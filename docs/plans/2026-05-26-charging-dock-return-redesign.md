# 回充电桩功能重新设计实现方案

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** APP 触发回充电桩时，在没有喷涂任务运行的前提下，让 B2W 使用现有导航主流程自动走到充电桩点位；回充过程中不执行机械臂喷涂、不触发电磁阀；不新增独立 `start_charging.sh` 之类的新启动脚本。

**Architecture:** 复用现有 `start_all.sh` + `b2w_nav_node` + `/tmp/start_all.*` 进程控制链路。`0x0C` 不再启动单独脚本，而是以“回桩任务模式”启动现有 `start_all.sh`；`b2w_nav_node` 增加 `operation_mode` 参数，`spray` 模式保持原喷涂行为，`dock_return` 模式只导航到点、不进入机械臂/喷枪状态。由于回桩也占用同一个 `start_all` 主流程，pause/restart/stop 与互斥都沿用现有安全路径，避免第二套 PID/PGID 文件和第二套 shell 生命周期。

**Tech Stack:** ROS 2 Humble, C++ rclcpp, bash `start_all.sh`, APP TCP 协议 `0x01/0x02/0x03/0x0C/0x10`, Unitree DDS SportClient。

---

## 一、需求确认与设计边界

### 明确需求

1. 未开启喷涂任务时，APP 按“回充电桩”按钮，可随时回到充电桩。
2. 喷涂任务进行过程中，如果直接 `0x03 stop` 终止当前任务，之后可以回充电桩。
3. 喷涂任务进行过程中：
   - `0x02 pause` 暂停任务，后续 `0x10 restart` 恢复任务，此过程中不能回充电桩。
   - `0x02 pause` 暂停任务，后续 `0x03 stop` 结束任务后，同第 2 条，可以回充电桩。
4. 回充过程中，`0x03 stop` 仍可中断当前回充任务。
5. 回充结束后，不自动开始喷涂任务；只有 APP 再发 `0x01 start` 才能重新开始喷涂任务。
6. 回充任务只是“自动走到那个点”，不执行机械臂动作、不喷涂、不触发继电器。
7. 不新建 `start_charging.sh` 等额外启动脚本；尽量复用现有启动、停止、安全控制链路。

### 设计边界

- 本方案保留 `gnss_charging.txt` 作为充电桩点位文件，但只作为导航点位输入，不代表新流程。
- `gnss_charging.txt` 推荐部署路径固定为：`/home/test/gnss_charging.txt`。
- 充电桩点位应使用与 `/epsg_position` 一致的 HEPOS-correct EPSG:2100 坐标。
- 不引入新的 ROS 服务类型；仅增加节点参数和少量启动参数。
- 不再使用 `/tmp/start_charging.pid`、`/tmp/start_charging.pgid`。

---

## 二、目标状态机

```
无任务 / stop 后 / 回桩完成
  ├─ 0x01 Start  → start_all.sh --mode spray       → 喷涂任务运行
  └─ 0x0C Dock   → start_all.sh --mode dock_return → 回桩任务运行

喷涂任务运行中
  ├─ 0x02 Pause   → 喷涂任务暂停，start_all 仍存活 → 0x0C 拒绝
  ├─ 0x10 Restart → 喷涂任务恢复，start_all 仍存活 → 0x0C 拒绝
  └─ 0x03 Stop    → 停止 start_all，清理 run files → 允许 0x0C

回桩任务运行中
  ├─ 0x03 Stop    → 停止 start_all，清理 run files → 回桩中断
  ├─ 0x01 Start   → 拒绝；必须先 stop 或等回桩完成
  ├─ 0x02 Pause   → 拒绝或忽略；回桩不支持 pause/restart
  └─ 0x10 Restart → 拒绝或忽略；回桩不支持 pause/restart

回桩完成
  └─ b2w_nav_node stop_on_completion=true 自动退出
     start_all.sh cleanup 删除 /tmp/start_all.*
     系统回到“无任务”状态；只有 0x01 Start 才会重新开始喷涂
```

关键点：回桩任务和喷涂任务共用 `/tmp/start_all.pid`、`/tmp/start_all.pgid`、`/tmp/start_all.ready`。因此只要任一任务正在运行，另一任务天然无法启动。

---

## 三、推荐实现方案

### 3.1 修改 `b2w_nav_node`：增加任务模式参数

文件：`b2w_navigation_ws/src/main.cpp`

新增参数：

```cpp
this->declare_parameter<std::string>("operation_mode", "spray");
this->declare_parameter<bool>("stop_on_completion", false);
```

新增成员：

```cpp
std::string operation_mode_ = "spray";
bool dock_return_mode_ = false;
bool stop_on_completion_ = false;
```

读取参数后校验：

```cpp
this->get_parameter("operation_mode", operation_mode_);
this->get_parameter("stop_on_completion", stop_on_completion_);

if (operation_mode_ == "dock_return") {
    dock_return_mode_ = true;
} else if (operation_mode_ == "spray") {
    dock_return_mode_ = false;
} else {
    RCLCPP_FATAL(this->get_logger(),
        "Invalid operation_mode='%s'. Expected 'spray' or 'dock_return'.",
        operation_mode_.c_str());
    rclcpp::shutdown();
    return;
}
```

### 3.2 回桩模式跳过机械臂和喷枪

当前主流程到达 waypoint 后会进入：

`WAITING_FOR_FRESH_RTK → EXECUTING_ARM_TASK → TRIGGERING_RELAY → RESETTING_ARM → GET_NEXT_WAYPOINT`

回桩模式不应进入这些状态。

推荐改法：在“到达目标点后准备进入机械臂流程”的位置，增加分支：

```cpp
if (dock_return_mode_) {
    RCLCPP_INFO(this->get_logger(),
        "Dock-return waypoint reached; skipping arm/relay spray sequence.");
    current_waypoint_index_++;
    state_ = GET_NEXT_WAYPOINT;
    return;
}
```

放置位置：到达目标点、`state_` 原本准备切到 `WAITING_FOR_FRESH_RTK` 之前。

实现原则：
- `dock_return_mode_ == true` 时绝不调用：
  - `/z1_move_to_target`
  - `/trigger_valve_ch1`
  - `/z1_reset_arm` 的正常喷涂复位流程
- 是否在回桩开始前主动复位机械臂不在本轮需求内；如必须做，也应走现有安全复位机制，不引入新脚本。

### 3.3 回桩完成后自动退出导航节点

`FINISH_ALL_POINTS` 中已有 `stop_on_completion_` 设计，应保留：

```cpp
case FINISH_ALL_POINTS:
{
    sport_client_.Move(0, 0, 0);
    if (stop_on_completion_) {
        RCLCPP_INFO(this->get_logger(),
            "All waypoints completed. stop_on_completion=true, shutting down.");
        rclcpp::shutdown();
        return;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "All waypoints completed. Robot idle at final position.");
    break;
}
```

喷涂模式：`stop_on_completion=false`，保持原行为。

回桩模式：`stop_on_completion=true`，到达充电桩后退出 `b2w_nav_node`，然后 `start_all.sh` 正常 cleanup，系统回到无任务状态。

---

## 四、修改 `start_all.sh`：复用现有脚本支持模式参数

文件：`start_all.sh`

### 4.1 增加模式参数

支持：

```bash
/home/test/start_all.sh --mode spray
/home/test/start_all.sh --mode dock_return
```

兼容旧调用：无参数等价 `--mode spray`。

脚本开头新增：

```bash
MISSION_MODE="spray"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode)
            MISSION_MODE="${2:-}"
            shift 2
            ;;
        --dock-return)
            MISSION_MODE="dock_return"
            shift
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 2
            ;;
    esac
done

case "$MISSION_MODE" in
    spray|dock_return) ;;
    *)
        echo "Invalid MISSION_MODE=$MISSION_MODE" >&2
        exit 2
        ;;
esac
```

### 4.2 根据模式选择点位文件和节点参数

新增：

```bash
if [[ "$MISSION_MODE" == "dock_return" ]]; then
    WAYPOINT_FILE="${ROBOT_HOME}/gnss_charging.txt"
    STOP_ON_COMPLETION="true"
else
    WAYPOINT_FILE="${ROBOT_HOME}/gnss_waypoints.txt"
    STOP_ON_COMPLETION="false"
fi
```

启动前检查：

```bash
if [[ ! -s "$WAYPOINT_FILE" ]]; then
    echo "ERROR: waypoint file not found or empty: $WAYPOINT_FILE" >&2
    exit 1
fi
```

启动 `b2w_nav_node` 时传参：

```bash
ros2 run b2w_navigation_controller b2w_nav_node \
    --ros-args \
    -p waypoint_file_path:="$WAYPOINT_FILE" \
    -p operation_mode:="$MISSION_MODE" \
    -p stop_on_completion:="$STOP_ON_COMPLETION" \
    >> "$NAV_LOG" 2>&1 &
PIDS+=("$!")
```

注意：这里必须用绝对路径传给 `waypoint_file_path`，不要再依赖当前工作目录。

### 4.3 ready 文件记录模式

当前 `/tmp/start_all.ready` 只记录 service/probe 状态。建议追加模式：

```bash
printf 'state=ready\nmode=%s\nservice_ready=1\nprobe_ok=1\n' "$MISSION_MODE" > "$READY_FILE"
```

其他状态同理写入：

```bash
mode=$MISSION_MODE
```

用途：APP 节点可区分当前 `start_all` 是喷涂任务还是回桩任务，从而拒绝回桩中的 pause/restart。

---

## 五、修改 `app_node.cpp`：0x0C 复用 start_all

文件：`app_ws/src/app_node.cpp`

### 5.1 删除旧回桩实现中的独立脚本逻辑

应删除/回滚：

- `/tmp/start_charging.pid` 检查。
- `/tmp/start_charging.pgid` 检查。
- `setsid /home/test/start_charging.sh &`。
- stop 里专门 kill start_charging 的逻辑。
- 新增文件 `start_charging.sh` 不再需要。

### 5.2 增加读取 start_all 当前模式的辅助函数

```cpp
std::optional<std::string> readStartAllMode() const
{
    std::ifstream ready_file("/tmp/start_all.ready");
    if (!ready_file.is_open()) {
        return std::nullopt;
    }
    std::string line;
    while (std::getline(ready_file, line)) {
        constexpr const char *prefix = "mode=";
        if (line.rfind(prefix, 0) == 0) {
            return line.substr(std::strlen(prefix));
        }
    }
    return std::nullopt;
}
```

### 5.3 0x01 Start：仅在没有任何 start_all 运行时启动喷涂

当前 0x01 已经检查 `/tmp/start_all.pid`，建议改成可信校验：

```cpp
StartAllTarget target;
if (findStartAllTarget(target)) {
    const auto mode = readStartAllMode().value_or("unknown");
    RCLCPP_WARN(this->get_logger(),
        "Start rejected: start_all already running, mode=%s, pid=%ld, pgid=%ld.",
        mode.c_str(), static_cast<long>(target.pid), static_cast<long>(target.pgid));
    return;
}
```

然后启动喷涂：

```cpp
const int rc = system("bash -c 'setsid /home/test/start_all.sh --mode spray &'");
```

保留 `markAppControlSessionActive("start command")`。

### 5.4 0x0C Dock Return：仅在没有任何 start_all 运行时启动回桩

新增/替换 0x0C 分支：

```cpp
if (instruction_type == 0x0C) {
    RCLCPP_INFO(this->get_logger(), "Received command: return to charging dock (0x0C)");

    StartAllTarget target;
    if (findStartAllTarget(target)) {
        const auto mode = readStartAllMode().value_or("unknown");
        RCLCPP_WARN(this->get_logger(),
            "Dock return rejected: start_all already running, mode=%s, pid=%ld, pgid=%ld. Stop current task first.",
            mode.c_str(), static_cast<long>(target.pid), static_cast<long>(target.pgid));
        return;
    }

    constexpr const char *charging_file = "/home/test/gnss_charging.txt";
    if (access(charging_file, R_OK) != 0) {
        RCLCPP_ERROR(this->get_logger(),
            "Dock return rejected: %s is not readable.", charging_file);
        return;
    }

    const int rc = system("bash -c 'setsid /home/test/start_all.sh --mode dock_return &' ");
    RCLCPP_INFO(this->get_logger(), "Dock return start_all system() returned %d", rc);
    markAppControlSessionActive("dock return command");
    return;
}
```

实现注意：
- 这里不要发布 `remote_command = "start"`，避免下游误认为喷涂任务开始。
- 可以新增命令字符串 `dock_return`，但如果没有下游消费，日志即可。
- 如果要发布 `/remote_command`，应发布明确值 `dock_return`，不要复用 `start`。

### 5.5 0x02 Pause / 0x10 Restart：只允许喷涂任务使用

回桩任务不支持 pause/restart，避免客户误解状态。

`handlePauseCommand()` 前增加：

```cpp
if (readStartAllMode().value_or("") == "dock_return") {
    RCLCPP_WARN(this->get_logger(), "Pause rejected: dock_return mode only supports stop.");
    return;
}
```

`0x10 restart` 分支同理：

```cpp
if (readStartAllMode().value_or("") == "dock_return") {
    RCLCPP_WARN(this->get_logger(), "Restart rejected: dock_return mode only supports stop.");
    return;
}
```

### 5.6 0x03 Stop：统一停止当前 start_all，不区分喷涂或回桩

保留现有 `triggerStartAllFailSafeStop("stop command")` 主体。

由于回桩也使用 `/tmp/start_all.pid/.pgid`，stop 会自然中断回桩任务，不需要任何 charging 专用逻辑。

建议调整日志：

```cpp
const auto mode = readStartAllMode().value_or("unknown");
RCLCPP_INFO(this->get_logger(), "Received stop command for start_all mode=%s", mode.c_str());
```

---

## 六、文件处理

### 6.1 删除不需要的文件

如果之前已经新增，应删除：

- `start_charging.sh`

如果之前已经加入代码，应回滚相关逻辑：

- `/tmp/start_charging.pid`
- `/tmp/start_charging.pgid`
- `isProcessRunning("/tmp/start_charging.pid")`
- `setsid /home/test/start_charging.sh &`
- stop 中 kill charging pgid 的代码

### 6.2 保留点位文件

保留：

- `gnss_charging.txt`

部署路径：

- `/home/test/gnss_charging.txt`

格式沿用 `gnss_waypoints.txt`：

```text
id,x,y,z,
```

建议仅放 1 个点：充电桩目标停车点。

---

## 七、实现任务拆分

### Task 1: 回滚独立 start_charging 方案

**Objective:** 移除第二套回桩脚本和第二套 PID/PGID 生命周期。

**Files:**
- Delete: `start_charging.sh`
- Modify: `app_ws/src/app_node.cpp`

**Steps:**
1. 删除仓库根目录 `start_charging.sh`。
2. 删除 `app_node.cpp` 中所有 `/tmp/start_charging.pid`、`/tmp/start_charging.pgid` 相关逻辑。
3. 删除 `0x01 Start` 中对 `start_charging.pid` 的检查。
4. 删除 `0x0C` 中 `setsid /home/test/start_charging.sh &`。
5. 删除 `triggerStartAllFailSafeStop()` 中 charging 专用清理代码。

**Verification:**

```bash
grep -R "start_charging" -n app_ws/src/app_node.cpp start_charging.sh
```

Expected:
- `start_charging.sh` 不存在。
- `app_node.cpp` 无 `start_charging` 命中。

---

### Task 2: b2w_nav_node 增加 operation_mode

**Objective:** 让同一个导航节点支持喷涂和回桩两种行为。

**Files:**
- Modify: `b2w_navigation_ws/src/main.cpp`

**Steps:**
1. 声明参数 `operation_mode`，默认 `spray`。
2. 读取参数并校验只允许 `spray` / `dock_return`。
3. 增加成员 `operation_mode_`、`dock_return_mode_`。
4. 启动日志打印当前模式。

**Verification:**

```bash
cd b2w_navigation_ws
colcon build --packages-select b2w_navigation_controller
```

Expected:
- 编译通过。

---

### Task 3: 回桩模式跳过机械臂和喷枪

**Objective:** 回桩模式只导航，不执行喷涂动作。

**Files:**
- Modify: `b2w_navigation_ws/src/main.cpp`

**Steps:**
1. 找到到达 waypoint 后进入 `WAITING_FOR_FRESH_RTK` 的位置。
2. 在该位置添加 `if (dock_return_mode_)` 分支。
3. 分支中递增 `current_waypoint_index_`，切到 `GET_NEXT_WAYPOINT`。
4. 确认回桩模式不会进入 `EXECUTING_ARM_TASK`、`TRIGGERING_RELAY`、`RESETTING_ARM`。

**Verification:**

```bash
cd b2w_navigation_ws
colcon build --packages-select b2w_navigation_controller
```

Expected:
- 编译通过。
- 日志中可看到 `Dock-return waypoint reached; skipping arm/relay spray sequence.`。

---

### Task 4: start_all.sh 支持 --mode

**Objective:** 不新增脚本，复用 start_all 启动喷涂或回桩。

**Files:**
- Modify: `start_all.sh`

**Steps:**
1. 在脚本开头解析 `--mode spray|dock_return`。
2. 默认无参数为 `spray`。
3. 根据模式选择：
   - spray → `/home/test/gnss_waypoints.txt`，`stop_on_completion=false`
   - dock_return → `/home/test/gnss_charging.txt`，`stop_on_completion=true`
4. 启动 `b2w_nav_node` 时传入：
   - `waypoint_file_path`
   - `operation_mode`
   - `stop_on_completion`
5. `/tmp/start_all.ready` 写入 `mode=$MISSION_MODE`。

**Verification:**

```bash
bash -n start_all.sh
```

Expected:
- 语法通过。

---

### Task 5: app_node 0x01/0x0C 统一使用 start_all

**Objective:** 让 APP start 和 dock return 互斥、都通过现有 start_all 生命周期管理。

**Files:**
- Modify: `app_ws/src/app_node.cpp`

**Steps:**
1. 新增 `readStartAllMode()`。
2. 0x01 Start：如果 `findStartAllTarget()` 成功，则拒绝；否则启动 `/home/test/start_all.sh --mode spray`。
3. 0x0C Dock Return：如果 `findStartAllTarget()` 成功，则拒绝；否则检查 `/home/test/gnss_charging.txt` 并启动 `/home/test/start_all.sh --mode dock_return`。
4. 0x0C 不发布 `remote_command=start`。
5. 0x02 Pause 和 0x10 Restart 在 `mode=dock_return` 时拒绝。
6. 0x03 Stop 保持统一停止 `start_all`。

**Verification:**

```bash
cd app_ws
colcon build --packages-select robot_tcp
```

Expected:
- 编译通过。

---

### Task 6: 部署与编译顺序

**Objective:** 按仓库要求部署修改后的二进制和脚本。

**Commands:**

```bash
cd /home/oneko/projects/spray_b2w_robot_project_greek

cd b2w_navigation_ws && colcon build --packages-select b2w_navigation_controller && source install/setup.bash && cd ..
cd app_ws && colcon build --packages-select robot_tcp && source install/setup.bash && cd ..

sudo setcap cap_net_raw+ep \
  b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_nav_node

cp start_all.sh /home/test/start_all.sh
chmod +x /home/test/start_all.sh
cp gnss_charging.txt /home/test/gnss_charging.txt
```

注意：如果在工控机上构建，按实际 workspace 路径执行；跨 workspace 依赖仍应遵守 `CLAUDE.md` 的 source 顺序。

---

## 八、验收测试清单

### 测试 1：空闲回桩

操作：

```text
确保 /tmp/start_all.pid 不存在或 start_all 未运行
APP 发送 0x0C
```

预期：

```text
start_all.sh --mode dock_return 启动
/tmp/start_all.ready 包含 mode=dock_return
b2w_nav_node 加载 /home/test/gnss_charging.txt
到达点后不调用 /z1_move_to_target，不调用 /trigger_valve_ch1
到达后 b2w_nav_node 退出，/tmp/start_all.* 被清理
```

### 测试 2：喷涂任务中拒绝回桩

操作：

```text
APP 发送 0x01 start
喷涂任务运行中发送 0x0C
```

预期：

```text
0x0C 被拒绝
日志显示 start_all already running mode=spray
不启动第二个 b2w_nav_node
```

### 测试 3：喷涂任务中 stop 后回桩

操作：

```text
APP 发送 0x01 start
喷涂任务运行中发送 0x03 stop
确认 start_all 停止
APP 发送 0x0C
```

预期：

```text
stop 清理 /tmp/start_all.*
0x0C 成功启动 mode=dock_return
```

### 测试 4：pause 后 restart 期间拒绝回桩

操作：

```text
APP 发送 0x01 start
APP 发送 0x02 pause
APP 发送 0x0C
APP 发送 0x10 restart
APP 再发送 0x0C
```

预期：

```text
pause 后 start_all 仍运行，0x0C 拒绝
restart 后 start_all 仍运行，0x0C 继续拒绝
```

### 测试 5：pause 后 stop，再回桩

操作：

```text
APP 发送 0x01 start
APP 发送 0x02 pause
APP 发送 0x03 stop
APP 发送 0x0C
```

预期：

```text
stop 清理 start_all
0x0C 成功启动 mode=dock_return
```

### 测试 6：回桩过程中 stop 可中断

操作：

```text
APP 发送 0x0C
回桩进行中发送 0x03 stop
```

预期：

```text
triggerStartAllFailSafeStop 停止当前 start_all 进程组
b2w_nav_node 被终止
/tmp/start_all.* 被清理
机器人停止运动
```

### 测试 7：回桩过程中 start 被拒绝

操作：

```text
APP 发送 0x0C
回桩进行中发送 0x01 start
```

预期：

```text
0x01 被拒绝
日志显示 start_all already running mode=dock_return
不启动喷涂任务
```

### 测试 8：回桩完成后仅 start 重新开始喷涂

操作：

```text
APP 发送 0x0C
等待回桩完成
不要自动发送 0x01
观察系统状态
再手动发送 0x01 start
```

预期：

```text
回桩完成后系统空闲，不自动喷涂
只有收到 0x01 后才启动 mode=spray
```

---

## 九、方案优点

1. 没有第二套脚本，没有 `/tmp/start_charging.*`，生命周期简单。
2. stop/pause/restart 继续复用现有 start_all 安全控制，不引入新的 kill 逻辑。
3. 喷涂任务和回桩任务天然互斥：同一时间只有一个 start_all。
4. 回桩只是一种 `operation_mode`，复用导航、避障、RTK、DDS 逻辑。
5. 回桩完成后 `stop_on_completion=true` 自动退出，满足“只有 start 才重新开始喷涂”。
6. 不改变 APP 协议帧，只重新定义 0x0C 的后端执行方式。

---

## 十、需要特别避免的实现错误

1. 不要再新建 `start_charging.sh`。
2. 不要再引入 `/tmp/start_charging.pid` / `/tmp/start_charging.pgid`。
3. 0x0C 不要发布 `remote_command=start`。
4. 回桩模式不要调用机械臂和喷枪服务。
5. `gnss_charging.txt` 不要用相对路径启动，必须传绝对路径。
6. pause/restart 不应作用于回桩任务；回桩任务只支持 stop 中断。
7. 回桩完成后不要自动转入喷涂任务。
