# Pause/Stop/断联时机械臂安全复位 改动报告

> 实施日期：2026-05-20
> 分支：ygs
> 类型：缺陷修复 + 安全加固（无破坏性接口变更）

> 完成状态：✅ 已完成代码实现与源码级回归验证。当前仓库中 `b2w_navigation_ws/src/main.cpp` 已包含 pause/断联 pause 时的机械臂安全复位子流程，`app_ws/src/app_node.cpp` 已包含 stop/fail-safe 路径的 best-effort `/z1_reset_arm` 请求；`tests/test_app_node_failsafe.py` 与 `tests/test_b2w_navigation_arm_safety_reset.py` 已通过。真实 ROS 2 Humble 目标机编译与现场动作验收仍以 §8.3 / §8.4 为准。

---

## 1. 问题陈述

现场观察：当 APP 触发 `0x02 pause`、`0x03 stop`，或 APP 心跳/TCP 断联触发自动 pause 时，如果当时 Z1 机械臂正处于工作位姿（已展开喷涂、或 MoveJ 进行中），机械臂会**停留在工作位姿不再自动复位**，需要现场人员手动介入。

## 2. 根因

主控状态机在 `b2w_navigation_ws/src/main.cpp` 中通过两条不同路径处理安全停止，两条路径**都绕过了正常喷涂状态机里的 `RESETTING_ARM` 阶段**：

1. **pause 路径**：`HandleEmergencyStop` 只设置 `paused_=true` 并发底盘 `Move(0,0,0)`，不调机械臂；`ControlLoop` 在 `paused_` 时直接 `return`，状态机冻结，永远不会自然进入 `RESETTING_ARM`。
2. **stop 路径**：`app_ws/src/app_node.cpp` 中 `triggerStartAllFailSafeStop` 发底盘 StopMove 后直接 kill `start_all.sh` 进程组，主导航节点被杀掉，机械臂复位逻辑随主控一起死掉。`z1_arm_controller_node` 是 `tcp_base_ctl.sh` 常驻基础节点，不属于 `start_all.sh`，stop 后仍存活，但没有任何节点向它请求复位。

## 3. 设计目标

1. pause / 断联 pause / 心跳超时 pause：底盘立即停，机械臂尽快进入安全复位流程
2. stop：底盘立即停 + 主任务停止，但不能因为 kill `start_all` 而丢失机械臂复位 → 由常驻 `app_node` 直接请求 `/z1_reset_arm`
3. 不引入 `Damp()` / `StandDown()` 作为兜底（沿用现有 StopMove-only 不变量）
4. 复位请求幂等、带日志、带超时；失败 ERROR 但不阻塞安全停止无限等待
5. 不强行中断正在执行的 MoveJ（Z1 SDK 没有 cancel 接口），接受"当前 MoveJ 完成后 backToStart 排队执行"

## 4. 实现方案

### 4.1 入口分工（双入口补复位） ✅

| 入口 | 谁负责调 `/z1_reset_arm` | 实现位置 |
|---|---|---|
| `/emergency_stop` (0x02 pause / 断联 pause / 心跳 pause) | `b2w_nav_node` | `b2w_navigation_ws/src/main.cpp` |
| fail-safe stop（0x03 stop / emergency stop timeout / bad response / exception / pause fallback） | `robot_tcp_node` (常驻 APP 节点) | `app_ws/src/app_node.cpp` |

### 4.2 `b2w_nav_node` 安全复位子流程 ✅

新增三个 private 方法（顺序固定，紧邻 `HandleEmergencyStop` 之前，被静态测试断言锁定）：

```text
IsArmRelatedState() → RequestArmSafetyReset() → PollArmSafetyReset() → HandleEmergencyStop()
```

**`IsArmRelatedState()`**：仅在以下状态返回 true，其他状态 pause 不会动机械臂：

- `EXECUTING_ARM_TASK`
- `RETRYING_ARM_AFTER_FORWARD`
- `RETRYING_ARM_AFTER_BACKUP`
- `TRIGGERING_RELAY`
- `RESETTING_ARM`

**`RequestArmSafetyReset(reason)`**（核心去重与复用逻辑）：

1. 不开启 (`arm_reset_on_pause_=false`) → skip
2. 不在机械臂状态 → skip
3. 已有 pending safety reset → skip
4. 若当前在 `RESETTING_ARM` 且主流程已发出 `/z1_reset_arm`（`arm_reset_task_requested_=true`）→ **复用** `arm_reset_task_future_`，避免向 Z1 控制器重复入队同一个 backToStart
5. `/z1_reset_arm` 服务未就绪 → 节流 WARN 并推迟
6. 否则 `async_send_request` 并保存 future + 时间戳

**`PollArmSafetyReset()`**（ControlLoop 顶部调用，pause 也跑得通）：

1. 没有 pending 请求 → 立即返回
2. future 已 ready → 读取结果：
   - 成功 → 强制 `state_ = GET_NEXT_WAYPOINT`（推进进度，跳过当前点）
   - 失败 / 异常 → 强制 `state_ = WAITING_FOR_WAYPOINT`（不推进进度，继续下一点）
   - **清零所有 in-flight 标记**：`arm_task_requested_` / `arm_reset_task_requested_` / `ch1_trigger_task_requested_` 以及对应 future，避免 resume 时残留标记导致误进 TRIGGERING_RELAY
3. 超时 (`arm_safety_reset_timeout_seconds_` 默认 15s) → ERROR，同样强制状态机回安全态并清零标记

**`HandleEmergencyStop` 修改**：

- 第一次 pause：原有逻辑（`paused_=true` + StopMove）+ `RequestArmSafetyReset("emergency stop at state X")`
- 已 paused 时重复收到 pause：原本直接返回，现在仍然调用 `RequestArmSafetyReset("emergency stop re-issued while already paused")` 兜底（覆盖第一次服务未就绪的情况）

**`ControlLoop` 修改（顶部三段）**：

```cpp
PublishOdom();
PollArmSafetyReset();                                    // 即使 paused 也轮询

if (paused_ && arm_reset_on_pause_ && IsArmRelatedState()
    && !arm_safety_reset_requested_) {
    RequestArmSafetyReset("ControlLoop retry while paused");   // 每 tick 重试派发
}

if (arm_safety_reset_requested_) {                       // 安全 hold 早返回
    sport_client_.Move(0,0,0);  // latched
    return;
}

if (paused_) { ... return; }                             // 原有 pause 冻结
```

> **不变量**：`if (paused_) { ... return; }` 块体内**不包含** `state_ =`。状态强制全部发生在 `PollArmSafetyReset()` 内（在 paused 块之外）。这条不变量被 `tests/test_b2w_navigation_arm_safety_reset.py::test_control_loop_polls_arm_safety_reset_before_paused_return` 锁定。

### 4.3 `robot_tcp_node` best-effort 复位 ✅

新增方法 `requestArmResetBestEffort(reason)`，放在 `triggerStartAllFailSafeStop` 之前（顺序被静态测试锁定）：

1. `wait_for_service(500ms)` —— 服务不可用只 WARN，不阻塞 stop
2. `async_send_request` `/z1_reset_arm`，callback 打印 success/failure/exception
3. 注册 `arm_reset_timeout_seconds_`（默认 15s）safety timer：超时只 ERROR 不重发不阻塞；与 `response_done` atomic 配合避免回调和 timer 双触发
4. 复用现有 `registerSafetyTimer/cancelSafetyTimer` 机制，timer 寿命自管

**`triggerStartAllFailSafeStop` 修改**：执行顺序硬性约束（被测试锁定）

```text
publishSafetyStopMove(reason)
    ↓
requestArmResetBestEffort(reason)        ← 新增
    ↓
findStartAllTarget + kill(-target.pgid, SIGTERM/SIGKILL)
    ↓
clearStartAllRunFiles
```

这条顺序保证：底盘最优先停 → 复位异步发出（不等响应）→ 才去 kill 主任务进程组。

### 4.4 为什么 PollArmSafetyReset 完成时要"强制状态机回到 GET_NEXT_WAYPOINT/WAITING_FOR_WAYPOINT" ✅

不强制的情况下会出现以下危险时序：

1. pause 发生在 `EXECUTING_ARM_TASK`，此时 `arm_task_future_` 还在等 MoveJ 完成
2. `RequestArmSafetyReset` 发出 backToStart，进入 Z1 单线程队列，排在 MoveJ 后面
3. MoveJ 完成（`arm_task_future_` 变 ready，结果为 success）
4. backToStart 执行完成（机械臂回到 home）
5. 用户 resume → `paused_=false`，状态机仍在 `EXECUTING_ARM_TASK`
6. ControlLoop 读取 `arm_task_future_` 已 ready 且 success → 跳转 `TRIGGERING_RELAY` → 打开喷枪 → 机械臂在 home 位置喷涂空气

强制状态机回到 `GET_NEXT_WAYPOINT` / `WAITING_FOR_WAYPOINT` 并清零所有 in-flight 标记后，resume 起步即为安全态，跳过当前点的喷涂周期。

## 5. 修改文件清单 ✅

| 文件 | 类型 | 改动摘要 |
|---|---|---|
| `b2w_navigation_ws/src/main.cpp` | MODIFY ✅ | +参数 `arm_reset_on_pause` / `arm_safety_reset_timeout_seconds`；+成员 3 个；+方法 `IsArmRelatedState` / `RequestArmSafetyReset` / `PollArmSafetyReset`；改 `HandleEmergencyStop` / `ControlLoop` |
| `app_ws/src/app_node.cpp` | MODIFY ✅ | +include `z1_arm_controller_cpp/srv/move_arm.hpp`；+参数 `arm_reset_timeout_seconds`；+成员 `z1_reset_arm_client_` / `arm_reset_timeout_seconds_`；+方法 `requestArmResetBestEffort`；改 `triggerStartAllFailSafeStop` |
| `app_ws/CMakeLists.txt` | MODIFY ✅ | +`find_package(z1_arm_controller_cpp REQUIRED)` 与 `ament_target_dependencies` 中追加 |
| `app_ws/package.xml` | MODIFY ✅ | +`<depend>std_srvs</depend>` 与 `<depend>z1_arm_controller_cpp</depend>` |
| `tests/test_app_node_failsafe.py` | MODIFY ✅ | +`test_fail_safe_stop_requests_best_effort_arm_reset_before_killing_start_all` |
| `tests/test_b2w_navigation_arm_safety_reset.py` | CREATE ✅ | 3 个测试覆盖状态枚举、ControlLoop 顺序、async/去重/超时 |

## 6. 新增可调参数 ✅

| 节点 | 参数 | 默认 | 说明 |
|---|---|---|---|
| `b2w_nav_node` | `arm_reset_on_pause` | `true` | 设为 false 可关闭 pause 时的机械臂安全复位（调试用） ✅ |
| `b2w_nav_node` | `arm_safety_reset_timeout_seconds` | `15.0` | 超时后强制状态机回 `WAITING_FOR_WAYPOINT` ✅ |
| `robot_tcp_node` | `arm_reset_timeout_seconds` | `15.0` | <1.0 会被 clamp 到 1.0 ✅ |

可在 `b2w_navigation_ws/config/b2w_controller_params.yaml` 中覆盖前两项；后者目前无对应 YAML，可通过 launch 参数注入或在 `tcp_base_ctl.sh` 启动行中追加。

## 7. 安全不变量（与历史 plan 对齐）

| # | 不变量 | 本次是否破坏 |
|---|---|---|
| 1 | 只有 APP 控制子命令 `0x0A` 可触发 `StandDown()` | 否 |
| 2 | 心跳功能码 `0xFF` 只更新在线状态 | 否 |
| 3 | TCP 断开/心跳超时/pause fallback/stop/emergency-stop 异常**只用 StopMove**，不用 StandDown/Damp | 否（新增的复位调用 `/z1_reset_arm`，与底盘姿态指令完全无关） |
| 4 | APP 未进入作业控制状态时不误暂停 | 否 |
| 5 | APP 控制会话中心跳超时必须保护 | 否 |
| 6 | 多客户端重连不误触发当前连接的保护 | 否 |
| **新增** | **pause/stop/断联 pause 发生在机械臂工作状态时，必须最终让机械臂返回 backToStart 位姿，或在超时后明确 ERROR 通报** | 由本改动建立 |

## 8. 测试矩阵

### 8.1 静态回归（已通过） ✅

```bash
python3 -m pytest tests/test_app_node_failsafe.py tests/test_b2w_navigation_arm_safety_reset.py -v
```

结果：**19 passed**

| 测试文件 | 用例数 | 关键断言 |
|---|---|---|
| `tests/test_b2w_navigation_arm_safety_reset.py` | 3 | `IsArmRelatedState` 含 EXECUTING_ARM_TASK / TRIGGERING_RELAY / RESETTING_ARM；`HandleEmergencyStop` 调用 `RequestArmSafetyReset`；`ControlLoop` 中 `PollArmSafetyReset()` 在 `if (paused_)` 之前；`if (paused_)` 块体内不含 `state_ =`；`RequestArmSafetyReset` 含 `async_send_request` / `z1_reset_arm_client_`；`PollArmSafetyReset` 含 `wait_for(std::chrono::milliseconds(0))` / `arm_safety_reset_timeout_seconds_` / 清零标记 |
| `tests/test_app_node_failsafe.py`（新增 1，原有 15） | 16 | include `move_arm.hpp`、有 `z1_reset_arm_client_` 与 `requestArmResetBestEffort`；`triggerStartAllFailSafeStop` 中调用顺序 `publishSafetyStopMove → requestArmResetBestEffort → kill SIGTERM`；`requestArmResetBestEffort` 体内含 `wait_for_service` / `async_send_request` / `arm_reset_timeout_seconds_` 且无递归调用 |

### 8.2 大括号/小括号平衡（已通过） ✅

```text
b2w main.cpp braces: {=154 }=154 parens: (=711 )=711
app_node.cpp braces: {=257 }=257 parens: (=1054 )=1054
```

### 8.3 真实 ROS 2 编译（待机器人本机执行）

本机为 ROS 2 Jazzy 且现有 `build/install/log` 目录 root 所有，无法本地复刻 Humble 编译。需在机器人本机 `/home/test/` 用户下：

```bash
# 依赖顺序：colcon_ws + z1_move_ws + spray_path_planner_ws 已存在则不重编
source /opt/ros/humble/setup.bash
source colcon_ws/install/setup.bash
source z1_move_ws/install/setup.bash
source spray_path_planner_ws/install/setup.bash

cd b2w_navigation_ws && colcon build --packages-select b2w_navigation_controller && source install/setup.bash && cd ..
cd app_ws && colcon build --packages-select robot_tcp && source install/setup.bash && cd ..

# 重新 setcap（重编后 cap_net_raw 会丢）
sudo setcap cap_net_raw+ep b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_nav_node
```

预期：0 warning / 0 error。

### 8.4 现场验证（待执行）

| 场景 | 步骤 | 预期日志 |
|---|---|---|
| 0x02 pause（机械臂喷涂中） | 让机械臂进入 `EXECUTING_ARM_TASK` 或 `TRIGGERING_RELAY` 中段，APP 下发 `0x02` | `b2w_navigation.log`: `Arm safety reset dispatched at state=EXECUTING_ARM_TASK (reason=emergency stop at state EXECUTING_ARM_TASK).` → `Arm safety reset succeeded ... Forcing state to GET_NEXT_WAYPOINT.`<br>`z1_arm.log`: 收到 `/z1_reset_arm` → `backToStart` 完成 |
| 0x03 stop（机械臂喷涂中） | 同上场景下下发 `0x03` | `robot_tcp.log`: `best-effort arm reset dispatched (reason=stop command, timeout=15.0s).` → `best-effort arm reset succeeded`<br>`z1_arm.log`: 复位 |
| APP 心跳超时（>3.5s 不送心跳） | 启动作业后 kill APP 端 | `robot_tcp.log`: `APP heartbeat timeout` → `handlePauseCommand`<br>`b2w_navigation.log`: `Arm safety reset dispatched ... (reason=emergency stop at state ...)` |
| APP TCP 断开 + 8s 重连超时 | 启动作业后断网 8s+ | 同上心跳超时路径 |
| 在非机械臂状态 pause（如 `MOVING_TO_TARGET`） | 在前进中下发 `0x02` | `b2w_navigation.log`: 无 "Arm safety reset dispatched"（`IsArmRelatedState`=false 跳过）；底盘正常停 |
| `/z1_reset_arm` 服务在 pause 时不可用 | 临时 `pkill z1_arm_controller`，pause 时下发 | `b2w_navigation.log`: 节流 WARN `/z1_reset_arm service not ready ... safety reset deferred`；恢复服务后下一 tick 自动派发 |
| 安全复位超时 | 仿真 Z1 卡住 >15s | `b2w_navigation.log`: `Arm safety reset timed out after Xs (limit=15.0s). Forcing state to WAITING_FOR_WAYPOINT.` |

## 9. 已知限制 & 后续可做的改进

1. **未强制关阀**：方案 B2（rs585_node 新增 `/force_close_relays`，复位前先关 CH1）本轮未实施。当前依赖 `/trigger_valve_ch1` 的脉冲式自关（默认 600ms 后自动关）。若现场使用带压实喷，建议后续接入 B2。
2. **不打断当前 MoveJ**：Z1 SDK 没有 cancel 接口，backToStart 会排在当前 MoveJ 之后执行。物理意义上是"先让当前动作做完，再立即回 home"，最坏情况下用户感知到 ~1-2 秒延迟。
3. **resume 跳过当前点**：安全复位完成后强制 `state_ = GET_NEXT_WAYPOINT`，意味着用户 pause 后 resume，**当前喷涂点会被跳过**。若需要"resume 后从当前点重做"，需要后续增加 `arm_safety_reset_recovery_policy` 参数与对应分支。
4. **`arm_reset_timeout_seconds` 没有 YAML 覆盖**：当前必须通过 launch 参数或命令行注入。

## 10. 回滚方式

如果现场出现非预期行为需要快速回滚到改动前：

1. **完全关闭新行为（不需重编译）**：
   ```bash
   ros2 param set /b2w_navigation_controller arm_reset_on_pause false
   ```
   只能关 b2w 侧的 pause 复位；stop 侧的 best-effort 没有运行时开关，需要回滚源码。
2. **源码回滚**：`git revert <本次 commit>` 后按 §8.3 重新编译。

## 11. 参考

- 上游 plan：`docs/plans/2026-05-20-app-heartbeat-disconnect-safety.md`（心跳/断联保护本身，本改动是其遗留的"机械臂残位"缺口的补丁）
- 关键代码：
  - `b2w_navigation_ws/src/main.cpp:382-590`（新增方法 + 修改的 HandleEmergencyStop / ControlLoop）
  - `app_ws/src/app_node.cpp:805-900`（requestArmResetBestEffort + triggerStartAllFailSafeStop）
- 测试：
  - `tests/test_b2w_navigation_arm_safety_reset.py`
  - `tests/test_app_node_failsafe.py::test_fail_safe_stop_requests_best_effort_arm_reset_before_killing_start_all`
- Gemini 审查记录：方案在 `/tmp/b2w_arm_reset_safety_plan.md` 中以 `gemini --approval-mode plan` 复核通过，重点关注的"喷枪安全顺序"风险记录在 §9.1 作为后续 follow-up
