# CLAUDE.md 更新计划（截至 2026-05-25）

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** 同步 CLAUDE.md 以反映自 commit `4207bde` 以来 30+ commits 的代码、配置、文档和架构变更。

**Architecture:** 逐节更新 CLAUDE.md，重点落在安全机制、话题归属变更（spray_path_planner → main.cpp）、新配置参数、新增测试和文档导航。

**Tech Stack:** Markdown, git

---

## 变更摘要（自上次 CLAUDE.md 更新：`4207bde`）

上次 CLAUDE.md 更新时同步了 Livox 登记、RS16 标记、话题表与 setcap 补丁。此后主要变更：

| 类别 | 关键变更 |
|------|---------|
| 安全机制 | Damp 禁用、心跳断联保护、pause/stop/断联强制机械臂安全复位、APP 暂停失效 StopMove 兜底 |
| 话题归属 | `/progress`、`/acquired_points`、`/unacquired_points` 从 spray_path_planner 移到 main.cpp |
| 配置新增 | `ins_parser_params.yaml`、`arm_safety_reset_timeout_seconds`、`arm_offset_x`、`arm_target_comp_x` |
| 配置调参 | pitch 90→84、heading_threshold 0.25→0.05、obstacle_range 1.1→1.2 |
| 文档新增 | 6+ 篇（安全机制、日志指南、网络拓扑、精度路线图、HEPOS 验证报告、Hermes 使用说明） |
| 测试 | `tests/` 目录 3 个静态验证脚本 |
| 工具脚本 | `resettime.sh`、`install_fake_hwclock.sh`、`record_gps_epsg_point.py`、`simulate_gnss_serial.py`、`compute_fj_plane.py` |

---

### Task 1: 更新「工作空间架构」表格

**Objective:** spray_path_planner 运行时角色变更

**Files:**
- Modify: `CLAUDE.md` — 第 113-127 行（工作空间架构表）

**Content:**

将 spray_path_planner 的描述从：
```
| `spray_path_planner_ws` | spray_path_planner | 读点文件、生成最短路径 |
```
改为：
```
| `spray_path_planner_ws` | spray_path_planner | 服务类型定义（GetNextWaypoint.srv / SetStartPoint.srv，仅编译时依赖；运行时节点不再启动，路径逻辑已内联到 main.cpp） |
```

---

### Task 2: 更新「ROS 2 话题总表」

**Objective:** 纠正 spray_path_planner 发布归属，补充 b2w_nav_node 的新发布话题

**Files:**
- Modify: `CLAUDE.md` — 第 133-173 行

**Content:**

删除以下三行（已从 spray_path_planner 移到 main.cpp）：
```
| `spray_path_planner_ws / spray_path_planner_node` | 发布 | `/progress` | ...
| `spray_path_planner_ws / spray_path_planner_node` | 发布 | `/acquired_points` | ...
| `spray_path_planner_ws / spray_path_planner_node` | 发布 | `/unacquired_points` | ...
```

在 `b2w_navigation_ws / b2w_nav_node` 的发布列表中添加（如果尚未完整，补充）：
```
| `b2w_navigation_ws / b2w_nav_node` | 发布 | `/progress` | `std_msgs/msg/Byte` | 喷涂进度百分比（0-100），main.cpp 直接计算发布 |
```

确认 `b2w_nav_node` 对 `/acquired_points` 和 `/unacquired_points` 的发布条目已存在（目前第 156-157 行已有，无需重复添加）。

添加 app_node 发布 `/joy` 的细分说明（替代旧 Damp 指令）：
```
| `app_ws / remote_control_node` | 发布 | `/joy` | `sensor_msgs/msg/Joy` | APP 手动控制输入（axes: 0x04-0x09 方向量，由 b2w_teleop_node 消费后调用 SportClient::Move()；buttons: 0x0A StandDown, 0x0B StandUp；buttons[2] 已禁用，不再触发 Damp） |
```

---

### Task 3: 更新「ROS 2 服务总表」

**Objective:** 标注服务调用状态变更

**Files:**
- Modify: `CLAUDE.md` — 第 175-195 行

**Content:**

将 `/get_next_waypoint` 和 `/set_start_point` 的说明更新：
```
| `spray_path_planner_ws / spray_path_planner_node` | 提供 | `/get_next_waypoint` | `spray_path_planner/srv/GetNextWaypoint` | （仅编译时依赖；运行时 main.cpp 直接读取 gnss_waypoints.txt，不再调用此服务） |
| `spray_path_planner_ws / spray_path_planner_node` | 提供 | `/set_start_point` | `spray_path_planner/srv/SetStartPoint` | （仅编译时依赖；运行时不再调用） |
```

删除这两行（main.cpp 不再调用）：
```
| `b2w_navigation_ws / b2w_nav_node` | 调用 | `/set_start_point` | ...
| `b2w_navigation_ws / b2w_nav_node` | 调用 | `/get_next_waypoint` | ...
```

---

### Task 4: 更新「接口注意事项」

**Objective:** 补充安全机制相关注意事项

**Files:**
- Modify: `CLAUDE.md` — 第 203-211 行

**Content:**

在现有注意事项末尾追加：
```
- `spray_path_planner` 节点在运行时不再启动；服务类型定义仍保留为编译时依赖（`b2w_navigation_ws` 的 CMakeLists.txt 和 package.xml 中 `find_package(spray_path_planner REQUIRED)`）。路径读取、点云发布、进度计算均在 `main.cpp` 内联完成。
- APP 端 `0xFF` 心跳仅用于应用层断联判断，不再触发 `Damp()`；断联后执行 `StopMove()` + 机械臂安全复位。
- `/joy.buttons[2]`（原 Damp 触发位）已禁用。b2w_teleop_node 不再处理 buttons[2]。
```

---

### Task 5: 新增「安全机制」章节

**Objective:** 系统化记录 pause/stop/断联的安全兜底

**Files:**
- Modify: `CLAUDE.md` — 在「TCP 应用协议」之后、「坐标系说明」之前插入新章节

**Content:**

```markdown
## 安全机制

### 断联保护

APP 心跳（0xFF）每 1 秒发送一次。`app_node.cpp` 以 `heartbeat_timeout_seconds`（默认 3.5s）为超时阈值。超时后：
1. 关闭 TCP 客户端 socket
2. 调用 `handlePauseCommand()`：优先尝试调用 `/emergency_stop` 服务
3. 如果 `/emergency_stop` 服务不可达，fallback：发布 StopMove（通过 `/joy` 全零）+ 对 `start_all.sh` 进程组发送 SIGTERM
4. 请求机械臂安全复位（如果处于 arm-related 状态）

注意：断联 **不会** 触发 `StandDown()` 或 `Damp()`（Damp 已全局禁用）。

### pause 安全兜底

APP 发送 `0x02 pause` → `handlePauseCommand()`：
1. 先尝试调用 `/emergency_stop` 服务（由 `b2w_nav_node` 提供）
2. 如果服务不可达（超时/节点未就绪），fallback：
   - 发布 `StopMove`（通过 `/joy` 全零）
   - 触发 `triggerStartAllFailSafeStop("pause fallback ...")` — 对 `start_all.sh` 进程组发送 SIGTERM
3. 请求机械臂安全复位（如果处于 arm-related 状态）

### 机械臂安全复位（arm_safety_reset）

在 `b2w_navigation_ws/src/main.cpp` 中实现：
- `IsArmRelatedState()` 判断当前状态是否为以下 5 种之一：
  EXECUTING_ARM_TASK / TRIGGERING_RELAY / RESETTING_ARM / RETRYING_ARM_AFTER_FORWARD / RETRYING_ARM_AFTER_BACKUP
- `RequestArmSafetyReset(reason)` 在 pause/stop/断联时被调用
- 超时由 `arm_safety_reset_timeout_seconds` 控制（默认 15s，在 main.cpp 中通过 `declare_parameter` 声明，非 YAML 配置项）
- 复位动作：调用 `/z1_reset_arm` 服务

### Damp 指令禁用

- `app_node.cpp`：心跳 0xFF 不再发布 `/joy.buttons[2]`
- `b2w_teleop.cpp`：不再处理 buttons[2] 的 Damp 调用
- 所有断联/暂停场景使用 `StopMove()` 而非 `Damp()`

### YAML 安全相关参数

| 参数 | 节点 | 默认值 | 说明 |
|------|------|--------|------|
| `heartbeat_timeout_seconds` | app_node | 3.5 | 心跳超时阈值（秒） |
| `heartbeat_required_after_control` | app_node | true | 遥控后是否强制要求心跳 |
| `arm_safety_reset_timeout_seconds` | b2w_nav_node | 15.0 | 机械臂安全复位超时（秒） |
```

---

### Task 6: 更新「TCP 应用协议」小节

**Objective:** 同步 Damp 禁用和心跳语义

**Files:**
- Modify: `CLAUDE.md` — 第 303-357 行

**Content:**

更新心跳指令说明（第 335-347 行）中最后一句改为：
```
... 不会触发 StandDown() 或 Damp()。断联后实际执行流程：关闭 TCP socket → 调用 handlePauseCommand() → 优先尝试 /emergency_stop 服务 → 仅在服务不可达时才 fallback 到 StopMove + triggerStartAllFailSafeStop(SIGTERM)。
```

更新控制链路说明（第 349-357 行）末尾追加：
```
- 0xFF heartbeat 不再触发 Damp()。b2w_teleop_node 已移除 buttons[2] 的 Damp 处理。
- pause 和 stop 流程中，app_node 会额外通过 /joy 发布 StopMove 指令，并在必要时对 start_all 进程组做 fail-safe kill。
- pause/stop/断联任一场景触发后，app_node 会请求 b2w_nav_node 执行机械臂安全复位（arm_safety_reset）。
```

---

### Task 7: 更新「配置文件」章节

**Objective:** 补充新增配置文件和参数

**Files:**
- Modify: `CLAUDE.md` — 第 282-301 行

**Content:**

在配置文件列表中添加：
```
- `rtk_nav_ws/config/ins_parser_params.yaml` — ins_parser 参数（串口、HEPOS 网格修正、GPGGA 日志）
```

`b2w_controller_params.yaml` 关键参数表追加：
```
| `arm_offset_x` | `b2w_nav_node` | 机械臂底座相对 base_link 的 x 偏移，默认 `0.3487` |
| `arm_target_comp_x` | `b2w_nav_node` | 喷涂目标前向补偿 (m)，默认 `0.015` |
| `heading_alignment_threshold` | `b2w_nav_node` | 航向对准阈值 (rad)，默认 `0.05`（约 3°） |
| `obstacle_detection_range` | `b2w_nav_node` | 障碍物检测距离 (m)，默认 `1.2` |
```

此外，以下参数在 `main.cpp` 中通过 `declare_parameter` 声明（非 YAML 配置项）：
| `arm_safety_reset_timeout_seconds` | `b2w_nav_node` | 15.0 | 安全复位超时（秒） |
| `heartbeat_timeout_seconds` | app_node | 3.5 | 心跳超时阈值（秒） |
| `heartbeat_required_after_control` | app_node | true | 遥控后是否强制要求心跳 |

更新已有参数：
```
| `z1_arm_target_pitch_deg` | `b2w_nav_node` | 机械臂末端目标 pitch 角度，默认 `84.0`（垂直向下约 83°，取84） |
```

---

### Task 8: 新增「测试」章节

**Objective:** 记录 `tests/` 目录的静态验证测试

**Files:**
- Modify: `CLAUDE.md` — 在「配置文件」之后插入

**Content:**

```markdown
## 测试

`tests/` 目录包含 3 个 Python 静态验证脚本，不需要 ROS 环境即可运行：

| 文件 | 验证内容 |
|------|---------|
| `test_app_node_failsafe.py` | pause fallback 逻辑、stop 进程组 kill、断联 StopMove、Damp 禁用 |
| `test_app_upload_protocol.py` | APP 上传协议（状态同步、无效数据过滤） |
| `test_b2w_navigation_arm_safety_reset.py` | arm_safety_reset 机制：IsArmRelatedState 覆盖的状态、RequestArmSafetyReset 流程 |

运行方式：
```bash
cd /home/oneko/projects/spray_b2w_robot_project_greek
python3 tests/test_app_node_failsafe.py
python3 tests/test_app_upload_protocol.py
python3 tests/test_b2w_navigation_arm_safety_reset.py
```

测试依赖：Python 3.8+，无第三方库。测试直接读取 `.cpp` 源码，用正则匹配关键字符串和函数签名。不执行编译或 ROS 运行时。
```

---

### Task 9: 新增「工具脚本」章节

**Objective:** 汇总仓库内独立脚本

**Files:**
- Modify: `CLAUDE.md` — 在「测试」章节之后插入

**Content:**

```markdown
## 工具脚本

| 脚本 | 用途 |
|------|------|
| `resettime.sh` | 快速设置系统时间（默认 2026 年），用法：`./resettime.sh -M-D [HH:MM:SS]` |
| `install_fake_hwclock.sh` | 安装假硬件时钟，RTC 不稳定时保证重启后时间不跳回 1970 |
| `record_gps_epsg_point.py` | RTK 录点链路分析脚本，将 GPGGA + /epsg_position 全链路写入 `gnss_waypoints_detail.txt` |
| `simulate_gnss_serial.py` | GNSS 串口模拟器，用 PTY 回放 GPGGA 日志给 `ins_parser_node` |
| `compute_fj_plane.py` | 丰疆平面坐标计算/转换工具（含 HEPOS 网格双线性插值） |
```

---

### Task 10: 新增「文档导航」章节

**Objective:** 汇总仓库关键文档

**Files:**
- Modify: `CLAUDE.md` — 在文末追加

**Content:**

```markdown
## 文档导航

| 文档 | 内容 |
|------|------|
| `docs/app_control_disconnect_safety.md` | APP 控制、断联与保护逻辑完整说明（必读） |
| `docs/log_view.md` | 日志查看指南：架构、路径、常用 tail 命令 |
| `docs/network_topology.md` | 网络拓扑实测（5-8）、换口故障复盘、MAC/IP 对照 |
| `docs/hermes_usage_guide.md` | Hermes Agent 在本仓库的使用说明 |
| `docs/mid360_lidar_migration.md` | MID-360 激光雷达迁移方案 |
| `rtk_nav_ws/issue.md` | HEPOS 5 点验证报告：PROJ default vs HEPOS-correct 对比 |
| `rtk_nav_ws/fj_dynamic/improve/precision_roadmap.md` | B2W 喷涂精度提升路线图（目标 3 cm） |
| `z1_move_ws/README.md` | Z1 机械臂 SDK、手动测试、零位恢复、故障排查 |
```

---

### Task 11: 新增「现场测试数据」章节

**Objective:** 记录 101 点喷涂测试归档和精度验证数据

**Files:**
- Modify: `CLAUDE.md` — 在「工具脚本」之后、「文档导航」之前插入

**Content:**

```markdown
## 现场测试数据

### 101 点喷涂精度测试

| 日期 | 目录 | 报告 |
|------|------|------|
| 5-13 | `rtk_nav_ws/fj_dynamic/improve/test-100/5-13/` | 分析报告、落点坐标 CSV、误差统计 |
| 5-14 | `rtk_nav_ws/fj_dynamic/improve/test-100/5-14/` | 分析报告 + 完整导航日志 |

目录结构（以 5-14 为例）：
- `analysis.md` — 精度分析报告
- `analysis_points.csv` — 机器狗落点
- `target_fj_101.csv` — 丰疆目标点
- `arm_trigger_101.csv` — 机械臂触发记录
- `spacing_error.csv` — 间距误差
- `sn/start_all_latest/b2w_navigation.log` — 导航日志（5-14 特有）

回录工具：
- `rtk_nav_ws/fj_dynamic/improve/test-100/recore_spary_points.py` — 从导航日志中回录喷涂点坐标
- `rtk_nav_ws/fj_dynamic/improve/test-100/README.md` — 测试数据目录说明
```

---

### Task 12: 更新「工具脚本」章节补充

**Objective:** 补充遗漏的脚本

**Files:**
- Modify: `CLAUDE.md` — 在「工具脚本」表追加一行

**Content:**

在工具脚本表中追加：
```
| `rtk_nav_ws/fj_dynamic/improve/test-100/recore_spary_points.py` | 从 b2w_navigation.log 回录喷涂点坐标 |
```

---

### Task 13: 更新「文档导航」补充

**Objective:** 补充协议文档和审计报告

**Files:**
- Modify: `CLAUDE.md` — 在「文档导航」表追加

**Content:**

追加：
```
| `docs/喷涂机器人通信协议05-25.md` | 喷涂机器人完整通信协议（2026-05-25 更新版） |
| `docs/source_compare_5-19.md` | 机器备份 vs 当前项目源码对比报告 |
| `dog_logs/5-22/upper_computer_protocol_audit_5-22.md` | 上位机通信协议三方核对报告 |
```

---

### Task 14: 更新「硬件设备与配置」章节

**Objective:** 同步 Z1 负载配置变更

**Files:**
- Modify: `CLAUDE.md` — 第 224 行（Z1 机械臂描述）之后

**Content:**

追加：
```
- Z1 控制器负载参数（`z1_controller/config/config.xml`）：自 commit `935189a` 起负载从 `0.0kg` 调整为 `1.0kg`（喷枪等末端负载），影响机械臂动力学计算。
```

---

### Task 15: 删除 Task 5 末尾的安全参数重复表

**Objective:** 避免参数表在多个位置重复

**Files:**
- Modify: `CLAUDE.md` — Task 5 插入内容末尾

**Content:**

将 Task 5 末尾的「YAML 安全相关参数」表替换为交叉引用：
```
安全相关参数（`arm_safety_reset_timeout_seconds`、`heartbeat_timeout_seconds`、`heartbeat_required_after_control`）详见「配置文件」章节。
```

---

## 执行顺序

按 Task 1→15 顺序执行，每完成一个 task 做一次 `git diff CLAUDE.md` 确认只改了预期内容。

## 验证

```bash
wc -l CLAUDE.md          # 确认行数在 550-700 范围（当前 430）
git diff --stat CLAUDE.md # 确认只改了 CLAUDE.md
python3 tests/test_app_node_failsafe.py       # 确认安全逻辑测试仍通过
python3 tests/test_b2w_navigation_arm_safety_reset.py  # 确认 arm reset 测试仍通过
```
