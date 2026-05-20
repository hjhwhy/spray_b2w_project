# APP 控制、断联与保护逻辑检查说明

本文按当前仓库代码整理 APP/TCP 控制链路、断联保护、底盘停止/趴下/Damp 相关安全逻辑，便于现场和代码复核。

检查对象：

- `app_ws/src/app_node.cpp`：TCP 9002 服务、APP 指令解析、断联自动暂停、pause/stop/restart 调用。
- `b2w_navigation_ws/src/b2w_teleop.cpp`：订阅 `/joy`，把 APP 手动控制转换为 Unitree B2W SportClient 动作。
- `b2w_navigation_ws/src/main.cpp`：自动喷涂主控、`/emergency_stop`、`/erase_emergency_stop`、暂停状态机。
- `start_all.sh`：主任务进程组、ready 文件、pause/restart 服务探测。
- `rs585_ws/src/rs485_node.cpp`：喷枪/水泵继电器动作。

## 1. 运行角色划分

### 1.1 常驻基础链路：`tcp_base_ctl.sh`

`tcp_base_ctl.sh` 是常驻基础控制链路，通常由 systemd 或人工启动。它负责启动：

- `robot_tcp_node`：APP TCP 服务，监听 9002。
- `b2w_teleop_node`：订阅 `/joy`，把手动控制发到 B2W。
- RTK/TF/RS485/Z1 等基础节点。

关键日志：

```bash
tail -F /home/test/logs/tcp_base_ctl_latest/robot_tcp.log \
        /home/test/logs/tcp_base_ctl_latest/b2w_teleop.log \
        /home/test/logs/tcp_base_ctl_latest/tcp_base_ctl.log
```

### 1.2 主作业链路：`start_all.sh`

APP 发送 `0x01 start` 后，`app_node.cpp` 会通过 `setsid /home/test/start_all.sh &` 启动主作业流程。

`start_all.sh` 会写入：

- `/tmp/start_all.pid`
- `/tmp/start_all.pgid`
- `/tmp/start_all.ready`

`/tmp/start_all.ready` 是 APP pause/restart 是否允许执行的重要门槛。当前 APP 逻辑只在 ready 文件至少进入 `ready` 或 `partial` 且服务可发现时，才尝试调用 `/emergency_stop` / `/erase_emergency_stop`。

## 2. APP TCP 输入到 ROS/Unitree 的链路

链路图：

```text
APP / TCP 客户端
  │  TCP 9002，协议帧 [0xF5][func][len][payload][CRC][0x5F]
  ▼
app_ws/src/app_node.cpp / robot_tcp_node
  │
  ├─ 0x01/0x02/0x03/0x10：start/pause/stop/resume 控制主作业
  └─ 0x04~0x0B：发布 /joy
        ▼
b2w_navigation_ws/src/b2w_teleop.cpp / b2w_teleop_node
  │
  ├─ /joy.axes    -> SportClient::Move(...) 或 StopMove()
  └─ /joy.buttons -> StandDown() / StandUp()
        ▼
Unitree B2W 底盘
```

注意：`b2w_teleop_node` 日志中写的 “from APP command” 只是日志文案。它实际只知道消息来自 ROS 话题 `/joy`，并不能证明发布者一定是手机 APP。

## 3. 当前 APP 子命令映射

按当前代码实际逻辑整理如下：

| 子命令 | APP 语义 | `app_node.cpp` 行为 | 下游结果 |
|---|---|---|---|
| `0x01` | 开始作业 | 拉起 `/home/test/start_all.sh`，发布 `/remote_command="start"` | 启动自动喷涂主流程 |
| `0x02` | 暂停 | 调用 `handlePauseCommand()`，再调用 `/emergency_stop` | `b2w_nav_node` 冻结状态机，底盘停止 |
| `0x03` | 停止 | 调用 `triggerStartAllFailSafeStop("stop command")` | 先发 StopMove，再终止 `start_all` 进程组 |
| `0x04` | 前进 | 发布 `/joy.axes` | `b2w_teleop_node` 调 `SportClient::Move(...)` |
| `0x05` | 后退 | 发布 `/joy.axes` | `SportClient::Move(...)` |
| `0x06` | 左移 | 发布 `/joy.axes` | `SportClient::Move(...)` |
| `0x07` | 右移 | 发布 `/joy.axes` | `SportClient::Move(...)` |
| `0x08` | 左转 | 发布 `/joy.axes` | `SportClient::Move(...)` |
| `0x09` | 右转 | 发布 `/joy.axes` | `SportClient::Move(...)` |
| `0x0A` | 趴下 StandDown | 发布 `/joy.buttons[0]=1` | `b2w_teleop_node` 调 `SportClient::StandDown()` |
| `0x0B` | 站立 StandUp | 发布 `/joy.buttons[1]=1` | `b2w_teleop_node` 调 `SportClient::StandUp()` |
| `0x10` | 恢复/继续 | 调用 `/erase_emergency_stop` | `b2w_nav_node` 解除暂停 |
| `0xFF` | APP 心跳 | 只更新 APP 在线/心跳时间 | 不发布 `/joy`，不会触发 Damp |

当前代码证据：

- `app_ws/src/app_node.cpp:329-415`：APP 子命令解析。
- `app_ws/src/app_node.cpp:390-401`：只有 `0x0A`、`0x0B` 发布姿态按钮。
- `b2w_navigation_ws/src/b2w_teleop.cpp:43-56`：`buttons[0]` 调 `StandDown()`，`buttons[1]` 调 `StandUp()`。
- 当前 `app_node.cpp` 只有 top-level `func_code == 0xFF` 心跳分支，没有 `instruction_type == 0xFF` 控制子命令；当前 `b2w_teleop.cpp` 没有 `Damp()` 调用。

## 4. 正常控制状态

### 4.1 APP 未连接

APP 未连接时：

- `robot_tcp_node` 仍监听 9002。
- 不会因为“没有 APP”自动 start/pause/stop。
- 原厂遥控器直控不经过 `robot_tcp_node`，不会在 `robot_tcp.log` 里出现 APP 子命令。

风险点：如果主作业已经运行，而 APP 从未连接过，当前代码没有“必须有 APP 在线才能继续作业”的门槛。断联保护只在曾经有活跃 TCP 客户端断开后才会调度。

### 4.2 APP 已连接但未开始作业

APP 连接后：

- `robot_tcp_node` 记录 `New client connected from <ip>`。
- 连接建立会取消之前 pending 的 disconnect auto-pause。
- APP 可以发手动移动、趴下、站立、start 等命令。

### 4.3 APP 开始作业

APP 发 `0x01` 后：

1. `app_node.cpp` 检查 `/tmp/start_all.pid`，如果旧流程不存在则清理旧 pid。
2. 通过 `setsid /home/test/start_all.sh &` 拉起主作业。
3. `start_all.sh` 启动 `b2w_nav_node` 等主任务节点。
4. `start_all.sh` 等待 `/emergency_stop`、`/erase_emergency_stop` 服务出现，并写 `/tmp/start_all.ready`。

## 5. APP 断联保护逻辑

### 5.1 断联检测入口

`app_node.cpp` 的客户端线程在 `recv()` 中等待 APP 数据：

- `recv()` 返回 `0`：认为客户端正常断开。
- `recv()` 返回 `<0` 且不是中断重试：认为接收失败，退出客户端线程。

代码位置：`app_ws/src/app_node.cpp:200-241`

如果断开的是当前活跃 `client_sock_`，则调用：

```cpp
scheduleDisconnectAutoPause();
```

### 5.2 断联延迟时间

参数：`client_disconnect_auto_pause_seconds`

默认值：`8.0` 秒。

如果配置值大于 0 且小于 3 秒，会被强制夹到 3 秒，避免 WiFi 短抖动造成误暂停。

代码位置：`app_ws/src/app_node.cpp:52-63`

### 5.3 重连取消断联保护

如果 APP 在超时前重连，`runTcpServer()` 接受新连接后会调用：

```cpp
cancelDisconnectAutoPause("APP client reconnect/cancel disconnect auto-pause");
```

同时断联定时器内部也会检查 `client_sock_ >= 0`，防止旧定时器误触发。

代码位置：

- `app_ws/src/app_node.cpp:186`
- `app_ws/src/app_node.cpp:726-738`
- `app_ws/src/app_node.cpp:747-780`

### 5.4 断联超时后的动作

断联超时后分两种：

#### 情况 A：`start_all` 不可控

如果 `/tmp/start_all.ready` 不存在，或 ready 状态不允许调用 emergency stop：

- 只发布 StopMove Joy：`axes=[0,0,0]`，`buttons=[]`。
- 不调用 `/emergency_stop`。
- 不会发 StandDown，不会发 Damp。

代码位置：`app_ws/src/app_node.cpp:781-787`

#### 情况 B：`start_all` 可控

如果 `start_all` 已进入可控状态：

- 调用 `handlePauseCommand("APP client disconnected timeout")`。
- `handlePauseCommand()` 调用 `/emergency_stop`。
- `/emergency_stop` 成功后，`b2w_nav_node` 进入 `paused_ = true`。
- 控制循环冻结状态机并保持底盘停止。

代码位置：

- `app_ws/src/app_node.cpp:789-793`
- `app_ws/src/app_node.cpp:674-682`
- `b2w_navigation_ws/src/main.cpp:382-404`
- `b2w_navigation_ws/src/main.cpp:423-439`

### 5.5 应用层心跳超时后的动作

除 TCP clean close 外，`robot_tcp_node` 还解析 top-level 心跳功能码 `0xFF`。APP 建议每 1 秒发送：

```text
F5 FF 01 00 FF 00 00 5F
```

默认参数：

- `heartbeat_timeout_seconds=3.5`
- `heartbeat_required_after_control=true`

当 APP 已经发过 `0x01 start`、`0x04~0x09` 移动、`0x0A/0x0B` 姿态，或新连接时检测到 `start_all` 已处于可控状态后，机器人进入 APP 控制/作业心跳监管模式。该模式下如果超过 `heartbeat_timeout_seconds` 未收到心跳：

1. `robot_tcp_node` 关闭当前 APP socket，释放单客户端占用，便于 APP 恢复后重连。
2. 若 `start_all` 不可控，只发布 StopMove Joy：`axes=[0,0,0]`、`buttons=[]`。
3. 若 `start_all` 可控，立即调用 `handlePauseCommand("APP heartbeat timeout")`，不再额外等待 8 秒 TCP 断联延迟。
4. 所有心跳超时保护路径都不发布姿态按钮，不触发 `StandDown()` 或 `Damp()`。

## 6. Pause / Stop / Fail-safe 的区别

### 6.1 Pause：冻结主控状态机

入口：

- APP 发 `0x02`
- APP 断联超时后自动调用

结果：

- 调 `/emergency_stop`。
- `b2w_nav_node` 设置 `paused_=true`。
- 立即 `sport_client_.Move(0,0,0)`。
- 后续 `ControlLoop()` 在 paused 状态下直接 return，状态机冻结。

这是“暂停任务等待恢复”，不是杀掉主作业进程。

### 6.2 Stop：终止主作业流程

入口：APP 发 `0x03`。

结果：

1. 先发布 StopMove Joy，确保底盘停止。
2. 校验 `/tmp/start_all.pgid` 或 `/tmp/start_all.pid` 指向可信的 `start_all.sh`。
3. 对 start_all 进程组发 `SIGTERM`。
4. 若仍存活，升级 `SIGKILL`。
5. 清理 `/tmp/start_all.pid`、`/tmp/start_all.pgid`、`/tmp/start_all.ready`。

代码位置：`app_ws/src/app_node.cpp:633-671`

### 6.3 Emergency stop 调用异常时的 fail-safe

如果 pause 调 `/emergency_stop` 失败，会进入 fail-safe stop：

- 服务不可用或 readiness 拒绝：`pause fallback`。
- emergency stop 响应超时：`emergency stop response timeout`。
- emergency stop 响应不包含 `Task paused` / `Task already paused`：`emergency stop bad response`。
- service future 异常：`emergency stop exception`。

这些 fail-safe 都先发布 StopMove，再终止 `start_all` 进程组。

代码位置：

- `app_ws/src/app_node.cpp:674-682`
- `app_ws/src/app_node.cpp:803-898`
- `app_ws/src/app_node.cpp:633-671`

## 7. 趴下 / Damp / 静止 的安全边界

### 7.1 当前哪些路径会 StandDown

当前只发现一条正式路径：

```text
APP 子命令 0x0A
  -> app_node.cpp 发布 /joy.buttons[0]=1
  -> b2w_teleop.cpp 调 SportClient::StandDown()
```

代码位置：

- `app_ws/src/app_node.cpp:390-401`
- `b2w_navigation_ws/src/b2w_teleop.cpp:43-47`

### 7.2 当前哪些路径会 Damp

当前 `app_node.cpp` 和 `b2w_teleop.cpp` 实际控制链路中没有发现 Damp 路径：

- `app_node.cpp` 的 `0xFF` 是 top-level 心跳功能码，不是 `/joy` 姿态/阻尼命令。
- `app_node.cpp` 没有 `joy_msg.buttons[2]=1`。
- `b2w_teleop.cpp` 没有 `SportClient::Damp()`。

因此，当前 APP 断联、pause、stop、fail-safe 都不会主动触发 Damp。

### 7.3 当前哪些路径会静止

底盘静止主要有三种：

1. `publishSafetyStopMove()` 发布 `/joy.axes=[0,0,0]`、`buttons=[]`，由 `b2w_teleop_node` 执行 `StopMove()`。
2. `b2w_nav_node` 在 emergency stop 中直接 `sport_client_.Move(0,0,0)`。
3. `b2w_nav_node` 主控状态机在多个状态切换点主动 `Move(0,0,0)`。

代码位置：

- `app_ws/src/app_node.cpp:620-630`
- `b2w_navigation_ws/src/b2w_teleop.cpp:65-72`
- `b2w_navigation_ws/src/main.cpp:382-404`
- `b2w_navigation_ws/src/main.cpp:423-439`

## 8. 已有保护点

### 8.1 断联自动暂停

APP 活跃连接断开后，默认 8 秒自动暂停。

### 8.2 重连取消旧定时器

APP 在超时前重连，会取消 pending 的断联定时器，并且定时器内部检查当前 socket 状态。

### 8.3 pause 必须确认下游状态

`app_node.cpp` 不只看 service 返回，还检查 response message 是否包含：

- `Task paused`
- `Task already paused`

如果没有确认暂停状态，就执行 fail-safe stop。

### 8.4 emergency stop 响应超时保护

调用 `/emergency_stop` 后 2 秒内未收到确认，会执行 fail-safe stop。

### 8.5 fail-safe stop 不使用趴下/Damp

fail-safe stop 使用 StopMove 和终止 `start_all` 进程组，不使用 `StandDown()` 或 `Damp()`。

### 8.6 stop 目标进程校验

终止 `start_all` 前会校验：

- pid/pgid 合法。
- 进程组不是 APP 节点自己的进程组。
- pid 仍运行。
- pid 实际 pgid 和文件记录一致。
- `/proc/<pid>/cmdline` 包含 `start_all.sh`。
- run file 不是 group/world writable，owner 是当前用户或 root。

这是为了避免误杀其他进程。

### 8.7 心跳 watchdog 和 send 失败保护

- APP 控制/作业会话中，心跳超过 `heartbeat_timeout_seconds` 会立即进入断联保护。
- 机器人向 APP `send()` 失败时，关闭 active socket 后也会调度 TCP 断联 auto-pause，避免“只关闭 socket、不暂停任务”的漏保护路径。

## 9. 当前仍需重点检查的潜在问题

### 9.1 已降低：APP 心跳 / 应用层 lease

当前已新增 APP 心跳 watchdog，可覆盖“手机 APP 卡死但 TCP 连接未关闭、socket 半开、WiFi 黑洞、APP 不再发控制包但连接仍 ESTABLISHED”等 TCP 不及时断开的场景。

仍需现场确认：APP 端必须按 1 Hz 持续发送 top-level `0xFF` 心跳；若 APP 版本未升级，机器人端在 APP 控制/作业会话中会按超时策略进入保护。

### 9.2 已降低：发送给 APP 失败会调度断联保护

`sendPacket()` 如果 `send()` 失败，会关闭 active socket，并在不持有 `client_mutex_` 的情况下调用 `scheduleDisconnectAutoPause()`。

这避免了“机器人向 APP 发送 progress/position/path/pointcloud 失败后只关闭 socket、不启动断联保护”的漏保护路径。

### 9.3 中高风险：9002 没有客户端认证/授权

`robot_tcp_node` 监听 `INADDR_ANY:9002`，代码中未看到认证、白名单、token 或控制模式门控。

潜在后果：同一网络内任何能连上 9002 的客户端，只要按协议发包，就能发 start/pause/stop/move/StandDown/StandUp。

建议现场排查时不要把日志里的 “APP client” 等同于手机 APP，需要结合连接 IP/MAC 判断来源。

### 9.4 中风险：`/joy` 没有发布者来源隔离

`b2w_teleop_node` 只订阅 `/joy`。任何 ROS 2 节点只要能发布 `/joy.buttons[0]=1`，都会触发 `StandDown()`。

建议后续考虑：

- 将 APP 控制拆成 `/app_joy`，只允许 APP 网关发布。
- 或在 teleop 层增加控制模式、来源白名单、作业状态门控。

### 9.5 中风险：断联 pause 只冻结底盘主控，不主动取消机械臂/喷枪已发动作

`/emergency_stop` 当前主要做：

- `paused_=true`
- `moving_=false`
- `sport_client_.Move(0,0,0)`

它不会主动取消已经发出的：

- `/z1_move_to_target`
- `/trigger_valve_ch1`
- `/z1_reset_arm`

如果 APP 断联发生在机械臂动作或喷枪触发窗口内，底盘会停，但已经发出的服务可能继续完成。

`rs485_node` 的 CH1 喷枪触发是打开 `trigger_time_ms` 后关闭，默认 600ms；CH2 水泵在节点启动时默认打开，节点退出时关闭。

代码位置：

- `b2w_navigation_ws/src/main.cpp:717-733`
- `b2w_navigation_ws/src/main.cpp:771-800`
- `b2w_navigation_ws/src/main.cpp:804-828`
- `rs585_ws/src/rs485_node.cpp:17`
- `rs585_ws/src/rs485_node.cpp:59-68`
- `rs585_ws/src/rs485_node.cpp:125-140`

### 9.6 已处理：旧 0xFF/Damp 映射文档

当前文档已统一：`0xFF` 是 APP 心跳功能码，不是姿态/阻尼指令；实际控制链路没有心跳触发 Damp 的路径。

## 10. 现场检查命令

### 10.1 看 APP 是否连接

```bash
ss -ltn | grep ':9002 '
ss -tnp | grep ':9002 '
```

### 10.2 看 APP 指令是否进入 robot_tcp

```bash
tail -F /home/test/logs/tcp_base_ctl_latest/robot_tcp.log
```

重点搜索：

```bash
grep -E "New client|Client disconnected|RX COMMAND|RX HEARTBEAT|APP heartbeat timeout|Decoded instruction|disconnect auto-pause|Emergency stop|fail-safe|StopMove" \
  /home/test/logs/tcp_base_ctl_latest/robot_tcp.log
```

### 10.3 看 `/joy` 是否触发姿态动作

```bash
tail -F /home/test/logs/tcp_base_ctl_latest/b2w_teleop.log
```

重点搜索：

```bash
grep -E "StandDown|StandUp|Damp|SportClient|StopMove" \
  /home/test/logs/tcp_base_ctl_latest/b2w_teleop.log
```

当前预期：

- 正常 APP 趴下：应看到 `Executed StandDown from APP command.`
- 正常 APP 站立：应看到 `Executed StandUp from APP command.`
- 不应看到 `Damp`。

### 10.4 看主控是否进入 emergency pause

```bash
tail -F /home/test/logs/start_all_latest/b2w_navigation.log
```

重点搜索：

```bash
grep -E "Task paused|Task already paused|Emergency pause active|Task resumed|Move command failed" \
  /home/test/logs/start_all_latest/b2w_navigation.log
```

### 10.5 看 ready 文件状态

```bash
cat /tmp/start_all.ready
cat /tmp/start_all.pid
cat /tmp/start_all.pgid
```

预期 ready 文件示例：

```text
state=ready
service_ready=1
probe_ok=1
```

或：

```text
state=partial
service_ready=1
probe_ok=0
```

`state=ready` 或 `state=partial` 且 `service_ready=1` 时，APP 才会尝试调用 pause/resume 服务。

## 11. 建议的现场验收用例

### 用例 A：APP 正常断开

步骤：

1. 启动 `tcp_base_ctl.sh`。
2. APP 连接 9002。
3. APP 发 `0x01 start`，等待 `/tmp/start_all.ready`。
4. 断开 APP 网络或关闭 APP。
5. 等待默认 8 秒。

预期：

- `robot_tcp.log` 出现 `Client disconnected`。
- 出现 `APP client disconnected; will auto-pause after 8.0s`。
- 超时后出现 `APP client disconnected timeout ... requesting automatic pause`。
- `/emergency_stop` response 包含 `Task paused` 或 `Task already paused`。
- `b2w_navigation.log` 出现 `Emergency pause active`。
- `b2w_teleop.log` 不应出现 `StandDown` 或 `Damp`。

### 用例 B：断开后 8 秒内重连

步骤：

1. APP 连接并 start。
2. 短暂断开 APP。
3. 8 秒内重新连接。

预期：

- `robot_tcp.log` 出现断联定时器启动。
- 重连后出现 `APP client reconnect/cancel disconnect auto-pause`。
- 不应自动 pause。

### 用例 C：APP 发趴下

步骤：APP 发送 `0x0A`。

预期：

- `robot_tcp.log` 显示 `Decoded instruction type: 0x0A`。
- `b2w_teleop.log` 显示 `Executed StandDown from APP command.`。
- 这是当前唯一正式 StandDown 路径。

### 用例 D：APP 断联时不应趴下/Damp

步骤：作业中断开 APP。

预期：

- `robot_tcp.log` 进入 auto-pause 或 StopMove/fail-safe。
- `b2w_teleop.log` 不出现 `Executed StandDown`。
- `b2w_teleop.log` 不出现 `Damp`。
- `b2w_navigation.log` 出现 pause/stop 相关日志。

### 用例 E：TCP 半开/APP 卡死/心跳停止

步骤：模拟 APP 进程卡死但不关闭 TCP，或网络黑洞。

预期：

- `robot_tcp.log` 出现 `APP heartbeat timeout ... entering disconnect protection`。
- 若 `start_all` 可控，立即请求 automatic pause；若不可控，发布 StopMove。
- `b2w_teleop.log` 不出现 `Executed StandDown` 或 `Damp`。

## 12. 独立复核结果

已使用 Gemini CLI 对当前未提交改动进行只读安全复核，重点检查：

- `docs/app_control_disconnect_safety.md` 是否准确反映 `app_ws/src/app_node.cpp` 实现。
- 是否仍存在旧 `0xFF -> Damp` 映射或误导性表述。
- 心跳超时、TCP 断开、`send()` 失败、原厂遥控器场景描述是否准确。
- APP 断联保护是否存在竞态、漏保护，或误触发 `StandDown()` / `Damp()` 的风险。

复核结论：

1. 必须修改项：无。
2. 文档与代码实现高度一致：`0xFF` 是 top-level APP 心跳功能码；心跳超时进入 `handleHeartbeatTimeoutProtection()`；`start_all` 可控时走 `handlePauseCommand("APP heartbeat timeout")`，不可控时发布 StopMove。
3. 旧 `0xFF -> Damp` 已清理：当前 `app_node.cpp` 没有 `instruction_type == 0xFF` 控制子命令，`b2w_teleop.cpp` 没有 `SportClient::Damp()` 调用，心跳不会发布 `/joy` 或姿态按钮。
4. 断联保护覆盖范围可接受：`recv()` 返回 0、`recv()` 失败、`send()` 失败、TCP clean disconnect、TCP 半开/APP 卡死/心跳停止等路径均会进入 auto-pause、heartbeat timeout protection 或 StopMove/fail-safe stop。
5. StopMove 路径安全：`publishSafetyStopMove()` 发布 `axes=[0,0,0]` 且不带 buttons，经 `b2w_teleop.cpp` 落入 `SportClient::StopMove()`，不会触发 `StandDown()`。
6. 心跳超时立即保护、TCP/`send()` 失败保留 8 秒断联延迟，是当前可接受的策略差异：前者用于 APP 卡死/黑洞等应用层失效，后者允许短暂 WiFi 抖动重连。

复核建议项：

- 可进一步细化心跳 watchdog 日志，区分“新连接尚未建立心跳导致超时”和“运行中丢失心跳导致超时”，便于现场排查 WiFi 黑洞。
- 现场验收仍需强调：`/emergency_stop` 主要冻结底盘主控，不保证主动撤回已经发出的机械臂或喷枪服务调用；这部分仍需结合流程管理和实机测试确认。

## 13. 当前结论

1. 当前代码中，APP 断联、pause、stop、emergency fail-safe 不会主动触发 StandDown 或 Damp。
2. 当前正式 StandDown 路径只有 APP 子命令 `0x0A -> /joy.buttons[0] -> b2w_teleop StandDown()`。
3. 当前实际控制链路没有心跳触发 Damp 的路径。
4. 正常 TCP 断开场景下，APP 断联默认 8 秒后会自动 pause，使主控状态机冻结、底盘停止。
5. APP/TCP 半开、APP 卡死、网络黑洞等“TCP 不及时断开”的场景下，应用层 `0xFF` 心跳 watchdog 会按 `heartbeat_timeout_seconds` 触发立即保护。
6. 当前已新增应用层心跳和 send 失败断联保护；仍需重点关注：9002 无认证、`/joy` 无来源隔离、emergency pause 不主动取消机械臂/喷枪已发动作。
