# APP 心跳 + TCP 双重断联保护 Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** 在现有 TCP 断联检测基础上，增加 APP/遥控器 1 Hz 心跳协议，形成“TCP socket 断开 + 应用层心跳超时”的双重断联判断；断联后安全地暂停/停止任务并保持机器狗静止，绝不触发 `StandDown()` 或 `Damp()`。

**Architecture:** `robot_tcp_node` 继续负责 TCP 9002 接入和 APP 协议解析。新增 top-level 功能码 `0xFF` 心跳包解析，更新 `last_client_activity/last_heartbeat` 时间戳，并用 wall timer 周期性检查心跳超时。所有断联入口统一收敛到同一个 safety handler：先确认当前 client 是否仍在线/心跳是否有效，再根据 `start_all.ready` 调用 `/emergency_stop` 或发布 StopMove；fail-safe 路径继续杀 `start_all` 进程组，但不得发布姿态按钮。

**Tech Stack:** C++17, ROS 2 Humble `rclcpp`, TCP socket, `sensor_msgs/msg/Joy`, `std_srvs/srv/Trigger`, pytest 源码级回归测试。

---

## 0. 安全不变量

实现中必须始终满足：

1. 只有 APP 控制子命令 `0x0A` 可以触发 `StandDown()`。
2. 心跳功能码 `0xFF` 只能更新在线状态，不能映射到 `/joy.buttons`，不能触发 `Damp()`。
3. TCP 断开、心跳超时、pause fallback、stop、emergency-stop timeout/bad-response/exception，全部使用 StopMove 或 emergency pause，不使用 `StandDown()` / `Damp()`。
4. APP 未进入作业控制状态时，不应因为“从未收到心跳”误暂停正常原厂遥控器使用场景。
5. APP 一旦开始作业或进入 APP 控制会话，心跳超时必须能保护，即使 TCP socket 没有立刻断开。
6. 多客户端重连、旧 timer、旧 socket 不能误触发当前新连接的保护动作。

---

## 1. 协议定义

### 1.1 控制帧保持不变

控制命令仍使用功能码 `0x08`：

```text
[0xF5][0x08][len_low][len_high][sub_command][...][crc_low][crc_high][0x5F]
```

### 1.2 新增心跳帧

APP/遥控器每 1 秒发送一次：

```text
[0xF5][0xFF][len_low][len_high][0xFF][crc_low][crc_high][0x5F]
```

建议数据段长度为 1，即 `len_low=0x01, len_high=0x00`。如果 APP 端已经按文档描述写成“数据段长度 2 字节”但未明确长度值，机器人端应按协议字段解释为 2 字节长度字段，而不是 data_len=2。实现时建议只接受 `data_len == 1 && payload[0] == 0xFF`，避免把异常包当作心跳。

CRC 当前现有控制解析只检查包尾，不校验 CRC。心跳第一版保持一致：检查包头/功能码/长度/包尾，暂不新增 CRC 校验，避免控制帧和心跳帧校验标准不一致。后续如补 CRC，应统一覆盖所有功能码。

---

## 2. 目标行为矩阵

| 场景 | 当前 TCP 状态 | 心跳状态 | 期望动作 |
|---|---|---|---|
| APP 从未连接，机器狗由原厂遥控器控制 | 无 client | 无心跳 | 不动作，不误暂停 |
| APP 连接但未 start | client 在线 | 心跳正常 | 不动作 |
| APP 连接但未 start，心跳停止 | client 可能半开 | 心跳超时 | 只关闭/标记 APP 会话，不杀主任务；可发布一次 StopMove，但不要影响原厂遥控器策略需现场确认 |
| APP 已 start，主作业 running | client 在线 | 心跳正常 | 正常工作 |
| APP 已 start，TCP clean close | client 断开 | 任意 | 延迟保护，默认 8 秒后 pause/fail-safe |
| APP 已 start，TCP 半开 | client 仍 ESTABLISHED | 心跳超时 | 触发断联保护：pause 或 fail-safe stop |
| APP 8 秒内重连并恢复心跳 | 新 client 在线 | 心跳正常 | 取消旧保护 timer，不 pause |
| `/emergency_stop` 服务不可用/无确认 | 任意 | 已超时 | StopMove + fail-safe stop `start_all` |
| APP 发 `0x0A` | client 在线 | 任意 | 允许 StandDown |
| APP 发心跳 `0xFF` | client 在线 | 任意 | 只更新心跳时间，不执行姿态动作 |

Gemini 复核后调整的最终语义：只要 APP 发过 `0x01 start`，或发过任何会影响机器人运动/姿态的 APP 控制指令（`0x04~0x09` 手动移动、`0x0A/0x0B` 姿态），或 `/tmp/start_all.ready` 显示主作业可控，就认为进入“APP 控制/作业监控模式”；该模式下心跳超时必须暂停/停止主任务或至少发布 StopMove。这样可覆盖“APP 手动遥控到作业点途中断联导致持续行走”的风险。

---

## 3. 需要修改的文件

- Modify: `app_ws/src/app_node.cpp`
- Modify: `tests/test_app_node_failsafe.py`
- Modify: `docs/app_control_disconnect_safety.md`
- Optional Modify: `docs/log_view.md`（同步移除旧 `0xFF -> Damp` 文档）
- Optional Modify: `CLAUDE.md`（已新增心跳协议，后续根据最终实现补超时参数名）

---

## 4. 设计细节

### 4.1 新增参数

在 `RemoteControlNode` 构造函数中新增：

```cpp
this->declare_parameter<double>("heartbeat_timeout_seconds", 3.5);
this->declare_parameter<bool>("heartbeat_required_after_control", true);
```

建议：

- APP 每 1 秒发一次心跳。
- 机器人端超时阈值默认 3.5 秒，容忍 2 次丢包/调度抖动。
- 参数小于等于 0 时禁用心跳 watchdog，但生产部署不建议禁用。

### 4.2 新增状态变量

放在 `RemoteControlNode` private 成员区：

```cpp
double heartbeat_timeout_seconds_ = 3.5;
bool heartbeat_required_after_control_ = true;
rclcpp::TimerBase::SharedPtr heartbeat_watchdog_timer_;
std::mutex heartbeat_mutex_;
rclcpp::Time last_heartbeat_time_;
rclcpp::Time last_client_activity_time_;
bool has_heartbeat_ = false;
bool app_control_session_active_ = false;
std::shared_ptr<std::atomic<bool>> heartbeat_timeout_pending_;
```

其中：

- `last_client_activity_time_`：任何合法 APP 帧都更新，可辅助诊断。
- `last_heartbeat_time_`：只有合法心跳帧更新。
- `app_control_session_active_`：APP 发 `0x01 start` 后置 true；`0x03 stop` 或 fail-safe stop 完成后置 false；新连接可保持 false 直到 start。
- 如果担心 APP 手动移动到作业点时也需要心跳保护，可在收到 `0x04~0x09` 或 `0x0A/0x0B` 后也置 true；这点需要现场策略确认。推荐第一版：`0x01 start` 后强制心跳。

### 4.3 新增统一活动记录 helper

```cpp
void markClientActivityLocked(const rclcpp::Time &now)
{
    last_client_activity_time_ = now;
}

void markHeartbeatReceived()
{
    const auto now = this->now();
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    last_client_activity_time_ = now;
    last_heartbeat_time_ = now;
    has_heartbeat_ = true;
    if (heartbeat_timeout_pending_) {
        heartbeat_timeout_pending_->store(false);
        heartbeat_timeout_pending_.reset();
    }
}
```

注意不要在持有 `heartbeat_mutex_` 时再去拿 `client_mutex_` 后执行复杂动作，避免锁顺序死锁。

### 4.4 解析心跳帧

在 `parseNextPacket()` 中新增 `func_code == 0xFF` 分支。逻辑应与 0x08/0x09 平级：

```cpp
if (func_code == 0xFF) {
    if (buffer.size() < 7) {
        return false;
    }
    const uint16_t data_len = buffer[2] | (static_cast<uint16_t>(buffer[3]) << 8);
    if (data_len != 1) {
        RCLCPP_WARN(this->get_logger(), "Invalid heartbeat data_len=%u", data_len);
        buffer.erase(buffer.begin());
        return true;
    }
    total_len = 1 + 1 + 2 + data_len + 2 + 1;
    if (buffer.size() < total_len) {
        return false;
    }
    if (buffer[total_len - 1] != 0x5F) {
        RCLCPP_WARN(this->get_logger(), "Invalid tail for heartbeat packet");
        buffer.erase(buffer.begin());
        return true;
    }
    const uint8_t heartbeat_value = buffer[4];
    if (heartbeat_value != 0xFF) {
        RCLCPP_WARN(this->get_logger(), "Invalid heartbeat value: 0x%02X", heartbeat_value);
        buffer.erase(buffer.begin(), buffer.begin() + total_len);
        return true;
    }
    logPacket("RX HEARTBEAT", buffer.data(), total_len, this->get_logger());
    handleHeartbeatPacket();
    buffer.erase(buffer.begin(), buffer.begin() + total_len);
    return true;
}
```

新增：

```cpp
void handleHeartbeatPacket()
{
    markHeartbeatReceived();
    RCLCPP_DEBUG(this->get_logger(), "APP heartbeat received.");
}
```

### 4.5 控制帧也更新活动时间

在 `handleCommandPacket()` 开头，解析出合法 payload 后：

```cpp
{
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    last_client_activity_time_ = this->now();
}
```

但不要把普通控制帧当心跳，否则 APP 卡死只要没有控制帧就应该超时。

### 4.6 start/stop 更新 APP 作业会话状态

收到 `0x01 start` 且 `system()` 调用返回后：

```cpp
{
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    app_control_session_active_ = true;
    if (!has_heartbeat_) {
        last_heartbeat_time_ = this->now();
        has_heartbeat_ = true;
    }
}
```

这样 start 后不会因为“上一秒没有心跳历史”立即误判；从 start 起开始计时。

`0x03 stop` 的 fail-safe stop 完成后：

```cpp
markAppControlSessionInactive("stop command");
```

新增 helper：

```cpp
void markAppControlSessionInactive(const std::string &reason)
{
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    app_control_session_active_ = false;
    has_heartbeat_ = false;
    if (heartbeat_timeout_pending_) {
        heartbeat_timeout_pending_->store(false);
        heartbeat_timeout_pending_.reset();
    }
    RCLCPP_INFO(this->get_logger(), "APP heartbeat session inactive: %s", reason.c_str());
}
```

### 4.7 心跳 watchdog timer

构造函数最后创建：

```cpp
heartbeat_watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RemoteControlNode::checkHeartbeatWatchdog, this));
```

实现：

```cpp
void checkHeartbeatWatchdog()
{
    if (heartbeat_timeout_seconds_ <= 0.0) {
        return;
    }

    bool should_timeout = false;
    double age = 0.0;
    {
        std::lock_guard<std::mutex> lock(heartbeat_mutex_);
        if (!app_control_session_active_) {
            return;
        }
        if (!has_heartbeat_) {
            return;
        }
        age = (this->now() - last_heartbeat_time_).seconds();
        if (age <= heartbeat_timeout_seconds_) {
            return;
        }
        if (heartbeat_timeout_pending_) {
            return;
        }
        heartbeat_timeout_pending_ = std::make_shared<std::atomic<bool>>(true);
        should_timeout = true;
    }

    if (!should_timeout) {
        return;
    }

    RCLCPP_ERROR(
        this->get_logger(),
        "APP heartbeat timeout %.2fs > %.2fs; entering disconnect protection.",
        age, heartbeat_timeout_seconds_);
    handleAppConnectionLost("APP heartbeat timeout");
}
```

### 4.8 统一断联处理入口

当前 TCP disconnect 使用 `scheduleDisconnectAutoPause()`。建议新增统一函数：

```cpp
void handleAppConnectionLost(const std::string &reason)
{
    // 保持原有策略：先走延迟保护还是立即保护，取决于来源。
}
```

更安全的策略：

- TCP clean close：保留原有 `client_disconnect_auto_pause_seconds_` 延迟，允许快速重连。
- 心跳超时：不要再额外等 8 秒，因为已经等了 `heartbeat_timeout_seconds_`；立即执行保护。

因此：

```cpp
void handleHeartbeatTimeoutProtection(const std::string &reason)
{
    {
        std::lock_guard<std::mutex> client_lock(client_mutex_);
        if (client_sock_ >= 0) {
            closeClientSocketLocked();
        }
    }

    if (!canAttemptStartAllControl()) {
        RCLCPP_WARN(this->get_logger(), "%s, but start_all is not controllable; publishing StopMove only.", reason.c_str());
        publishSafetyStopMove(reason + ": start_all not controllable");
        return;
    }
    handlePauseCommand(reason);
}
```

`checkHeartbeatWatchdog()` 调这个函数，而不是 `scheduleDisconnectAutoPause()`。

### 4.9 TCP disconnect 和心跳 timeout 的互相取消

- 新 client 连接时：取消 `disconnect_auto_pause`；重置 heartbeat 状态，但不要默认激活作业会话。
- 收到心跳时：更新 last heartbeat；如果已有 heartbeat timeout pending，则取消 pending。
- 收到 stop/fail-safe 完成时：会话 inactive，取消 heartbeat pending。

`runTcpServer()` 新连接后增加：

```cpp
resetHeartbeatForNewClient("APP client connected");
```

但注意：如果 start_all 正在运行且 APP 重连，是否立即要求心跳？推荐：新连接时如果 `canAttemptStartAllControl()` 为 true，可设置 `app_control_session_active_=true` 且从当前时间开始计时，避免重连后 APP 不发心跳仍长期不保护。

### 4.10 send 失败也进入断联处理

当前 `sendPacket()` 在发送失败时只关闭 socket。建议改为：

```cpp
bool sendPacket(const uint8_t *data, size_t len)
{
    bool closed_active_client = false;
    {
        std::lock_guard<std::mutex> lock(client_mutex_);
        if (client_sock_ < 0) {
            return false;
        }
        if (!sendAllBytesLocked(client_sock_, data, len)) {
            closeClientSocketLocked();
            closed_active_client = true;
        } else {
            return true;
        }
    }
    if (closed_active_client && !stop_requested_.load()) {
        scheduleDisconnectAutoPause();
    }
    return false;
}
```

这样“向 APP 发送数据失败”也会触发原有 TCP 断联保护。

---

## 5. TDD 任务拆分

### Task 1: 添加心跳协议源码级回归测试

**Objective:** 先用测试锁定 `0xFF` 是 heartbeat，不是 Damp/姿态命令。

**Files:**
- Modify: `tests/test_app_node_failsafe.py`

**Step 1: Add tests**

新增测试：

```python
def test_heartbeat_function_code_is_not_damp_or_joy_button():
    text = source()
    teleop = teleop_source()
    assert "func_code == 0xFF" in text
    assert "RX HEARTBEAT" in text
    assert "handleHeartbeatPacket" in text
    assert "instruction_type == 0xFF" not in text
    assert "joy_msg.buttons[2] = 1" not in text
    assert ".Damp()" not in teleop
```

**Step 2: Run test to verify failure**

Run:

```bash
pytest -q tests/test_app_node_failsafe.py::test_heartbeat_function_code_is_not_damp_or_joy_button
```

Expected: FAIL，因为还未实现 `func_code == 0xFF` heartbeat 分支。

---

### Task 2: 实现心跳帧解析

**Objective:** `parseNextPacket()` 能识别 top-level `func_code == 0xFF` 心跳帧，并只更新心跳状态。

**Files:**
- Modify: `app_ws/src/app_node.cpp:245-315`

**Step 1: Add `func_code == 0xFF` branch**

按 4.4 代码实现。

**Step 2: Add `handleHeartbeatPacket()` and `markHeartbeatReceived()`**

放在 `handleRegionPacket()` 附近或 safety helper 前。

**Step 3: Run test**

```bash
pytest -q tests/test_app_node_failsafe.py::test_heartbeat_function_code_is_not_damp_or_joy_button
```

Expected: PASS。

---

### Task 3: 添加心跳状态和 watchdog 参数测试

**Objective:** 确认代码有心跳超时参数、会话状态、watchdog timer。

**Files:**
- Modify: `tests/test_app_node_failsafe.py`

**Step 1: Add test**

```python
def test_heartbeat_watchdog_has_timeout_and_session_gate():
    text = source()
    assert "heartbeat_timeout_seconds" in text
    assert "heartbeat_required_after_start" in text
    assert "heartbeat_watchdog_timer_" in text
    assert "checkHeartbeatWatchdog" in text
    assert "app_control_session_active_" in text
    assert "last_heartbeat_time_" in text
    assert "APP heartbeat timeout" in text
```

**Step 2: Run failing test**

```bash
pytest -q tests/test_app_node_failsafe.py::test_heartbeat_watchdog_has_timeout_and_session_gate
```

Expected: FAIL。

---

### Task 4: 实现心跳 watchdog

**Objective:** APP 作业会话中，超过心跳超时时间未收到心跳，立即进入断联保护。

**Files:**
- Modify: `app_ws/src/app_node.cpp`

**Step 1: Add parameters in constructor**

新增 `heartbeat_timeout_seconds` 和 `heartbeat_required_after_start`。

**Step 2: Add private members**

按 4.2 添加。

**Step 3: Add watchdog timer**

构造函数中创建 500ms timer。

**Step 4: Add `checkHeartbeatWatchdog()`**

按 4.7 实现。

**Step 5: Run test**

```bash
pytest -q tests/test_app_node_failsafe.py::test_heartbeat_watchdog_has_timeout_and_session_gate
```

Expected: PASS。

---

### Task 5: start/stop 与心跳会话状态联动

**Objective:** APP start 后进入心跳监管；stop/fail-safe 后退出监管，避免非作业场景误判。

**Files:**
- Modify: `app_ws/src/app_node.cpp:329-369, 685-690`
- Modify: `tests/test_app_node_failsafe.py`

**Step 1: Add tests**

```python
def test_start_activates_heartbeat_session_and_stop_deactivates_it():
    text = source()
    start_case = re.search(r"case 0x01:[\s\S]+?break;", text)
    assert start_case is not None
    assert "markAppControlSessionActive" in start_case.group(0)
    stop_handler = re.search(r"void handleStopCommand[\s\S]+?\n    void clearSafetyTimers", text)
    assert stop_handler is not None
    assert "markAppControlSessionInactive" in stop_handler.group(0)
```

**Step 2: Implement helpers**

新增：

```cpp
void markAppControlSessionActive(const std::string &reason)
void markAppControlSessionInactive(const std::string &reason)
void resetHeartbeatForNewClient(const std::string &reason)
```

**Step 3: Wire start/stop**

- `case 0x01` 调 `markAppControlSessionActive("start command")`。
- `handleStopCommand()` fail-safe 返回后调 `markAppControlSessionInactive("stop command")`。

**Step 4: Run test**

```bash
pytest -q tests/test_app_node_failsafe.py::test_start_activates_heartbeat_session_and_stop_deactivates_it
```

Expected: PASS。

---

### Task 6: 心跳超时保护必须走 StopMove/Pause，不走 StandDown/Damp

**Objective:** 确保心跳 timeout 和现有 fail-safe 一样安全。

**Files:**
- Modify: `tests/test_app_node_failsafe.py`
- Modify: `app_ws/src/app_node.cpp`

**Step 1: Add test**

```python
def test_heartbeat_timeout_uses_disconnect_protection_not_posture_buttons():
    text = source()
    match = re.search(r"void handleHeartbeatTimeoutProtection[\s\S]+?\n    bool requestTrigger", text)
    assert match is not None
    body = match.group(0)
    assert "publishSafetyStopMove" in body
    assert "handlePauseCommand" in body
    assert "StandDown" not in body
    assert "Damp" not in body
    assert "joy_msg.buttons" not in body
```

**Step 2: Implement `handleHeartbeatTimeoutProtection()`**

按 4.8 实现。

**Step 3: Run test**

```bash
pytest -q tests/test_app_node_failsafe.py::test_heartbeat_timeout_uses_disconnect_protection_not_posture_buttons
```

Expected: PASS。

---

### Task 7: send 失败进入 TCP 断联保护

**Objective:** 修复现有潜在问题：向 APP 发送失败时只关闭 socket，不调度 auto-pause。

**Files:**
- Modify: `app_ws/src/app_node.cpp:983-991`
- Modify: `tests/test_app_node_failsafe.py`

**Step 1: Add test**

```python
def test_send_failure_schedules_disconnect_auto_pause():
    text = source()
    match = re.search(r"bool sendPacket\(const uint8_t \*data[\s\S]+?\n    bool sendPacket\(const std::vector", text)
    assert match is not None
    body = match.group(0)
    assert "sendAllBytesLocked" in body
    assert "closeClientSocketLocked" in body
    assert "scheduleDisconnectAutoPause" in body
    assert body.find("closeClientSocketLocked") < body.find("scheduleDisconnectAutoPause")
```

**Step 2: Modify sendPacket**

按 4.10 改造，避免在持有 `client_mutex_` 时调用 `scheduleDisconnectAutoPause()`。

**Step 3: Run test**

```bash
pytest -q tests/test_app_node_failsafe.py::test_send_failure_schedules_disconnect_auto_pause
```

Expected: PASS。

---

### Task 8: 集成测试和源码安全扫描

**Objective:** 所有源码级 fail-safe 测试通过，并且源码中没有误触发 Damp/StandDown 的新增路径。

**Files:**
- Test: `tests/test_app_node_failsafe.py`

**Step 1: Run tests**

```bash
pytest -q tests/test_app_node_failsafe.py
```

Expected: all passed。

**Step 2: Search unsafe patterns**

```bash
grep -R "Damp()\|buttons\[2\]\|instruction_type == 0xFF\|publishSafetyDamp\|allow_damp_fallback" \
  app_ws/src/app_node.cpp b2w_navigation_ws/src/b2w_teleop.cpp tests/test_app_node_failsafe.py
```

Expected:

- `app_node.cpp` 不出现 `instruction_type == 0xFF`。
- `app_node.cpp` 不出现 `buttons[2]`。
- `b2w_teleop.cpp` 不出现 `Damp()`。
- tests 中可以出现这些字符串作为 negative assertions。

---

### Task 9: 编译 app_ws

**Objective:** 确认 C++ 编译通过。

**Files:**
- Build: `app_ws`

**Step 1: Build**

```bash
source /opt/ros/humble/setup.bash
source colcon_ws/install/setup.bash 2>/dev/null || true
source b2w_navigation_ws/install/setup.bash 2>/dev/null || true
cd app_ws
colcon build --packages-select robot_tcp
```

Expected: build finished with exit code 0。

如果本机 ROS 环境不可用，在 Jetson/目标机上按仓库 CLAUDE.md 的 workspace 顺序编译验证。

---

### Task 10: 更新文档

**Objective:** 文档与实际代码一致，现场人员知道心跳和断联策略。

**Files:**
- Modify: `docs/app_control_disconnect_safety.md`
- Modify: `docs/log_view.md`
- Modify: `CLAUDE.md` if final parameter names changed

**Step 1: Update protocol docs**

写明：

- `0xFF` 是 top-level 心跳功能码，不是 `0x08` 控制子命令。
- APP 每 1 秒发送。
- 机器人默认 `heartbeat_timeout_seconds=3.5` 秒超时。
- 超时后保持静止，不触发 StandDown/Damp。

**Step 2: Update log troubleshooting docs**

`docs/log_view.md` 中旧的 `0xFF -> Damp()` 要删除或改成 heartbeat。

**Step 3: Verify docs**

```bash
grep -R "0xFF.*Damp\|0xFF.*急停阻尼" CLAUDE.md docs/*.md
```

Expected: no match，除非是在“旧版本风险说明”里明确标注已废弃。

---

## 6. 手工/现场验收

### 6.1 正常心跳

1. APP 连接。
2. APP 每秒发送 `F5 FF 01 00 FF 00 00 5F`。
3. APP 发送 `0x01 start`。
4. 观察 10 秒。

预期：不触发 heartbeat timeout，不 pause。

### 6.2 心跳超时但 TCP 半开

1. APP 连接并 start。
2. 停止 APP 心跳发送，但不主动关闭 socket。
3. 等待 `heartbeat_timeout_seconds`。

预期：

- `robot_tcp.log` 出现 `APP heartbeat timeout`。
- 如果 `start_all` 可控，调用 `/emergency_stop` 并确认 `Task paused`。
- 如果不可控，发布 StopMove。
- 不出现 `StandDown` / `Damp`。

### 6.3 TCP clean disconnect

1. APP 连接并 start。
2. 关闭 APP socket。
3. 等待 `client_disconnect_auto_pause_seconds`。

预期：沿用原断联 auto-pause 流程。

### 6.4 快速重连

1. APP start 后断开。
2. 在 8 秒内重连并恢复心跳。

预期：旧 disconnect timer 取消，不自动 pause。

### 6.5 趴下按钮

1. APP 发 `0x0A`。

预期：只有该动作触发 StandDown。

---

## 7. Gemini 复核问题清单

提交给 Gemini 复核时重点问：

1. 心跳 timeout 是否应该立即保护，还是复用 8 秒 disconnect delay？
2. `app_control_session_active_` 只由 `0x01 start` 激活是否足够？APP 手动遥控到作业点阶段是否也应强制心跳？
3. `sendPacket()` 失败后调度 disconnect auto-pause 是否会和 `handleClient()` 的断开处理重复？是否需要 token 去重？
4. 锁顺序是否安全：`client_mutex_`、`heartbeat_mutex_`、`disconnect_timer_mutex_` 是否可能死锁？
5. 心跳包 `data_len` 是否严格要求 1 更安全，还是兼容 `data_len==0/1/2`？
6. heartbeat timeout 后是否应关闭 active socket，防止半开连接继续占用唯一 client slot？
7. 是否需要在 `/erase_emergency_stop` resume 前要求心跳新鲜？

---

## 8. 风险和待决策点

1. 心跳超时时间：建议 3.5 秒，但现场 WiFi 抖动大时可调到 5 秒。
2. 手动遥控阶段是否强制心跳：如果 APP 接管手动遥控时也必须断联保护，应把 `0x04~0x09` 也视为激活 APP 控制会话。
3. 心跳 timeout 后是否 kill `start_all`：当前建议先 pause，pause 不确认再 fail-safe stop；若用户定义“断联必须终止任务”，则直接走 `triggerStartAllFailSafeStop()`。
4. CRC：当前暂不校验 CRC；如要严格，应同时补齐控制帧和心跳帧 CRC 校验。
5. 喷枪/机械臂：本计划主要保护底盘和主控状态机；若要求断联立刻停止喷枪/机械臂，需要另开执行器急停任务。
