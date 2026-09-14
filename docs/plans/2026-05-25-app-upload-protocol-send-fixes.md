# 机器狗端 APP 上传协议发送问题修复 Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** 修复当前机器狗端发给 APP 的协议帧中仍存在的内容正确性和状态清空问题，确保进度、点位、坐标、温度、轨迹等上传数据可被 APP 稳定解析和显示。

**Architecture:** `app_ws/src/app_node.cpp` 继续作为 TCP 9002 协议出口，负责把 ROS 话题转换为 APP TCP 帧；`b2w_navigation_ws/src/main.cpp` 负责发布导航状态话题。修复重点放在 TCP 发送前的数据类型匹配、无效值过滤、空列表显式发送，以及发送频率/边界条件保护。协议格式保持与 `docs/喷涂机器人通信协议05-25.md` 一致。

**Tech Stack:** C++17, ROS 2 Humble `rclcpp`, `sensor_msgs/msg/PointCloud2`, `nav_msgs/msg/Odometry`, TCP socket, pytest / 静态源码级回归测试, colcon。

---

## 0. 当前问题总表

| 优先级 | 功能码 / 数据 | 问题 | 影响 | 主要文件 |
|---|---|---|---|---|
| P0 | `0x02` 已完成点、`0x03` 未完成点 | TCP 节点用 `PointCloud2ConstIterator<float>` 读取发布端 `FLOAT64` 点云 | 点数正确，但 x/y/z 坐标大概率错误 | `app_ws/src/app_node.cpp` |
| P0 | `0x07` 当前坐标 | RTK 未有效时会发送 NaN 坐标 | APP 地图/坐标显示可能异常 | `app_ws/src/app_node.cpp`, `b2w_navigation_ws/src/main.cpp` |
| P1 | `0x02` / `0x03` 空点列表 | `num_points == 0` 时直接 return，不发送清空帧 | APP 已完成/未完成点列表可能残留旧数据 | `app_ws/src/app_node.cpp` |
| P1 | `0x01` 全部目标点 | `all_points_.empty()` 时不发送任何帧 | APP 无法区分未发送、读取失败和目标点为空 | `app_ws/src/app_node.cpp` |
| P1 | `0x04` 轨迹 | 空轨迹时不发送清空帧 | APP 轨迹可能残留旧线 | `app_ws/src/app_node.cpp` |
| P1 | `0x04` 轨迹 | 超过 255 点时只发送最早 255 个旧轨迹点 | APP 长时间看不到最新运动轨迹 | `app_ws/src/app_node.cpp` |
| P1 | TCP 新 client 重连 | 新连接只主动发送全部目标点，不重放进度/已完成/未完成/轨迹/位置/温度缓存 | APP 中途断线重连后状态不完整，需等下一次 ROS 话题更新 | `app_ws/src/app_node.cpp` |
| P2 | `0x01` 全部目标点 | 超过 65535 点时 packet 按截断数量分配，但循环仍遍历全部点 | 极端情况下内存越界 | `app_ws/src/app_node.cpp` |
| P2 | `0x07` 当前坐标 | 约 20Hz 高频上传 | APP/网络弱时可能积压或 UI 卡顿 | `app_ws/src/app_node.cpp` |

---

## 1. 证据摘要

### 1.1 代码证据

- `app_ws/src/app_node.cpp:1285-1324`
  - `sendPointCloud()` 当前使用 `PointCloud2ConstIterator<float>`。
  - 但发送协议中又把读取值转成 double 写入 TCP 帧。
- `b2w_navigation_ws/src/main.cpp:1171-1192`
  - `makeWaypointCloud()` 使用 `PointField::FLOAT64`。
  - 写入时使用 `PointCloud2Iterator<double>`。
- `app_ws/src/app_node.cpp:1290`
  - `if (num_points == 0) return;` 导致空 `0x02/0x03` 不发送。
- `b2w_navigation_ws/src/main.cpp:1131`
  - `current_x_ / current_y_ / current_z_` 初始为 `NAN`。
- `app_ws/src/app_node.cpp:1374-1392`
  - `sendPosition()` 不检查 finite，直接发送 NaN。
- `app_ws/src/app_node.cpp:1327-1357`
  - `sendAllPoints()` 空点直接 return；超过 65535 点时循环仍遍历 `all_points_`。
- `app_ws/src/app_node.cpp:1413-1457`
  - `sendPath()` 空轨迹直接 return。
  - 超过 255 点时 `N=255`，但循环仍从 `poses[0]` 开始发送最旧轨迹点。
- `app_ws/src/app_node.cpp:209-212`
  - 新 TCP client 连接后只调用 `sendAllPoints()`，没有重放已完成/未完成点、进度、轨迹、位置和温度缓存。

### 1.2 日志证据

5-22 日志统计：

```text
robot_tcp.log 文件数: 76
TX ALL_POINTS: 95 次
TX PROGRESS: 226 次，值范围 0~100
TX POSITION: 80012 次，帧长度固定 29
Sent max temperature: 2815 次，值范围 26~49
Sent Path: 942 次，点数范围 1~255
Received /acquired_points: 503 次
Received /unacquired_points: 503 次
Sent PointCloud: 998 次
Cloud step=24 fields=3
NaN position: 153 次
```

关键样例：

```text
Cloud step=24 fields=3
Sent PointCloud: 1 points func=0x02
Sent PointCloud: 99 points func=0x03

Sent position : (nan, nan, nan)
TX POSITION [len=29] : F5 07 00 00 00 00 00 00 F8 7F ... 00 00 5F
```

---

## 2. 修复原则

1. 不改变已更新协议文档中的帧格式。
2. 所有坐标/点位/轨迹 TCP payload 中的 x/y/z 仍使用 little-endian double。
3. 空列表应显式发送 `count=0` 或 `N=0`，让 APP 能清空旧状态。
4. 机器人端不应发送 NaN/Inf 坐标给 APP。
5. 对超过协议上限的数据必须安全截断，不能越界写内存。
6. `PointCloud2` 字段类型检查是强制要求，不是可选增强；字段不存在或不是 `FLOAT64` 时应拒绝发送并打印可诊断日志。
7. `0x04` 轨迹超过 255 点时应发送最新 255 点，而不是最旧 255 点。
8. 新 TCP client 连接后应重放最近一次 APP 上传状态，避免 APP 断线重连后 UI 状态不完整。
9. `0x07` 节流必须使用类成员保存上次发送时间和位置，不能用回调局部变量。
10. 默认不在 `b2w_navigation_ws` 停止发布 `/b2w_odom`；非 finite 坐标过滤放在 `app_ws` TCP 边缘层，避免影响其他 ROS 消费者。
11. 修改后必须用测试覆盖：点云 double 读取、字段类型校验、空列表帧、NaN 过滤、边界截断、最新轨迹截断、重连状态重放。

---

## 3. 需要修改 / 新增的文件

- Modify: `app_ws/src/app_node.cpp`
- Optional Modify: `b2w_navigation_ws/src/main.cpp`
- Create or Modify: `tests/test_app_upload_protocol.py`
- Optional Modify: `docs/喷涂机器人通信协议05-25.md`（如果实现决定改变空帧策略或发送频率，需要同步）
- Optional Modify: `docs/log_view.md`（增加如何排查 APP 上传协议日志）

---

## 3.1 Gemini Reviewer 追加审查意见（必须纳入实施）

本计划经 Gemini 作为独立 reviewer 审查后，结论为“可行，但需补充修改”。实施时必须吸收以下 5 类意见：

1. **轨迹超过 255 点时发送最新轨迹段**：当前 `sendPath()` 截断后仍从 `poses[0]` 开始发，APP 只能看到旧轨迹。应改为从 `poses.size() - N` 开始发送最新 255 点。
2. **TCP 新连接重放最近状态**：新 client 连接后不能只发送 `0x01` 全部目标点，还要重放最近一次 `0x02/0x03/0x04/0x05/0x06/0x07`，使 APP 断线重连后立即恢复 UI 状态。
3. **点云字段类型校验改为强制**：`PointCloud2ConstIterator<double>` 前必须检查 `x/y/z` 字段存在且 datatype 为 `FLOAT64`，避免字段来源变化时异常丢包且难排查。
4. **不要在导航端停止发布 `/b2w_odom`**：NaN/Inf 过滤应默认只放在 `app_node.cpp` TCP 发送层；除非单独确认无其他消费者依赖，否则不要让 `b2w_navigation_ws` 因 RTK 无效而停止发布 odom。
5. **位置上传节流需要类级状态**：`position_upload_hz` / `position_upload_min_distance` 需要配套 `last_position_sent_time_`、`has_last_sent_position_`、`last_sent_x_/y_/z_` 等成员变量。

---

## 4. 任务清单

### Task 1: 为 APP 上传协议添加源码级回归测试骨架

**Objective:** 新增测试文件，用源码扫描/小型解析方式锁定协议关键实现，先覆盖当前已知问题。

**Files:**

- Create: `tests/test_app_upload_protocol.py`

**Steps:**

1. 新建测试文件。
2. 加入 helper 读取 `app_ws/src/app_node.cpp` 和 `b2w_navigation_ws/src/main.cpp`。
3. 添加测试占位：
   - `test_pointcloud_sender_reads_float64_cloud_as_double()`
   - `test_empty_pointcloud_sends_zero_count_frame()`
   - `test_position_sender_filters_non_finite_values()`
   - `test_all_points_truncation_loop_uses_num_points()`
   - `test_empty_path_sends_zero_count_frame()`
   - `test_path_truncation_sends_latest_255_points()`
   - `test_new_client_replays_cached_upload_state()`
   - `test_pointcloud_sender_requires_float64_xyz_fields()`
4. 先运行测试，确认当前代码至少 P0/P1 用例失败。

**Verification:**

```bash
pytest tests/test_app_upload_protocol.py -v
```

Expected before implementation: tests fail on current known issues.

---

### Task 2: 修复 `0x02/0x03` 点云 double/float 类型不匹配

**Objective:** 让已完成/未完成目标点按发布端 `FLOAT64` 正确读取并按协议写入 double。

**Files:**

- Modify: `app_ws/src/app_node.cpp:1285-1324`
- Test: `tests/test_app_upload_protocol.py`

**Current problem:**

```cpp
sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
```

**Target implementation:**

```cpp
sensor_msgs::PointCloud2ConstIterator<double> iter_x(*cloud, "x");
sensor_msgs::PointCloud2ConstIterator<double> iter_y(*cloud, "y");
sensor_msgs::PointCloud2ConstIterator<double> iter_z(*cloud, "z");
```

**Required robust version:**

必须增加字段类型检查；如果 `x/y/z` 字段缺失、不是 `FLOAT64`，或 `point_step / row_step / data.size()` 与 `width * height` 不匹配，打印 error 并跳过发送，避免误读或异常丢包。不要只依赖 `PointCloud2ConstIterator<double>` 抛异常。

建议 helper：

```cpp
bool hasFloat64XYZFields(const sensor_msgs::msg::PointCloud2 &cloud)
{
    auto has_field = [&cloud](const char *name) {
        return std::any_of(cloud.fields.begin(), cloud.fields.end(),
            [name](const sensor_msgs::msg::PointField &field) {
                return field.name == name &&
                       field.datatype == sensor_msgs::msg::PointField::FLOAT64;
            });
    };
    return has_field("x") && has_field("y") && has_field("z") &&
           cloud.point_step >= 3 * sizeof(double) &&
           cloud.data.size() >= static_cast<size_t>(cloud.width) * cloud.height * cloud.point_step;
}
```

**Verification:**

```bash
pytest tests/test_app_upload_protocol.py::test_pointcloud_sender_reads_float64_cloud_as_double -v
pytest tests/test_app_upload_protocol.py::test_pointcloud_sender_requires_float64_xyz_fields -v
```

Expected: PASS.

---

### Task 3: 修复 `0x02/0x03` 空点列表不发送

**Objective:** 当 `/acquired_points` 或 `/unacquired_points` 为空时，仍发送合法清空帧。

**Files:**

- Modify: `app_ws/src/app_node.cpp:1289-1290`
- Test: `tests/test_app_upload_protocol.py`

**Current problem:**

```cpp
size_t num_points = cloud->width * cloud->height;
if (num_points == 0) return;
```

**Target behavior:**

- 对 `func_code=0x02`：发送 `F5 02 00 00 00 00 5F`。
- 对 `func_code=0x03`：发送 `F5 03 00 00 00 00 5F`。

**Suggested helper:**

```cpp
void sendEmptyPointCloud(uint8_t func_code)
{
    uint8_t packet[] = {0xF5, func_code, 0x00, 0x00, 0x00, 0x00, 0x5F};
    sendPacket(packet, sizeof(packet));
    RCLCPP_INFO(this->get_logger(), "Sent empty PointCloud func=0x%02X", func_code);
}
```

**Verification:**

```bash
pytest tests/test_app_upload_protocol.py::test_empty_pointcloud_sends_zero_count_frame -v
```

Expected: PASS.

---

### Task 4: 修复 `0x07` 当前坐标 NaN/Inf 发送

**Objective:** TCP 层不要向 APP 发送无效坐标帧。

**Files:**

- Modify: `app_ws/src/app_node.cpp:1374-1392`
- Test: `tests/test_app_upload_protocol.py`

**Target implementation in TCP sender:**

```cpp
void sendPosition(double x, double y, double z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Skip invalid position: x=%.3f y=%.3f z=%.3f", x, y, z);
        return;
    }
    ...
}
```

**Scope boundary:**

默认不要修改 `b2w_navigation_ws/src/main.cpp` 来停止发布 `/b2w_odom`。`/b2w_odom` 可能被其他本地 ROS 节点、TF、调试工具依赖；本任务只在 TCP 边缘发送层过滤 NaN/Inf，确保 APP 不收到无效坐标，同时不改变 ROS 内部话题行为。

**Verification:**

```bash
pytest tests/test_app_upload_protocol.py::test_position_sender_filters_non_finite_values -v
```

Expected: PASS.

Manual log expectation after deployment:

```text
Skip invalid position: x=nan y=nan z=nan
```

而不是：

```text
TX POSITION [len=29] : F5 07 ... F8 7F ... 5F
```

---

### Task 5: 修复 `0x01` 全部目标点为空时不发送

**Objective:** 新 APP client 连接后，即使目标点为空，也发送 `0x01 count=0`，让 APP 明确清空全部目标点列表。

**Files:**

- Modify: `app_ws/src/app_node.cpp:1327-1357`
- Test: `tests/test_app_upload_protocol.py`

**Target behavior:**

当 `all_points_.empty()` 时发送：

```text
F5 01 00 00 00 00 5F
```

**Suggested implementation:**

```cpp
if (all_points_.empty()) {
    uint8_t packet[] = {0xF5, 0x01, 0x00, 0x00, 0x00, 0x00, 0x5F};
    sendPacket(packet, sizeof(packet));
    logPacket("TX ALL_POINTS_EMPTY", packet, sizeof(packet), this->get_logger());
    return;
}
```

**Verification:**

```bash
pytest tests/test_app_upload_protocol.py::test_all_points_empty_sends_zero_count_frame -v
```

Expected: PASS.

---

### Task 6: 修复 `0x01` 全部目标点超过 65535 时潜在越界

**Objective:** 确保截断后只写入 `num_points` 个点，不遍历完整 `all_points_`。

**Files:**

- Modify: `app_ws/src/app_node.cpp:1330-1349`
- Test: `tests/test_app_upload_protocol.py`

**Current problem:**

```cpp
if (num_points > std::numeric_limits<uint16_t>::max()) {
    num_points = std::numeric_limits<uint16_t>::max();
}
...
for (const auto& p : all_points_) {
    ...
}
```

**Target implementation:**

```cpp
for (size_t i = 0; i < num_points; ++i) {
    const auto& p = all_points_[i];
    std::memcpy(&packet[idx], &p.x, sizeof(double)); idx += sizeof(double);
    std::memcpy(&packet[idx], &p.y, sizeof(double)); idx += sizeof(double);
    std::memcpy(&packet[idx], &p.z, sizeof(double)); idx += sizeof(double);
}
```

**Verification:**

```bash
pytest tests/test_app_upload_protocol.py::test_all_points_truncation_loop_uses_num_points -v
```

Expected: PASS.

---

### Task 7: 修复 `0x04` 空轨迹不发送

**Objective:** 轨迹为空时发送合法清空帧，避免 APP 保留旧轨迹。

**Files:**

- Modify: `app_ws/src/app_node.cpp:1413-1415`
- Test: `tests/test_app_upload_protocol.py`

**Current problem:**

```cpp
if (path_msg->poses.empty()) return;
```

**Target behavior:**

发送：

```text
F5 04 00 00 00 5F
```

含义：

- `0x04`：轨迹功能码
- `N=0`：轨迹点数为 0
- `00 00`：CRC 占位
- `5F`：包尾

**Verification:**

```bash
pytest tests/test_app_upload_protocol.py::test_empty_path_sends_zero_count_frame -v
```

Expected: PASS.

---

### Task 8: 修复 `0x04` 轨迹超过 255 点时发送旧轨迹段

**Objective:** 轨迹超过协议上限时发送最新 255 点，确保 APP 看到最近运动轨迹。

**Files:**

- Modify: `app_ws/src/app_node.cpp:1417-1437`
- Test: `tests/test_app_upload_protocol.py`

**Current problem:**

```cpp
uint8_t N = static_cast<uint8_t>(path_msg->poses.size());
if (path_msg->poses.size() > 255) {
     N = 255;
}
for (int i = 0; i < N; ++i) {
    const auto &pose_stamped = path_msg->poses[i];
    ...
}
```

当轨迹超过 255 点时，上述代码发送最早的 255 个点，导致 APP 看不到最新运动轨迹。

**Target implementation:**

```cpp
const size_t total_points = path_msg->poses.size();
const size_t num_points = std::min<size_t>(total_points, 255);
const uint8_t N = static_cast<uint8_t>(num_points);
const size_t start_idx = total_points > num_points ? total_points - num_points : 0;

for (size_t i = 0; i < num_points; ++i) {
    const auto &pose_stamped = path_msg->poses[start_idx + i];
    double x = pose_stamped.pose.position.x;
    double y = pose_stamped.pose.position.y;
    double z = pose_stamped.pose.position.z;
    std::memcpy(&packet[idx], &x, sizeof(double)); idx += sizeof(double);
    std::memcpy(&packet[idx], &y, sizeof(double)); idx += sizeof(double);
    std::memcpy(&packet[idx], &z, sizeof(double)); idx += sizeof(double);
}
```

**Verification:**

```bash
pytest tests/test_app_upload_protocol.py::test_path_truncation_sends_latest_255_points -v
```

Expected: PASS.

---

### Task 9: TCP 新连接重放最近 APP 上传状态

**Objective:** APP 中途断线重连后，立即收到最近一次已完成/未完成点、进度、轨迹、位置和温度状态。

**Files:**

- Modify: `app_ws/src/app_node.cpp:89-115`
- Modify: `app_ws/src/app_node.cpp:209-212`
- Modify: `app_ws/src/app_node.cpp:1518-1560`
- Test: `tests/test_app_upload_protocol.py`

**Current problem:**

```cpp
RCLCPP_INFO(this->get_logger(), "New client connected from %s", client_ip);
sendAllPoints();
client_thread_ = std::thread(&RemoteControlNode::handleClient, this, new_socket);
```

新 client 只收到全部目标点。若 APP 在任务中途重连，可能收不到当前进度、已完成/未完成点、历史轨迹、当前位置和温度，直到下一次 ROS 话题更新。

**Target implementation:**

1. 在各 ROS subscription 回调里先缓存最近消息，再调用原发送函数。
2. 新增 `replayLatestUploadStateToClient()`，在 `sendAllPoints()` 之后调用。
3. 缓存访问需要 mutex 保护，避免 TCP accept 线程和 ROS executor 回调并发读写。

建议成员：

```cpp
std::mutex upload_state_mutex_;
std::optional<uint8_t> latest_progress_;
sensor_msgs::msg::PointCloud2::SharedPtr latest_acquired_points_;
sensor_msgs::msg::PointCloud2::SharedPtr latest_unacquired_points_;
nav_msgs::msg::Path::SharedPtr latest_path_;
std::optional<std::array<double, 3>> latest_position_;
std::optional<uint8_t> latest_max_temperature_;
```

建议重放顺序：

```cpp
void replayLatestUploadStateToClient()
{
    // 复制 shared_ptr / optional 到局部变量后释放 mutex，再调用 send*，避免锁内网络发送阻塞 ROS 回调。
    // 顺序：0x02 acquired -> 0x03 unacquired -> 0x06 progress -> 0x04 path -> 0x07 position -> 0x05 temperature。
}
```

**Important:**

- `sendAllPoints()` 仍在新连接后首先发送，因为它来自点位文件，不一定有 ROS 缓存。
- `latest_position_` 只缓存 finite 坐标。
- 温度建议缓存已经 clamp 后的 `uint8_t`，重放时避免重新依赖原始数组生命周期。
- 不要在 `upload_state_mutex_` 锁内调用 `sendPacket()`，避免 TCP 阻塞导致 ROS 回调卡住。

**Verification:**

```bash
pytest tests/test_app_upload_protocol.py::test_new_client_replays_cached_upload_state -v
```

Expected: PASS.

---

### Task 10: 降低或节流 `0x07` 当前坐标上传频率

**Objective:** 降低 TCP 和 APP UI 压力，同时保留足够流畅的当前位置显示。

**Files:**

- Modify: `app_ws/src/app_node.cpp:94-97` or `app_ws/src/app_node.cpp:1374-1392`
- Test: `tests/test_app_upload_protocol.py`

**Current behavior:**

`/b2w_odom` 约 20Hz 发布，TCP 节点收到后每帧都发送 `0x07`。

**Suggested target behavior:**

新增参数：

```cpp
this->declare_parameter<double>("position_upload_hz", 5.0);
this->declare_parameter<double>("position_upload_min_distance", 0.02);
```

新增类成员：

```cpp
double position_upload_hz_{5.0};
double position_upload_min_distance_{0.02};
rclcpp::Time last_position_sent_time_;
bool has_last_sent_position_{false};
double last_sent_x_{0.0};
double last_sent_y_{0.0};
double last_sent_z_{0.0};
```

发送条件：

- 距离上次发送超过 `1 / position_upload_hz` 秒；或
- 位置变化超过 `position_upload_min_distance`；
- 且坐标 finite。

**Verification:**

1. 源码测试确认存在节流逻辑。
2. 现场日志确认单位时间内 `TX POSITION` 数量下降。

Example command after deployment:

```bash
grep -R "TX POSITION" /home/test/logs/tcp_base_ctl_latest/robot_tcp.log | wc -l
```

Expected: 约 5Hz，而不是 20Hz。

---

### Task 11: 同步协议排查文档和日志说明

**Objective:** 把修复后的行为写入文档，便于现场 APP 联调和日志排查。

**Files:**

- Modify: `docs/喷涂机器人通信协议05-25.md`
- Optional Modify: `docs/log_view.md`

**Update points:**

1. `0x01/0x02/0x03`：明确 `count=0` 是合法清空列表帧，且机器人端应发送。
2. `0x04`：明确 `N=0` 是合法清空轨迹帧。
3. `0x07`：明确机器人端应过滤 NaN/Inf，不向 APP 发送无效坐标。
4. `0x07`：如果实现了节流，记录默认上传频率。
5. `0x04`：明确超过 255 点时发送最新 255 点。
6. TCP 新连接：明确会重放最近一次 APP 上传状态。

**Verification:**

```bash
grep -nE "count=0|N=0|NaN|Inf|position_upload_hz|最新 255|重放" docs/喷涂机器人通信协议05-25.md docs/log_view.md
```

Expected: 能查到对应说明。

---

### Task 12: 编译和静态回归验证

**Objective:** 确认代码可编译，测试通过，且 ROS workspace 依赖顺序正确。

**Files:**

- Modify/Test: `app_ws`
- Optional Modify/Test: `b2w_navigation_ws`

**Commands:**

按仓库约定，涉及 `app_ws` 单独编译可执行：

```bash
cd /home/oneko/projects/spray_b2w_robot_project_greek
pytest tests/test_app_upload_protocol.py -v
cd app_ws && colcon build && source install/setup.bash && cd ..
```

如果修改了 `b2w_navigation_ws`，需要先 source 依赖工作空间：

```bash
cd /home/oneko/projects/spray_b2w_robot_project_greek
cd z1_move_ws && source install/setup.bash && cd ..
cd spray_path_planner_ws && source install/setup.bash && cd ..
cd b2w_navigation_ws && colcon build && source install/setup.bash && cd ..
```

**Expected:**

- pytest 通过。
- colcon build 通过。
- 无新增编译错误。

---

### Task 13: 现场日志验收

**Objective:** 部署到狗端后，用日志确认协议发送行为已经修复。

**Files / Logs:**

- `/home/test/logs/tcp_base_ctl_latest/robot_tcp.log`
- `/home/test/logs/tcp_base_ctl_latest/b2w_navigation.log`

**Checks:**

```bash
# 1. 不应再发送 NaN 坐标
grep -i "nan" /home/test/logs/tcp_base_ctl_latest/robot_tcp.log

# 2. 应能看到空点列表/空轨迹发送日志
 grep -E "empty PointCloud|ALL_POINTS_EMPTY|empty Path|count=0|N=0" /home/test/logs/tcp_base_ctl_latest/robot_tcp.log

# 3. 点云仍正常发送
 grep "Sent PointCloud" /home/test/logs/tcp_base_ctl_latest/robot_tcp.log | tail -20

# 4. 进度仍为 0~100
 grep "TX PROGRESS" /home/test/logs/tcp_base_ctl_latest/robot_tcp.log | tail -20

# 5. 位置帧长度仍为 29
 grep "TX POSITION" /home/test/logs/tcp_base_ctl_latest/robot_tcp.log | tail -20

# 6. 轨迹超过 255 点时应发送最新轨迹段，现场结合 APP 显示或抓包确认尾部点在变化
grep "Sent Path" /home/test/logs/tcp_base_ctl_latest/robot_tcp.log | tail -20

# 7. APP 断线重连后应立即看到状态重放日志
grep -E "New client connected|replay|Replay" /home/test/logs/tcp_base_ctl_latest/robot_tcp.log | tail -40
```

**Expected:**

- `nan` 不再出现在 `TX POSITION` 中。
- `0x02/0x03` 在空列表时发送 `count=0` 帧。
- `0x04` 在空轨迹时发送 `N=0` 帧。
- `0x04` 超过 255 点时发送最新 255 点，而不是最早 255 点。
- APP 断线重连后能立即恢复进度、已完成/未完成点、轨迹、位置、温度状态。
- `TX PROGRESS` 仍是 `F5 06 01 00 PP 00 00 5F`。
- `TX POSITION` 仍是 29 字节，但不包含 `F8 7F` NaN double。

---

## 5. 建议提交拆分

不要一次性大提交，建议按风险拆分：

1. `test: add app upload protocol regression tests`
2. `fix: correct pointcloud double upload for app protocol`
3. `fix: send empty app point lists and path frames`
4. `fix: send latest truncated path points`
5. `fix: replay cached app upload state on reconnect`
6. `fix: filter invalid app position uploads`
7. `fix: guard all-points upload truncation`
8. `perf: throttle app position upload rate`
9. `docs: update app upload protocol verification notes`

提交信息按仓库偏好可用 gitmoji 中文风格，例如：

```text
🐛 修复 APP 点云上传 double 解析错误

- 将 PointCloud2ConstIterator<float> 改为 double
- 保持 0x02/0x03 TCP payload 为 x/y/z double
- 增加协议回归测试覆盖 FLOAT64 点云
```

---

## 6. 验收标准

修复完成后必须满足：

1. `0x02/0x03` 点位坐标与 `gnss_waypoints.txt` / `/acquired_points` / `/unacquired_points` 发布值一致。
2. APP 可收到 `count=0` 清空已完成/未完成列表。
3. APP 可收到 `N=0` 清空轨迹。
4. `0x07` 不再发送 NaN/Inf 坐标。
5. `0x01` 目标点为空时仍发送 `count=0`。
6. `0x01` 点数超过 65535 时安全截断，不越界。
7. `0x04` 点数超过 255 时发送最新 255 个轨迹点。
8. TCP 新 client 连接后会重放最近一次 `0x02/0x03/0x04/0x05/0x06/0x07` 状态。
9. `0x05` 温度、`0x06` 进度、`0x04` 非空轨迹原有格式不被破坏。
10. 相关 pytest 和 colcon build 通过。

---

## 7. 当前不在本计划内的事项

以下事项不在本轮修复范围，避免扩大改动：

1. 不重新设计 CRC；当前仍保持 `00 00` 占位。
2. 不改变 APP 下发控制协议 `0x08`。
3. 不改变心跳协议 `0xFF`。
4. 不新增完整电机温度数组协议；`0x05` 仍只表示最高电机温度。
5. 不改变坐标系定义；目标点和当前位置仍使用当前 EPSG:2100 / HEPOS-correct 坐标链路。
