# B2W 喷涂作业精度提升路线图

> 文档目的：在项目固定结构下（**丰疆录目标 + 司南 RTK 跟踪 + 丰疆复测落点**），把端到端误差 ③ 压到 **3 cm 以内**。
>
> **当前进度**（截至 5-7 实测）：③ 已从 5-6 基线 **11.1 cm 降至 8.0 cm**（test2 同 5 点位置重测），最佳单点 **2.9 cm**（test5 pt2）。剩余 5 cm 主要来自 `rtk_x_offset` 未标定的 dE +5.4 cm 系统偏置。
>
> 配套报告：
> - `logs/5-6/test_logs/5-6.md`（5-6 基线，①②③ 误差拆解、链路一致性验证）
> - `logs/5-7/ana.md`（5-7 三组实验：test2 同点重测、test4 8 点公共点标定、test5 新点验证）

---

## 目录

- [0. 项目约束（不可改）](#0-项目约束不可改)
- [1. 误差结构](#1-误差结构)
  - [1.1 误差链路图](#11-误差链路图)
  - [1.2 为什么"换录点设备"无法消除 ①](#12-为什么换录点设备无法消除-)
  - [1.3 两条腿走路](#13-两条腿走路)
- [2. 腿 A：机器人侧改进](#2-腿-a机器人侧改进)
  - [2.1 已完成 — C：车头阈值收紧](#21-已完成--c车头阈值收紧)
  - [2.2 A1：天线 + 机械臂外参现场标定](#22-a1天线--机械臂外参现场标定)
  - [2.3 已完成 — A2：多帧 RTK 均值](#23-已完成--a2多帧-rtk-均值)
  - [2.4 A4：对录点方的流程要求](#24-a4对录点方的流程要求)
- [3. 腿 B：司南-丰疆系统偏差校正](#3-腿-b司南-丰疆系统偏差校正)
  - [3.1 偏差性质判定](#31-偏差性质判定)
  - [3.2 B2：建立公共点网络](#32-b2建立公共点网络)
  - [3.3 B3：录入流程加 T 校正](#33-b3录入流程加-t-校正)
  - [3.4 标定的复用范围](#34-标定的复用范围)
- [4. 闭环 SOP（每次作业要做的事）](#4-闭环-sop每次作业要做的事)
- [5. 推进路线 / 时间表](#5-推进路线--时间表)
- [6. 待开发工具清单](#6-待开发工具清单)
- [7. 验收标准与里程碑](#7-验收标准与里程碑)
- [附录：参考资料](#附录参考资料)

---

## 0. 项目约束（不可改）

| 环节 | 设备 | 不可更改原因 |
|---|---|---|
| 地面物理标记的目标点 | **丰疆 RTK** | 项目规范 |
| 机器人作业时实时定位 | **司南 RTK**（机器狗自带） | 硬件约束，机器狗只能用司南 |
| 喷涂落点的真值复测 | **丰疆 RTK** | 项目规范 |

也就是说：

- 目标点（输入）= 丰疆视角下物理点的 EPSG 数值
- 作业实时跟踪 = 司南视角下的 EPSG 数值
- 真值（输出）= 丰疆视角下物理点的 EPSG 数值

**机器人本质上是"按司南视角去跟踪丰疆给的数值"**，跨系统结构是固定的。

---

## 1. 误差结构

### 1.1 误差链路图

```
丰疆录目标 (E_fj, N_fj)
   │
   ▼
gnss_waypoints.txt          ← 这里直接录入丰疆值（项目规范）
   │
   ▼
b2w_nav_node 跟踪司南 /epsg_position
   │
   │  跟踪误差源（机器人侧）：
   │    - rtk_x_offset 外参偏差
   │    - arm_offset_x 外参偏差
   │    - 机械臂运动学到位精度
   │    - 车头对齐残差 → 横向偏移
   │    - 多帧 RTK 噪声
   │
   ▼
机器人按"司南视角下 (E_fj, N_fj) 的位置"喷涂
   │
   │  系统差（坐标系侧）：
   │    司南视角 (E_fj, N_fj) 物理位置 ≠ 丰疆视角 (E_fj, N_fj) 物理位置
   │    实测为 ~5 cm（5-6 测试 ① 偏差）
   │
   ▼
物理喷涂落点
   │
   ▼
丰疆复测 → 端到端误差 ③ = 落点 - 真值
```

### 1.2 为什么"换录点设备"无法消除 ①

5-6 测试中 §3.3 已经用 lat/lon 直接对比验证：**司南给出的同一物理点 lat/lon ≠ 丰疆给出的同一物理点 lat/lon，差 4-7 cm**。这种差在 lat/lon 源头就存在，与坐标转换链路无关。

机器人内部用司南实时跟踪。**无论目标点是司南录还是丰疆录的数值**，机器人按这个数值停下后的物理位置都是"司南视角下该数值的位置"。

- 用司南录目标 → 物理落点 = 司南视角下数值的位置 = 物理目标位置（误差仅来自机器人侧）
- 用丰疆录目标 → 物理落点 = 司南视角下数值的位置 ≠ 丰疆视角下同一数值的物理位置（差 ① 那 5 cm）

**结论**：项目规范决定了 ① 的偏差**直接进入端到端误差 ③**，必须通过坐标系侧校正（腿 B）来消除。

### 1.3 两条腿走路

| 腿 | 解决的问题 | 不做的话上限 |
|---|---|---:|
| A 机器人侧 | "机器人物理位置 vs 司南给的目标值"的偏差 | ③ 触底约 5-6 cm |
| B 坐标系侧 | "司南视角值 vs 丰疆视角值"的偏差 | ③ 触底约 8-10 cm |
| **A + B** | 两个偏差源都消除 | ③ 可达 **2-3 cm** |

---

## 2. 腿 A：机器人侧改进

### 2.1 已完成 — C：车头阈值收紧

**改动文件**：`b2w_navigation_ws/config/b2w_controller_params.yaml`

```yaml
heading_alignment_threshold: 0.05   # 原 0.25 (≈14°)，调到 0.05 (≈3°)
```

**预期收益**：5-6 测试中车头偏差最大 4.5°，机械臂伸 0.35 m 横向偏移 ~27 mm。新阈值后偏差 < 3°，横向偏移降到 < 18 mm。**减约 9 mm**。

**代价**：每个 waypoint 多花 0.5-1 秒在 yaw 对齐上。

**生效**：重新启动 `tcp_base_ctl.service` 即可（YAML 在 service 启动时加载）。

### 2.2 A1：天线 + 机械臂外参现场标定

**这是当前最大单项收益项**（预期减约 5 cm）。原因是 5-7 test2 中仍有 dE 系统性 +5.4 cm 偏置，最可能的根因就是 `rtk_x_offset` 不准。

#### 步骤 1：标定 RTK 天线相对 base_link 的实际偏移

```
准备：
  - 用丰疆测量 base_link 中心垂直投影点 P（EPSG:2100）
  - 机器人静止放置，采集时姿态不能变化
  - 推荐先用单一朝向完成一次测量；多朝向只用于一致性验证

测量：
  - 司南 /epsg_position 读 N=10 帧均值，得到 RTK 天线的实际世界坐标 A
  - 用当前 yaw 把 A - P 投影到机体系：
    - body_x = (A_E - P_E) * cos(yaw) + (A_N - P_N) * sin(yaw)
    - body_y = -(A_E - P_E) * sin(yaw) + (A_N - P_N) * cos(yaw)
  - 按当前 main.cpp 符号约定：rtk_x_offset_real = -body_x
  - 注意：当前 main.cpp 使用 abs(rtk_x_offset)，YAML 中保持负值是为了沿用既有配置习惯
  
验证：
  - 如果转向后 base_link 不能保持在同一个 P 上，不要强行复用旧 P
  - 每次转向停稳后，用丰疆重新测当前 base_link 投影点 P_i
  - 每轮用自己的 P_i 和同一姿态下的司南 A_i 计算 body_x/body_y
  - 多轮 rtk_x_offset 应该稳定；不稳定说明 P_i 对中、yaw 或 RTK 帧质量有问题
```

现场脚本：

```bash
# 固定 P 模式：适合能保证转向后 base_link 仍在同一物理点上
python3 rtk_nav_ws/fj_dynamic/improve/scripts/2-2-A-step-1.py \
  --base-link-e <P_E> --base-link-n <P_N>

# 每轮 P_i 模式：适合四足机器人转向后 base_link 会平移的实际现场
python3 rtk_nav_ws/fj_dynamic/improve/scripts/2-2-A-step-1.py \
  --per-round-base-link
```

现场执行流程（推荐按此对照操作）：

```bash
source /opt/ros/humble/setup.bash
source /home/test/rtk_nav_ws/install/setup.bash

# 确认司南 RTK 位置话题正常
ros2 topic echo /epsg_position --once

# 进入每轮 P_i 标定模式
python3 rtk_nav_ws/fj_dynamic/improve/scripts/2-2-A-step-1.py \
  --per-round-base-link \
  --frames 20 \
  --current-offset -0.4477
```

操作步骤：

1. 让机器人停稳在第一个朝向。
2. 用丰疆 RTK 测当前 `base_link` 中心垂直投影点，得到 EPSG:2100 的 `E/N`。
3. 脚本提示输入第 1 次 `P_i` 时，填入这组 `E/N`。
4. 按 Enter，脚本采集司南 `/epsg_position` 多帧均值，并输出本轮：
   - RTK 天线均值 `A`
   - 当前 yaw
   - `body_x/body_y`
   - 建议的 `rtk_x_offset`
5. 转向到新朝向并停稳；不要强求回到上一个 P 点。
6. 再用丰疆测当前 `base_link` 投影点 `P_i`，输入脚本，重复采集。
7. 建议至少测 4 个朝向，尽量覆盖 0°、90°、180°、270°。

结果判定：

- `rtk_x_offset std < 10 mm`：结果可信，可以写入 YAML。
- `10-20 mm`：勉强可用，建议复测。
- `> 20 mm`：不要更新配置，通常是 `P_i` 对中不准、机器人没停稳、RTK 跳点或 yaw 不稳定。

结果文件会保存到：

```text
rtk_nav_ws/fj_dynamic/improve/calibration_results/
```

拿到推荐值后，更新：

```yaml
# b2w_navigation_ws/config/b2w_controller_params.yaml
rtk_x_offset: <脚本推荐均值>
```

当前 `main.cpp` 只使用 `rtk_x_offset`，`rtk_y_offset` 是脚本诊断值；如果 `rtk_y_offset` 长期超过 1 cm，说明 RTK 天线存在明显横向安装偏差，后续需要改导航代码支持二维外参。

关键约束：**每一轮只要求 P_i 和 A_i 属于同一静止姿态**，不要求所有朝向共用同一个 P。没有每轮真实 P_i 时，单靠司南 RTK 天线轨迹无法区分“天线外参”和“底盘转向带来的平移”，外参不可辨识。

**更新文件**：`b2w_navigation_ws/config/b2w_controller_params.yaml`

```yaml
rtk_x_offset: <实测值>   # 当前 -0.4477，可能差几 cm
```

#### 步骤 2：✅ 已确认 — arm_offset_x 使用 URDF 设计值，无需重新标定

`tf_broadcast_ws/urdf/my_robot.urdf` 中已明确定义：

```xml
<joint name="base_to_z1" type="fixed">
  <origin xyz="0.3487 0.0 0.05" rpy="0 0 0"/>
</joint>
```

z1_base 是**刚性螺栓安装**在机身上，安装公差 ±1 mm，属于纯机械量，**不存在 RTK 天线那种"相位中心 ≠ 机械中心"的问题**。URDF 值 0.3487 m 即机械安装距离，直接使用。

> 若日后机械臂重新安装或更换机身，用卷尺重新量 z1_base 中心到 base_link 参考面的 x 方向距离即可，精度 ±2 mm 已足够。

```yaml
arm_offset_x: 0.3487   # ✅ 已确认（来自 URDF 设计值，刚性安装，无需 RTK 标定）
```

#### 步骤 3：验证机械臂运动学到位精度

> **核心认知**：不需要额外写脚本或单独搭实验台。
> 利用**正式打点作业 + ARM_TRIGGER 日志 + 司南录落点**，一次作业就能同时拿到验证数据。

##### 打点流程与误差来源（代码分析）

```
WAITING_FOR_FRESH_RTK（5 帧 RTK 均值 → arm_avg_x/y/yaw）
    ↓
EXECUTING_ARM_TASK
  z1_world = arm_avg + arm_offset_x * (cos yaw, sin yaw)   ← z1_base 世界坐标
  dx_local = (target − z1_world) 投影到车头方向            ← 机械臂指令 x
  dy_local = (target − z1_world) 投影到车左方向            ← 机械臂指令 y
  → /z1_move_to_target: MoveJ(dx_local, dy_local, z=0, pitch=90°) + IK
    ↓
TRIGGERING_RELAY（喷漆）→ RESETTING_ARM（backToStart）
```

ARM_TRIGGER 日志（A2 改动后已自动打印）：

```
ARM_TRIGGER base=(base_E,base_N) yaw=yaw° z1_world=(z1_E,z1_N) target=(target_E,target_N)
```

`target=(target_E, target_N)` = **机械臂被指令要落到的世界坐标**（等于 gnss_waypoints.txt 里的目标点）。
实际末端落到哪，就是"机械臂运动学精度"的体现。

##### 验证方法（A1.1 完成后跑一次）

**步骤：**

1. 地面贴 5 个物理标记，用**司南**录目标点（`record_epsg_waypoint.py`）
   - 目标点和机器人定位同在司南坐标系，消除"丰疆-司南跨系"的混淆

2. 正常跑打点作业，ARM_TRIGGER 日志自动记录每次喷漆时的 `target` 坐标

3. 作业完成后：
   - a. 用**丰疆**测 5 个喷漆落点（得到端到端验收指标 ③）
   - b. 用**司南**再录一次 5 个落点：把司南天线杆竖在喷漆圆点旁，取 10 帧均值
     （不需要专门设备，`record_epsg_waypoint.py` 直接用）

4. 对比两组数据：

| 对比组合 | 计算方式 | 含义 |
|---|---|---|
| 司南录落点 − 司南录目标 | 纯司南系内相减 | **纯机械臂运动学误差**（无跨系混入） |
| 丰疆测落点 − 丰疆测目标 | 纯丰疆系内相减 | 端到端总误差 ③（验收指标） |

##### 判定标准

| 司南系内机械臂误差 | 判断 | 处置 |
|---|---|---|
| < 1 cm | ✅ 运动学精度足够，继续做 B2/B3 | 无需处理 |
| 1-3 cm | ⚠ 存在偏差但可接受 | 记录并纳入误差预算 |
| > 3 cm | ❌ 需要排查关节零位 | 联系 Unitree，或从软件层加机械臂末端校正变换 |

#### A1 总预期收益

- 步骤 1（rtk_x_offset 校准）：消除 5-7 test2 中 dE +5.4 cm 系统偏置 → **待标定**
- 步骤 2（arm_offset_x）：✅ 已确认使用 URDF 值 0.3487 m，无需标定
- 步骤 3（机械臂运动学）：随 A1.1 验证实验同步测量，无需单独实验

**总减约 5 cm**（主要来自步骤 1）。

### 2.3 已完成 — A2：多帧 RTK 均值

**改动文件**：`b2w_navigation_ws/src/main.cpp`（4 处）

**原逻辑**：停车后等 1 帧 fresh RTK，用瞬时 `current_x_/y_/yaw_` 计算机械臂目标

**新逻辑**：停车后等 5 帧 RTK，计算平均位置（平面均值 + yaw 圆周均值），用均值驱动机械臂

#### 改动 1：新增成员变量（main.cpp 成员变量区）

```cpp
// A2: 多帧 RTK 均值
struct RtkSample { double x, y, sin_yaw, cos_yaw; };
static constexpr int ARM_RTK_FRAMES = 5;
std::vector<RtkSample> arm_rtk_buf_;
double arm_avg_x_ = 0.0, arm_avg_y_ = 0.0, arm_avg_yaw_ = 0.0;
```

#### 改动 2：WAYPOINT_REACHED 时清空 buffer

```cpp
arm_wait_rtk_seq_ = rtk_update_seq_;
arm_rtk_buf_.clear();              // ← 新增
state_ = WAITING_FOR_FRESH_RTK;
```

#### 改动 3：WAITING_FOR_FRESH_RTK 改为多帧收集

```cpp
// arm_wait_rtk_seq_ 是停车时基准序号，buf.size() 是已收集帧数
if (rtk_update_seq_ <= arm_wait_rtk_seq_ + (uint64_t)arm_rtk_buf_.size()) {
    // 新帧还没来，等
    return;
}
// 新帧到了，入 buffer
arm_rtk_buf_.push_back({current_x_, current_y_,
                         std::sin(current_yaw_), std::cos(current_yaw_)});
if ((int)arm_rtk_buf_.size() < ARM_RTK_FRAMES) {
    return;  // 帧数不足，继续等
}
// 收满 5 帧，计算均值
double sum_x = 0, sum_y = 0, sum_sin = 0, sum_cos = 0;
for (const auto & s : arm_rtk_buf_) {
    sum_x += s.x; sum_y += s.y;
    sum_sin += s.sin_yaw; sum_cos += s.cos_yaw;
}
arm_avg_x_   = sum_x / ARM_RTK_FRAMES;
arm_avg_y_   = sum_y / ARM_RTK_FRAMES;
arm_avg_yaw_ = std::atan2(sum_sin, sum_cos);  // yaw 圆周均值
arm_rtk_buf_.clear();
RCLCPP_INFO(..., "RTK 5-frame avg: (%.4f, %.4f) yaw=%.2f°", ...);
state_ = EXECUTING_ARM_TASK;
```

#### 改动 4：EXECUTING_ARM_TASK 改用均值 + 加 ARM_TRIGGER 日志

```cpp
// 原来用 current_x_/y_/yaw_，现在用均值
double robot_yaw = arm_avg_yaw_;
double z1_world_x = arm_avg_x_ + arm_offset_x * std::cos(robot_yaw);
double z1_world_y = arm_avg_y_ + arm_offset_x * std::sin(robot_yaw);
RCLCPP_INFO(...,
    "ARM_TRIGGER base=(%.4f,%.4f) yaw=%.2f° z1_world=(%.4f,%.4f) target=(%.4f,%.4f)",
    arm_avg_x_, arm_avg_y_, robot_yaw * 180/M_PI,
    z1_world_x, z1_world_y, target_x_, target_y_);
```

**实际收益**：1-3 mm（消除瞬时 RTK 跳点对机械臂落点的影响）

**额外收益**：`ARM_TRIGGER` 日志让每次喷涂时的 base 位置、yaw、z1_world 都记录到 `b2w_navigation.log`，方便后续外参标定（A1）时做系统性分析。

**代价**：每个 waypoint 多 0.5 秒（5 帧 @ 10 Hz RTK）

### 2.4 A4：对录点方的流程要求

由于目标点必须由丰疆录，**录点方流程**直接决定 ① 偏差里"物理对中"那部分的下限。要求录点方：

1. **每点用三脚架精密对中**，不要手持杆
2. **每点录至少 30 秒**，取**中位数**（不是简单均值，抗瞬时跳点）
3. **同一物理点在不同时刻（间隔 ≥ 10 分钟）录 2-3 次取均值**，消除卫星几何漂移
4. **避免在 GDOP 较差的时段录点**（早晨日出/傍晚日落前后卫星几何最差）

如果录点方拒绝改流程，残留几 cm 物理对中误差就是没法绕过的下限——这时只能靠**腿 B 的统计校正**间接弥补。

---

## 3. 腿 B：司南-丰疆系统偏差校正

### 3.1 偏差性质判定（已由 5-7 test4 8 点数据更新）

**5-7 test4 实验（8 点公共点，覆盖 ~60×45 m 工作区）**：

| 点 | dE = 司南−丰疆 (m) | dN = 司南−丰疆 (m) | 距离 (m) |
|---|---:|---:|---:|
| pt1 | −0.036 | +0.049 | 0.061 |
| pt2 | −0.037 | −0.012 | 0.039 |
| pt3 | −0.044 | +0.024 | 0.050 |
| pt4 | −0.017 | −0.005 | 0.018 |
| pt5 | −0.033 | −0.002 | 0.033 |
| pt6 | **+0.041** | −0.004 | 0.041 |
| pt7 | **+0.027** | +0.015 | 0.031 |
| pt8 | **+0.038** | −0.018 | 0.042 |
| **均值** | **−0.008** | **+0.006** | **0.039** |
| std | 34 mm | 21 mm | — |

**关键发现（颠覆 5-6 推断）**：

1. **均值接近 0（−0.8 mm, +5.9 mm）** —— 不存在系统性常数偏置
2. **dE 在 pt5 → pt6 之间方向反转**（前 5 点全负、后 3 点全正）—— 空间非线性分布
3. **平移变换 RMSE 几乎不变**（4.0 → 3.8 cm，仅减 2 mm） —— 平移变换收益可忽略
4. **绝对量级 ~4 cm**（vs 5-6 估计的 5 cm，更精确）

详见 `logs/5-7/ana.md` §3。

### 3.2 B2：建立公共点网络（⏸ 暂停 — 已被 5-7 test4 验证无显著收益）

> 5-7 test4 已用 8 个公共点验证（覆盖 ~60×45 m 工作区），结果见 §3.1：
> - 均值偏差 ≈ 0 → 平移变换无效
> - 局部 4 cm 非线性 → 仿射变换至多减到 3 cm
>
> **结论**：相比 A1.1 标定 `rtk_x_offset` 的 5+ cm 单步收益，B2 全局变换至多 1-2 cm 增益。
> 优先做完 A1.1，若 ③ 仍 > 4 cm 再考虑做 B2 的非线性变换（TPS / IDW），可作为后续优化项。

下面保留原 B2 实施步骤作为日后参考：



#### 步骤 1：选定 8-12 个永久物理标记

- 在工作区域内**分布开**（不要集中一处）
- 至少覆盖工作区四角 + 中心
- 用油漆/打孔做永久标记（这些点不参与作业，纯粹用来标定）
- 命名：`cal_pt_01`, `cal_pt_02`, ...

#### 步骤 2：丰疆 + 司南"准同时"录这些点

```
方法：
  1. 架设三脚架在 cal_pt_01 上，精密对中
  2. 装丰疆 RTK 杆，录 30 帧（约 1 分钟）
  3. 立即换司南 RTK 杆（< 1 分钟内，不挪三脚架），录 30 帧
  4. 移动到 cal_pt_02，重复

每个点：
  - 取 30 帧的中位数作为该次测量值
  - 记录采集时间戳

每个点重复 3 次，间隔 30 分钟以上：
  - 第一轮（早上 09:00）
  - 第二轮（中午 13:00）
  - 第三轮（下午 17:00）
  - 三轮取均值，消除卫星几何漂移
```

输出文件：`calibration_pairs.csv`

```csv
name,fj_E,fj_N,fj_Z,sn_E,sn_N,sn_Z,timestamp_round
cal_pt_01,481600.123,4210293.456,183.450,481600.165,4210293.502,185.180,2026-05-08T09:30:00
cal_pt_01,481600.121,4210293.457,183.449,481600.167,4210293.504,185.179,2026-05-08T13:30:00
...
```

#### 步骤 3：拟合 fj → sn 的仿射变换 T

候选模型，按复杂度递增：

**模型 1：纯平移（2 参数）**

```
E_sn = E_fj + dE_const
N_sn = N_fj + dN_const

dE_const = mean(E_sn_i - E_fj_i)
dN_const = mean(N_sn_i - N_fj_i)
```

**模型 2：相似变换 / 4 参数仿射（含旋转 + 缩放）**

```
[E_sn]   [a -b] [E_fj]   [tx]
[N_sn] = [b  a] [N_fj] + [ty]

最小二乘解 (a, b, tx, ty)
```

**模型 3：6 参数仿射（含剪切）**

```
[E_sn]   [a11 a12] [E_fj]   [tx]
[N_sn] = [a21 a22] [N_fj] + [ty]
```

#### 步骤 4：评估 T 是否值得用

对每个模型计算：

- **拟合前 RMSE**：直接把 (E_fj, N_fj) 当 (E_sn, N_sn) 的残差
- **拟合后 RMSE**：应用 T 后的残差
- **AIC / BIC**：避免过拟合（参数越多模型越容易拟合采样点但不一定泛化）

判定：

| 拟合前 RMSE | 拟合后 RMSE | 结论 |
|---|---|---|
| 5 cm | < 1 cm | T 显著有效，写进流程 |
| 5 cm | 2-3 cm | T 部分有效，可用但不万能 |
| 5 cm | ≈ 5 cm | T 没用（偏差是纯随机），只能改善 A4 录点流程 |

输出文件：`calibration.json`

```json
{
  "model": "affine_4param",
  "params": {"a": 1.0000123, "b": -0.0000456, "tx": 0.0234, "ty": 0.0152},
  "rmse_before_m": 0.0521,
  "rmse_after_m": 0.0089,
  "n_points": 12,
  "n_rounds": 3,
  "fit_date": "2026-05-08",
  "valid_region": {"E_min": 481590, "E_max": 481600, "N_min": 4210290, "N_max": 4210295},
  "notes": "首次标定，覆盖工作区主体"
}
```

### 3.3 B3：录入流程加 T 校正（⏸ 暂停）

> 当前状态：**不要在现场作业流程中启用 B3**。
>
> 原因：5-7 test4 证明全局平移/仿射变换收益很小，且仓库当前没有 `tools/import_fj_waypoints.py`、`calibration.json` 的完整生产工具链。当前阶段优先完成 A1.1 `rtk_x_offset` 标定，再用实测 ③ 判断是否需要恢复 B2/B3。

新增工具脚本：`tools/import_fj_waypoints.py`

```
输入：
  - fj_export.csv（丰疆控制器导出，name,E,N,Z 格式）
  - calibration.json（B2 拟合的 T 参数）
  - --z-mode {fixed | nearest_sn | region_mean}（z 字段处理策略）

处理：
  1. 读 calibration.json 的 T
  2. 对每个 fj 点 (E_fj, N_fj) 应用 T → (E_sn, N_sn)
  3. z 按 --z-mode 处理：
     - fixed：用命令行指定的固定值
     - nearest_sn：用最近的司南实测点的 z
     - region_mean：用 calibration_pairs.csv 中所有 sn_Z 的均值
  4. 检查 (E_sn, N_sn) 是否在 calibration.json 的 valid_region 内
     - 不在则警告（外推不可靠）

输出：
  - gnss_waypoints.txt（项目格式：name,E,N,Z,）
  - import_log.json（记录使用的 T 版本、输入输出对照）
```

### 3.4 标定的复用范围

| 维度 | 有效范围 | 注意 |
|---|---|---|
| 空间 | 同一工作区域 | 仿射变换是局部线性近似，跨 km 级会失效 |
| 时间 | 数月 | RTK 基站不变 + 设备不变 + 大气条件类似 |
| 设备 | 同一对司南/丰疆 | 任一设备更换或固件更新都需要重标定 |
| 跨工地 | 重新标定 | 一个工地一份 calibration.json |

---

## 4. 闭环 SOP（每次作业要做的事）

固化成标准流程：

### 4.1 作业前

当前阶段（B2/B3 暂停）：

1. 确认已经完成 A1.1，`b2w_navigation_ws/config/b2w_controller_params.yaml` 中的 `rtk_x_offset` 是脚本推荐值。
2. 录点方用丰疆录目标点，按当前项目格式生成或手动整理 `gnss_waypoints.txt`。
3. 如果丰疆输出的是 WGS84 经纬度，先走狗端同一套转换链路生成 EPSG:2100 坐标；如果丰疆已经输出 EPSG:2100 平面坐标，直接使用该 E/N。
4. SSH 到机器狗确认 waypoint 文件已更新：
   ```bash
   wc -l /home/test/gnss_waypoints.txt
   head /home/test/gnss_waypoints.txt
   ```
5. 重启 service：`sudo systemctl restart tcp_base_ctl.service`

仅当后续重新启用 B2/B3 时，才恢复 `calibration.json` 检查和 `tools/import_fj_waypoints.py` 导入流程。

### 4.2 作业中

按现有流程跑，机器人自动作业。监控日志：

```bash
tail -F /home/test/logs/start_all_latest/b2w_navigation.log
```

### 4.3 作业后

1. 录点方用丰疆复测每个喷涂落点 → 输出 `fj_spray_<date>.csv`
2. 用目标点 E/N 与落点 E/N 计算本次作业的端到端 ③ 误差，记录在作业日志里。
3. 手工保存目标-落点对 CSV，字段至少包含 `name,target_E,target_N,spray_E,spray_N,error_m`。后续若恢复 B2/B3，可再导入这些历史数据。

### 4.4 长期

- 每月或每 100 个作业点，汇总一次目标-落点误差，检查是否重新出现稳定方向的系统偏置。
- 若 A1.1 后 ③ 仍长期 > 4 cm，再恢复 §3.2/§3.3 的 B2/B3 工具链。
- 跨工地工作时，至少重新跑 A1.1；如果新工地 ③ 超标，再重新建立公共点网络。

---

## 5. 推进路线 / 时间表

| 阶段 | 工作内容 | 工作量 | 实测 / 预期 ③ |
|---|---|---|---:|
| ✅ 已完成 | C：heading 阈值 0.25 → 0.05 rad（14° → 3°） | — | — |
| ✅ 已完成 | A2：多帧 RTK 均值（5 帧，改 main.cpp 4 处） | — | — |
| ✅ 已确认 | A1.2：arm_offset_x = 0.3487（URDF，刚性安装） | — | — |
| ✅ 已实测 | **5-7 test2**：丰疆录目标 + 丰疆复测（5 点同 5-6 位置） | — | **8.0 cm 均值** |
| ✅ 已实测 | **5-7 test4**：8 点公共点标定（60×45 m） | — | 偏差均值 ≈ 0、std=34 mm |
| ✅ 已实测 | **5-7 test5**：丰疆 2 新点重测（pt2 单点） | — | **2.9 cm（最佳）** |
| ⏸ 已暂停 | ~~B2 / B3：公共点 + 仿射 T~~（test4 证明全局变换无显著收益） | — | — |
| **Step 1** | A1.1：标定 rtk_x_offset（脚本就绪） | 2-3 小时 | 预期 3-4 cm |
| **Step 2** | 标定后跑 5+ 点验证（看 dE 偏置是否归零） | 1 小时 | — |
| **Step 3** | A1.3：机械臂运动学（若 ③ 仍 > 3 cm） | 随作业 | ~2-3 cm |
| 可选 | 非线性变换（TPS/IDW，仅在需要降到亚 cm 级时） | 半天 | ~1-2 cm |
| 长期 | 闭环数据积累，监控偏差稳定性 | 持续 | 维持 |

**剩余 5 cm 误差的拆解（来自 5-7 test2）**：

```
③ test2 = 8.0 cm
       ↓ 拆解
  机器人 dE 系统偏置（rtk_x_offset 未标定） ≈ 5.4 cm
  + 机器人 dN 偏置 + 丰疆复测对中误差     ≈ 2.6 cm
       ↓ A1.1 标定后预期
③ A1.1 后 ≈ 3-4 cm（仅剩对中 + 机械臂运动学）
```

**关键决策点**：Step 1（A1.1 rtk_x_offset 标定）后做 Step 2 复测：

- 若 ③ ≤ 3 cm → ✅ 项目目标达成
- 若 ③ 在 3-5 cm → 做 Step 3（机械臂运动学验证）
- 若 ③ 仍 > 5 cm → 排查 rtk_x_offset 标定流程是否正确

---

## 6. 待开发工具清单

当前可执行工具只有 A1.1 标定脚本：`rtk_nav_ws/fj_dynamic/improve/scripts/2-2-A-step-1.py`。B2/B3 相关工具当前未实现，且已暂停，不是完成下一轮现场测试的前置条件。

| # | 文件名 | 用途 | 优先级 |
|---|---|---|---|
| 1 | `rtk_nav_ws/fj_dynamic/improve/scripts/2-2-A-step-1.py` | A1.1：读司南 N 帧 + 每轮丰疆 P_i → 输出 `rtk_x_offset` 建议值 | 已有 |
| 2 | `tools/fit_fj_sn_transform.py` | B2：从 `calibration_pairs.csv` 拟合 T，输出 `calibration.json` + RMSE 报告 | 暂停 |
| 3 | `tools/import_fj_waypoints.py` | B3：丰疆 CSV → 应用 T → 输出 `gnss_waypoints.txt` | 暂停 |
| 4 | `tools/append_calibration_log.py` | 把作业的 (target_fj, spray_fj) 对追加到 `calibration_log.csv` | 可选 |
| 5 | `tools/run_arm_kinematics_test.py` | 自动化 A1.3 步骤：让机械臂依次伸到 N 个目标，记录 RTK | 低 |

如果恢复开发 B2/B3 工具链，建议都放在项目根的 `tools/` 目录，跟 `record_gps_epsg_point.py` 等已有脚本同级。

---

## 7. 验收标准与里程碑

| 里程碑 | 验收标准 | 当前状态 |
|---|---|---|
| M0 — 基线确立 | 完成 5 点测试，③ 端到端误差有量化值 | ✅ 5-6 测试，③ = 11.1 cm 均值 |
| M0.5 — 软件侧调优 | C（heading 阈值）+ A2（多帧均值）已合并入代码 | ✅ 已完成 |
| **M1a — 项目流程基线复测** | 丰疆录目标 + 丰疆复测落点，③ 较 5-6 基线减少 ≥ 20% | **✅ 已达成**：5-7 test2 ③ = **8.0 cm**（−28% 改善） |
| M1b — 公共点偏差刻画 | ≥ 8 个点定量化司南-丰疆系统偏差的空间分布 | ✅ 5-7 test4：均值 ≈ 0、std=34 mm、空间非线性 |
| M2 — 外参标定 | 跑 A1.1 标定 rtk_x_offset，③ ≤ 4 cm（均值） | ⏳ Step 1：标定脚本已就绪 |
| **M3 — 项目目标** | **③ ≤ 3 cm（均值），最大 ≤ 5 cm** | ⏳ Step 1+2 后验证，单点已实测 2.9 cm（test5 pt2） |
| M4 — 流程标准化 | SOP 全部固化，工具齐备，新人按文档可独立操作 | ⏳ 待 |

**精度等级对应应用**：

- 5 cm：粗放喷涂、大色块
- 3 cm：标准喷涂作业（项目目标）
- 1-2 cm：精细作业（如细线/小图案，需要 A1+B+A2 全部完成）

---

## 附录：参考资料

- **本项目内**：
  - `logs/5-6/test_logs/5-6.md`：5-6 实验完整报告（误差 ①②③④⑤ 拆解、链路一致性验证）
  - `logs/5-7/ana.md`：**5-7 实验报告**（test2 同点重测、test4 8 点公共点、test5 新点验证）
  - `CLAUDE.md` "坐标数据变化链路"章节：完整 INS → PROJ → HEPOS → 发布流程
  - `rtk_nav_ws/issue.md`：HEPOS 5 点验证报告
  - `b2w_navigation_ws/config/b2w_controller_params.yaml`：当前所有可调参数
  - `b2w_navigation_ws/src/main.cpp`：导航主控源码（state machine 在 line 536, 600-660）

- **相关脚本**：
  - `record_gps_epsg_point.py`：录点 + 链路诊断（可作为公共点采集工具的基础）
  - `record_epsg_waypoint.py`：实机录点（写 gnss_waypoints.txt 的工具）
  - `convert_gps_convert_to_csv.py` / `convert_var_point_no_datum.py`：离线坐标转换工具

- **核心结论速查**：
  - ① 的 cm 级偏差完全在 RTK 录点层面，与坐标转换链路无关（5-6.md §3.4）
  - 丰疆控制器内部链路与项目链路毫米级一致（5-6.md §3.2）
  - 司南内部单帧精度 < 2 mm（5-6.md §3.1）
  - 机器人停车精度 std = 9 mm（5-6.md §4.3）
  - **司南-丰疆系统偏差均值 ≈ 0、std=34 mm、空间非线性**（5-7 ana.md §3，8 点公共点）
  - **同系（丰疆录目标 + 丰疆复测）端到端 ③ = 8.0 cm**，dE 系统偏置 +5.4 cm（5-7 ana.md §2）
  - **同系单点最佳 ③ = 2.9 cm**（5-7 test5 pt2，已接近项目目标）
  - `rtk_x_offset = -0.4477`（URDF 设计值，**需用脚本实测**，RTK 相位中心 ≠ 机械安装位置）
  - `arm_offset_x = 0.3487`（URDF 设计值，✅ **已确认**，z1_base 刚性安装，机械量直接可用）
