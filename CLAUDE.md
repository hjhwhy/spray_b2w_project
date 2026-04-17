# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目简介

自主喷涂机器人系统，使用 Unitree B2W 四足机器狗 + Z1 机械臂，基于 RTK-GNSS 定位进行精准喷涂作业。部署于希腊，使用 EPSG 2100 坐标投影。

平台：ARM64（Jetson/aarch64），ROS 2 Humble，Ubuntu。

## 构建命令

### 编译各工作空间（依赖顺序）

跨工作空间依赖关系：
- `gnss_driver_ws` 依赖 `colcon_ws`（使用 nmea_msgs 消息类型）
- `b2w_navigation_ws` 依赖 `z1_move_ws`（使用 MoveArm.srv / MoveArmWithRPY.srv）
- `b2w_navigation_ws` 依赖 `spray_path_planner_ws`（使用 GetNextWaypoint.srv / SetStartPoint.srv）

每个 `source install/setup.bash` 不可省略，后续工作空间需要通过它发现前面安装的包。

```bash
# ── Tier 0：基础消息/服务包（必须最先编译，三者之间无依赖可并行） ──
cd colcon_ws && colcon build && source install/setup.bash && cd ..
cd z1_move_ws && colcon build && source install/setup.bash && cd ..
cd spray_path_planner_ws && colcon build && source install/setup.bash && cd ..

# ── Tier 1：依赖 colcon_ws（nmea_msgs） ──
cd gnss_driver_ws && colcon build && source install/setup.bash && cd ..

# ── Tier 2：需要先编译unitree_sdk2，依赖 z1_move_ws + spray_path_planner_ws ──
cd b2w_navigation_ws && colcon build && source install/setup.bash && cd ..
# FIX：[b2w_nav_node-1] free(): invalid pointer
sudo setcap  cap_net_raw+ep  b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_nav_node 


# ── 无跨工作空间依赖，可任意顺序编译 ──
cd rtk_nav_ws && colcon build && source install/setup.bash && cd ..
cd tf_broadcast_ws && colcon build && source install/setup.bash && cd ..
cd rs585_ws && colcon build && source install/setup.bash && cd ..
cd robose_airy_ws && colcon build --cmake-args '-DENABLE_TRANSFORM=ON'  && source install/setup.bash && cd ..
cd app_ws && colcon build && source install/setup.bash && cd ..
```

编译单个包：`colcon build --packages-select <package_name>`

### 编译外部 SDK（非 ROS）

```bash
# Z1 SDK（机械臂底层库，ARM64 预编译，无需重新编译）
# 库位于 z1_sdk/lib/libZ1_SDK_aarch64

# Z1 控制器（独立二进制）
cd z1_controller && mkdir -p build && cd build && cmake .. && make

# Unitree SDK2（B2W 四足）
cd unitree_sdk2 && mkdir -p build && cd build && cmake .. && make 
make install 
```

### 运行所有节点

```bash
./start_all.sh  # 管理所有节点的生命周期，日志写入 logs/ 目录
```

> **注意**：`start_all.sh` 启动以下节点：rs485_node、robot_tf_broadcaster、ins_driver_node、spray_path_planner、z1_controller（二进制）、z1_arm_controller_node、rslidar_sdk，以及 `ros2 bag record /b2w_odom`。
> `b2w_navigation_controller`（主导航节点）和 `app_ws`（TCP服务）**不在** `start_all.sh` 中，需单独启动或由 APP 发送 0x01 指令触发。

## 工作空间架构

| 工作空间 | 包名 | 功能 |
|---------|------|------|
| `colcon_ws` | nmea_msgs, serial | 基础依赖（需最先编译） |
| `rtk_nav_ws` | ins_driver_node | 司南 RTK 串口解析，发布位置和方向 |
| `gnss_driver_ws` | gnss_driver | NovAtel/司南 RTK 驱动，坐标投影 |
| `tf_broadcast_ws` | robot_tf_broadcaster | 发布静态 TF 树（B2W→Z1→Lidar→GNSS） |
| `z1_move_ws` | z1_arm_controller_cpp | Z1 机械臂 ROS2 服务接口 |
| `b2w_navigation_ws` | b2w_navigation_controller | B2W 导航、里程计、EKF 融合 |
| `spray_path_planner_ws` | spray_path_planner | 读点文件、生成最短路径 |
| `rs585_ws` | rs485_node | Modbus RS485 继电器控制（喷枪） |
| `robose_airy_ws` | rslidar_sdk | RS16 激光雷达驱动 |
| `app_ws` | robot_tcp | TCP 长连接服务（与 APP/遥控器通信） |

## 关键 ROS 2 话题与服务

**话题（ins_driver_node 发布）**：
- `/fix` (PointStamped) — 原始 GPS 经纬高调试话题
- `/utm_fix` (PoseStamped) — UTM 投影坐标 + 航向四元数
- `/epsg_position` (PoseStamped) — EPSG 2100 投影坐标 + 航向四元数（yaw=0→朝东，yaw=π/2→朝北）

**话题（b2w_navigation_controller 发布）**：
- `/b2w_odom` (Odometry) — B2W 融合里程计（RTK + DDS）
- `/b2w_path` (Path) — 运行轨迹历史
- `/imu` (Imu) — B2W IMU 数据（来自 DDS LowState）
- `/battery` (BatteryState) — B2W 电池状态
- `/motors_temperatures` (Float32MultiArray) — 电机温度（12 个电机）

**话题（spray_path_planner 发布）**：
- `/progress` (Byte) — 作业进度 0-255
- `/acquired_points` (PointCloud2) — 已完成喷涂点云
- `/unacquired_points` (PointCloud2) — 待喷涂点云

**话题（app_ws 发布）**：
- `/remote_command` (String) — 遥控命令字符串（"start"/"pause"/"stop"）
- `/joy` (Joy) — 方向控制摇杆消息（axes[0]=左右平移, axes[1]=前后, axes[2]=旋转）

**自定义服务**：
- `/get_next_waypoint` (GetNextWaypoint) — 请求下一个喷涂点（spray_path_planner 提供）
- `/set_start_point` (SetStartPoint) — 设置起始点（spray_path_planner 提供）
- `/z1_move_to_target` (MoveArm) — 移动机械臂到笛卡尔坐标（z1_arm_controller 提供）
- `/z1_reset_arm` (MoveArm) — 复位机械臂（z1_arm_controller 提供）
- `/trigger_valve_ch1` (Trigger) — 触发 RS485 继电器通道1（喷枪，rs485_node 提供）
- `/trigger_valve_ch2` (Trigger) — 触发 RS485 继电器通道2（rs485_node 提供）
- `/emergency_stop` (Trigger) — 紧急停止（b2w_navigation_controller 提供）
- `/erase_emergency_stop` (Trigger) — 解除紧急停止（b2w_navigation_controller 提供）

**服务消息类型定义**：
- `spray_path_planner/GetNextWaypoint.srv`
- `spray_path_planner/SetStartPoint.srv`
- `z1_arm_controller_cpp/MoveArm.srv`
- `z1_arm_controller_cpp/MoveArmWithRPY.srv`

## 硬件设备与配置

- `/dev/ttyTHS1` — RS485 继电器（需通过 `tac3kp_uart_mode_config.sh 485` 设为 485 模式，权限 777）
- `/dev/ttyTHS2` — 司南 RTK 串口（波特率 115200，权限 777）
- B2W 通信通过 DDS（Unitree SDK2），需配置网络接口（eth2 等）
- Z1 SDK 库为 ARM64 预编译：`z1_sdk/lib/libZ1_SDK_aarch64`
- Z1 控制器二进制部署路径：`/home/test/z1_controller/build/z1_ctrl`（机器人本机路径）

## 部署说明

- 机器人本机用户路径：`/home/test/`（与开发机 `/home/oneko/` 不同）
- `start_all.sh` 中硬编码了 `/home/test/z1_controller/build/` 路径，部署时需确认
- 喷涂点文件（如 `gnss_waypoints.txt`、`points_test_mikinwn.txt`）为经纬度文本，由 `spray_path_planner` 读取并转为 EPSG 2100 坐标

## 关键源文件

- `b2w_navigation_ws/src/main.cpp` — B2W 导航主控逻辑（约 807 行）
- `b2w_navigation_ws/src/main_ekf.cpp` — EKF 融合版本（约 1041 行）
- `b2w_navigation_ws/src/b2w_teleop.cpp` — 遥控/手动控制逻辑
- `app_ws/src/app_node.cpp` — TCP 通信协议实现（约 533 行）
- `rtk_nav_ws/src/ins_parser.cpp` — 司南 RTK 串口解析与坐标投影（发布 /fix、/utm_fix、/epsg_position）
- `rtk_nav_ws/src/dog_controller.cpp` — 遥控手柄控制辅助
- `z1_move_ws/src/z1_arm_controller_node.cpp` — Z1 控制节点
- `spray_path_planner_ws/src/spray_path_planner_node.cpp` — 路径规划
- `gnss_driver_ws/src/pub_rtk_save_pt_node.cpp` — GNSS 驱动与坐标转换（NovAtel/司南）

## 配置文件

- `b2w_navigation_ws/config/b2w_controller_params.yaml` — 速度、阈值等运动参数
- `b2w_navigation_ws/config/b2w_controller_params_ekf.yaml` — EKF 参数
- `spray_path_planner_ws/config/path_planner.yaml` — 喷涂点文件路径
- `rs585_ws/config/rs485_params.yaml` — RS485 通信参数
- `gnss_driver_ws/config/gnss_params.yaml` — GNSS 驱动参数（端口、波特率、坐标系）
- `01-wifi-ap.yaml` — 机器人 WiFi AP 配置（Netplan，部署时热点设置）

## TCP 应用协议

APP ↔ 主控通过 TCP 长连接通信：
- 包格式：`[0xF5][功能码][长度][数据][CRC低][CRC高][0x5F]`

**机器人 → APP（主动推送）**：
| 功能码 | 内容 |
|--------|------|
| 0x01 | 全部喷涂点（新连接后立即发送） |
| 0x02 | 已完成喷涂点云（PointCloud2） |
| 0x03 | 待喷涂点云（PointCloud2） |
| 0x04 | 机器人轨迹（/b2w_path） |
| 0x05 | 作业进度（/progress） |
| 0x06 | 日志/报错信息 |
| 0x07 | 当前位置（/b2w_odom） |

**APP → 机器人（控制指令，功能码 0x08，数据第1字节为子命令）**：
| 子命令 | 含义 |
|--------|------|
| 0x01 | 开始喷涂（发布 "start" 到 /remote_command） |
| 0x02 | 暂停 |
| 0x03 | 停止/结束 |
| 0x04 | 前进 |
| 0x05 | 后退 |
| 0x06 | 左移 |
| 0x07 | 右移 |
| 0x08 | 左转 |
| 0x09 | 右转 |
| 0x10 | 恢复（解除暂停） |

## 坐标系说明

- 使用 EPSG 2100（希腊 GGRS87 / Greek Grid）投影
- 使用 `proj` 库进行 WGS84 ↔ EPSG2100 转换
- TF 树：`base_link` → `z1_base` → `rslidar` → `gnss_antenna`
