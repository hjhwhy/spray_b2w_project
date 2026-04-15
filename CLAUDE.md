# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目简介

自主喷涂机器人系统，使用 Unitree B2W 四足机器狗 + Z1 机械臂，基于 RTK-GNSS 定位进行精准喷涂作业。部署于希腊，使用 EPSG 2100 坐标投影。

平台：ARM64（Jetson/aarch64），ROS 2 Humble，Ubuntu。

## 构建命令

### 编译各工作空间（依赖顺序）

```bash
# 1. 先编译基础依赖
cd colcon_ws && colcon build && source install/setup.bash && cd ..

# 2. 编译其他 ROS 工作空间（独立）
cd <workspace> && colcon build && source install/setup.bash

# 编译单个包
colcon build --packages-select <package_name>
```

### 编译外部 SDK（非 ROS）

```bash
# Z1 SDK（机械臂底层库，ARM64 预编译，无需重新编译）
# 库位于 z1_sdk/lib/libZ1_SDK_aarch64

# Z1 控制器（独立二进制）
cd z1_controller && mkdir -p build && cd build && cmake .. && make

# Unitree SDK2（B2W 四足）
cd unitree_sdk2 && mkdir -p build && cd build && cmake .. && make
```

### 运行所有节点

```bash
./start_all.sh  # 管理所有节点的生命周期，日志写入 logs/ 目录
```

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

**话题**：
- `/b2w_odom` (Odometry) — B2W 位置里程计
- `/b2w_path` (Path) — 运行轨迹
- `/progress` (Byte) — 作业进度 0-255
- `/motors_temperatures` (Float32MultiArray) — 电机温度
- `/epsg_position` — 希腊 EPSG 2100 投影坐标

**自定义服务**：
- `spray_path_planner/GetNextWaypoint.srv` — 请求下一个喷涂点
- `spray_path_planner/SetStartPoint.srv` — 设置起始点
- `z1_arm_controller_cpp/MoveArm.srv` — 移动机械臂到笛卡尔坐标
- `z1_arm_controller_cpp/MoveArmWithRPY.srv` — 移动到笛卡尔坐标 + RPY 姿态

## 硬件设备与配置

- `/dev/ttyTHS1` — RS485 继电器（需设为 485 模式，权限 777）
- `/dev/ttyTHS2` — 司南 RTK 串口（波特率 115200，权限 777）
- B2W 通信通过 DDS（Unitree SDK2），需配置网络接口（eth2 等）
- Z1 SDK 库为 ARM64 预编译：`z1_sdk/lib/libZ1_SDK_aarch64`

## 关键源文件

- `b2w_navigation_ws/src/main.cpp` — B2W 导航主控逻辑（37KB）
- `b2w_navigation_ws/src/main_ekf.cpp` — EKF 融合版本（48KB）
- `app_ws/src/app_node.cpp` — TCP 通信协议实现（23KB）
- `z1_move_ws/src/z1_arm_controller_node.cpp` — Z1 控制节点
- `spray_path_planner_ws/src/spray_path_planner_node.cpp` — 路径规划
- `gnss_driver_ws/src/pub_rtk_save_pt_node.cpp` — GNSS 驱动与坐标转换

## 配置文件

- `b2w_navigation_ws/config/b2w_controller_params.yaml` — 速度、阈值等运动参数
- `b2w_navigation_ws/config/b2w_controller_params_ekf.yaml` — EKF 参数
- `spray_path_planner_ws/config/path_planner.yaml` — 喷涂点文件路径
- `rs585_ws/config/rs485_params.yaml` — RS485 通信参数

## TCP 应用协议

APP ↔ 主控通过 TCP 长连接通信：
- 包格式：`[0xF5][功能码][长度][数据][CRC低][CRC高][0x5F]`
- 主要功能码：0x01 开始喷涂、0x02 暂停/上传数据、0x03 结束、0x04-0x09 方向控制、0x10 恢复

## 坐标系说明

- 使用 EPSG 2100（希腊 GGRS87 / Greek Grid）投影
- 使用 `proj` 库进行 WGS84 ↔ EPSG2100 转换
- TF 树：`base_link` → `z1_base` → `rslidar` → `gnss_antenna`
