# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目简介

自主喷涂机器人系统，使用 Unitree B2W 四足机器狗 + Z1 机械臂，基于 RTK-GNSS 定位进行精准喷涂作业。部署于希腊，使用 EPSG 2100 坐标投影。

平台：ARM64（Jetson/aarch64），ROS 2 Humble，Ubuntu。

## 构建命令

### 命令行更改工控机时间

机器离线且 RTC 不稳定时，优先使用仓库脚本设置日期。默认年份为 2026：

```bash
./resettime.sh -4-29
./resettime.sh -4-29 08:30:00
```

等价手动命令：

```bash
sudo timedatectl set-ntp false
sudo timedatectl set-time "2026-04-29 08:30:00"
```

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
# Unitree DDS 需要 raw socket 权限；手动部署/重新编译后两个二进制都要设置。
sudo setcap  cap_net_raw+ep  b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_nav_node 
sudo setcap  cap_net_raw+ep  b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_teleop_node


# ── 无跨工作空间依赖，可任意顺序编译 ──
cd rtk_nav_ws && colcon build && source install/setup.bash && cd ..
cd tf_broadcast_ws && colcon build && source install/setup.bash && cd ..
cd rs585_ws && colcon build && source install/setup.bash && cd ..
cd robose_airy_ws && colcon build --cmake-args '-DENABLE_TRANSFORM=ON' --packages-select rslidar_sdk  && source install/setup.bash && cd ..
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

### 运行控制链路与主任务

```bash
./tcp_base_ctl.sh  # 常驻控制链路：APP TCP、底盘遥控、RTK/TF/RS485/Z1 基础节点
./start_all.sh     # 主任务：只启动 b2w_navigation 主控流程
```

推荐运行方式：

- APP/遥控器测试：先启动 `tcp_base_ctl.sh`（或 systemd 服务），APP 发送 `0x01 start` 后由 `robot_tcp_node` 拉起 `start_all.sh`。
- 纯自动流程测试：可手动先启动 `tcp_base_ctl.sh`，再手动执行 `start_all.sh`。
- `pause/restart` 只在 `start_all.sh` 生成 `/tmp/start_all.ready` 后可用；未 ready 时 APP 节点会拒绝暂停/恢复请求。

`tcp_base_ctl.sh` 日志：

```bash
tail -f /home/test/logs/tcp_base_ctl_latest/tcp_base_ctl.log
tail -f /home/test/logs/tcp_base_ctl_latest/robot_tcp.log
tail -f /home/test/logs/tcp_base_ctl_latest/b2w_teleop.log
```

systemd 自启动：

```bash
sudo cp tcp_base_ctl.sh /home/test/tcp_base_ctl.sh
sudo chmod +x /home/test/tcp_base_ctl.sh
sudo cp tcp_base_ctl.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable tcp_base_ctl.service
sudo systemctl restart tcp_base_ctl.service
journalctl -u tcp_base_ctl.service -f
```

`tcp_base_ctl.service` 使用 `test` 用户运行主进程，并通过 `ExecStartPre` 以 root 权限完成 `setcap`、485 串口模式和串口权限配置。

## 工作空间架构

| 工作空间 | 包名 | 功能 |
|---------|------|------|
| `colcon_ws` | nmea_msgs | 基础消息依赖（需最先编译） |
| `rtk_nav_ws` | ins_driver_node | 司南 RTK 串口解析，发布位置和方向 |
| `gnss_driver_ws` | gnss_driver | NovAtel/司南 RTK 驱动，坐标投影 |
| `tf_broadcast_ws` | robot_tf_broadcaster | 发布静态 TF 树（B2W→Z1→Lidar→GNSS） |
| `z1_move_ws` | z1_arm_controller_cpp | Z1 机械臂 ROS2 服务接口 |
| `b2w_navigation_ws` | b2w_navigation_controller | B2W 导航、里程计、EKF 融合 |
| `spray_path_planner_ws` | spray_path_planner | 读点文件、生成最短路径 |
| `rs585_ws` | rs485_node | Modbus RS485 继电器控制（喷枪） |
| `robose_airy_ws` | rslidar_sdk | 旧 RS16 激光雷达驱动（保留但当前未自动启动） |
| `livox_mid360_ws` | livox_ros_driver2 | MID-360 激光雷达驱动（已放入仓库，生产链路待接入） |
| `app_ws` | robot_tcp | TCP 长连接服务（与 APP/遥控器通信） |

## 关键 ROS 2 话题与服务

以下按“代码中真实出现的接口”整理，包含发布、订阅和服务。

### ROS 2 话题总表

| 工作空间 / 节点 | 方向 | 话题名 | 类型 | 说明 |
|---|---|---|---|---|
| `rtk_nav_ws / ins_parser_node` | 发布 | `/fix` | `geometry_msgs/msg/PointStamped` | 原始经纬高调试话题，`frame_id=gps_link` |
| `rtk_nav_ws / ins_parser_node` | 发布 | `/utm_fix` | `geometry_msgs/msg/PoseStamped` | UTM 坐标 + 朝向四元数 |
| `rtk_nav_ws / ins_parser_node` | 发布 | `/epsg_position` | `geometry_msgs/msg/PoseStamped` | EPSG:2100 坐标 + 朝向四元数，主导航节点当前订阅这个版本 |
| `rtk_nav_ws / ins_parser_node` | 发布 | `/gps` | `sensor_msgs/msg/NavSatFix` | 标准 GPS 消息 |
| `rtk_nav_ws / dog_controller` | 订阅 | `/utm_fix` | `geometry_msgs/msg/PoseStamped` | 简化导航测试节点输入 |
| `rtk_nav_ws / dog_controller` | 订阅 | `/scan` | `sensor_msgs/msg/LaserScan` | 障碍检测 |
| `rtk_nav_ws / dog_controller` | 发布 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 简化底盘控制输出 |
| `gnss_driver_ws / gnss_driver` | 发布 | `/nmea_sentence` | `nmea_msgs/msg/Sentence` | 原始 NMEA 报文 |
| `gnss_driver_ws / gnss_driver` | 发布 | `/gnss_pos` | `sensor_msgs/msg/NavSatFix` | GNSS 定位结果 |
| `gnss_driver_ws / gnss_driver` | 发布 | `/gnss_yaw` | `sensor_msgs/msg/Imu` | 航向转成的 IMU 姿态消息 |
| `gnss_driver_ws / gnss_driver` | 发布 | `/utm_position` | `geometry_msgs/msg/PointStamped` | UTM 投影坐标 |
| `gnss_driver_ws / gnss_driver` | 发布 | `/epsg_position` | `geometry_msgs/msg/PointStamped` | EPSG:2100 坐标，仅点，不含姿态 |
| `b2w_navigation_ws / b2w_nav_node` | 订阅 | `/epsg_position` | `geometry_msgs/msg/PoseStamped` | 依赖 `ins_parser_node` 发布的 Pose 版本 |
| `b2w_navigation_ws / b2w_nav_node` | 订阅 | `/scan` | `sensor_msgs/msg/LaserScan` | 激光避障 |
| `b2w_navigation_ws / b2w_nav_node` | 发布 | `/imu` | `sensor_msgs/msg/Imu` | B2W DDS LowState 转 ROS IMU |
| `b2w_navigation_ws / b2w_nav_node` | 发布 | `/battery` | `sensor_msgs/msg/BatteryState` | 电量状态 |
| `b2w_navigation_ws / b2w_nav_node` | 发布 | `/motors_temperatures` | `std_msgs/msg/Float32MultiArray` | 电机温度数组 |
| `b2w_navigation_ws / b2w_nav_node` | 发布 | `/b2w_odom` | `nav_msgs/msg/Odometry` | 主导航里程计输出 |
| `b2w_navigation_ws / b2w_nav_node` | 发布 | `/b2w_path` | `nav_msgs/msg/Path` | 运动轨迹历史 |
| `b2w_navigation_ws / b2w_nav_node` | 发布 | `/acquired_points` | `sensor_msgs/msg/PointCloud2` | 主导航根据当前 waypoint 状态发布已完成点云 |
| `b2w_navigation_ws / b2w_nav_node` | 发布 | `/unacquired_points` | `sensor_msgs/msg/PointCloud2` | 主导航根据当前 waypoint 状态发布未完成点云 |
| `b2w_navigation_ws / b2w_teleop_node` | 订阅 | `/joy` | `sensor_msgs/msg/Joy` | APP/遥控输入，直接转 Unitree 运动命令 |
| `spray_path_planner_ws / spray_path_planner_node` | 发布 | `/progress` | `std_msgs/msg/Byte` | 当前实现实际发布 0-100，不是 0-255 |
| `spray_path_planner_ws / spray_path_planner_node` | 发布 | `/acquired_points` | `sensor_msgs/msg/PointCloud2` | 已完成喷涂点云 |
| `spray_path_planner_ws / spray_path_planner_node` | 发布 | `/unacquired_points` | `sensor_msgs/msg/PointCloud2` | 未完成喷涂点云 |
| `z1_move_ws / z1_arm_controller` | 发布 | `/z1_complete` | `std_msgs/msg/Bool` | 机械臂动作/复位完成标志 |
| `app_ws / remote_control_node` | 发布 | `/remote_command` | `std_msgs/msg/String` | `"start" / "pause" / "stop"` 等命令字符串 |
| `app_ws / remote_control_node` | 发布 | `/joy` | `sensor_msgs/msg/Joy` | TCP 指令转手动控制 |
| `app_ws / remote_control_node` | 订阅 | `/progress` | `std_msgs/msg/Byte` | 转发给 TCP 客户端 |
| `app_ws / remote_control_node` | 订阅 | `/b2w_odom` | `nav_msgs/msg/Odometry` | 转发位置 |
| `app_ws / remote_control_node` | 订阅 | `/motors_temperatures` | `std_msgs/msg/Float32MultiArray` | 转发最高温度 |
| `app_ws / remote_control_node` | 订阅 | `/b2w_path` | `nav_msgs/msg/Path` | 转发轨迹 |
| `app_ws / remote_control_node` | 订阅 | `/acquired_points` | `sensor_msgs/msg/PointCloud2` | 转发已喷涂点云 |
| `app_ws / remote_control_node` | 订阅 | `/unacquired_points` | `sensor_msgs/msg/PointCloud2` | 转发未喷涂点云 |
| `robose_airy_ws / rslidar_sdk` | 发布 | `/scan` | `sensor_msgs/msg/LaserScan` | 旧 RS16 驱动输出；当前导航和 `dog_controller` 仍订阅 `/scan`，但启动脚本未自动拉起雷达 |
| `tf_broadcast_ws / robot_state_publisher` | 发布 | `/tf` | `tf2_msgs/msg/TFMessage` | 标准 `robot_state_publisher` 动态 TF 输出 |
| `tf_broadcast_ws / robot_state_publisher` | 发布 | `/tf_static` | `tf2_msgs/msg/TFMessage` | 标准静态 TF 输出 |

### ROS 2 服务总表

| 工作空间 / 节点 | 方向 | 服务名 | 类型 | 说明 |
|---|---|---|---|---|
| `spray_path_planner_ws / spray_path_planner_node` | 提供 | `/get_next_waypoint` | `spray_path_planner/srv/GetNextWaypoint` | 依次返回下一个喷涂点 |
| `spray_path_planner_ws / spray_path_planner_node` | 提供 | `/set_start_point` | `spray_path_planner/srv/SetStartPoint` | 根据起点重排喷涂路径 |
| `z1_move_ws / z1_arm_controller` | 提供 | `/z1_move_to_target` | `z1_arm_controller_cpp/srv/MoveArm` | 直接按目标位姿控制 Z1 |
| `z1_move_ws / z1_arm_controller` | 提供 | `/z1_reset_arm` | `z1_arm_controller_cpp/srv/MoveArm` | 机械臂复位 |
| `z1_move_ws / z1_arm_controller` | 提供 | `/z1_move_in_base_frame` | `z1_arm_controller_cpp/srv/MoveArmWithRPY` | 输入 `base_link` 坐标，内部转到 `z1_base` |
| `rs585_ws / relay_control_node` | 提供 | `/trigger_valve_ch1` | `std_srvs/srv/Trigger` | 触发 CH1 电磁阀 |
| `rs585_ws / relay_control_node` | 提供 | `/trigger_pump_ch2` | `std_srvs/srv/Trigger` | 触发 CH2 水泵 |
| `gnss_driver_ws / gnss_driver` | 提供 | `/save_utm_point` | `std_srvs/srv/Trigger` | 将当前点写入 waypoint 文件 |
| `b2w_navigation_ws / b2w_nav_node` | 提供 | `/emergency_stop` | `std_srvs/srv/Trigger` | 暂停当前自动任务，冻结导航状态机并保持底盘停止 |
| `b2w_navigation_ws / b2w_nav_node` | 提供 | `/erase_emergency_stop` | `std_srvs/srv/Trigger` | 解除暂停，恢复当前任务状态机 |
| `app_ws / remote_control_node` | 调用 | `/emergency_stop` | `std_srvs/srv/Trigger` | APP 暂停时调用；调用前检查 `/tmp/start_all.ready` |
| `app_ws / remote_control_node` | 调用 | `/erase_emergency_stop` | `std_srvs/srv/Trigger` | APP 恢复时调用；调用前检查 `/tmp/start_all.ready` |
| `b2w_navigation_ws / b2w_nav_node` | 调用 | `/set_start_point` | `spray_path_planner/srv/SetStartPoint` | 初始化时设置起点 |
| `b2w_navigation_ws / b2w_nav_node` | 调用 | `/get_next_waypoint` | `spray_path_planner/srv/GetNextWaypoint` | 获取下一个喷涂目标 |
| `b2w_navigation_ws / b2w_nav_node` | 调用 | `/z1_move_to_target` | `z1_arm_controller_cpp/srv/MoveArm` | 调机械臂执行喷涂位姿 |
| `b2w_navigation_ws / b2w_nav_node` | 调用 | `/z1_reset_arm` | `z1_arm_controller_cpp/srv/MoveArm` | 喷涂完成后复位 |
| `b2w_navigation_ws / b2w_nav_node` | 调用 | `/trigger_valve_ch1` | `std_srvs/srv/Trigger` | 喷枪触发 |

### 非 ROS 通道

| 接口 | 类型 | 用途 |
|---|---|---|
| `rt/lowstate` | Unitree DDS topic | `b2w_nav_node` 订阅底盘低层状态，不是 ROS 2 话题 |

### 接口注意事项

- `/epsg_position` 在仓库里有两种实现：
  - ✅ **主导航输入（推荐）**：`rtk_nav_ws/ins_parser.cpp` 发布 `geometry_msgs/msg/PoseStamped`，包含位置和航向。
  - ⚠️ **辅助调试用**：`gnss_driver_ws/pub_rtk_save_pt_node.cpp` 发布 `geometry_msgs/msg/PointStamped`，仅包含坐标，不满足主导航需求。
- 当前主导航 `b2w_navigation_ws/src/main.cpp` 订阅的是 `PoseStamped` 版本，因此运行主流程时必须启动 `rtk_nav_ws` 的 `ins_parser_node`。
- `spray_path_planner` 当前 `/progress` 实际发布的是 `0~100` 百分比整数；如果 APP 协议仍按 `0~255` 处理，需要后续统一。
- `/emergency_stop` 和 `/erase_emergency_stop` 由 `b2w_nav_node` 提供；`app_node.cpp` 作为客户端调用。若 `/tmp/start_all.ready` 不存在或未进入 `ready/partial`，APP 暂停/恢复会被拒绝，避免主控未就绪时误判成功。

### 自定义服务消息类型定义

- `spray_path_planner/GetNextWaypoint.srv`
- `spray_path_planner/SetStartPoint.srv`
- `z1_arm_controller_cpp/MoveArm.srv`
- `z1_arm_controller_cpp/MoveArmWithRPY.srv`

## 硬件设备与配置

- `/dev/ttyTHS1` — RS485 继电器（需通过 `tac3kp_uart_mode_config.sh 485` 设为 485 模式，权限 777）
- `/dev/ttyTHS2` — 司南 RTK 串口（波特率 115200，权限 777）
- B2W 通信通过 DDS（Unitree SDK2），网卡见下方“网络与端口”。
- Z1 机械臂 SDK、手动测试、零位恢复和故障排查统一见 `z1_move_ws/README.md`。
- 导航大师配置： 4G配置中APN需要改成 internet，改完后能连上网

## 网络与端口

### 拓扑与接口对照表（2026-05-08 现场实测）

| 用途 | 接口 | 主机 IP | 对端设备 IP / MAC | 端口 | 配置来源 |
|---|---|---|---|---|---|
| 机器人 WiFi AP（热点）| `wlan0` | `192.168.88.1/24` | 手机 / APP 客户端 | — | `01-wifi-ap.yaml`（**唯一在 netplan 里的**）|
| Z1 机械臂下位机（z1_ctrl 主连）| `eth1`（100 Mb/s）| `192.168.122.222/24` | `192.168.122.110` / `00:80:fb:5e:ad:49` | UDP `8881` | NM `Wired connection 2` + `z1_controller/config/config.xml` |
| **B2W 狗 DDS 通信（Unitree SDK2）** | **`eth2`（1000 Mb/s）** | **`192.168.123.222/24`** | **`192.168.123.161` / `48:21:0b:3d:4a:2e`** | — | NM `Wired connection 3` + `tcp_base_ctl.sh:369,372` + `b2w_navigation_ws/launch/b2w_navigation.launch:20` |
| Z1 出厂 IP 设置工具（一次性写入 MCU）| 临时手配 | — | `192.168.123.110` | UDP `8880` | `z1_controller/unitreeArmTools.py` |
| APP / 遥控器 TCP 服务（机器人侧监听）| 任意 | — | — | TCP `9002`（参数 `listen_port`）| `app_ws/src/app_node.cpp:45` |
| 4G 上网（默认路由）| `wwan0` | DHCP `192.168.225.x/22` | 运营商 | — | NM `Wired connection 4`，APN `internet` |
| RSLidar RSAIRY 默认配置（单播）| 不绑定主机地址 | — | — | MSOP `6699`、DIFOP `7788` | `robose_airy_ws/src/rslidar_sdk/config/config.yaml` |
| RSLidar 备用配置（双雷达组播）| 主机 `10.21.31.100` | — | 组播组 `224.10.10.201/202` | MSOP `6691/6692`、DIFOP `7781/7782` | `robose_airy_ws/src/rslidar_sdk/config/config_trans.yaml` |
| `eth0` | DOWN | `192.168.1.102/24`（残留配置）| — | — | NM `Wired connection 1`，物理无线，无效 |

完整拓扑图、ARP 证据和换口故障复盘见 `docs/network_topology.md`。

注意事项：

- **网卡 IP 的真实信源是 `/etc/NetworkManager/system-connections/Wired connection N.nmconnection`，不是 netplan**。netplan 里只有 `wlan0`，所有 eth* 都靠 NetworkManager 维护静态 IP。
- **狗本体上多个 RJ45 槽中只有一个内部走线接通 DDS**。5-8 现场把网线从狗端有效槽换到另一个槽，整条链路在狗那一头就断了，和 IPC 端用什么接口名无关。狗身上"对外 DDS 槽"是固定的，**不要换槽**；排查链路先在狗端拔插测 `ethtool eth2 \| grep "Link detected"`。
- **NetworkManager 用 `interface-name=ethN` 把配置绑到接口名而不是 MAC**——这是日后**改 IPC 端**接口名（如把 `eth2` 改 `eth1`）时会撞的独立坑：eth1 仍只挂 122.x 段，狗的 123.x 段不会跟着搬过去。**与 5-8 故障无关**，但作为通用警示保留。换口正确做法见 `docs/network_topology.md` §5。
- `b2w_nav_node` / `b2w_teleop_node` 二进制必须带 `cap_net_raw+ep`，否则无法打开 `eth2`。当前 `tcp_base_ctl.service` 的 `ExecStartPre` 会自动给 `b2w_teleop_node` 设置权限；`b2w_nav_node` 手动编译/部署后需要单独确认或执行：

  ```bash
  sudo setcap cap_net_raw+ep b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_nav_node
  sudo setcap cap_net_raw+ep b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_teleop_node
  ```

  排查方法见 `log_view.md`（搜 `cap_net_raw`、`eth2`）。
- Z1 控制器下位机 IP 自 commit `d2c1b73` 起由 `192.168.123.110` 改为 `192.168.122.110`（`config.xml`）。若更换/重置 MCU，需先用 `unitreeArmTools.py`（仍默认连 `192.168.123.110:8880`）写入新 IP，再回写 `config.xml`。
- `app_node.cpp` 只对外开 TCP `9002` 一个监听端口，机器人侧没有主动外联第三方服务；APP 端通过这条 TCP 长连接收发协议（参见“TCP 应用协议”小节）。
- `01-wifi-ap.yaml` 是 Netplan 部署文件，将 `wlan0` 配成 AP 热点；调试时上位机/手机连入 `KifferB2wLocalLAN` 后可走 `192.168.88.0/24` 直连机器人。
- `sport_client.launch` 的默认 `interface` 是 `lo`（仅供仿真/回环），现场启动 B2W 必须显式传 `eth2` 或使用 `b2w_navigation.launch`（已硬编码 `eth2`）。
- RSLidar 当前没有被 `tcp_base_ctl.sh` / `start_all.sh` 自动拉起；若使用 `config_trans.yaml`，主机网卡需配置在 `10.21.31.0/24` 网段并加入对应组播组。

## 部署说明

- 机器人本机用户路径：`/home/test/`（与开发机 `/home/oneko/` 不同）
- `tcp_base_ctl.sh` 中使用 `/home/test/z1_controller/build/` 路径启动 `z1_ctrl`，部署时需确认该二进制存在
- `start_all.sh` 只负责主任务 `b2w_navigation.launch`；基础节点由 `tcp_base_ctl.sh` 常驻维护
- 喷涂点文件（如 `gnss_waypoints.txt`、`points_test_mikinwn.txt`）当前应使用与 `/epsg_position` 同框架的 EPSG:2100 + HEPOS 平面坐标；历史点文件可能来自经纬度离线转换。

## 关键源文件

- `b2w_navigation_ws/src/main.cpp` — B2W 导航主控逻辑
- `b2w_navigation_ws/src/b2w_teleop.cpp` — 遥控/手动控制逻辑
- `app_ws/src/app_node.cpp` — TCP 通信协议实现
- `rtk_nav_ws/src/ins_parser.cpp` — 司南 RTK 串口解析与坐标投影（发布 /fix、/utm_fix、/epsg_position、/gps）
- `rtk_nav_ws/src/dog_controller.cpp` — 遥控手柄控制辅助
- `z1_move_ws/src/z1_arm_controller_node.cpp` — Z1 控制节点
- `spray_path_planner_ws/src/spray_path_planner_node.cpp` — 路径规划
- `gnss_driver_ws/src/pub_rtk_save_pt_node.cpp` — GNSS 驱动与坐标转换（NovAtel/司南）

## 配置文件

- `b2w_navigation_ws/config/b2w_controller_params.yaml` — 速度、阈值、RTK 外参、机械臂目标姿态和 APP 遥控速度参数
- `b2w_navigation_ws/config/b2w_controller_params_ekf.yaml` — EKF 参数
- `spray_path_planner_ws/config/path_planner.yaml` — 喷涂点文件路径
- `rs585_ws/config/rs485_params.yaml` — RS485 通信参数
- `gnss_driver_ws/config/gnss_params.yaml` — GNSS 驱动参数（端口、波特率、坐标系）
- `01-wifi-ap.yaml` — 机器人 WiFi AP 配置（Netplan，部署时热点设置）

`b2w_controller_params.yaml` 关键参数：

| 参数 | 节点 | 说明 |
|---|---|---|
| `moving_to_target_forward_speed` | `b2w_nav_node` | 自动作业前进速度 |
| `rtk_x_offset` | `b2w_nav_node` | RTK 天线相对 `base_link` 的 x 偏移，默认 `-0.4477` |
| `z1_arm_target_pitch_deg` | `b2w_nav_node` | 机械臂末端目标 pitch 角度，默认 `90.0` |
| `teleop_linear_x_speed` | `b2w_teleop_node` | APP 遥控前进/后退速度 |
| `teleop_linear_y_speed` | `b2w_teleop_node` | APP 遥控左移/右移速度 |
| `teleop_yaw_speed` | `b2w_teleop_node` | APP 遥控旋转角速度 |
| `teleop_deadzone` | `b2w_teleop_node` | 遥控死区，低于该速度时发送 `StopMove()` |

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
| 0x0A | 趴下（`b2w_teleop_node` 调用 `StandDown()`） |
| 0x0B | 站立（`b2w_teleop_node` 调用 `StandUp()`） |
| 0x10 | 恢复（解除暂停） |
| 0xFF | 急停阻尼（`b2w_teleop_node` 调用 `Damp()`） |

控制链路说明：

- `0x01 start`：`app_node.cpp` 检查 `/tmp/start_all.pid` 后通过 `setsid /home/test/start_all.sh &` 拉起主任务。
- `0x02 pause`：`app_node.cpp` 调用 `/emergency_stop`，由 `b2w_nav_node` 暂停状态机。
- `0x03 stop`：`app_node.cpp` 读取 `/tmp/start_all.pid`，对 `start_all.sh` 进程组发送 `SIGTERM`。
- `0x04` ~ `0x09`：`app_node.cpp` 发布 `/joy.axes` 方向量，`b2w_teleop_node` 按 YAML 速度缩放后调用 `SportClient::Move()`。
- `0x0A`、`0x0B`、`0xFF`：`app_node.cpp` 发布 `/joy.buttons`，`b2w_teleop_node` 分别调用 `StandDown()`、`StandUp()`、`Damp()`。
- `0x10 restart/resume`：`app_node.cpp` 调用 `/erase_emergency_stop`，由 `b2w_nav_node` 解除暂停。

## 坐标系说明

- 使用 EPSG 2100（希腊 GGRS87 / Greek Grid）投影
- 使用 `proj` 库进行 WGS84 ↔ EPSG2100 转换
- 当前 URDF 中的固定链路：`base_link` → `z1_base`，`base_link` → `lidar_link`，`base_link` → `rtk_link`

## 坐标数据变化链路

参考 `rtk_nav_ws/issue.md`（HEPOS 5 点验证报告）和 commit `391d7f4`（接入 HEPOS 网格修正）。当前导航实际使用的是 **PROJ EPSG:2100 + HEPOS plane grid 修正**，与丰疆 RTK 控制器显示坐标对齐到毫米级残差。

### 完整链路（INS → 发布）

`rtk_nav_ws/src/ins_parser.cpp` 和 `gnss_driver_ws/src/pub_rtk_save_pt_node.cpp` 共用同一套链路：

```
司南 INS 串口 (/dev/ttyTHS2, 115200)
  └─ NMEA + 私有报文 → (lat, lon, alt, yaw_deg, heading_valid)

[Step 1] PROJ 基础投影  EPSG:4326 → EPSG:2100
  ├─ use_epsg_crs_datum = true（YAML 默认）
  │     proj_create_crs_to_crs("EPSG:4326","EPSG:2100") + proj_normalize_for_visualization
  │     自动叠加 GGRS87↔WGS84 datum 平移
  └─ use_epsg_crs_datum = false（备用）
        +proj=tmerc +lat_0=0 +lon_0=24 +k=0.9996 +x_0=500000 +ellps=GRS80 +no_defs
        仅做投影，无 datum 平移
  → (epsg_x_proj, epsg_y_proj)        // 称为 "PROJ default" 坐标

[Step 2] HEPOS plane grid 修正（use_hepos_grid_correction = true，YAML 实际开启）
  ├─ 读取 rtk_nav_ws/fj_dynamic/dE_2km_V1-0.egrd, dN_2km_V1-0.ngrd
  ├─ 网格参数：spacing=2000 m, origin=(-64400, 3840000), 值单位 cm
  ├─ 用 (epsg_x_proj, epsg_y_proj) 作为索引，双线性插值得到 dE_m, dN_m
  └─ epsg_x = epsg_x_proj + dE_m;  epsg_y = epsg_y_proj + dN_m
  → (epsg_x, epsg_y)                  // 称为 "HEPOS-correct ΕΓΣΑ87" 坐标，与丰疆控制器显示残差 ~4 mm

[Step 2.5] TM07（仅诊断）
  project_to_tm07() 同时输出，不进入任何 publish，仅打印用于排查

[Step 3] 航向转换（INS → ROS）
  yaw_ros_deg = 90.0 - yaw_deg        // INS 北向 0°、顺时针正 → ROS 东向 0°、逆时针正
  归一化到 (-π, π]，再转为 tf2::Quaternion

[Step 4] 发布
  /fix          PointStamped(lon, lat, alt)            // 原始经纬度，调试
  /utm_fix      PoseStamped(UTM E, UTM N, alt) + quat   // 旧链路保留，主线已不用
  /epsg_position PoseStamped(epsg_x, epsg_y, alt) + quat // 主导航唯一输入源
  /gps          NavSatFix(lat, lon, alt)
```

每帧链路输出会被打印为 `COORD_CHAIN ... EPSG2100_PROJ ... HEPOS(...) EPSG2100_FINAL ...`，可在日志中直接看到每一阶段坐标。

### 下游消费

- `b2w_navigation_ws/src/main.cpp` 中的 `b2w_nav_node` **只**订阅 `/epsg_position`，并按 YAML 中 `rtk_x_offset`（默认 `-0.4477`）把 RTK 天线坐标平移回 `base_link`。
- `gnss_waypoints.txt` 中的目标点必须与 `/epsg_position` 在同一坐标框架，因此**也必须经过 HEPOS plane grid 修正**，否则会出现约 1 m 的系统性偏差（详见 `issue.md §5/§6`）。

### 点位录入流程（HEPOS 修正接入后，commit `391d7f4` 起生效）

狗端 `use_hepos_grid_correction=true` 接入 HEPOS plane grid 后，狗端 `/epsg_position` 的 `(x, y)` 与丰疆控制器显示的 `Easting/Northing` 在同一 HEPOS-correct ΕΓΣΑ87 平面框架下，残差量级 ≤ 10 cm（`issue.md §6.3` 的 5 点验证为 ~4 mm）。因此：

- ✅ **可以直接把丰疆控制器显示的 `Easting/Northing` 录入 `gnss_waypoints.txt` 的 `x/y` 字段**。
- ✅ 司南 RTK 通过 `record_epsg_waypoint.py` 现场录点同样可用，输出已经走完整 `EPSG:4326 → EPSG:2100 → HEPOS grid` 链路。
- ✅ 也可以用丰疆 RTK 只采 WGS84 经纬度，离线通过狗端同一套链路（如 `convert_var_point_no_datum.py` / `convert_gps_convert_to_csv.py`）转换得到 `x/y`，三种来源等价。
- ⚠️ **前置条件**：录入丰疆显示坐标前，确认狗端 YAML `use_hepos_grid_correction=true` 且 `fj_dynamic/dE_2km_V1-0.egrd`、`dN_2km_V1-0.ngrd` 在配置路径下可读；若关闭 HEPOS，狗端坐标会退回 PROJ default，与丰疆显示值再次出现 ~1 m 偏差。
- ⚠️ `z` 字段仍使用司南 RTK 体系（与 `/epsg_position.z` 同源），**不直接使用丰疆 `Elevation`**：两者高程基准不同（丰疆 184.143 vs 司南 223.5418，落点文件 z=185.077），混用会引入米级偏差。地形不敏感时可用区域均值或邻近司南点 z。
- 抽检建议：录入后挑 2–3 个点现场复核，狗端到达位置与目标点应在 10 cm 量级；若出现 ~1 m 偏差，先排查 HEPOS 配置是否生效，再排查高程字段是否被混用。

### HEPOS 修正涉及文件

- `rtk_nav_ws/fj_dynamic/dE_2km_V1-0.egrd` / `dN_2km_V1-0.ngrd` — 平面格网修正量（2 km 间距，cm 单位）
- `rtk_nav_ws/fj_dynamic/GEOID_GR.GRD` — 大地水准面格网（高程修正，当前未接入）
- `rtk_nav_ws/fj_dynamic/parsms.json` — 丰疆参数文件
- `rtk_nav_ws/include/hepos_grid_corrector.hpp` — 双线性插值实现，被 `ins_parser.cpp` 与 `pub_rtk_save_pt_node.cpp` 共用
