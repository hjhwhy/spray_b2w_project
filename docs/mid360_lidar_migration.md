# MID-360 雷达更换与斜向安装改造说明

本文用于梳理从原头部 RS16/Robosense 雷达更换为 Livox MID-360 后，需要检查和修改的工程位置。

当前结论：

- 这次改造不是只换驱动，还必须重新定义 `base_link` 到 `lidar_link` 的外参。
- 当前仓库里已有 `livox_mid360_ws`，并已放入 `Livox-SDK2` 与 `livox_ros_driver2`。
- 当前 `start_all.sh` 已不启动旧 `rslidar_sdk`，`tcp_base_ctl.sh` 也未启动雷达。
- 当前 TF 中仍保留旧雷达安装关系：`base_link -> lidar_link`，位置为 `0.4197 0.0 0.0`，yaw 为 `90°`。

## 1. 硬件变化

原安装方式：

- 雷达在狗头部。
- 原描述为 `Z` 轴向前。
- 当前 URDF 里 `base_to_lidar` 使用 `rpy="0 0 1.5708"`，说明软件里曾经做过一个绕 `Z` 轴 90° 的固定旋转。

新安装方式：

- 雷达更换为 Livox MID-360。
- 雷达斜向安装，描述为 `Z` 轴上移 60°。
- 电源线从侧边引出。
- 由于线缆朝向，雷达自身 `X` 轴可能朝向机器狗左边。

需要现场确认的机械参数：

- MID-360 中心点相对 `base_link` 的 `x y z`，单位 m。
- 雷达坐标系相对 `base_link` 的 roll/pitch/yaw，单位 rad 或 deg。
- 线缆侧边引出后，是否确实导致 MID-360 的 `X` 轴朝向机器狗左侧。
- “Z 轴上移 60°”具体是绕机器狗哪个轴旋转：绕 `base_link` 的 `Y` 轴俯仰，还是绕雷达自身某个轴倾斜。

## 2. 必须修改的位置

### 2.1 TF / URDF 外参

文件：

- `tf_broadcast_ws/urdf/my_robot.urdf`

当前内容：

```xml
<link name="lidar_link"/>
<joint name="base_to_lidar" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.4197 0.0 0.0" rpy="0 0 1.5708"/>
</joint>
```

需要修改：

- `xyz` 改为 MID-360 新安装中心相对 `base_link` 的实际位置。
- `rpy` 改为 MID-360 新安装姿态。
- 如果 MID-360 驱动发布的 frame 不是 `lidar_link`，要统一 frame 名称。

建议保留 frame 名称为：

- `base_link`
- `lidar_link`

这样下游不用改 frame 名。

### 2.2 Livox MID-360 驱动

目录：

- `livox_mid360_ws`

当前实际目录：

- `livox_mid360_ws/src/Livox-SDK2`
- `livox_mid360_ws/src/livox_ros_driver2`
- `livox_mid360_ws/src/编译和运行命令.txt`

SDK 编译安装：

```bash
cd /home/test/livox_mid360_ws/src/Livox-SDK2
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

开发机路径示例：

```bash
cd /home/oneko/projects/spray_b2w_robot_project_greek/livox_mid360_ws/src/Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install
```

ROS2 驱动编译：

```bash
cd /home/test/livox_mid360_ws/src/livox_ros_driver2
./build.sh humble
```

开发机路径示例：

```bash
cd /home/oneko/projects/spray_b2w_robot_project_greek/livox_mid360_ws/src/livox_ros_driver2
./build.sh humble
```

注意：

- 目录名是 `livox_ros_driver2`，不是 `livox_ros_driver`。
- `launch` 文件安装在 `launch_ROS2/` 子目录下，不能直接用 `ros2 launch livox_ros_driver2 rviz_MID360_launch.py`。
- 如果 `build/install/log` 属主变成 `nobody:nogroup`，普通用户重新编译会失败，需要先修复权限。

权限修复：

```bash
sudo chown -R $USER:$USER /home/test/livox_mid360_ws/build \
  /home/test/livox_mid360_ws/install \
  /home/test/livox_mid360_ws/log
```

需要补齐或确认：

- MID-360 IP、主机 IP、点云 topic、frame_id 是否配置正确。
- 驱动输出 topic 是否为下游需要的 `PointCloud2`。

驱动运行命令：

```bash
cd /home/test/livox_mid360_ws
source install/setup.bash
ros2 launch livox_ros_driver2 launch_ROS2/rviz_MID360_launch.py
```

如果只启动驱动、不启动 RViz：

```bash
cd /home/test/livox_mid360_ws
source install/setup.bash
ros2 launch livox_ros_driver2 launch_ROS2/msg_MID360_launch.py
```

`rviz_MID360_launch.py` 与 `msg_MID360_launch.py` 的差异：

- `rviz_MID360_launch.py` 会同时启动 `livox_ros_driver2_node` 和 `rviz2`。
- `rviz_MID360_launch.py` 默认 `xfer_format = 0`，输出标准 `sensor_msgs/msg/PointCloud2`。
- `msg_MID360_launch.py` 只启动驱动，不启动 RViz。
- `msg_MID360_launch.py` 默认 `xfer_format = 1`，输出 Livox 自定义消息。
- 如果下游需要标准点云，生产运行建议使用无 RViz launch，并把 `xfer_format` 改为 `0`。

### 2.3 启动脚本

相关文件：

- `tcp_base_ctl.sh`
- `start_all.sh`

当前状态：

- `tcp_base_ctl.sh` 启动基础节点：RS485、TF、GNSS、Z1、APP 底盘遥控、TCP 通信。
- `start_all.sh` 只启动 `b2w_navigation.launch` 主任务。
- `start_all.sh` 中旧 `rslidar_sdk` 启动已注释。

推荐方案：

- 如果 MID-360 是基础感知节点，应该放入 `tcp_base_ctl.sh` 常驻启动。
- 不建议放入 `start_all.sh`，避免每次 APP start/stop 反复重启雷达驱动。
- `start_all.sh` 继续只负责主任务。

需要新增：

- MID-360 日志路径，例如 `MID360_LOG="${RUN_LOG_DIR}/mid360.log"`。
- source `livox_mid360_ws/install/setup.bash`。
- 启动命令，例如：

```bash
ros2 launch livox_ros_driver2 launch_ROS2/msg_MID360_launch.py >>"$MID360_LOG" 2>&1 &
MID360_PID=$!
```

生产运行不要使用 `rviz_MID360_launch.py`，避免 systemd 或后台脚本启动图形界面。

如果使用 `msg_MID360_launch.py` 作为生产 launch，需要先确认或修改：

```python
xfer_format = 0
frame_id = 'lidar_link'
```

否则默认可能发布 Livox 自定义消息，且 frame_id 为 `livox_frame`。

### 2.4 文档与构建说明

相关文件：

- `CLAUDE.md`
- `livox_mid360_ws/src/编译和运行命令.txt`

需要更新：

- 将旧 `robose_airy_ws / rslidar_sdk / RS16` 说明改为 MID-360。
- 增加 Livox SDK2 编译步骤。
- 增加 `livox_ros_driver2` 编译步骤。
- 增加 MID-360 IP 与网卡配置说明。
- 增加 TF 外参测量方式和验证方式。

## 3. 可能需要修改的下游逻辑

### 3.1 话题名称

当前仓库中旧雷达相关说明提到：

- `/scan`
- `rslidar_points`
- `rslidar_imu_data`

MID-360 通常输出：

- 点云 `PointCloud2`
- IMU 数据

需要确认实际 topic：

```bash
ros2 topic list
ros2 topic info <pointcloud_topic>
ros2 topic echo <pointcloud_topic> --once
```

如果下游节点订阅了旧 topic，需要做 topic remap 或修改订阅名。

### 3.2 frame_id

需要确认 MID-360 点云消息中的：

```bash
ros2 topic echo <pointcloud_topic> --once | grep frame_id
```

要求：

- 点云 frame_id 最好统一为 `lidar_link`。
- 如果驱动输出为 `livox_frame` 或 `livox_lidar`，可以通过驱动配置改 frame_id，或在 TF 中增加 `lidar_link -> livox_frame`。

推荐：

- 驱动直接发布 `frame_id: lidar_link`。
- URDF 只维护 `base_link -> lidar_link`。

### 3.3 点云坐标方向

由于你描述“雷达 X 可能朝向机器狗左边”，这会影响下游对点云方向的理解。

需要验证：

- 狗正前方障碍物，在 RViz 里是否出现在 `base_link` 的 `+X` 方向。
- 狗左侧障碍物，是否出现在 `base_link` 的 `+Y` 方向。
- 地面点是否在合理的 `Z` 方向。

如果方向不对，优先改 `base_to_lidar` 的 `rpy`，不要在算法里硬改点云。

## 4. 初始外参推导建议

ROS 常用约定：

- `base_link`：`X` 向前，`Y` 向左，`Z` 向上。
- `lidar_link`：建议转换后也能正确映射到 `base_link`。

如果现场确认：

- MID-360 自身 `X` 轴朝向机器狗左侧，即对应 `base_link +Y`。
- MID-360 自身 `Z` 轴斜向上，且相对竖直方向倾斜 60°。

则需要组合两个旋转：

- 先把雷达 `X` 轴转到 `base_link +Y`。
- 再加入 60° 倾角。

注意：

- 不能仅凭“线从侧边出”确定坐标轴。
- 必须以 MID-360 官方坐标系标识或实际点云方向验证为准。
- `rpy` 的旋转顺序是 URDF 固定 joint 的 roll、pitch、yaw，调试时建议每次只改一个角。

## 5. 验证流程

### 5.1 驱动连通性

```bash
source /opt/ros/humble/setup.bash
source /home/test/livox_mid360_ws/install/setup.bash
ros2 launch livox_ros_driver2 launch_ROS2/msg_MID360_launch.py
```

检查：

```bash
ros2 topic list
ros2 topic hz <pointcloud_topic>
ros2 topic echo <pointcloud_topic> --once
```

如果需要临时用 RViz 看点云：

```bash
source /opt/ros/humble/setup.bash
source /home/test/livox_mid360_ws/install/setup.bash
ros2 launch livox_ros_driver2 launch_ROS2/rviz_MID360_launch.py
```

### 5.2 TF 连通性

启动：

```bash
ros2 launch robot_tf_broadcaster tf_publisher.launch
```

检查：

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

### 5.3 RViz 方向验证

固定坐标系设为：

```text
base_link
```

观察：

- 前方物体是否在 `+X`。
- 左侧物体是否在 `+Y`。
- 地面是否在 `-Z` 或合理高度。
- 斜装 60° 后，点云扇区是否覆盖预期作业区域。

### 5.4 启动链路验证

如果把 MID-360 放进 `tcp_base_ctl.sh`：

```bash
./tcp_base_ctl.sh
```

检查日志：

```bash
tail -f /home/test/logs/tcp_base_ctl_latest/mid360.log
```

如果使用 systemd：

```bash
journalctl -u tcp_base_ctl.service -f
```

## 6. 推荐改造顺序

1. 先单独跑通 MID-360 驱动，不接入主流程。
2. 确认点云 topic、frame_id、频率、IP 配置。
3. 修改 `tf_broadcast_ws/urdf/my_robot.urdf` 的 `base_to_lidar` 外参。
4. 用 RViz 验证方向，确认前方为 `+X`、左方为 `+Y`。
5. 将 MID-360 启动加入 `tcp_base_ctl.sh`。
6. 更新 `CLAUDE.md` 和部署说明。
7. 再接入自动作业流程。

## 7. 当前暂不建议直接修改的地方

- 不建议先改 `b2w_navigation_ws/src/main.cpp`。
- 不建议先改喷涂路径算法。
- 不建议把雷达启动放进 `start_all.sh`。
- 不建议在点云消费者里手动交换 `x/y/z`。

优先级应为：

1. 驱动参数。
2. TF 外参。
3. topic/frame remap。
4. 下游算法。

## 8. 待确认清单

- MID-360 设备 IP：
- 主机网卡名：
- 主机 IP：
- 点云 topic：
- IMU topic：
- 点云 frame_id：
- MID-360 中心相对 `base_link` 的 `x y z`：
- MID-360 相对 `base_link` 的 `roll pitch yaw`：
- 是否需要使用 MID-360 IMU：
- 是否需要替代旧 `/scan`：
- 是否需要保留旧 `lidar_link` 名称：
