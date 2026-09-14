# Z1 机械臂控制与排查

本文集中记录 Z1 机械臂 ROS2 控制节点、手动测试、零位恢复和常见故障排查。机械臂相关说明以本文为准，避免散落在顶层文档中重复维护。

## 组件关系

- `z1_sdk/lib/libZ1_SDK_aarch64`：Z1 SDK ARM64 预编译库，`z1_move_ws` 编译时链接它。
- `/home/test/z1_controller/build/z1_ctrl`：机器人本机 Z1 底层控制程序，负责连接 Z1 下位机。
- `z1_move_ws`：ROS2 服务层，包名 `z1_arm_controller_cpp`，节点名 `z1_arm_controller`。
- `tf_broadcast_ws`：提供 `base_link -> z1_base`，`/z1_move_in_base_frame` 服务依赖该 TF。

当前 Z1 下位机配置见 `z1_controller/config/config.xml`：

```xml
<IP>192.168.122.110</IP>
<Port>8881</Port>
```

官方键盘控制参考：

- https://support.unitree.com/home/zh/Z1_developer/keyboard
- https://support.unitree.com/home/en/Z1_developer/z1

## 正常启动链路

在机器人本机上，`tcp_base_ctl.sh` 会常驻启动 Z1 相关进程：

```bash
cd /home/test
./tcp_base_ctl.sh
```

对应日志：

```bash
tail -f /home/test/logs/tcp_base_ctl_latest/z1_ctrl.log
tail -f /home/test/logs/tcp_base_ctl_latest/z1.log
```

如果需要手动启动，按这个顺序：

```bash
source /opt/ros/humble/setup.bash
source /home/test/z1_move_ws/install/setup.bash
source /home/test/tf_broadcast_ws/install/setup.bash

ros2 launch robot_tf_broadcaster tf_publisher.launch
```

另开一个终端启动底层控制：

```bash
cd /home/test/z1_controller/build
./z1_ctrl
```

再另开一个终端启动 ROS2 服务层：

```bash
source /opt/ros/humble/setup.bash
source /home/test/z1_move_ws/install/setup.bash
ros2 run z1_arm_controller_cpp z1_arm_controller_node
```

## 服务检查

确认服务存在：

```bash
ros2 service list | grep z1
```

应至少看到：

```text
/z1_move_to_target
/z1_reset_arm
/z1_move_in_base_frame
```

确认节点和 TF：

```bash
ros2 node list | grep z1
ros2 run tf2_ros tf2_echo base_link z1_base
```

## 手动动作测试

直接调用 `/z1_move_to_target`：

```bash
ros2 service call /z1_move_to_target z1_arm_controller_cpp/srv/MoveArm \
"{target_pose: {position: {x: 0.424, y: -0.053, z: 0.0}, orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}}}"
```

复位机械臂：

```bash
ros2 service call /z1_reset_arm z1_arm_controller_cpp/srv/MoveArm "{}"
```

用脚本生成或发送 RPY 测试命令：

```bash
cd /home/test/z1_move_ws

# 只生成命令，不发送
python3 manual_test_pose.py --x 0.424 --y -0.053 --z 0.0 --pitch 90

# 直接发送
python3 manual_test_pose.py --x 0.424 --y -0.053 --z 0.0 --pitch 88 --send
```

连续微调 pitch 时，建议从这些值开始：

```bash
python3 manual_test_pose.py --x 0.424 --y -0.053 --z 0.0 --pitch 92 --send
python3 manual_test_pose.py --x 0.424 --y -0.053 --z 0.0 --pitch 90 --send
python3 manual_test_pose.py --x 0.424 --y -0.053 --z 0.0 --pitch 88 --send
python3 manual_test_pose.py --x 0.424 --y -0.053 --z 0.0 --pitch 83 --send
```

同时微调 roll / pitch / yaw：

```bash
python3 manual_test_pose.py --x 0.424 --y -0.053 --z 0.0 --roll 0 --pitch 88 --yaw 0 --send
```

## 手动调整并恢复零位

适用于机械臂关节刻度偏了、自动复位姿态不对、或者需要人工把关节对回机械零位的场景。

1. 停止 ROS2 机械臂控制节点和主任务，避免自动控制同时发命令。

```bash
pkill -TERM -x z1_arm_controller_node 2>/dev/null || true
pkill -TERM -f "ros2 run z1_arm_controller_cpp z1_arm_controller_node" 2>/dev/null || true
```

2. 进入 Z1 底层控制程序目录，启动键盘模式：

```bash
cd /home/test/z1_controller/build
./z1_ctrl k
```

3. 按键盘数字 `2` 进入 `JOINTCTRL` 模式。

4. 按 Unitree 官方键盘说明长按对应关节按键，逐个调整关节，直到机械臂关节刻度对齐。

5. 对齐后按 `Ctrl+C` 退出终端。

6. 断电重启 Z1 机械臂，再重新启动 `tcp_base_ctl.sh` 或手动启动链路。

注意：键盘模式会直接控制关节，操作前确认机械臂周围无人、喷枪/管线不会被拉扯。

## 常见故障排查

### 服务不存在

现象：

```bash
ros2 service list | grep z1
```

没有输出。

处理：

- 看 `/home/test/logs/tcp_base_ctl_latest/z1.log`，确认 `z1_arm_controller_node` 是否启动。
- 确认已 source `/home/test/z1_move_ws/install/setup.bash`。
- 重新手动启动 `ros2 run z1_arm_controller_cpp z1_arm_controller_node`，观察终端报错。

### z1_ctrl 未连接或机械臂不动

优先看底层日志：

```bash
tail -n 100 /home/test/logs/tcp_base_ctl_latest/z1_ctrl.log
```

检查项：

- `z1_ctrl` 是否正在运行：`pgrep -a z1_ctrl`
- Z1 下位机 IP 是否仍为 `192.168.122.110`
- `z1_controller/config/config.xml` 中 IP/Port 是否为 `192.168.122.110:8881`
- 工控机 `eth1` 是否是 `192.168.122.222/24`
- `ip neigh show dev eth1` 是否能看到 `192.168.122.110`

### /z1_move_in_base_frame 失败

这个服务会把 `base_link` 下的目标转换到 `z1_base`。如果失败，先查 TF：

```bash
ros2 run tf2_ros tf2_echo base_link z1_base
```

没有 TF 时，启动：

```bash
ros2 launch robot_tf_broadcaster tf_publisher.launch
```

### 动作完成标志异常

`z1_arm_controller_node` 会发布：

```bash
ros2 topic echo /z1_complete
```

如果服务调用返回但 `/z1_complete` 不变化，查看 `z1.log`，确认节点是否卡在 SDK 调用或姿态到达判断。

### 自动流程里机械臂不动作

按顺序看三段：

1. `start_all_latest/b2w_navigation.log`：主导航是否调用 `/z1_move_to_target` 或 `/z1_reset_arm`。
2. `tcp_base_ctl_latest/z1.log`：ROS2 机械臂节点是否收到服务请求。
3. `tcp_base_ctl_latest/z1_ctrl.log`：底层 `z1_ctrl` 是否正常连接下位机。

如果第 1 段没调用，问题在主导航状态机或任务点条件；如果第 2 段没收到，问题在 ROS service discovery；如果第 3 段异常，问题在 Z1 底层控制链路。
