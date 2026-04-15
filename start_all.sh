#!/bin/bash

mkdir -p logs

cleanup() {
    echo "正在关闭所有节点..."

    for pid in "${PIDS[@]}"; do
        echo "关闭 PID: $pid"
        kill $pid 2>/dev/null
    done

    echo "全部已关闭。"
    exit 0
}
trap cleanup SIGINT SIGTERM

# 让脚本在当前 shell 中支持 ros2 环境
source /opt/ros/humble/setup.bash

# 用于保存所有后台 PID
PIDS=()

echo "设置COM2 THS1 为 485 模式..."
sudo /opt/vendor_test/tac3kp_uart_mode_config.sh 485
sleep 1

echo "修改 /dev/ttyTHS1 THS2  权限..."
sudo chmod 777 /dev/ttyTHS1
sudo chmod 777 /dev/ttyTHS2
sleep 1

echo "启动 继电器 .."
ros2 launch rs485_node rs485.launch.py  > logs/485.log 2>&1 	&
PIDS+=($!)
sleep 1

echo "启动 tf_publisher..."
ros2 launch robot_tf_broadcaster tf_publisher.launch &
PIDS+=($!)
sleep 1

echo "启动司南 gnss ..."
ros2 run ins_driver_node ins_parser --ros-args -p port:=/dev/ttyTHS2 -p baudrate:=115200  &
PIDS+=($!)
sleep 1

echo "启动 path_planner..."
ros2 launch spray_path_planner path_planner.launch.py > logs/path_planner.log 2>&1 &
PIDS+=($!)
sleep 1

echo "启动 z1_ctrl 程序..."
cd /home/test/z1_controller/build/
./z1_ctrl &
PIDS+=($!)
sleep 1

echo "启动 z1_arm_controller_node..."
cd /home/test
ros2 run z1_arm_controller_cpp z1_arm_controller_node > logs/z1.log 2>&1 &
PIDS+=($!)
sleep 1

echo "启动 rslidar_sdk..."
ros2 launch rslidar_sdk start.py &
PIDS+=($!)
sleep 1

echo "记录轨迹"
ros2 bag record /b2w_odom &
PIDS+=($!)
sleep 1

echo "所有节点已启动！"
wait

