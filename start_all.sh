#!/bin/bash

mkdir -p logs
PID_FILE="/tmp/start_all.pid"

if [ -f "$PID_FILE" ]; then
    existing_pid=$(cat "$PID_FILE")
    if [ -n "$existing_pid" ] && kill -0 "$existing_pid" 2>/dev/null; then
        echo "start_all.sh 已在运行，PID: $existing_pid"
        exit 0
    fi
    rm -f "$PID_FILE"
fi

echo $$ > "$PID_FILE"

cleanup() {
    echo "正在关闭所有节点..."

    for pid in "${PIDS[@]}"; do
        echo "发送 SIGINT 到 PID: $pid"
        kill -INT "$pid" 2>/dev/null
    done

    sleep 1

    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "发送 SIGTERM 到 PID: $pid"
            kill -TERM "$pid" 2>/dev/null
        fi
    done

    echo "全部已关闭。"
    rm -f "$PID_FILE"
    exit 0
}
trap cleanup SIGINT SIGTERM

# 让脚本在当前 shell 中支持 ros2 环境
source /opt/ros/humble/setup.bash

ROBOT_HOME="${ROBOT_HOME:-/home/test}"
WORKSPACES=(
    "$ROBOT_HOME/colcon_ws/install/setup.bash"
    "$ROBOT_HOME/z1_move_ws/install/setup.bash"
    "$ROBOT_HOME/spray_path_planner_ws/install/setup.bash"
    "$ROBOT_HOME/gnss_driver_ws/install/setup.bash"
    "$ROBOT_HOME/b2w_navigation_ws/install/setup.bash"
    "$ROBOT_HOME/rtk_nav_ws/install/setup.bash"
    "$ROBOT_HOME/tf_broadcast_ws/install/setup.bash"
    "$ROBOT_HOME/robose_airy_ws/install/setup.bash"
    "$ROBOT_HOME/app_ws/install/setup.bash"
)

for setup_file in "${WORKSPACES[@]}"; do
    if [ -f "$setup_file" ]; then
        source "$setup_file"
    fi
done

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"

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

echo "启动 z1_ctrl 程序..."
cd /home/test/z1_controller/build/
./z1_ctrl > /home/test/logs/z1_ctrl.log 2>&1 &
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

echo "启动 b2w 主控节点"
ros2 launch b2w_navigation_controller b2w_navigation.launch > logs/b2w_navigation.log 2>&1 &
NAV_PID=$!
PIDS+=($!)
sleep 1

echo "等待 b2w 主控服务就绪..."
service_ready=0
for _ in $(seq 1 20); do
    if ros2 service list 2>/dev/null | grep -qx "/emergency_stop" && \
       ros2 service list 2>/dev/null | grep -qx "/erase_emergency_stop"; then
        if timeout 5s ros2 service call /erase_emergency_stop std_srvs/srv/Trigger "{}" \
            > logs/emergency_stop_probe.log 2>&1; then
            service_ready=1
            break
        fi
    fi
    sleep 1
done

if [ "$service_ready" -eq 1 ]; then
    echo "b2w 主控服务已就绪。"
else
    echo "错误：等待 /emergency_stop 和 /erase_emergency_stop 超时。"
    if ! kill -0 "$NAV_PID" 2>/dev/null; then
        echo "b2w_navigation.launch 进程已退出。"
    else
        echo "b2w_navigation.launch 进程仍在运行，但服务未注册。"
    fi
    echo "当前 emergency stop probe 输出："
    tail -n 40 logs/emergency_stop_probe.log 2>/dev/null || true
    echo "最近的 b2w_navigation.log："
    tail -n 80 logs/b2w_navigation.log 2>/dev/null || true
    cleanup
fi

echo "所有节点已启动！"
wait
