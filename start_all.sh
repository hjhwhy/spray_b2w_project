#!/bin/bash

mkdir -p logs
PID_FILE="/tmp/start_all.pid"
PGID_FILE="/tmp/start_all.pgid"
READY_FILE="/tmp/start_all.ready"

owns_start_all_group() {
    local pid="$1"
    local expected_pgid="$2"

    if [ -z "$pid" ] || [ -z "$expected_pgid" ]; then
        return 1
    fi
    if ! kill -0 "$pid" 2>/dev/null; then
        return 1
    fi

    local current_pgid
    current_pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    if [ "$current_pgid" != "$expected_pgid" ]; then
        return 1
    fi

    if [ -r "/proc/$pid/cmdline" ] && tr '\0' ' ' < "/proc/$pid/cmdline" | grep -Fq "start_all.sh"; then
        return 0
    fi

    return 1
}

if [ -f "$PID_FILE" ]; then
    existing_pid=$(cat "$PID_FILE" 2>/dev/null || echo "")
    if [ -n "$existing_pid" ] && kill -0 "$existing_pid" 2>/dev/null; then
        echo "start_all.sh 已在运行，PID: $existing_pid"
        exit 0
    fi
    rm -f "$PID_FILE"
fi
rm -f "$READY_FILE"

# 处理上次进程组的残留（基于 PGID 文件，比按全局进程名 pkill -x 更精准；
# 不会误杀手工调试启动的同名节点，也不会碰 tcp_base_ctl.sh 的 b2w_teleop_node / robot_tcp_node，
# 因为它们在 tcp_base_ctl.sh 的进程组里，而本脚本通过 setsid 启动，进程组互不相同）
SELF_PGID=$(ps -o pgid= -p $$ | tr -d ' ')
if [ -f "$PGID_FILE" ]; then
    OLD_PID=""
    OLD_PGID=""
    OLD_TAG=""
    read -r OLD_PID OLD_PGID OLD_TAG < "$PGID_FILE" || true
    rm -f "$PGID_FILE"
    if owns_start_all_group "$OLD_PID" "$OLD_PGID" && [ "$OLD_PGID" != "$SELF_PGID" ]; then
        echo "检测到上次 start_all.sh 进程组残留 PGID=$OLD_PGID，包含进程："
        ps -o pid,pgid,comm -g "$OLD_PGID" 2>/dev/null || true
        echo "  发送 SIGTERM 到进程组 $OLD_PGID"
        kill -TERM -"$OLD_PGID" 2>/dev/null || true
        sleep 2
        if pgrep -g "$OLD_PGID" >/dev/null 2>&1; then
            echo "  PGID=$OLD_PGID 仍残留，强制 SIGKILL"
            kill -KILL -"$OLD_PGID" 2>/dev/null || true
        fi
    elif [ -n "$OLD_PGID" ] && pgrep -g "$OLD_PGID" >/dev/null 2>&1; then
        echo "警告：发现旧 PGID=$OLD_PGID 仍有进程，但无法确认其仍属于上次 start_all.sh；"
        echo "      为避免误杀其他进程组，本次跳过该 PGID 的强制清理。"
        ps -o pid,pgid,comm -g "$OLD_PGID" 2>/dev/null || true
    fi
fi

echo $$ > "$PID_FILE"
printf '%s %s %s\n' "$$" "$SELF_PGID" "start_all.sh" > "$PGID_FILE"
SCRIPT_START_TS=$(date +%s)

cleanup() {
    local exit_code="${1:-0}"
    echo "正在关闭所有节点..."

    for pid in "${PIDS[@]}"; do
        echo "发送 SIGINT 到 PID: $pid"
        kill -INT "$pid" 2>/dev/null
    done

    sleep 1

    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            local term_comm
            term_comm=$(ps -o comm= -p "$pid" 2>/dev/null | tr -d ' ' || echo "?")
            echo "发送 SIGTERM 到 PID: $pid (comm=${term_comm:-?})"
            kill -TERM "$pid" 2>/dev/null
        fi
    done

    sleep 2

    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            local kill_comm
            kill_comm=$(ps -o comm= -p "$pid" 2>/dev/null | tr -d ' ' || echo "?")
            echo "发送 SIGKILL 到 PID: $pid (comm=${kill_comm:-?})"
            kill -KILL "$pid" 2>/dev/null
        fi
    done

    echo "全部已关闭。"
    rm -f "$PID_FILE" "$PGID_FILE" "$READY_FILE"
    exit "$exit_code"
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

echo "================ 手动自动作业模式 ================"
echo "当前脚本将直接启动喷涂主流程，即使 APP 故障也可独立运行。"
echo "目标点文件: ${ROBOT_HOME}/gnss_waypoints.txt"
if [ -f "${ROBOT_HOME}/gnss_waypoints.txt" ]; then
    echo "目标点文件存在。"
else
    echo "警告：目标点文件不存在，主控可能无法正常读取任务点。"
fi

if pgrep -x robot_tcp_node >/dev/null 2>&1; then
    echo "检测到 robot_tcp_node 正在运行：当前为 自动作业 + APP远程可控 模式。"
else
    echo "未检测到 robot_tcp_node：当前为 纯自动作业 模式。"
    echo "说明：此模式下可直接自动执行任务，但 APP 无法下发 start/pause/stop/restart。"
fi
echo "如需恢复 APP 控制，请另外启动: ${ROBOT_HOME}/tcp_base_ctl.sh"
echo "==============================================="

# 用于保存所有后台 PID
PIDS=()

# 注意：残留进程的清理已经通过 PGID 文件机制在脚本头部完成，这里不再按进程名 pkill -x，
# 避免误杀手工启动的同名诊断节点。

# 清理 FastRTPS 残留共享内存：仅在 tcp_base_ctl.sh 的常驻控制链路不在时执行。
# 这里只避开已知必须常驻的 robot_tcp_node / b2w_teleop_node 及其 ros2 run 包装进程，
# 避免误伤运行中的控制链路；同时避免把清理条件放得过宽，导致共享内存残留长期得不到处理。
if pgrep -x robot_tcp_node >/dev/null 2>&1 || \
   pgrep -x b2w_teleop_node >/dev/null 2>&1 || \
   pgrep -f "ros2 run robot_tcp robot_tcp_node" >/dev/null 2>&1 || \
   pgrep -f "ros2 run b2w_navigation_controller b2w_teleop_node" >/dev/null 2>&1; then
    echo "检测到 tcp_base_ctl.sh 常驻控制链路正在运行，跳过 /dev/shm/fastrtps_* 清理（避免误伤运行中的 DDS）"
else
    echo "清理 FastRTPS 残留共享内存..."
    find /dev/shm -maxdepth 1 \( -name 'fastrtps_*' -o -name 'sem.fastrtps_*' \) -delete 2>/dev/null || true
fi
sleep 1

echo "启动 b2w 主控节点"
ros2 launch b2w_navigation_controller b2w_navigation.launch > logs/b2w_navigation.log 2>&1 &
NAV_PID=$!
PIDS+=($!)
sleep 1

echo "start_all.sh PID: $$"
echo "b2w_navigation.launch PID: $NAV_PID"

# 清除旧 probe 日志，避免 else 分支误读上一次的探测结果
rm -f logs/emergency_stop_probe.log

echo "等待 b2w 主控服务就绪..."
service_ready=0
probe_ok=0
service_wait_start_ts=$(date +%s)
attempt=0
for _ in $(seq 1 20); do
    attempt=$((attempt + 1))
    if ros2 service list 2>/dev/null | grep -qx "/emergency_stop" && \
       ros2 service list 2>/dev/null | grep -qx "/erase_emergency_stop"; then
        service_ready=1
        service_wait_end_ts=$(date +%s)
        echo "第 $attempt 次检查检测到 emergency stop 服务，耗时 $((service_wait_end_ts - service_wait_start_ts))s。"
        break
    fi
    sleep 1
done

if [ "$service_ready" -eq 1 ]; then
    echo "b2w 主控服务已就绪。"

    probe_start_ts=$(date +%s)
    timeout 5s ros2 service call /erase_emergency_stop std_srvs/srv/Trigger "{}" \
        > logs/emergency_stop_probe.log 2>&1 || true
    probe_end_ts=$(date +%s)
    if grep -q "Trigger_Response" logs/emergency_stop_probe.log 2>/dev/null; then
        probe_ok=1
        echo "b2w 主控服务探测调用成功，耗时 $((probe_end_ts - probe_start_ts))s。"
        echo "probe 摘要："
        grep -E "waiting for service|requester:|Trigger_Response|success=|message=" logs/emergency_stop_probe.log 2>/dev/null || true
    else
        echo "警告：b2w 主控服务探测调用未拿到标准响应，但不终止主流程。"
        echo "探测调用耗时 $((probe_end_ts - probe_start_ts))s。"
        echo "当前 emergency stop probe 输出："
        tail -n 40 logs/emergency_stop_probe.log 2>/dev/null || true
    fi
else
    if ! kill -0 "$NAV_PID" 2>/dev/null; then
        echo "错误：b2w_navigation.launch 进程已退出，主控不可用，主流程无法执行。"
        echo "最近的 b2w_navigation.log："
        tail -n 80 logs/b2w_navigation.log 2>/dev/null || true
        echo "正在清理已启动的其他节点..."
        cleanup 1
        # cleanup 已 exit，下面不会执行
    fi
    echo "警告：等待 /emergency_stop 和 /erase_emergency_stop 超时，但 b2w_navigation.launch 仍在运行；"
    echo "      service discovery 可能尚未完成，继续运行（不退出）。"
    echo "最近的 b2w_navigation.log："
    tail -n 80 logs/b2w_navigation.log 2>/dev/null || true
fi

echo "start_all 启动总耗时: $(( $(date +%s) - SCRIPT_START_TS ))s"
if [ "$service_ready" -eq 1 ] && [ "$probe_ok" -eq 1 ]; then
    printf 'state=ready\nservice_ready=1\nprobe_ok=1\n' > "$READY_FILE"
    echo "所有节点已启动，pause/restart 服务已验证。"
elif [ "$service_ready" -eq 1 ]; then
    printf 'state=partial\nservice_ready=1\nprobe_ok=0\n' > "$READY_FILE"
    echo "所有节点已启动，但 pause/restart 服务尚未验证通过。"
else
    printf 'state=discovery_pending\nservice_ready=0\nprobe_ok=0\n' > "$READY_FILE"
    echo "所有节点已启动，但 emergency stop 服务尚未完成发现，pause/restart 可能暂不可用。"
fi
wait
