#!/bin/bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_HOME="${ROBOT_HOME:-/home/test}"
LOG_DIR="${ROBOT_HOME}/logs"
RUN_LOG="${LOG_DIR}/tcp_base_ctl.log"
TELEOP_LOG="${LOG_DIR}/b2w_teleop.log"
TCP_LOG="${LOG_DIR}/robot_tcp.log"
TELEOP_REL="b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_teleop_node"
TCP_REL="app_ws/install/robot_tcp/lib/robot_tcp/robot_tcp_node"
TELEOP_BIN=""
TCP_BIN=""

for candidate in \
    "$ROBOT_HOME/$TELEOP_REL" \
    "$SCRIPT_DIR/$TELEOP_REL"
do
    if [[ -x "$candidate" ]]; then
        TELEOP_BIN="$candidate"
        break
    fi
done

for candidate in \
    "$ROBOT_HOME/$TCP_REL" \
    "$SCRIPT_DIR/$TCP_REL"
do
    if [[ -x "$candidate" ]]; then
        TCP_BIN="$candidate"
        break
    fi
done

cleanup() {
    echo "[$(date '+%F %T')] tcp_base_ctl cleanup triggered"
    if [[ -f /tmp/start_all.pid ]]; then
        local start_all_pid=""
        start_all_pid="$(cat /tmp/start_all.pid 2>/dev/null || true)"
        if [[ -n "$start_all_pid" ]] && kill -0 "$start_all_pid" 2>/dev/null; then
            echo "检测到主流程仍在运行，先停止 start_all.sh 进程组..."
            kill -TERM -"$start_all_pid" 2>/dev/null || true
            sleep 1
        fi
    fi
    if [[ -n "${TCP_PID:-}" ]] && kill -0 "$TCP_PID" 2>/dev/null; then
        echo "停止 app 通讯..."
        kill -TERM "$TCP_PID" 2>/dev/null || true
        wait "$TCP_PID" 2>/dev/null || true
    fi
    if [[ -n "${TELEOP_PID:-}" ]] && kill -0 "$TELEOP_PID" 2>/dev/null; then
        echo "停止 app 底盘遥控..."
        kill -TERM "$TELEOP_PID" 2>/dev/null || true
        wait "$TELEOP_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT SIGINT SIGTERM

mkdir -p "$LOG_DIR"
touch "$RUN_LOG" "$TELEOP_LOG" "$TCP_LOG"

exec > >(tee -a "$RUN_LOG") 2>&1

echo "[$(date '+%F %T')] tcp_base_ctl starting"
echo "[$(date '+%F %T')] ROBOT_HOME=$ROBOT_HOME"
echo "[$(date '+%F %T')] LOG_DIR=$LOG_DIR"

source /opt/ros/humble/setup.bash

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
    if [[ -f "$setup_file" ]]; then
        source "$setup_file"
    fi
done

set -u

if [[ -z "$TELEOP_BIN" ]]; then
    echo "未找到 b2w_teleop_node，可检查以下路径：" >&2
    echo "  $ROBOT_HOME/$TELEOP_REL" >&2
    echo "  $SCRIPT_DIR/$TELEOP_REL" >&2
    exit 1
fi

if [[ -z "$TCP_BIN" ]]; then
    echo "未找到 robot_tcp_node，可检查以下路径：" >&2
    echo "  $ROBOT_HOME/$TCP_REL" >&2
    echo "  $SCRIPT_DIR/$TCP_REL" >&2
    exit 1
fi

echo "[$(date '+%F %T')] TELEOP_BIN=$TELEOP_BIN"
echo "[$(date '+%F %T')] TCP_BIN=$TCP_BIN"

ensure_teleop_capability() {
    local current_caps=""
    current_caps="$(/usr/sbin/getcap "$TELEOP_BIN" 2>/dev/null || true)"
    if [[ "$current_caps" == *"cap_net_raw=ep"* ]]; then
        return 0
    fi

    if [[ "$(id -u)" -eq 0 ]]; then
        /usr/sbin/setcap cap_net_raw+ep "$TELEOP_BIN"
        return 0
    fi

    echo "检测到 $TELEOP_BIN 尚未具备 cap_net_raw+ep，正在尝试通过 sudo 自动设置..."
    if sudo /usr/sbin/setcap cap_net_raw+ep "$TELEOP_BIN"; then
        return 0
    fi

    echo "自动设置 cap_net_raw+ep 失败。" >&2
    echo "请手动执行：" >&2
    echo "  sudo /usr/sbin/setcap cap_net_raw+ep \"$TELEOP_BIN\"" >&2
    exit 1
}

print_tcp_cleanup_diagnostics() {
    echo "robot_tcp_node 相关进程："
    pgrep -a -x robot_tcp_node 2>/dev/null || echo "  无 robot_tcp_node 进程"

    echo "ros2 run robot_tcp 包装进程："
    pgrep -a -f "ros2 run robot_tcp robot_tcp_node" 2>/dev/null || echo "  无 ros2 run robot_tcp 包装进程"

    echo "9002 端口监听情况："
    ss -ltn 2>/dev/null | grep ':9002 ' || echo "  9002 未处于 LISTEN"

    if command -v fuser >/dev/null 2>&1; then
        echo "9002 端口占用 PID："
        fuser -n tcp 9002 2>/dev/null || echo "  无法通过 fuser 获取，或 9002 未占用"
        if [[ "$(id -u)" -ne 0 ]]; then
            echo "9002 端口占用 PID（sudo）："
            sudo fuser -n tcp 9002 2>/dev/null || echo "  sudo fuser 仍无法获取，或 9002 未占用"
        fi
    fi
}

cleanup_stale_processes() {
    local names=(
        "robot_tcp_node"
        "b2w_teleop_node"
    )
    local legacy_patterns=(
        "ros2 run robot_tcp robot_tcp_node"
        "ros2 run b2w_navigation_controller b2w_teleop_node"
    )

    for name in "${names[@]}"; do
        if pgrep -x "$name" >/dev/null 2>&1; then
            echo "检测到旧进程，正在停止: $name"
            pkill -TERM -x "$name" 2>/dev/null || true
        fi
    done

    for pattern in "${legacy_patterns[@]}"; do
        if pgrep -f "$pattern" >/dev/null 2>&1; then
            echo "检测到旧包装进程，正在停止: $pattern"
            pkill -TERM -f "$pattern" 2>/dev/null || true
        fi
    done

    local port_pids=""
    local can_query_port_pids=0
    if ss -ltnp 2>/dev/null | grep -q 'pid='; then
        can_query_port_pids=1
    fi

    if [[ "$can_query_port_pids" -eq 1 ]]; then
        port_pids="$(ss -ltnp 2>/dev/null | awk '/:9002 / {print $NF}' | grep -o 'pid=[0-9]\+' | cut -d= -f2 | sort -u || true)"
    fi
    for pid in $port_pids; do
        if [[ -n "$pid" ]]; then
            echo "检测到 9002 端口占用，正在停止 PID: $pid"
            kill -TERM "$pid" 2>/dev/null || true
        fi
    done

    if ss -ltn 2>/dev/null | grep -q ':9002 '; then
        if [[ "$(id -u)" -eq 0 ]]; then
            fuser -k -TERM 9002/tcp 2>/dev/null || true
        else
            sudo fuser -k -TERM 9002/tcp 2>/dev/null || true
        fi
    fi

    sleep 1

    for name in "${names[@]}"; do
        if pgrep -x "$name" >/dev/null 2>&1; then
            echo "旧进程仍存在，强制停止: $name"
            pkill -KILL -x "$name" 2>/dev/null || true
        fi
    done

    for pattern in "${legacy_patterns[@]}"; do
        if pgrep -f "$pattern" >/dev/null 2>&1; then
            echo "旧包装进程仍存在，强制停止: $pattern"
            pkill -KILL -f "$pattern" 2>/dev/null || true
        fi
    done

    if [[ "$can_query_port_pids" -eq 1 ]]; then
        port_pids="$(ss -ltnp 2>/dev/null | awk '/:9002 / {print $NF}' | grep -o 'pid=[0-9]\+' | cut -d= -f2 | sort -u || true)"
    else
        port_pids=""
    fi
    for pid in $port_pids; do
        if [[ -n "$pid" ]]; then
            echo "9002 端口仍被占用，强制停止 PID: $pid"
            kill -KILL "$pid" 2>/dev/null || true
        fi
    done

    if ss -ltn 2>/dev/null | grep -q ':9002 '; then
        if [[ "$(id -u)" -eq 0 ]]; then
            fuser -k -KILL 9002/tcp 2>/dev/null || true
        else
            sudo fuser -k -KILL 9002/tcp 2>/dev/null || true
        fi
    fi

    find /dev/shm -maxdepth 1 \( -name 'fastrtps_port*' -o -name 'fastrtps_*' \) -print -delete 2>/dev/null || true

    for _ in $(seq 1 20); do
        if ! pgrep -x robot_tcp_node >/dev/null 2>&1; then
            if [[ "$can_query_port_pids" -eq 0 ]]; then
                if ! ss -ltn 2>/dev/null | grep -q ':9002 '; then
                    return 0
                fi
            else
                if ! ss -ltnp 2>/dev/null | grep -q ':9002 '; then
                    return 0
                fi
            fi
        fi
        sleep 0.5
    done

    echo "9002 端口或 robot_tcp_node 残留进程仍未释放，请手动检查。" >&2
    print_tcp_cleanup_diagnostics
    exit 1
}

ensure_teleop_capability
cleanup_stale_processes

echo "启动 app 底盘遥控"
"$TELEOP_BIN" eth2 >>"$TELEOP_LOG" 2>&1 &
TELEOP_PID=$!
echo "[$(date '+%F %T')] b2w_teleop_node pid=$TELEOP_PID"

echo "启动 app 通讯"
"$TCP_BIN" > >(tee -a "$TCP_LOG") 2>&1 &
TCP_PID=$!
echo "[$(date '+%F %T')] robot_tcp_node pid=$TCP_PID"

wait "$TCP_PID"
