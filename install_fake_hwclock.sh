#!/bin/bash
# 一键安装"假硬件时钟"机制：开机恢复时间，关机/定时保存时间。
# 三层防御：拒绝在错时间下安装；save 拒绝写 < 2024 的时间；restore 取多源最大值并校验。
# 使用：sudo bash install_fake_hwclock.sh

set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
    echo "请用 sudo 执行: sudo bash $0" >&2
    exit 1
fi

# 安装时第一道闸：当前时间必须晚于 2026-01-01，否则会把脏值带进 floor 文件
SANITY_EPOCH=1735689600   # 2026-01-01 00:00:00 UTC
NOW_EPOCH=$(date +%s)
if (( NOW_EPOCH < SANITY_EPOCH )); then
    echo "============================================================"
    echo "ERROR: 当前系统时间是 $(date)，明显不正确。"
    echo "请先手动把时间改对，再运行本脚本，否则 floor 会被污染："
    echo "  sudo timedatectl set-ntp false"
    echo "  sudo date -s '2026-05-08 14:00:00'   # 改成真实当前时间"
    echo "  sudo bash $0"
    echo "============================================================"
    exit 1
fi

DATA_FILE=/var/lib/fake-hwclock.data
FLOOR_FILE=/var/lib/fake-hwclock.floor
SAVE_BIN=/usr/local/sbin/fake-hwclock-save
RESTORE_BIN=/usr/local/sbin/fake-hwclock-restore
RESTORE_UNIT=/etc/systemd/system/fake-hwclock-restore.service
SAVE_UNIT=/etc/systemd/system/fake-hwclock-save.service
SAVE_TIMER=/etc/systemd/system/fake-hwclock-save.timer
SHUTDOWN_UNIT=/etc/systemd/system/fake-hwclock-shutdown.service

echo "[1/6] 写入保存脚本: $SAVE_BIN"
cat > "$SAVE_BIN" <<'SAVE_EOF'
#!/bin/bash
# 拒绝在系统时间明显错误时写入数据文件，避免污染恢复链路
DATA_FILE=/var/lib/fake-hwclock.data
SANITY_EPOCH=1735689600   # 2026-01-01

now=$(date +%s)
if (( now < SANITY_EPOCH )); then
    logger -t fake-hwclock "save REJECTED: current time looks wrong ($(date))" 2>/dev/null || true
    exit 0
fi

date +"%Y-%m-%d %H:%M:%S" > "$DATA_FILE"
sync
SAVE_EOF
chmod 755 "$SAVE_BIN"

echo "[2/6] 写入恢复脚本: $RESTORE_BIN"
# 用占位符方式把安装时刻硬编码进恢复脚本，作为最后一层 floor
cat > "$RESTORE_BIN" <<'RESTORE_EOF'
#!/bin/bash
# 三个来源取 MAX：硬编码安装时戳 / floor 文件 / data 文件；< 2026 全部忽略
DATA_FILE=/var/lib/fake-hwclock.data
FLOOR_FILE=/var/lib/fake-hwclock.floor
SANITY_EPOCH=1735689600
HARDCODED_INSTALL_EPOCH=__INSTALL_EPOCH__

target=0
src=""

# 来源 1：脚本内硬编码的安装时戳（即使 /var/lib 全没了也有兜底）
if (( HARDCODED_INSTALL_EPOCH >= SANITY_EPOCH )); then
    target=$HARDCODED_INSTALL_EPOCH
    src="hardcoded"
fi

# 来源 2：floor 文件（安装时写入，正常情况下不变）
if [[ -r "$FLOOR_FILE" ]]; then
    f=$(cat "$FLOOR_FILE" 2>/dev/null || echo 0)
    if [[ "$f" =~ ^[0-9]+$ ]] && (( f >= SANITY_EPOCH )) && (( f > target )); then
        target=$f
        src="floor"
    fi
fi

# 来源 3：data 文件（timer / shutdown 周期更新）
if [[ -r "$DATA_FILE" ]]; then
    saved=$(cat "$DATA_FILE" 2>/dev/null || echo "")
    if [[ -n "$saved" ]]; then
        s=$(date -d "$saved" +%s 2>/dev/null || echo 0)
        if (( s >= SANITY_EPOCH )) && (( s > target )); then
            target=$s
            src="data"
        fi
    fi
fi

if (( target == 0 )); then
    logger -t fake-hwclock "no valid baseline (all sources < 2026), skip" 2>/dev/null || true
    exit 0
fi

now=$(date +%s)
if (( target > now )); then
    /usr/bin/date -s "@$target" >/dev/null
    logger -t fake-hwclock "restored from $src to $(date -d "@$target" '+%F %T') (was $(date -d "@$now" '+%F %T'))" 2>/dev/null || true
else
    logger -t fake-hwclock "current $(date -d "@$now" '+%F %T') >= baseline $(date -d "@$target" '+%F %T') from $src, skip" 2>/dev/null || true
fi
exit 0
RESTORE_EOF
# 把硬编码的安装时戳替换进去
sed -i "s/__INSTALL_EPOCH__/$NOW_EPOCH/" "$RESTORE_BIN"
chmod 755 "$RESTORE_BIN"

echo "[3/6] 重置数据文件和 floor 文件（强制覆盖，清除可能的 1970 污染）"
mkdir -p "$(dirname "$DATA_FILE")"
date +"%Y-%m-%d %H:%M:%S" > "$DATA_FILE"
date +%s > "$FLOOR_FILE"
chmod 644 "$DATA_FILE" "$FLOOR_FILE"
echo "  data  = $(cat "$DATA_FILE")"
echo "  floor = $(cat "$FLOOR_FILE")  ($(date -d "@$(cat "$FLOOR_FILE")" '+%F %T'))"

echo "[4/6] 写入 systemd unit 文件"
cat > "$RESTORE_UNIT" <<'UNIT_EOF'
[Unit]
Description=Restore fake hardware clock at boot
DefaultDependencies=no
After=local-fs.target
Before=sysinit.target time-sync.target
ConditionPathExists=/var/lib/fake-hwclock.data

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/sbin/fake-hwclock-restore

[Install]
WantedBy=sysinit.target
UNIT_EOF

cat > "$SAVE_UNIT" <<'UNIT_EOF'
[Unit]
Description=Save fake hardware clock (timer-triggered)
After=local-fs.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/fake-hwclock-save

[Install]
WantedBy=multi-user.target
UNIT_EOF

cat > "$SHUTDOWN_UNIT" <<'UNIT_EOF'
[Unit]
Description=Save fake hardware clock on shutdown
DefaultDependencies=no
Before=shutdown.target halt.target reboot.target poweroff.target
Requires=local-fs.target
After=local-fs.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/fake-hwclock-save

[Install]
WantedBy=shutdown.target halt.target reboot.target poweroff.target
UNIT_EOF

cat > "$SAVE_TIMER" <<'UNIT_EOF'
[Unit]
Description=Periodically save fake hardware clock

[Timer]
OnBootSec=5min
OnUnitActiveSec=10min
Unit=fake-hwclock-save.service

[Install]
WantedBy=timers.target
UNIT_EOF

echo "[5/6] 启用并立即保存一次"
systemctl daemon-reload
systemctl enable fake-hwclock-restore.service >/dev/null
systemctl enable fake-hwclock-shutdown.service >/dev/null
systemctl enable fake-hwclock-save.timer >/dev/null
systemctl start  fake-hwclock-save.timer
"$SAVE_BIN"

echo "[6/6] 完成"
echo
echo "==================================================="
echo "安装完成。状态如下："
echo "  当前时间    : $(date)"
echo "  data 文件   : $(cat "$DATA_FILE")"
echo "  floor 文件  : $(date -d "@$(cat "$FLOOR_FILE")" '+%F %T')"
echo "  硬编码 floor: $(date -d "@$NOW_EPOCH" '+%F %T')"
echo "==================================================="
echo
echo "验证命令："
echo "  systemctl status fake-hwclock-restore.service"
echo "  journalctl -t fake-hwclock --no-pager"
echo
echo "模拟测试（手动把时间调回 1970，看是否会被恢复）："
echo "  sudo timedatectl set-ntp false"
echo "  sudo date -s '1970-01-01 08:00:00'"
echo "  sudo /usr/local/sbin/fake-hwclock-restore"
echo "  date"
