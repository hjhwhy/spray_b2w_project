#!/bin/bash
# 一键安装"假硬件时钟"机制：开机恢复时间，关机/定时保存时间。
# 不依赖外网，不依赖 apt，纯 systemd + bash。
# 使用：sudo bash install_fake_hwclock.sh

set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
    echo "请用 sudo 执行: sudo bash $0" >&2
    exit 1
fi

DATA_FILE=/var/lib/fake-hwclock.data
SAVE_BIN=/usr/local/sbin/fake-hwclock-save
RESTORE_BIN=/usr/local/sbin/fake-hwclock-restore
RESTORE_UNIT=/etc/systemd/system/fake-hwclock-restore.service
SAVE_UNIT=/etc/systemd/system/fake-hwclock-save.service
SAVE_TIMER=/etc/systemd/system/fake-hwclock-save.timer

echo "[1/5] 写入保存脚本: $SAVE_BIN"
cat > "$SAVE_BIN" <<'SAVE_EOF'
#!/bin/bash
DATA_FILE=/var/lib/fake-hwclock.data
date +"%Y-%m-%d %H:%M:%S" > "$DATA_FILE"
sync
SAVE_EOF
chmod 755 "$SAVE_BIN"

echo "[2/5] 写入恢复脚本: $RESTORE_BIN"
cat > "$RESTORE_BIN" <<'RESTORE_EOF'
#!/bin/bash
DATA_FILE=/var/lib/fake-hwclock.data
[[ -r "$DATA_FILE" ]] || exit 0
saved=$(cat "$DATA_FILE")
[[ -n "$saved" ]] || exit 0

saved_epoch=$(date -d "$saved" +%s 2>/dev/null || echo 0)
now_epoch=$(date +%s)

if (( saved_epoch > now_epoch )); then
    /usr/bin/timedatectl set-ntp false 2>/dev/null || true
    /usr/bin/timedatectl set-time "$saved" 2>/dev/null || true
    logger -t fake-hwclock "restored time to $saved (was $(date -d "@$now_epoch"))" 2>/dev/null || true
else
    logger -t fake-hwclock "current time is newer than saved, skip" 2>/dev/null || true
fi
exit 0
RESTORE_EOF
chmod 755 "$RESTORE_BIN"

echo "[3/5] 初始化数据文件: $DATA_FILE"
mkdir -p "$(dirname "$DATA_FILE")"
if [[ ! -f "$DATA_FILE" ]]; then
    date +"%Y-%m-%d %H:%M:%S" > "$DATA_FILE"
fi
chmod 644 "$DATA_FILE"

echo "[4/5] 写入 systemd unit 文件"
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
Description=Save fake hardware clock
DefaultDependencies=no
After=local-fs.target
Before=shutdown.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/sbin/fake-hwclock-save
ExecStop=/usr/local/sbin/fake-hwclock-save

[Install]
WantedBy=shutdown.target
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

echo "[5/5] 启用并立即保存一次"
systemctl daemon-reload
systemctl enable fake-hwclock-restore.service >/dev/null
systemctl enable fake-hwclock-save.service >/dev/null
systemctl enable fake-hwclock-save.timer >/dev/null
systemctl start  fake-hwclock-save.timer
"$SAVE_BIN"

echo
echo "==================================================="
echo "安装完成。当前保存时间："
cat "$DATA_FILE"
echo "==================================================="
echo
echo "验证命令："
echo "  systemctl status fake-hwclock-restore.service"
echo "  systemctl list-timers | grep fake-hwclock"
echo "  journalctl -t fake-hwclock --no-pager"
echo
echo "模拟测试（手动把时间调回 1970，看是否会被恢复）："
echo "  sudo timedatectl set-ntp false"
echo "  sudo timedatectl set-time '1970-01-01 08:00:00'"
echo "  sudo /usr/local/sbin/fake-hwclock-restore"
echo "  date"
