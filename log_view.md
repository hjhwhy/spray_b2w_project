# 日志查看指南

机器人本机所有运行日志统一落在 `/home/test/logs/` 下。本文档面向部署后的日常运维和现场排错。

> 装好 fake-hwclock 之后，所有 RUN_ID 时间戳都准确，可以放心按时间检索。

---

## 一、日志架构总览

```
/home/test/logs/
├── tcp_base_ctl_latest          → 软链，指向最新一次 tcp_base_ctl 启动的日志目录
├── tcp_base_ctl_runs/
│   └── <RUN_ID>/                每次启动一个目录，RUN_ID = YYYYMMDD_HHMMSS_PID
│       ├── tcp_base_ctl.log     脚本框架 + 各节点 PID/启动信息
│       ├── robot_tcp.log        APP TCP 通信节点
│       ├── b2w_teleop.log       手动遥控节点
│       ├── 485.log              RS485 继电器（喷枪）
│       ├── tf_publisher.log     TF 树发布
│       ├── ins_parser.log       司南 RTK ROS 节点 stdout/stderr
│       ├── ins_parser_runs/     司南 RTK 原始 NMEA 流（按 PID 分文件）
│       │   └── gpgga_raw_<时间>_<PID>.log
│       ├── z1_ctrl.log          Z1 底层 controller（UDP 等待 SDK 客户端）
│       └── z1.log               Z1 ROS 节点（z1_arm_controller_node）
│
├── start_all_latest             → 软链，指向最新一次 start_all 主任务的日志目录
└── start_all_runs/
    └── <RUN_ID>/
        ├── b2w_navigation.log       B2W 主控（ros2 launch 全部输出）
        └── emergency_stop_probe.log 启动时的服务发现验证日志
```

两套 runs：

- `tcp_base_ctl_runs/` —— 常驻控制链路（systemd 拉起，APP/遥控/RTK/TF/RS485/Z1 基础节点）
- `start_all_runs/` —— 主任务（喷涂自动作业）

它们独立计数。一次完整作业流程通常对应：1 个 tcp_base_ctl run + 多个 start_all run（每次按 APP 的 0x01 启动一次）。

---

## 二、记住三个入口

```bash
/home/test/logs/tcp_base_ctl_latest/    # 基础控制链路最新日志
/home/test/logs/start_all_latest/       # 主任务最新日志
journalctl -u tcp_base_ctl.service       # systemd 层（service 重启原因）
```

`*_latest` 是软链，每次脚本启动会自动跳到新的 RUN_ID，**不要手动改它们**。

---

## 三、高频场景

### 3.1 现在不对劲，看正在发生什么

```bash
cd /home/test/logs/tcp_base_ctl_latest

# 脚本框架（哪个节点起没起、PID 多少）
tail -n 100 tcp_base_ctl.log

# 同时跟多个关键日志
tail -f tcp_base_ctl.log z1.log z1_ctrl.log

# 主任务侧
tail -f /home/test/logs/start_all_latest/b2w_navigation.log
```

### 3.2 刚才出过一次问题，要回去看那一次

```bash
# 倒序列最近 10 次基础链路启动
ls -lt /home/test/logs/tcp_base_ctl_runs/ | head -11

# 倒序列最近 10 次主任务启动
ls -lt /home/test/logs/start_all_runs/ | head -11

# 进入指定那次
cd /home/test/logs/tcp_base_ctl_runs/20260506_142315_3421/
ls -la
```

### 3.3 service 又自己重启了，看为什么

```bash
# 最近的 systemd 日志（含 ExecStartPre 失败、退出码、重启原因）
journalctl -u tcp_base_ctl.service -n 200 --no-pager

# 本次开机以来
journalctl -u tcp_base_ctl.service -b --no-pager

# 指定时间段
journalctl -u tcp_base_ctl.service --since '2026-05-06 08:00' --until '2026-05-06 09:00'

# 实时跟踪
journalctl -u tcp_base_ctl.service -f
```

---

## 四、按故障类型对应到日志

| 症状 | 主看 | 次看 |
|---|---|---|
| 机械臂不动 | `tcp_base_ctl_latest/z1.log`（节点是否启起来） | `tcp_base_ctl_latest/z1_ctrl.log`（底层 controller、UDP 连接） |
| RTK 没位置 / 跳点 | `tcp_base_ctl_latest/ins_parser.log` | `tcp_base_ctl_latest/ins_parser_runs/gpgga_raw_*.log`（原始 NMEA） |
| APP 连不上 / 9002 报错 | `tcp_base_ctl_latest/robot_tcp.log` | `ss -ltn \| grep 9002` 看端口 |
| 喷枪不喷 | `tcp_base_ctl_latest/485.log` | `start_all_latest/b2w_navigation.log`（是否调用了 `/trigger_valve_ch1`） |
| 路径规划异常 | `start_all_latest/b2w_navigation.log` | path_planner 在主任务日志里输出 |
| 主任务起不来 | `start_all_latest/b2w_navigation.log` | `start_all_latest/emergency_stop_probe.log`（启动期服务验证） |
| 遥控/手柄不响应 | `tcp_base_ctl_latest/b2w_teleop.log` | `tcp_base_ctl_latest/robot_tcp.log`（看 0x04~0x09 命令是否到达） |
| APP 指令发出但车不动 | 见第五章 APP 控制链路实时跟踪 | — |
| TF 树缺帧 | `tcp_base_ctl_latest/tf_publisher.log` | `ros2 run tf2_tools view_frames`（机器上跑） |
| service 频繁重启 | `journalctl -u tcp_base_ctl.service` | 各 run 目录的 `tcp_base_ctl.log` |

---

## 五、APP 控制链路实时跟踪

APP 一次"按下控制键"涉及三个 ROS 节点串联，每段对应一份独立日志。**`tcp_base_ctl.log` 现在只记录脚本启动框架，不再 mirror robot_tcp 的实时输出**——想看 APP 控制实时反应，必须 tail `robot_tcp.log` 和 `b2w_teleop.log` 这两份。

### 5.1 链路图（以"按下前进 0x04"为例）

```
APP
 │ TCP 报文 [0xF5][0x08][len][0x04][...][CRC][0x5F]
 ▼
robot_tcp_node                       ←  robot_tcp.log（看到收到的子命令字节）
 │ 发布 /joy.axes 或 /remote_command 或调 /emergency_stop 服务
 ▼
b2w_teleop_node                      ←  b2w_teleop.log（看到 SportClient::Move(...) 调用）
 │ DDS rt/lowcmd
 ▼
B2W 底盘
```

子命令字节对应：

| 子命令 | 含义 | robot_tcp 发的 ROS 消息 | b2w_teleop 反应 |
|---|---|---|---|
| 0x01 | start 喷涂 | `/remote_command "start"` | （app_node 拉起 start_all.sh） |
| 0x02 | 暂停 | 调用 `/emergency_stop` 服务 | b2w_nav_node 冻结状态机 |
| 0x03 | 停止 | 杀 start_all.sh 进程组 | — |
| 0x04~0x09 | 前后左右 / 左右转 | `/joy.axes` 方向量 | `SportClient::Move(vx,vy,vyaw)` |
| 0x0A | 趴下 | `/joy.buttons` | `StandDown()` |
| 0x0B | 站立 | `/joy.buttons` | `StandUp()` |
| 0x10 | 恢复 | 调用 `/erase_emergency_stop` 服务 | b2w_nav_node 解除暂停 |
| 0xFF | 急停阻尼 | `/joy.buttons` | `Damp()` |

### 5.2 实时跟踪命令

```bash
# 推荐：同时跟 robot_tcp + b2w_teleop，按任何控制键能立刻看到两段反应
tail -F /home/test/logs/tcp_base_ctl_latest/robot_tcp.log \
        /home/test/logs/tcp_base_ctl_latest/b2w_teleop.log

# 或者用 multitail（如果装了）独立分屏
multitail /home/test/logs/tcp_base_ctl_latest/robot_tcp.log \
          /home/test/logs/tcp_base_ctl_latest/b2w_teleop.log
```

`-F` 比 `-f` 更稳：文件被 RUN_ID 切换或 logrotate 时会自动重新打开。

### 5.3 按段排错

按下"前进"但底盘不动时，按这个顺序定位：

**第 1 段：APP → robot_tcp**

`robot_tcp.log` 有新输出？

- 没有 → APP 没连上，或 TCP 链路断了。
  ```bash
  ss -ltn | grep 9002              # 9002 是否 LISTEN
  ss -tn  | grep 9002              # 是否有 ESTABLISHED 连接
  ```
- 有 → 进入第 2 段。

**第 2 段：robot_tcp → b2w_teleop**

`b2w_teleop.log` 有 `SportClient::Move` / `StandUp` / `StandDown` 之类调用？

- 没有 → ROS pub/sub 没通。
  ```bash
  ros2 topic info /joy             # 看发布者和订阅者数量是否都 ≥ 1
  ros2 topic echo /joy             # 实时看 robot_tcp 是否真在发
  echo $ROS_DOMAIN_ID              # 两个节点必须一致
  ```
- 有 → 进入第 3 段。

**第 3 段：b2w_teleop → 底盘**

DDS 链路能否到 B2W：

```bash
ros2 topic echo /imu --once        # B2W 是否在发 IMU
ros2 topic echo /b2w_odom --once   # 是否在发里程计
getcap /home/test/b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_teleop_node
# 应该输出 cap_net_raw=ep；缺了就打不开 eth2
```

都没数据 → 多半是 eth2 网卡 / Unitree DDS NetworkInterface 配错，或者 `cap_net_raw+ep` 丢了（`tcp_base_ctl.service` 的 `ExecStartPre` 里的 setcap 没执行成功）。

---

## 六、ins_parser 两份日志对应起来

ins_parser 的 ROS stdout 在 `tcp_base_ctl_latest/ins_parser.log`，原始 NMEA 流在 `tcp_base_ctl_latest/ins_parser_runs/gpgga_raw_<时间>_<PID>.log`。要对应到同一次启动：

```bash
cd /home/test/logs/tcp_base_ctl_latest

# 拿到本次 ins_parser 的 PID
grep -E "ins_parser pid=" tcp_base_ctl.log
# 输出形如：[2026-05-06 14:23:18] ins_parser pid=12345

# 找对应 PID 的原始流文件
ls ins_parser_runs/ | grep "_12345.log"
```

---

## 七、推荐 alias

写到 `~/.bashrc`，登录后用着方便：

```bash
# 日志快捷入口
alias logs='cd /home/test/logs/tcp_base_ctl_latest'
alias navlogs='cd /home/test/logs/start_all_latest'

# 列最近启动
alias logsls='ls -lt /home/test/logs/tcp_base_ctl_runs/ | head -11'
alias navls='ls -lt /home/test/logs/start_all_runs/ | head -11'

# 一键 tail 关键日志
alias tail-tcp='tail -n 100 -f /home/test/logs/tcp_base_ctl_latest/tcp_base_ctl.log'
alias tail-nav='tail -n 100 -f /home/test/logs/start_all_latest/b2w_navigation.log'
alias tail-z1='tail -n 100 -f /home/test/logs/tcp_base_ctl_latest/z1.log /home/test/logs/tcp_base_ctl_latest/z1_ctrl.log'
alias tail-rtk='tail -n 100 -f /home/test/logs/tcp_base_ctl_latest/ins_parser.log'
alias tail-app='tail -F /home/test/logs/tcp_base_ctl_latest/robot_tcp.log /home/test/logs/tcp_base_ctl_latest/b2w_teleop.log'

# service 状态
alias tcpst='systemctl status tcp_base_ctl.service'
alias tcpjlog='journalctl -u tcp_base_ctl.service -n 200 --no-pager'
alias tcpfollow='journalctl -u tcp_base_ctl.service -f'
```

`source ~/.bashrc` 之后，日常常用的就只剩 `logs`、`tail-z1`、`tail-nav`、`tail-app`、`tcpfollow` 这几个。

---

## 八、远程拉日志回开发机分析

机器上不好用 grep / vim 翻大文件时，整个 run 目录拷回开发机：

```bash
# 在开发机
mkdir -p ./logs/$(date +%F)
scp -r test@<robot-ip>:/home/test/logs/tcp_base_ctl_latest ./logs/$(date +%F)/tcp_base_ctl
scp -r test@<robot-ip>:/home/test/logs/start_all_latest    ./logs/$(date +%F)/start_all
```

`*_latest` 是软链，`scp -r` 默认跟随软链，拷过来的就是实际目录内容。

如果想拷"具体某次 RUN_ID"而不是"当前最新"，先在机器上 `readlink -f` 拿到真实路径：

```bash
ssh test@<robot-ip> 'readlink -f /home/test/logs/tcp_base_ctl_latest'
# 输出形如：/home/test/logs/tcp_base_ctl_runs/20260506_142315_3421
```

---

## 九、定期清理（建议）

`tcp_base_ctl_runs/` 和 `start_all_runs/` 当前**没有自动清理**，长期运行会累积。手动清理一次：

```bash
# 删 7 天前的归档（不影响 _latest 软链指向的当前 run）
find /home/test/logs/tcp_base_ctl_runs -mindepth 1 -maxdepth 1 -type d -mtime +7 -exec rm -rf {} +
find /home/test/logs/start_all_runs    -mindepth 1 -maxdepth 1 -type d -mtime +7 -exec rm -rf {} +
```

需要长期自动化时，可加 systemd-timer 或 cron。

---

## 十、关键约定（不要忘）

- **RUN_ID** 由 `date '+%Y%m%d_%H%M%S'_$$` 生成，含 PID，所以即便时间戳冲突也不会覆盖。
- **`tcp_base_ctl_latest` / `start_all_latest`** 是脚本启动时主动 `ln -sfn` 切换的，反映"最近启动"，不是"最近成功启动"。如果脚本起来就崩了，软链照样跳过去——所以排错时如果发现 latest 里很空，去 `*_runs/` 看上一次。
- **`tcp_base_ctl.log`** 现在只记录脚本框架（启动各节点的"启动 X / pid=Y"那种），**不再**包含 robot_tcp 的全量输出。robot_tcp 自己的输出独立写到 `robot_tcp.log`。
- **`b2w_navigation.log`** 每次 start_all 启动会写到一个新的 RUN_LOG_DIR，**不再覆盖**前一次。历史可在 `start_all_runs/<RUN_ID>/` 下追溯。
