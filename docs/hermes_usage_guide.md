# Hermes Agent 使用说明

本文面向本仓库的日常开发、审查和排障。Hermes 与 Codex 命令已按当前机器上的 `--help` 核对；Claude Code 命令以目标机器上的 `claude --help` 和官方文档为准。Hermes CLI 与外部编码代理版本变化较快，未覆盖或不确定的参数以 `hermes <command> --help`、`claude --help`、`codex --help` 为准。

## Hermes 是什么

Hermes Agent 是一个支持工具调用的 AI 代理 CLI。它可以读取当前仓库上下文、调用 shell、启用工具 toolsets、加载 skills、管理会话、配置模型 Provider，并通过 profile 隔离不同项目或不同工作方式。

在本 ROS2 机器人项目里，Hermes 适合做这些事：

- 快速理解多工作空间依赖、ROS2 topic/service、启动脚本和现场配置。
- 生成或修改代码、文档、脚本，并让工具执行构建和检查。
- 常见配比是把 Codex 作为写代码代理，把 Claude Code 作为审查代理，形成“实现 + 复核”的闭环。
- 保存会话，针对不同任务使用不同 profile，例如 `ros2-dev`、`review`、`field-debug`。

## 启动方式

在仓库根目录启动：

```bash
cd /home/oneko/projects/spray_b2w_robot_project_greek
hermes
```

显式进入聊天：

```bash
hermes chat
```

单次问题：

```bash
hermes chat -q "阅读 CLAUDE.md，总结本项目启动顺序"
```

只输出最终结果，适合脚本：

```bash
hermes -z "列出本仓库所有 ROS2 工作空间及其依赖关系"
```

启动 TUI：

```bash
hermes --tui
hermes chat --tui
```

二者等价，都是启动 TUI 模式。

继续最近会话或指定会话：

```bash
hermes -c
hermes -c "ros2 navigation debug"
hermes --resume <session_id>
hermes sessions list
hermes sessions browse
```

并行或隔离实验时使用独立 git worktree：

```bash
hermes --worktree
hermes chat --worktree
```

危险命令自动放行：

```bash
hermes --yolo
```

`--yolo` 会绕过危险命令审批，只应在可信、可恢复的环境中使用。机器人现场机器上不建议默认开启。

## 模型 / Provider 配置

交互式选择默认模型和 Provider：

```bash
hermes model
```

本次调用临时指定模型：

```bash
hermes -m anthropic/claude-sonnet-4.6
hermes chat -m anthropic/claude-sonnet-4.6
```

本次调用临时指定 Provider：

```bash
hermes --provider openrouter
hermes chat --provider anthropic
```

同时指定：

```bash
hermes chat --provider anthropic -m anthropic/claude-sonnet-4.6
```

也可以用环境变量覆盖：

```bash
export HERMES_INFERENCE_PROVIDER=anthropic
export HERMES_INFERENCE_MODEL=anthropic/claude-sonnet-4.6
hermes chat
```

登录 Provider：

```bash
hermes login
hermes login --provider nous
hermes login --provider openai-codex
```

管理认证池：

```bash
hermes auth add <provider>
hermes auth list
hermes auth status <provider>
hermes auth reset <provider>
hermes logout
```

配置 fallback provider 链，主模型限流、过载或网络失败时按顺序尝试：

```bash
hermes fallback list
hermes fallback add
hermes fallback remove
hermes fallback clear
```

查看和编辑配置：

```bash
hermes config show
hermes config path
hermes config env-path
hermes config edit
hermes config set model.default anthropic/claude-sonnet-4.6
hermes config check
```

模型名称和 Provider 名称会随安装版本、账号和网关配置变化；不确定时运行：

```bash
hermes model
hermes config show
hermes chat --help
```

## 工具 toolsets

Hermes 的 toolsets 控制代理能用哪些能力。内置 toolset 使用普通名称，例如 `web`、`memory`；MCP 工具使用 `server:tool` 形式，例如 `github:create_issue`。真实可用名称以当前版本 `hermes tools list` 输出为准。

查看工具状态：

```bash
hermes tools list
hermes tools --summary
```

`cli` 是默认平台；如果要检查 Telegram、Discord 等平台的工具配置，再使用 `--platform <platform>`。

启用或禁用工具：

```bash
hermes tools enable web memory
hermes tools disable web
hermes tools enable github:create_issue
hermes tools disable github:create_issue
```

对一次会话临时启用 toolsets：

```bash
hermes chat -t web,memory
hermes -t web,memory -z "查找本项目网络配置相关文档并总结"
```

建议：

- 修改代码时保留 shell/filesystem 相关默认能力，便于读取、构建和测试。
- 需要外部资料时再启用 `web`，机器人现场排障优先读仓库和本机日志。
- 不确定工具名时先运行 `hermes tools list`。

## Skills

Skills 是可安装、可启用的专项能力说明，适合把固定工作流固化下来，例如 ROS2 排障、代码审查、现场部署检查。

常用命令：

```bash
hermes skills list
hermes skills list --enabled-only
hermes skills search ros2
hermes skills inspect <identifier>
hermes skills install <identifier>
hermes skills install <identifier> --yes
hermes skills update
hermes skills check
hermes skills config
```

会话启动时预加载 skill：

```bash
hermes -s <skill_name>
hermes chat -s <skill_name>
hermes chat -s skill_a,skill_b
```

本项目建议沉淀的 skills：

- `ros2-build-order`：固定本仓库各工作空间编译顺序。
- `robot-field-debug`：读取 `CLAUDE.md`、`docs/log_view.md`、`docs/network_topology.md` 后再排障。
- `safety-review`：重点检查底盘控制、喷枪触发、急停、串口权限和网卡配置。

如果 skill 标识符、来源或安装方式不确定，按当前版本 help 为准：

```bash
hermes skills --help
hermes skills install --help
```

## Codex 写代码 + Claude Code 审查流程

推荐的一种配比是把 Codex 用作实现代理，把 Claude Code 用作审查代理。原则是：Codex 只做明确范围的改动，Claude 重点找 bug、回归风险和漏测。

1. 查看当前工作区：

```bash
git status --short
```

2. 让 Codex 在当前仓库实现改动：

```bash
codex
```

或用非交互模式：

```bash
codex exec -C /home/oneko/projects/spray_b2w_robot_project_greek \
  "阅读 CLAUDE.md，只修改与 <任务> 相关的文件；完成后运行必要检查。"
```

限制执行权限的例子：

```bash
codex exec -C /home/oneko/projects/spray_b2w_robot_project_greek \
  -s workspace-write \
  -a on-request \
  "实现 <任务>，不要修改无关文件。"
```

3. 运行项目相关检查。按变更范围选择，不要盲目全量编译现场机器：

```bash
colcon build --packages-select <package_name>
```

跨工作空间依赖按 `CLAUDE.md` 顺序 source 和编译：

```bash
cd colcon_ws && colcon build && source install/setup.bash && cd ..
cd z1_move_ws && colcon build && source install/setup.bash && cd ..
cd spray_path_planner_ws && colcon build && source install/setup.bash && cd ..
cd gnss_driver_ws && colcon build && source install/setup.bash && cd ..
cd b2w_navigation_ws && colcon build && source install/setup.bash && cd ..
```

4. 用 Codex 做本地 review：

```bash
codex review --uncommitted
codex review --base main
codex review --commit <sha>
```

5. 用 Claude Code 做第二轮审查：

```bash
claude
```

非交互审查当前改动：

```bash
claude -p "请以代码审查方式检查当前未提交改动，重点关注 ROS2 接口、线程安全、硬件安全、启动脚本回归和漏测。"
```

如果团队额外安装了 Claude Code review 插件或自定义 wrapper，可按该插件文档使用；不要把本地未安装的插件命令写进固定流程。通用可用方式仍建议使用 `claude -p` 或交互式 `claude`。

6. 根据审查意见修复后再次检查：

```bash
git diff --check
codex review --uncommitted
```

## Profiles

Hermes profile 用于隔离配置、skills、状态和别名。适合区分机器人项目、通用开发、审查和现场排障。

查看、创建和切换：

```bash
hermes profile list
hermes profile create ros2-dev
hermes profile create review --clone
hermes profile use ros2-dev
hermes profile show ros2-dev
```

管理 profile：

```bash
hermes profile rename old_name new_name
hermes profile delete old_name
hermes profile export ros2-dev
hermes profile import <archive>
hermes profile info ros2-dev
```

建议：

- `ros2-dev`：用于日常实现，启用代码、shell、必要 skills。
- `review`：用于审查，减少写入倾向，重点检查 diff。
- `field-debug`：用于现场排障，优先读日志、网络、systemd 和设备权限，不默认改代码。

profile 子命令参数可能随 Hermes 版本变化；不确定时运行：

```bash
hermes profile --help
hermes profile create --help
```

## 常用命令

Hermes：

```bash
hermes --version
hermes status
hermes doctor
hermes doctor --fix
hermes logs
hermes logs -f
hermes logs errors
hermes logs --since 1h
hermes sessions list
hermes sessions rename <id> "ros2 navigation debug"
hermes update
```

Claude Code：

```bash
claude --help
claude
claude -p "总结当前仓库架构"
claude -c
claude -r <session_id>
claude --model sonnet
```

Codex：

```bash
codex --help
codex
codex exec "阅读 CLAUDE.md，总结构建顺序"
codex exec -C /home/oneko/projects/spray_b2w_robot_project_greek "实现 <任务>"
codex review --uncommitted
codex resume --last
```

本项目高频 ROS2 / 系统命令：

```bash
./resettime.sh -4-29
./resettime.sh -4-29 08:30:00
./tcp_base_ctl.sh
./start_all.sh
tail -f /home/test/logs/tcp_base_ctl_latest/tcp_base_ctl.log
tail -f /home/test/logs/tcp_base_ctl_latest/robot_tcp.log
journalctl -u tcp_base_ctl.service -f
sudo systemctl restart tcp_base_ctl.service
sudo setcap cap_net_raw+ep b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_nav_node
sudo setcap cap_net_raw+ep b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_teleop_node
```

## 排障

Hermes 本身：

```bash
hermes doctor
hermes doctor --fix
hermes status
hermes logs errors
hermes logs --level WARNING
hermes logs --session <session_id>
hermes config check
hermes auth list
hermes fallback list
```

常见问题：

- 找不到模型或 Provider：先运行 `hermes model`，再检查 `hermes config show` 和 `hermes auth status <provider>`。
- 工具不可用：运行 `hermes tools list`，确认对应 toolset 或 MCP 工具已启用。
- skill 没加载：运行 `hermes skills list --enabled-only`，或启动时显式加 `-s <skill_name>`。
- 会话上下文混乱：使用 `hermes sessions list` 找到正确会话，或新开 `hermes chat`。
- CLI 参数报错：运行对应 help，例如 `hermes chat --help`，按当前版本 help 为准。

本项目排障优先级：

1. 先读 `CLAUDE.md`，确认启动顺序、topic/service、网络和硬件注意事项。
2. 再读相关文档，例如 `docs/network_topology.md`、`docs/log_view.md`、`docs/mid360_lidar_migration.md`。
3. 先看日志和状态，不急着改代码：

```bash
tail -f /home/test/logs/tcp_base_ctl_latest/tcp_base_ctl.log
tail -f /home/test/logs/tcp_base_ctl_latest/b2w_teleop.log
journalctl -u tcp_base_ctl.service -f
```

4. 检查 DDS raw socket 权限：

```bash
getcap b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_nav_node
getcap b2w_navigation_ws/install/b2w_navigation_controller/lib/b2w_navigation_controller/b2w_teleop_node
```

5. 检查关键网卡和链路。B2W DDS 走 `eth2`，主机静态 IP 应在 `192.168.123.222/24`，狗端 DDS 槽不要随意换：

```bash
ip addr show eth2
ethtool eth2
```

6. 检查 ROS2 接口是否与 `CLAUDE.md` 一致，尤其是 `/epsg_position` 的消息类型。主导航订阅 `geometry_msgs/msg/PoseStamped`，运行主流程应优先使用 `rtk_nav_ws` 的 `ins_parser_node`。

## 本 ROS2 机器人项目推荐工作流

### 新功能或 bugfix

1. 让 Hermes 或 Codex 先读取 `CLAUDE.md` 和相关包代码，明确只改哪些文件。
2. 优先小范围修改：一个 ROS2 包、一个启动脚本或一个文档主题。
3. 按依赖顺序构建受影响工作空间。涉及服务消息时，先编译提供消息的工作空间，再编译依赖方。
4. 改到底盘、喷枪、急停、串口、DDS、机械臂时，必须做审查，不只看能否编译。
5. 用 `codex review --uncommitted` 和 `claude -p` 做复核。

### 现场排障

1. 记录当前时间、网络口、日志路径和正在运行的 systemd 服务。
2. 先查 `tcp_base_ctl.service`、`tcp_base_ctl_latest` 日志、`eth2` 链路、`setcap`、串口权限。
3. 不要在未确认 ready 状态时判断暂停/恢复失败；`pause/restart` 依赖 `/tmp/start_all.ready`。
4. 不要随意更换狗端 DDS 网线槽。现场排查先看 `ethtool eth2` 的 `Link detected`。
5. 需要改代码时，先复现、再改最小范围、再按包构建、最后回看日志。

### 启动链路验证

基础链路：

```bash
./tcp_base_ctl.sh
```

主任务：

```bash
./start_all.sh
```

systemd 自启动链路：

```bash
sudo systemctl restart tcp_base_ctl.service
journalctl -u tcp_base_ctl.service -f
```

APP/遥控器测试建议先启动 `tcp_base_ctl.sh` 或 systemd 服务，再由 APP 发送 `0x01 start`，让 `app_ws` 的 TCP 节点（可执行 `robot_tcp_node` / 节点 `remote_control_node`）拉起 `start_all.sh`。纯自动流程测试可以手动先启动 `tcp_base_ctl.sh`，再执行 `start_all.sh`。

### 提交前检查

```bash
git status --short
git diff --check
codex review --uncommitted
```

根据变更范围选择构建：

```bash
colcon build --packages-select <package_name>
```

如果改动跨工作空间，按 `CLAUDE.md` 的依赖顺序构建，不要省略 `source install/setup.bash`。
