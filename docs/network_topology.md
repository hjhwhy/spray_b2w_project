# 网络拓扑梳理（2026-05-08）

记录工控机上所有接口的真实角色、IP 配置、对端设备，以及"换网口"为何会让狗的 DDS 失联。日后再遇到 `eth1: does not match an available interface` 或换口后控制失灵，先回到本文档定位。

## 一、最终拓扑（健康状态快照）

```
┌─────────────────────────── 工控机（test@tac3000pro-ubuntu）─────────────────────────┐
│                                                                                     │
│  eth0  c4:..:cf            DOWN  无线缆               未使用                        │
│        NM "Wired conn 1"   192.168.1.102/24（残留配置，无影响）                     │
│                                                                                     │
│  eth1  c4:83:72:12:1d:e5   UP   100Mb/s   ───────────► Z1 机械臂下位机              │
│        NM "Wired conn 2"   192.168.122.222/24                  192.168.122.110      │
│                                                                MAC 00:80:fb:5e:ad:49│
│                                                                                     │
│  eth2  c4:83:72:12:1d:e6   UP   1000Mb/s  ───────────► B2W 狗（DDS 通道）           │
│        NM "Wired conn 3"   192.168.123.222/24                  192.168.123.161      │
│                                                                MAC 48:21:0b:3d:4a:2e│
│                                                                                     │
│  wlan0 4c:b7:e0:9a:39:d8   UP   AP        ───────────► APP / 手机                   │
│        netplan 01-wifi-ap  192.168.88.1/24                                          │
│        SSID KifferB2wLocalLAN                                                       │
│                                                                                     │
│  wwan0 f6:a2:f8:78:20:a6   UP   4G        ───────────► 互联网（默认路由）           │
│        NM "Wired conn 4"   192.168.225.150/22 (DHCP)                                │
│                                                                                     │
│  l4tbr0                    DOWN           USB bridge                                │
│        192.168.55.1/24（未连接 host）                                               │
│                                                                                     │
│  can0 / can1               UP   NOARP                                               │
│                                                                                     │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

## 二、关键事实

| 项目 | 值 | 来源 |
|---|---|---|
| Z1 下位机 IP | 192.168.122.110 | `ip neigh show dev eth1` |
| Z1 下位机 MAC | 00:80:fb:5e:ad:49 | ARP |
| **狗 IP** | **192.168.123.161** | `ip neigh show dev eth2` |
| **狗 MAC** | **48:21:0b:3d:4a:2e** | ARP |
| 主机 eth1 IP | 192.168.122.222/24 | `nmcli` "Wired connection 2" |
| 主机 eth2 IP | 192.168.123.222/24 | `nmcli` "Wired connection 3" |
| eth1 速率 | 100 Mbps | `ethtool eth1` |
| eth2 速率 | 1000 Mbps | `ethtool eth2` |

CLAUDE.md 之前只说"B2W DDS 走 eth2"，没记录狗的实际 IP。**狗在 192.168.123.161**，是 Unitree 默认 192.168.123.x 网段中的非典型号（不是常说的 .18 / .161 是它真实分配）。

## 三、IP 配置在哪里（唯一信源）

netplan **只声明 wlan0**（顶部 `renderer: NetworkManager`），所有 eth* 的 IP 都在：

```
/etc/NetworkManager/system-connections/
├── Wired connection 1.nmconnection   eth0  192.168.1.102/24    （线没插，无影响）
├── Wired connection 2.nmconnection   eth1  192.168.122.222/24  Z1
└── Wired connection 3.nmconnection   eth2  192.168.123.222/24  狗
```

每个 `.nmconnection` 都用 `interface-name=ethN` 把配置**绑死在接口名上**，不是绑 MAC。**这就是换网口必坏的根本原因**：换接口名 = 换配置。

## 四、5-8 故障复盘（订正版）

### 4.1 实际根因（用户 5-8 下午确认）

**狗本体上有多个 RJ45 物理插槽，只有一个槽内部布线连到对外的 DDS 通道**。本次操作实际是把狗端那根线从"有效槽"换到了"另一个看起来一样、但内部没接走线的槽"，结果是**整条链路在狗那一头就断了**——和工控机侧用 eth1/eth2 哪个口、NetworkManager 怎么配，完全没关系。

恢复办法：把狗端的网线插回原来那个槽。**狗身上的"对外 DDS 槽"是固定的，不要随便换**。

### 4.2 工控机侧观测到的现象

工控机这一侧用户**没有做任何改动**，eth2 端口的网线一直插着。但故障时观测到：

```
eth1 UP    192.168.122.222/24                       ← Z1 链路一直好的，无关
eth2 DOWN  <NO-CARRIER,BROADCAST,MULTICAST,UP>      ← 线还在，但对端（狗死口）无信号
```

注意 eth2 的标志位是 `NO-CARRIER`，不是没插线——是 IPC 端线缆完好、但狗端那头插到了内部没走线的槽，电气层就没握上手。Unitree SDK 看到 eth2 处于 DOWN，所以报 `eth2: does not match an available interface` 拒绝绑定。这一连串现象都只是物理层断在狗端的下游表现，**没改过 IPC 配置**。

### 4.3 易混淆的 IPC 端通用警示（与 5-8 无关，仅作未来参考）

如果**未来**真要在 IPC 侧把代码里的 `eth2` 改成 `eth1` 之类，会撞上一个独立的坑：NetworkManager 用 `interface-name=ethN` 绑 IP，eth1 上**只有 192.168.122.x 一段**，狗的 192.168.123.x 段不会跟着搬过去。这条已经写进 CLAUDE.md 网络与端口段，**和本次 5-8 故障无关**，不要混进根因分析。

### 4.4 排错时的判别要点

- **物理层先排狗端**：把网线从狗身上拔下来，看 `ethtool eth2 | grep "Link detected"` 是否变 no，再插回去看是否变 yes。如果插回去 link 不上，那就是插错槽了——这就是 5-8 这次的现象。判断标志：`ip -br link` 中 eth2 出现 `NO-CARRIER`，而 IPC 端线缆未动。
- **IP 层再排工控机**：`ip neigh show dev eth2` 若长时间空（没有 192.168.123.161 这条），说明链路虽然 UP 但狗本体没在通讯。
- **DDS 层最后排**：teleop 日志的 `Executed` 字样**不是**狗收到的证据，只能证明本地 SDK 调用返回了。要看狗的物理反应。

## 五、要换网口（譬如把狗从 eth2 改到 eth1），正确的做法

不要只改代码里的接口名，那只是把节点的 socket 绑到新接口，**IP 子网不会跟着搬**。

### 方案 A（推荐，物理层固定）：用 udev / NM 把"狗那根线插的物理口"永久叫 eth2

```bash
# 假设你想把现 eth1 物理口（MAC c4:83:72:12:1d:e5）改名叫 eth2
sudo tee /etc/systemd/network/10-b2w-eth2.link >/dev/null <<'LNK'
[Match]
MACAddress=c4:83:72:12:1d:e5

[Link]
Name=eth2
LNK
sudo update-initramfs -u
sudo reboot
```

代码、netplan、NetworkManager 配置全都不用动。注意要先把现 eth2 改个别名，避免重名冲突。

### 方案 B（软件层搬迁）：把 IP 配置从 eth2 搬到 eth1

```bash
# 1. 改 NM "Wired connection 3" 绑到 eth1（或修改 ipv4 给 "Wired connection 2" 加第二地址）
sudo nmcli connection modify "Wired connection 3" \
    connection.interface-name eth1 \
    ipv4.addresses 192.168.123.222/24

# 2. 让两段 IP 共用 eth1（如果还有别的设备占着 eth1）
sudo nmcli connection modify "Wired connection 2" \
    +ipv4.addresses 192.168.123.222/24

# 3. 改代码硬编码（参考 CLAUDE.md "网络与端口"小节）
#    tcp_base_ctl.sh:369,372
#    b2w_navigation_ws/launch/b2w_navigation.launch:20

# 4. 重新 colcon build + setcap + systemctl restart tcp_base_ctl
```

方案 B 适合临时调试，长期请走方案 A。

## 六、验证清单（换口后必跑）

```bash
# 1. 物理层：哪些口插了线
sudo ethtool eth1 2>/dev/null | grep "Link detected"
sudo ethtool eth2 2>/dev/null | grep "Link detected"

# 2. IP 层：每个接口的实际 IP 和路由
ip -br addr | grep -E '^eth'
ip route | grep '192.168.12'

# 3. 设备发现：狗和 Z1 是不是真的能 ARP 出来
ip neigh show dev eth1   # 应该看到 192.168.122.110（Z1）
ip neigh show dev eth2   # 应该看到 192.168.123.161（狗）

# 4. 应用层：狗能否对 ping 响应
ping -c 3 192.168.123.161
ping -c 3 192.168.122.110

# 5. DDS 层：teleop 实际能不能让狗动（看狗本体反应，不要看 "Executed" 日志）
#    APP 发"站立"，物理观察狗是否真的起立
```

任何一层断了就不要去往上一层找原因。

## 七、关联文件索引

| 路径 | 用途 |
|---|---|
| `/etc/netplan/*.yaml` | wlan0 AP 配置（唯一在 netplan 里的） |
| `/etc/NetworkManager/system-connections/Wired connection 2.nmconnection` | eth1 / Z1 IP |
| `/etc/NetworkManager/system-connections/Wired connection 3.nmconnection` | eth2 / 狗 IP |
| `/etc/NetworkManager/system-connections/Wired connection 1.nmconnection` | eth0 残留，无效 |
| `tcp_base_ctl.sh:369,372` | 启动 b2w_teleop，硬编码接口名 |
| `b2w_navigation_ws/launch/b2w_navigation.launch:20` | 启动 b2w_nav_node，硬编码接口名 |
| `z1_controller/config/config.xml` | Z1 下位机 IP 192.168.122.110 |

