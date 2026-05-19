# 5-19 机器备份代码与当前项目源码对比报告

更新时间：2026-05-19

## 0. 本次更新说明

根据最新同步状态，本报告已更新以下结论：

1. 机器备份中的 `tcp_base_ctl.sh` 已经把 `eth22` 修正为 `eth2`。
   - 当前项目和备份目录现在一致。
   - 该文件不再列入“路径相同但内容不同”。

2. 当前项目中的 `z1_controller/config/config.xml` 已经把 Z1 负载改回 `1.0`。
   - 当前项目和备份目录现在一致。
   - 该文件不再列入“路径相同但内容不同”。

3. 备份中的部分点位/文档已归档到当前项目 `docs/dog_backup/` 下。
   - 例如 `5-7/test2`、`5-7/test4`、`5-7/test5`、`waypoints_backup/5-6`、根目录点位备份等。
   - 它们在“备份目录独有”中仍会按原路径显示，因为路径不同；但已在“已归档到当前项目的备份文件”中单独标注。

4. 本报告文件自身 `docs/source_compare_5-19.md` 未计入统计，避免影响当前项目独有文件数量。

## 1. 对比范围

当前项目目录：

```text
/home/oneko/projects/spray_b2w_robot_project_greek
```

机器备份目录：

```text
/home/oneko/文档/希腊材料/kiefer/projects/backup/5-19
```

说明：用户原始输入路径为 `ome/oneko/...`，实际存在路径为 `/home/oneko/...`，本报告按实际存在路径对比。

本次对比排除了以下构建/缓存目录：

```text
.git
build
install
log
.vscode
.idea
__pycache__
.cache
```

纳入对比的文件类型包括：

```text
.c .cc .cpp .cxx
.h .hh .hpp .hxx
.py .sh .bash
.launch .xml .yaml .yml
.srv .msg .idl
.cmake CMakeLists.txt package.xml Makefile
.txt .md .urdf .xacro .json .toml
```

## 2. 总体统计

| 类别 | 数量 |
|---|---:|
| 当前项目纳入对比文件 | 1175 |
| 备份目录纳入对比文件 | 1149 |
| 两边路径相同的文件 | 1122 |
| 路径相同但内容不同 | 13 |
| 当前项目独有 | 53 |
| 备份目录独有 | 27 |

和上一版报告相比：

| 项目 | 上一版 | 当前 |
|---|---:|---:|
| 路径相同但内容不同 | 15 | 13 |
| 当前项目独有 | 43 | 53 |
| 备份目录独有 | 27 | 27 |

变化原因：

- `tcp_base_ctl.sh` 已统一为 `eth2`，不再不同。
- `z1_controller/config/config.xml` 已统一为 `<load>1.0</load>`，不再不同。
- 当前项目新增了从备份归档进来的 `docs/dog_backup/*` 文件。

## 3. 路径相同但内容不同的文件

以下文件在当前项目和备份目录中路径相同，但内容仍不同。

| 序号 | 文件 | 差异性质 | 建议 |
|---:|---|---|---|
| 1 | `app_ws/test.py` | APP 测试协议从 2 字节指令改为 1 字节指令 | 保留当前版本，更符合现有 APP 协议 |
| 2 | `gnss_waypoints.txt` | 坐标一致，仅点名从 `pt1..pt101` 改为 `task_new..task_new100` | 点名体系不影响系统算法执行，无需作为统一重点 |
| 3 | `record_epsg_waypoint.py` | 仅末尾换行差异 | 可忽略 |
| 4 | `robose_airy_ws/src/rslidar_sdk/src/source/source_pointcloud_ros.hpp` | LaserScan 过滤参数不同 | 需要结合避障效果确认 |
| 5 | `rtk_nav_ws/fj_dynamic/improve/precision_roadmap.md` | 精度路线图文档更新 | 保留当前文档更完整 |
| 6 | `rtk_nav_ws/fj_dynamic/improve/test-100/README.md` | 100 点说明扩展为 100/101 点 | 保留当前文档更符合现有点位 |
| 7 | `tf_broadcast_ws/urdf/my_robot.urdf` | RTK 外参不同 | 需要统一到当前标定值或现场实测值 |
| 8 | `unitree_sdk2/example/b2w/CMakeLists.txt` | 当前不再编译 `b2w_move_to_xy` 示例 | 通常无影响 |
| 9 | `unitree_sdk2/example/g1/CMakeLists.txt` | 当前启用更多 G1 示例 | 影响 SDK example 编译范围 |
| 10 | `unitree_sdk2/example/go2/CMakeLists.txt` | 当前启用更多 Go2 示例 | 影响 SDK example 编译范围 |
| 11 | `unitree_sdk2/example/go2w/CMakeLists.txt` | 当前启用 Go2W 示例 | 影响 SDK example 编译范围 |
| 12 | `unitree_sdk2/example/h1/CMakeLists.txt` | 当前启用更多 H1 示例 | 影响 SDK example 编译范围 |
| 13 | `z1_move_ws/src/z1_arm_controller_node.cpp` | 删除两条日志 | 可忽略 |

### 3.1 已不再不同的文件

以下两个文件上一版报告中列为不同，目前已经统一：

#### `tcp_base_ctl.sh`

当前项目：

```bash
"$TELEOP_BIN" eth2 --ros-args --params-file "$TELEOP_PARAMS_FILE" >>"$TELEOP_LOG" 2>&1 &
"$TELEOP_BIN" eth2 >>"$TELEOP_LOG" 2>&1 &
```

备份目录：

```bash
"$TELEOP_BIN" eth2 --ros-args --params-file "$TELEOP_PARAMS_FILE" >>"$TELEOP_LOG" 2>&1 &
"$TELEOP_BIN" eth2 >>"$TELEOP_LOG" 2>&1 &
```

结论：已统一为 `eth2`，无需再处理。

#### `z1_controller/config/config.xml`

当前项目：

```xml
<load>1.0</load><!-- kg-->
```

备份目录：

```xml
<load>1.0</load><!-- kg-->
```

结论：已统一为 `1.0 kg`，无需再处理。

### 3.2 `app_ws/test.py`

备份版本：

```python
packet.append(0x08)               # func_code (node ONLY supports 0x01)
packet.extend([0x02, 0x00])       # data_len = 2 bytes, little-endian
packet.extend(cmd_type.to_bytes(2, 'little'))  # instruction type
```

当前版本：

```python
packet.append(0x08)               # func_code
packet.extend([0x01, 0x00])       # data_len = 1 byte (little-endian)
packet.extend(cmd_type.to_bytes(1, 'little'))  # 1-byte instruction type
```

当前版本还把示例从 `send_command(0x0005)` 改为：

```python
send_command(0x05)
```

结论：当前版本更符合现有 APP 控制协议，即指令为 1 字节。

统一建议：保留当前版本。

### 3.3 `gnss_waypoints.txt`

两边均为 101 行，坐标和高程一致，差异主要是点名。

备份开头/结尾：

```text
pt1,481597.843,4210291.194,183.382,
pt2,481597.794,4210293.029,183.386,
pt3,481598.423,4210294.017,183.386,
...
pt99,481564.786,4210307.583,183.640,
pt100,481565.760,4210306.936,183.608,
pt101,481567.643,4210305.925,183.618,
```

当前开头/结尾：

```text
task_new,481597.843,4210291.194,183.382,
task_new1,481597.794,4210293.029,183.386,
task_new2,481598.423,4210294.017,183.386,
...
task_new98,481564.786,4210307.583,183.640,
task_new99,481565.760,4210306.936,183.608,
task_new100,481567.643,4210305.925,183.618,
```

结论：这是点位 ID 命名差异，不是坐标差异；点名体系不影响系统算法执行。

统一建议：

- 不需要因为系统运行而统一点名。
- 当前差异可保留，仅在做人工分析、复测配对或文档归档时注意同一批数据内部点号对应关系。

### 3.4 `record_epsg_waypoint.py`

差异：仅文件末尾是否有最后一个换行符。

```text
备份：3843 bytes，末尾有换行
当前：3842 bytes，末尾无换行
```

结论：逻辑无差异，不影响运行。

统一建议：可忽略；若要规范，可给当前文件补末尾换行。

### 3.5 `robose_airy_ws/src/rslidar_sdk/src/source/source_pointcloud_ros.hpp`

备份版本：

```cpp
const double z_min = -0.1;
const double z_max = 0.06;
const float range_min = 0.4f;
const float range_max = 5.0f;
```

当前版本：

```cpp
const double z_min = -0.18;
const double z_max = 0.06;
const float range_min = 0.3f;
const float range_max = 5.0f;
```

影响：

- 当前版本允许更低的点进入 LaserScan：`z_min -0.18`。
- 当前版本允许更近的点进入 LaserScan：`range_min 0.3`。
- 当前避障可能更敏感，但也可能带入更多近距离噪声或地面点。

统一建议：

- 若现场近距离障碍检测不足，保留当前版本。
- 若出现地面噪声、误触发避障，可考虑回退到备份参数或重新现场标定。

### 3.6 `tf_broadcast_ws/urdf/my_robot.urdf`

备份版本：

```xml
<origin xyz="-0.3485 0.0 0.30" rpy="0 0 0"/>
```

当前版本：

```xml
<origin xyz="-0.4477 0.0 0.10" rpy="0 0 0"/>
```

影响：

- RTK 天线相对 `base_link` 的 x 偏移从 `-0.3485` 改为 `-0.4477`。
- RTK 天线高度从 `0.30` 改为 `0.10`。
- 这会影响 TF 树和基于 RTK 的 base_link 推算。

统一建议：

- 当前项目 CLAUDE.md 中也记录主导航 `rtk_x_offset` 默认 `-0.4477`。
- 若这是 5-7 后标定结果，建议保留当前版本。
- 若现场机械安装已变化，应重新实测 RTK 天线到 base_link 的外参。

### 3.7 `z1_move_ws/src/z1_arm_controller_node.cpp`

备份版本多两条日志：

```cpp
RCLCPP_INFO(this->get_logger(),"start FORWARD");
arm_.labelRun("forward");
RCLCPP_INFO(this->get_logger(),"forward finished");
```

当前版本：

```cpp
arm_.labelRun("forward");
```

结论：只是日志差异，不影响控制逻辑。

统一建议：保留当前版本即可；如果现场调试需要更详细日志，可以恢复这两条日志。

### 3.8 Unitree SDK example CMake 差异

涉及文件：

```text
unitree_sdk2/example/b2w/CMakeLists.txt
unitree_sdk2/example/g1/CMakeLists.txt
unitree_sdk2/example/go2/CMakeLists.txt
unitree_sdk2/example/go2w/CMakeLists.txt
unitree_sdk2/example/h1/CMakeLists.txt
```

主要差异：

- 当前版本启用了更多 Unitree 官方 example 的编译。
- 备份版本很多 example 被 `##` 注释。
- 当前 `b2w/CMakeLists.txt` 删除了 `b2w_move_to_xy` 示例编译。

影响：

- 影响 `unitree_sdk2` 的 example 编译范围。
- 可能引入 Boost、yaml-cpp 等额外依赖。
- 一般不影响当前项目主导航，因为主导航在 `b2w_navigation_ws`。

统一建议：

- 如果目标是稳定构建生产链路，建议只保留必要 B2W example，减少无关 example 编译。
- 如果目标是保留 Unitree SDK 原始示例能力，可以保留当前启用状态。

## 4. 当前项目独有文件

这些文件在当前项目中存在，但备份目录中没有。

### 4.1 当前项目独有：源码/配置类

```text
01-wifi-ap.yaml
app_ws/publish_acquired_points.py
colcon_ws/src/nmea_msgs/CMakeLists.txt
colcon_ws/src/nmea_msgs/msg/Gpgga.msg
colcon_ws/src/nmea_msgs/msg/Gpgsa.msg
colcon_ws/src/nmea_msgs/msg/Gpgst.msg
colcon_ws/src/nmea_msgs/msg/Gpgsv.msg
colcon_ws/src/nmea_msgs/msg/GpgsvSatellite.msg
colcon_ws/src/nmea_msgs/msg/Gphdt.msg
colcon_ws/src/nmea_msgs/msg/Gprmc.msg
colcon_ws/src/nmea_msgs/msg/Gpvtg.msg
colcon_ws/src/nmea_msgs/msg/Gpzda.msg
colcon_ws/src/nmea_msgs/msg/Sentence.msg
colcon_ws/src/nmea_msgs/package.xml
compute_fj_plane.py
install_fake_hwclock.sh
rs585_ws/CMakeLists.txt
rs585_ws/config/rs485_params.yaml
rs585_ws/launch/rs485.launch.py
rs585_ws/package.xml
rs585_ws/src/rs485_node.cpp
simulate_gnss_serial.py
spray_path_planner_ws/CMakeLists.txt
spray_path_planner_ws/package.xml
spray_path_planner_ws/src/spray_path_planner_node.cpp
tests/test_app_node_failsafe.py
```

重点说明：

#### `colcon_ws/src/nmea_msgs/*`

当前项目包含完整 `nmea_msgs` 包。`gnss_driver_ws` 依赖这个包。

统一建议：保留当前项目版本。备份目录缺少它，不适合作为完整可编译工程直接使用。

#### `spray_path_planner_ws/*`

当前项目包含路径规划服务包，提供：

```text
/get_next_waypoint
/set_start_point
```

`b2w_navigation_ws` 依赖该工作空间。

统一建议：保留当前项目版本。备份目录缺少它，会影响主导航完整编译/运行。

#### `rs585_ws/*`

当前项目为 `rs585_ws`，备份目录对应为 `rs485_ws`。

我已比对以下对应文件，内容一致：

| 备份路径 | 当前路径 | 内容 |
|---|---|---|
| `rs485_ws/CMakeLists.txt` | `rs585_ws/CMakeLists.txt` | 一致 |
| `rs485_ws/package.xml` | `rs585_ws/package.xml` | 一致 |
| `rs485_ws/launch/rs485.launch.py` | `rs585_ws/launch/rs485.launch.py` | 一致 |
| `rs485_ws/config/rs485_params.yaml` | `rs585_ws/config/rs485_params.yaml` | 一致 |
| `rs485_ws/src/rs485_node.cpp` | `rs585_ws/src/rs485_node.cpp` | 一致 |

统一建议：只需要决定目录名。当前 CLAUDE.md 和工程使用 `rs585_ws`，建议保留当前目录名。

### 4.2 当前项目独有：文档/数据类

```text
CLAUDE.md
docs/gnss_waypoints_back.txt
docs/hermes_usage_guide.md
docs/log_view.md
docs/mid360_lidar_migration.md
docs/network_topology.md
livox_mid360_ws/src/编译和运行命令.txt
points_test_mikinwn.txt
rtk_nav_ws/fj_dynamic/improve/test-100/5-13/analysis.md
rtk_nav_ws/fj_dynamic/improve/test-100/5-13/fj/dog2026051301_20260513155712_035830.txt
rtk_nav_ws/fj_dynamic/improve/test-100/5-13/fj/dog2026051301_20260513155733_035835.txt
rtk_nav_ws/fj_dynamic/improve/test-100/5-14/analysis.md
rtk_nav_ws/fj_dynamic/improve/test-100/5-14/fj/dogtest_1_023920.txt
rtk_nav_ws/fj_dynamic/improve/test-100/5-14/fj/dogtest_20260514143428_023725.txt
rtk_nav_ws/issue.md
rtk_nav_ws/issue.txt
z1_move_ws/README.md
```

统一建议：这些文档和测试数据体现当前项目的维护状态，建议保留当前版本。

### 4.3 已从备份归档到当前项目 `docs/dog_backup/` 的文件

以下文件在备份中仍按原路径存在；当前项目已另存到 `docs/dog_backup/` 下，因此“路径”不同但“内容”已归档。

| 备份路径 | 当前项目归档路径 |
|---|---|
| `5-7/test2/gnss_waypoints.txt` | `docs/dog_backup/5-7/test2/gnss_waypoints.txt` |
| `5-7/test4/gnss_waypoints.txt` | `docs/dog_backup/5-7/test4/gnss_waypoints.txt` |
| `5-7/test4/gnss_waypoints_detail.txt` | `docs/dog_backup/5-7/test4/gnss_waypoints_detail.txt` |
| `5-7/test5/gnss_waypoints.txt` | `docs/dog_backup/5-7/test5/gnss_waypoints.txt` |
| `gnss_waypoints_back.txt` | `docs/dog_backup/gnss_waypoints_back.txt`，另有 `docs/gnss_waypoints_back.txt` |
| `gnss_waypoints_detail.txt` | `docs/dog_backup/gnss_waypoints_detail.txt` |
| `gnss_waypoints_pt1.txt` | `docs/dog_backup/gnss_waypoints_pt1.txt` |
| `waypoints_backup/5-6/gnss_waypoints.txt` | `docs/dog_backup/waypoints_backup/5-6/gnss_waypoints.txt` |
| `waypoints_backup/5-6/gnss_waypoints_detail.txt` | `docs/dog_backup/waypoints_backup/5-6/gnss_waypoints_detail.txt` |

当前项目还新增了：

```text
docs/dog_backup/gnss_waypoints.txt
```

说明：该文件在备份独有清单中没有同名根路径对应项，可能是从当前/备份点位再归档出来的总表，后续统一点位时需要人工确认来源。

## 5. 备份目录独有文件

这些文件在备份目录存在，但当前项目相同路径下没有。

注意：其中一部分已经按新路径归档到 `docs/dog_backup/`，详见上一节。

### 5.1 备份目录独有：源码/配置类

```text
app_ws/app_node.cpp
robose_airy_ws/src/rslidar_sdk/src/rs_driver/cmake/rs_driverConfig.cmake
robose_airy_ws/src/rslidar_sdk/src/rs_driver/cmake/rs_driverConfigVersion.cmake
rs485_ws/CMakeLists.txt
rs485_ws/config/rs485_params.yaml
rs485_ws/launch/rs485.launch.py
rs485_ws/package.xml
rs485_ws/src/rs485_node.cpp
rtk_nav_ws/fj_dynamic/improve/calibration_results/rtk_offset_calib_20260507_143128.json
unitree_sdk2/example/b2w/b2w_localization_node.cpp
unitree_sdk2/example/b2w/b2w_move_to_xy.cpp
unitree_sdk2/example/b2w/b2w_navigation_controller/CMakeLists.txt
unitree_sdk2/example/b2w/b2w_navigation_controller/package.xml
unitree_sdk2/example/b2w/b2w_navigation_controller/src/main.cpp
unitree_sdk2/ros2_ws/src/b2w_navigation/CMakeLists.txt
unitree_sdk2/ros2_ws/src/b2w_navigation/package.xml
unitree_sdk2/ros2_ws/src/b2w_navigation/src/b2w_navigation_node.cpp
```

#### `app_ws/app_node.cpp`

备份目录有旧路径：

```text
app_ws/app_node.cpp
```

当前项目没有这个路径。

但备份目录和当前项目都有正式路径：

```text
app_ws/src/app_node.cpp
```

并且两边正式路径文件哈希一致：

```text
backup/app_ws/src/app_node.cpp  ==  current/app_ws/src/app_node.cpp
```

统一建议：不要恢复备份中的 `app_ws/app_node.cpp`。它更像旧位置遗留文件。以 `app_ws/src/app_node.cpp` 为准。

#### `rs485_ws/*`

这是当前 `rs585_ws/*` 的旧目录名版本，内容一致。

统一建议：不要同时保留两个目录，避免 colcon 或脚本混淆。当前工程建议统一为 `rs585_ws`。

#### Unitree SDK 旧 B2W 导航示例

备份目录独有：

```text
unitree_sdk2/example/b2w/b2w_localization_node.cpp
unitree_sdk2/example/b2w/b2w_move_to_xy.cpp
unitree_sdk2/example/b2w/b2w_navigation_controller/*
unitree_sdk2/ros2_ws/src/b2w_navigation/*
```

这些看起来是早期或示例性质的 B2W 导航代码。

当前项目主导航位置是：

```text
b2w_navigation_ws/src/main.cpp
```

统一建议：

- 不建议直接并入当前生产代码。
- 如果需要追溯早期算法，可单独归档到 `docs/archive/` 或 `experiments/`，不要混进主构建链路。

#### `robose_airy_ws/src/rslidar_sdk/src/rs_driver/cmake/*`

备份目录独有：

```text
rs_driverConfig.cmake
rs_driverConfigVersion.cmake
```

这类文件更像 SDK package config 或生成文件。

统一建议：除非有外部项目通过 `find_package(rs_driver)` 依赖它，否则不需要恢复。

#### `rtk_offset_calib_20260507_143128.json`

备份目录独有标定结果 JSON。

统一建议：可作为历史标定数据归档，但不要作为运行源码合并。

### 5.2 备份目录独有：文档/点位类

```text
5-7/test2/gnss_waypoints.txt
5-7/test4/gnss_waypoints.txt
5-7/test4/gnss_waypoints_detail.txt
5-7/test5/gnss_waypoints.txt
gnss_waypoints_back.txt
gnss_waypoints_detail.txt
gnss_waypoints_pt1.txt
rtk_nav_ws/fj_dynamic/improve/calibration_results/rtk_offset_calib_20260507_143128_report.txt
waypoints_backup/5-6/gnss_waypoints.txt
waypoints_backup/5-6/gnss_waypoints_detail.txt
```

其中多数已归档到当前项目 `docs/dog_backup/`，详见 4.3。

目前仍未在当前项目中发现完全相同内容归档的备份文档/数据：

```text
rtk_nav_ws/fj_dynamic/improve/calibration_results/rtk_offset_calib_20260507_143128_report.txt
```

统一建议：

- 已归档到 `docs/dog_backup/` 的点位文件无需再重复放在根目录。
- `rtk_offset_calib_20260507_143128_report.txt` 如果仍有参考价值，建议归档到：

```text
docs/dog_backup/rtk_nav_ws/fj_dynamic/improve/calibration_results/rtk_offset_calib_20260507_143128_report.txt
```

或归档到：

```text
rtk_nav_ws/fj_dynamic/improve/calibration_results/
```

## 6. 建议统一策略

### 6.1 已经统一，无需再处理

```text
tcp_base_ctl.sh
z1_controller/config/config.xml
```

当前状态：

- `tcp_base_ctl.sh`：两边均使用 `eth2`。
- `z1_controller/config/config.xml`：两边均为 `<load>1.0</load>`。

### 6.2 必须保留当前版本

```text
colcon_ws/src/nmea_msgs/*
spray_path_planner_ws/*
app_ws/src/app_node.cpp
tests/test_app_node_failsafe.py
```

理由：

- `nmea_msgs` 和 `spray_path_planner_ws` 是当前完整构建链路的一部分。
- `app_ws/src/app_node.cpp` 两边一致，是正式 APP 节点源码。
- tests 是当前 failsafe 逻辑的验证资产。

### 6.3 需要人工确认后统一

```text
tf_broadcast_ws/urdf/my_robot.urdf
robose_airy_ws/src/rslidar_sdk/src/source/source_pointcloud_ros.hpp
```

确认点：

1. `my_robot.urdf`
   - 确认 RTK 天线相对 `base_link` 的真实外参。
   - 当前项目使用 `-0.4477 0.0 0.10`。
   - 备份使用 `-0.3485 0.0 0.30`。

2. `source_pointcloud_ros.hpp`
   - 确认避障参数是否应使用当前更敏感配置。

说明：`gnss_waypoints.txt` 仅点名不同，坐标一致；点名体系不影响系统算法执行，不再列为需要统一的核心项。

### 6.4 可以忽略或只归档

```text
record_epsg_waypoint.py
z1_move_ws/src/z1_arm_controller_node.cpp
unitree_sdk2/example/*/CMakeLists.txt
unitree_sdk2/example/b2w/* 旧示例
unitree_sdk2/ros2_ws/src/b2w_navigation/* 旧示例
```

理由：

- `record_epsg_waypoint.py` 仅末尾换行差异。
- `z1_arm_controller_node.cpp` 只是日志差异。
- Unitree SDK example 差异不属于当前生产主链路。

## 7. 逐个比对命令

如果需要手动打开每个差异文件逐个确认，可用以下命令。

### 7.1 查看路径相同但内容不同的文件 diff

```bash
BACKUP='/home/oneko/文档/希腊材料/kiefer/projects/backup/5-19'
CURRENT='/home/oneko/projects/spray_b2w_robot_project_greek'

for f in \
  app_ws/test.py \
  gnss_waypoints.txt \
  record_epsg_waypoint.py \
  robose_airy_ws/src/rslidar_sdk/src/source/source_pointcloud_ros.hpp \
  rtk_nav_ws/fj_dynamic/improve/precision_roadmap.md \
  rtk_nav_ws/fj_dynamic/improve/test-100/README.md \
  tf_broadcast_ws/urdf/my_robot.urdf \
  unitree_sdk2/example/b2w/CMakeLists.txt \
  unitree_sdk2/example/g1/CMakeLists.txt \
  unitree_sdk2/example/go2/CMakeLists.txt \
  unitree_sdk2/example/go2w/CMakeLists.txt \
  unitree_sdk2/example/h1/CMakeLists.txt \
  z1_move_ws/src/z1_arm_controller_node.cpp
  do
    echo "================ $f ================"
    diff -u "$BACKUP/$f" "$CURRENT/$f" | less
  done
```

### 7.2 验证已统一文件

```bash
BACKUP='/home/oneko/文档/希腊材料/kiefer/projects/backup/5-19'
CURRENT='/home/oneko/projects/spray_b2w_robot_project_greek'

diff -u "$BACKUP/tcp_base_ctl.sh" "$CURRENT/tcp_base_ctl.sh"
diff -u "$BACKUP/z1_controller/config/config.xml" "$CURRENT/z1_controller/config/config.xml"
```

当前预期：无输出，表示一致。

### 7.3 单独确认 APP 正式源码是否一致

```bash
sha256sum \
  '/home/oneko/文档/希腊材料/kiefer/projects/backup/5-19/app_ws/src/app_node.cpp' \
  '/home/oneko/projects/spray_b2w_robot_project_greek/app_ws/src/app_node.cpp'
```

本次结果为一致。

### 7.4 单独确认 RS485/RS585 是否只是目录名不同

```bash
BACKUP='/home/oneko/文档/希腊材料/kiefer/projects/backup/5-19'
CURRENT='/home/oneko/projects/spray_b2w_robot_project_greek'

diff -u "$BACKUP/rs485_ws/CMakeLists.txt" "$CURRENT/rs585_ws/CMakeLists.txt"
diff -u "$BACKUP/rs485_ws/package.xml" "$CURRENT/rs585_ws/package.xml"
diff -u "$BACKUP/rs485_ws/launch/rs485.launch.py" "$CURRENT/rs585_ws/launch/rs485.launch.py"
diff -u "$BACKUP/rs485_ws/config/rs485_params.yaml" "$CURRENT/rs585_ws/config/rs485_params.yaml"
diff -u "$BACKUP/rs485_ws/src/rs485_node.cpp" "$CURRENT/rs585_ws/src/rs485_node.cpp"
```

本次结果为内容一致。

### 7.5 验证已归档到 `docs/dog_backup/` 的文件

```bash
BACKUP='/home/oneko/文档/希腊材料/kiefer/projects/backup/5-19'
CURRENT='/home/oneko/projects/spray_b2w_robot_project_greek'

diff -u "$BACKUP/5-7/test2/gnss_waypoints.txt" "$CURRENT/docs/dog_backup/5-7/test2/gnss_waypoints.txt"
diff -u "$BACKUP/5-7/test4/gnss_waypoints.txt" "$CURRENT/docs/dog_backup/5-7/test4/gnss_waypoints.txt"
diff -u "$BACKUP/5-7/test4/gnss_waypoints_detail.txt" "$CURRENT/docs/dog_backup/5-7/test4/gnss_waypoints_detail.txt"
diff -u "$BACKUP/5-7/test5/gnss_waypoints.txt" "$CURRENT/docs/dog_backup/5-7/test5/gnss_waypoints.txt"
diff -u "$BACKUP/gnss_waypoints_back.txt" "$CURRENT/docs/dog_backup/gnss_waypoints_back.txt"
diff -u "$BACKUP/gnss_waypoints_detail.txt" "$CURRENT/docs/dog_backup/gnss_waypoints_detail.txt"
diff -u "$BACKUP/gnss_waypoints_pt1.txt" "$CURRENT/docs/dog_backup/gnss_waypoints_pt1.txt"
diff -u "$BACKUP/waypoints_backup/5-6/gnss_waypoints.txt" "$CURRENT/docs/dog_backup/waypoints_backup/5-6/gnss_waypoints.txt"
diff -u "$BACKUP/waypoints_backup/5-6/gnss_waypoints_detail.txt" "$CURRENT/docs/dog_backup/waypoints_backup/5-6/gnss_waypoints_detail.txt"
```

当前预期：无输出，表示已完整归档。

## 8. 最终判断

根据最新同步状态：

- `tcp_base_ctl.sh` 已经统一，不再存在 `eth22` 问题。
- `z1_controller/config/config.xml` 已经统一为 `load=1.0`。
- 备份中的多数历史点位/文档已经归档到当前项目 `docs/dog_backup/`。
- 当前项目仍然整体更完整，更接近当前生产链路。

当前仍需要人工逐个确认并统一的核心差异只剩 2 类：

```text
tf_broadcast_ws/urdf/my_robot.urdf         # RTK 外参
robose_airy_ws/src/rslidar_sdk/src/source/source_pointcloud_ros.hpp  # 雷达过滤参数
```

`gnss_waypoints.txt` 仍有点名差异，但坐标一致，且点名体系不影响系统算法执行，因此不作为核心统一项。

另外还有一些低风险差异：

```text
app_ws/test.py                             # 测试脚本协议，建议保留当前 1 字节版本
record_epsg_waypoint.py                    # 末尾换行
z1_move_ws/src/z1_arm_controller_node.cpp   # 日志差异
unitree_sdk2/example/*/CMakeLists.txt       # SDK 示例编译范围
```

备份独有的旧路径/旧示例代码仍建议只归档、不直接混入生产构建链路：

```text
app_ws/app_node.cpp
unitree_sdk2/example/b2w/b2w_navigation_controller/*
unitree_sdk2/ros2_ws/src/b2w_navigation/*
```
