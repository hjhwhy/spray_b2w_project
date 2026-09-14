# 丰疆 RTK 点位录入与司南 RTK 点位偏差分析

## 2026.4.30

## 1. 问题描述

当前机器狗使用上装中的司南 RTK 手动记录点位信息，并将点位记录到 `gnss_waypoints.txt` 中。

机器狗随后读取 `gnss_waypoints.txt`，自动到达点位并执行喷涂作业。

当使用一台丰疆 RTK 独立打点，然后将丰疆 RTK 平面坐标记录结果手动录入 `gnss_waypoints.txt`，直接让机器狗执行作业。

实际测试发现：

- **使用司南 RTK 自己记录的点位时，机器狗能正常到达目标点。**
- **使用丰疆 RTK 记录并手动录入的点位时，机器狗到达位置与预期位置偏差较大（~1m左右）。**

对同一个物理点分别记录了丰疆 RTK 数据和机器狗司南 RTK 数据。

## 2. 原始数据

### 2.1 丰疆 RTK 数据

```text
Latitude:  38°02'33.90924"N
Longitude: 23°47'31.14485"E
Northing:  4210292.041
Easting:   481598.134
Elevation: 184.143
```

丰疆 RTK 参数：

```text
Projection Scale: 0.9996
坐标系统椭球参数：
Ellipsoid Name: RS 1980 (IUGG,1980)
a: 6378137
1/f: 298.257222101
Coord Axis: North-East
```

### 2.2 机器狗司南 RTK 数据

原始经纬度：

```text
Longitude: 23°47'31.14759"E
Latitude:  38°02'33.90247"N
Height:    223.5418 m
```

通过 `record_epsg_waypoint.py` 记录到 `gnss_waypoints.txt` 的结果：

```text
Parsed point:
id='pt3'
x=481597.758286845
y=4210290.955223978
z=185.077100000
```

其中：

- `x` 表示 EPSG:2100 Easting。
- `y` 表示 EPSG:2100 Northing。
- `z` 表示记录到点位文件中的高度字段。

## 3. 丰疆经纬度按狗端方式转换

丰疆 RTK 经纬度转十进制度：

```text
Latitude  = 38 + 2/60 + 33.90924/3600
          = 38.042752566666664

Longitude = 23 + 47/60 + 31.14485/3600
          = 23.791984680555558
```

使用狗端相同的坐标转换链路：

```text
EPSG:4326 -> EPSG:2100
```

转换结果：

```text
x = 481597.757455160
y = 4210291.053724137
```

## 4. 与司南记录点对比

司南通过 `record_epsg_waypoint.py` 记录的点：

```text
x = 481597.758286845
y = 4210290.955223978
```

丰疆经纬度按狗端方式转换后的点：

```text
x = 481597.757455160
y = 4210291.053724137
```

差值：

```text
dx = 司南 x - 丰疆转换 x
   = 0.000831685 m

dy = 司南 y - 丰疆转换 y
   = -0.098500159 m

水平距离差 ≈ 0.099 m
```

结论：

**如果使用丰疆 RTK 的原始经纬度作为输入，并使用狗端同一套 EPSG:2100 转换方式生成点位，
丰疆点和司南点只差约 10 cm。**


这个误差属于很小的点位差异，不是导致机器狗明显跑偏的主要原因。

## 5. 丰疆平面坐标与狗端转换结果对比

丰疆 RTK 直接显示的平面坐标：

```text
Easting  = 481598.134
Northing = 4210292.041
```

丰疆经纬度按狗端方式转换后的平面坐标：

```text
Easting  = 481597.757455160
Northing = 4210291.053724137
```

差值：

```text
dx = 丰疆显示 Easting - 狗端转换 Easting
   = 481598.134 - 481597.757455160
   = 0.376544840 m

dy = 丰疆显示 Northing - 狗端转换 Northing
   = 4210292.041 - 4210291.053724137
   = 0.987275863 m

水平距离差 = sqrt(dx^2 + dy^2)
           = sqrt(0.376544840^2 + 0.987275863^2)
           ≈ 1.056551 m
```

完整对比：

```text
丰疆显示坐标：
Easting  = 481598.134000000
Northing = 4210292.041000000

狗端转换坐标：
Easting  = 481597.757455160
Northing = 4210291.053724137

差值：
East 差  = +0.376544840 m
North 差 = +0.987275863 m
水平差   = 1.056551 m
```

说明：

- **水平误差~1m，和之前的2点测试结果一致。**
- **丰疆 RTK 显示的 Easting/Northing 与狗端当前 EPSG:2100 转换结果并不完全一致。**
- **结合后续 5 点验证，关键原因已经明确：丰疆控制器显示值叠加了 HEPOS plane grid 修正，而狗端当前转换链路没有叠加该修正。**

## 6. 进一步验证：HEPOS 平面格网修正

后续补充了 `rtk_nav_ws/fj_dynamic/HEPOS_5point_validation_1.xlsx`，对 5 个真实点进行了更完整的坐标框架对比。

该表把同一批点放入以下几个坐标框架中比较：

- 丰疆 RTK 原始 WGS84 经纬度。
- 司南 RTK 原始 WGS84 经纬度。
- 狗端当前 `PROJ default` 的 EPSG:2100 转换结果。
- 丰疆控制器显示的 HEPOS-correct ΕΓΣΑ87 平面坐标。
- 狗端当前写入 `gnss_waypoints.txt` 的点位坐标。

### 6.1 WGS84 经纬度本身没有明显问题

`WGS84 Compare` 表明，丰疆 RTK 和司南 RTK 在原始经纬度域基本一致。

5 个点的水平差异为：

```text
pt1: 5.78 cm
pt2: 3.60 cm
pt3: 1.19 cm
pt4: 9.56 cm
pt5: 3.46 cm
```

结论：

```text
丰疆和司南在 WGS84 经纬度上只存在厘米级 RTK 噪声。
约 1 m 的平面偏差不是由原始经纬度采集错误导致的。
```

### 6.2 PROJ default 与 HEPOS-correct 的差异

`EPSG2100 Frames` 表明，狗端当前 `PROJ default` 结果与丰疆控制器显示的 HEPOS-correct 坐标存在稳定偏差。

5 个点的 `PROJ default -> HEPOS-correct` 平移量约为：

```text
dE ≈ +0.379 m
dN ≈ +0.984 m
```

这与第 5 节中单点计算得到的约 `1.056 m` 水平差一致。

### 6.3 HEPOS 格网修正可以复现丰疆控制器结果

`HEPOS Validation` 表明，在狗端 `PROJ default` 结果基础上叠加 HEPOS plane grid 修正后，可以复现丰疆控制器显示值。

验证结果：

| 点位 | HEPOS E | HEPOS N | 丰疆 E | 丰疆 N | 水平残差 |
| --- | --- | --- | --- | --- | --- |
| pt1 | 481596.207550 | 4210296.554471 | 481596.204 | 4210296.557 | 4.36 mm |
| pt2 | 481596.235077 | 4210294.596399 | 481596.232 | 4210294.599 | 4.03 mm |
| pt3 | 481597.740780 | 4210293.125972 | 481597.738 | 4210293.129 | 4.11 mm |
| pt4 | 481594.399420 | 4210297.074773 | 481594.396 | 4210297.078 | 4.70 mm |
| pt5 | 481592.792360 | 4210298.454815 | 481592.789 | 4210298.458 | 4.63 mm |

统计结果：

```text
平均残差 ≈ 4.4 mm
最大残差 ≈ 4.7 mm
```

结论：

```text
丰疆控制器显示的平面坐标使用了 HEPOS plane grid 修正。
狗端当前只做 PROJ default EPSG:2100 转换，没有叠加 HEPOS 平面格网修正。
这就是约 1 m 偏差的主要原因。
```

### 6.4 高程类型不一致

丰疆 RTK：

```text
Elevation = 184.143
```

司南 RTK 原始高程：

```text
Height = 223.5418 m
```

司南记录点文件中的 z：

```text
z = 185.077100000
```

这说明不同设备或不同链路中的高度字段可能不是同一种高程：

- 椭球高。
- 正常高。
- 海拔高。
- 或经过 geoid / datum 修正后的高度。

当前导航主要依赖平面 `x/y`，但不要直接混用不同设备的高度字段。

## 7. 结论

```text
丰疆 RTK 的原始经纬度与司南 RTK 的原始经纬度基本一致。
```

如果只比较原始 WGS84 经纬度，丰疆和司南只存在厘米级 RTK 噪声。

因此，问题不在丰疆 RTK 的原始经纬度精度，而在：

- **直接使用丰疆显示的 Easting/Northing。**
- **丰疆控制器显示值是 HEPOS-correct ΕΓΣΑ87，狗端当前是 PROJ default EPSG:2100。**
- **狗端当前缺少 HEPOS plane grid 修正，因此与丰疆平面坐标存在约 1 m 系统性偏差。**

## 8. 后续处理方式


核心原则：

```text
如果希望机器狗使用丰疆 RTK 采集点位，并且点位与丰疆控制器显示的平面坐标一致，
狗端必须实现与丰疆控制器一致的 HEPOS plane grid 修正链路。
```

### 8.1 必须修改狗端坐标转换链路

后续应在狗端或离线点位转换工具中加入：

- `PROJ default EPSG:4326 -> EPSG:2100` 基础投影。
- HEPOS plane grid 修正。
- 与丰疆控制器一致的 Easting/Northing 输出。

涉及文件：

```text
rtk_nav_ws/fj_dynamic/dN_2km_V1-0.ngrd
rtk_nav_ws/fj_dynamic/dE_2km_V1-0.egrd
rtk_nav_ws/fj_dynamic/GEOID_GR.GRD
rtk_nav_ws/fj_dynamic/parsms.json
```

目标：

```text
输入：WGS84 经纬度
输出：HEPOS-correct ΕΓΣΑ87 Easting/Northing
验证标准：与丰疆控制器显示坐标误差小于厘米级
```

### 8.2 推荐点位生成流程

1. 使用丰疆 RTK 记录原始 WGS84 经纬度。
2. 将丰疆经纬度输入狗端统一转换工具。
3. 转换工具执行 `PROJ default + HEPOS plane grid`，生成 HEPOS-correct `x/y`。
4. `z` 使用司南 RTK 体系下的高度，不直接使用丰疆 `Elevation`。
5. 机器狗只使用统一转换后的点位文件作业。

### 8.3 短期未接入 HEPOS plane grid 时的限制

如果狗端还没有接入 HEPOS plane grid：

- 不要把丰疆控制器显示的 `Easting/Northing` 直接写入 `gnss_waypoints.txt`。
- 不要把狗端当前 `PROJ default` 生成的平面坐标与丰疆显示坐标混用。
- 可以临时继续使用司南 RTK 在狗端直接记录点位。
- 如果必须使用丰疆采点，应只使用丰疆 WGS84 经纬度，并明确知道狗端当前生成的是 PROJ default 坐标，不会与丰疆显示坐标完全一致。

### 8.4 高程处理原则

不直接混用丰疆高程和司南高程。

- 如果作业区域地面高度变化不大，可以使用附近司南记录点的 `z` 或区域平均 `z`。
- 如果喷涂高度对地形敏感，应使用司南 RTK 在作业区域补采若干高度参考点，再按区域赋值或插值。
- 不建议把丰疆 `Elevation` 直接写入 `gnss_waypoints.txt` 的 `z` 字段，除非已经确认丰疆 `Elevation` 与司南记录的 `z` 使用同一高程基准。

这样可以避免平面格网、高程基准不同造成的系统性偏差。

## 9. 狗端程序投影参数校验

为了排查问题是否出在狗端代码本身，对仓库里所有做坐标投影的位置进行了逐行核对。

涉及的文件：

- `rtk_nav_ws/src/ins_parser.cpp`
- `gnss_driver_ws/src/pub_rtk_save_pt_node.cpp`

两份代码使用同一套投影逻辑，只是消息类型不同。

### 9.1 程序内两条转换路径

代码里通过参数 `use_epsg_crs_datum` 切换两条路径。

**路径 A：`use_epsg_crs_datum = true`（当前 YAML 中实际启用的）**

```cpp
P = proj_create_crs_to_crs(C, "EPSG:4326", "EPSG:2100", nullptr);
P = proj_normalize_for_visualization(C, P);
PJ_COORD coord = proj_coord(lon, lat, alt, 0);
coord = proj_trans(P, PJ_FWD, coord);
```

走的是 PROJ 完整 CRS-to-CRS 转换，**自动应用 GGRS87 ↔ WGS84 的 datum 平移**。

**路径 B：`use_epsg_crs_datum = false`（备用）**

```text
+proj=tmerc
+lat_0=0
+lon_0=24
+k=0.9996
+x_0=500000
+y_0=0
+ellps=GRS80
+units=m
+no_defs
```

只做投影，不带 datum 平移。

### 9.2 与 EPSG:2100 官方定义逐项核对

| 参数 | 程序使用值 | EPSG:2100 官方定义 | 是否一致 |
| --- | --- | --- | --- |
| 投影方式 | `tmerc`（横轴墨卡托） | Transverse Mercator | 一致 |
| 中央经线 `lon_0` | 24° | 24°E | 一致 |
| 起始纬度 `lat_0` | 0° | 0° | 一致 |
| 比例因子 `k` | 0.9996 | 0.9996 | 一致 |
| 东偏 `x_0` | 500000 | 500000 | 一致 |
| 北偏 `y_0` | 0 | 0 | 一致 |
| 椭球 | GRS80（a=6378137, 1/f=298.257222101） | GRS 1980（GGRS87 datum） | 一致 |

椭球参数也与丰疆 RTK 设备显示的参数完全相同：

```text
Ellipsoid Name: RS 1980 (IUGG, 1980)
a:   6378137
1/f: 298.257222101
```

### 9.3 路径 A 与路径 B 不等价

虽然两条路径都从 WGS84 经纬度出发，但狗端当前只做基础投影，而丰疆控制器显示值还叠加了 HEPOS plane grid 修正，因此两者不是同一个结果。

| 项 | 路径 A `use_epsg_crs_datum = true` | 路径 B `use_epsg_crs_datum = false` |
| --- | --- | --- |
| 投影 | 基础 EPSG:2100 / PROJ default | 仅做 tmerc 投影 |
| HEPOS plane grid 修正 | 额外叠加 | 不做 |
| 在希腊位置上的水平差异 | 与丰疆控制器一致 | 相比丰疆显示值偏 ~1 m |

第 5 节里观察到丰疆显示坐标与狗端转换坐标相差 ≈ 1.06 m。结合 `HEPOS_5point_validation_1.xlsx` 的 5 点验证，当前更准确的解释是：

```text
丰疆控制器显示的是叠加 HEPOS plane grid 后的坐标。
狗端当前 PROJ default 没有叠加该平面格网。
```

### 9.4 校验结论

```text
狗端程序使用的基础投影参数和椭球参数没有问题。
问题出在狗端没有叠加与丰疆一致的 HEPOS plane grid 修正。
如果希望直接复现丰疆控制器显示值，必须在狗端补上这层修正。
```
