# test-100 数据记录说明

用途：A1.1 标定后，执行 100/101 点喷涂验证，并把丰疆目标点、丰疆喷涂复测点、司南喷涂复测点按同一 `id` 配对保存，供后续误差分析使用。

## 文件约定

本次 100/101 点任务必须归档以下原始文件，后续分析都从这些文件配对生成：

```text
target_fj_100.csv       # 丰疆目标点：写入 gnss_waypoints 的目标点；101 点时可命名 target_fj_101.csv
spray_fj_100.csv        # 丰疆复测喷涂点；101 点时可命名 spray_fj_101.csv
spray_sn_100.txt        # 司南复测喷涂点：本目录脚本直接生成，格式 ptN,E,N,Z,
spray_sn_100_detail.csv # 司南复测细节：均值、标准差、yaw、时间戳
b2w_navigation.log      # 主导航日志：包含 ARM_TRIGGER base/yaw/z1_world/target
```

如果后续分析脚本要求 `spray_sn_100.csv`，先由 `spray_sn_100.txt` 或 `spray_sn_100_detail.csv` 整理生成；现场原始记录不要覆盖。

丰疆数据由设备手动导出，建议保存为：

```text
target_fj_100.csv    # 丰疆记录的 100 个目标点
spray_fj_100.csv     # 丰疆复测的 100 个实际喷涂点
```

如果实际是 101 个点，可用 `target_fj_101.csv` / `spray_fj_101.csv`，但同一次分析里命名要保持一致。

司南复测脚本会生成：

```text
spray_sn_100.txt         # 简洁点位文件：pt1,E,N,Z,
spray_sn_100_detail.csv  # 详细记录：均值、标准差、yaw、时间戳
```

关键要求：三份数据的点号必须一致。若直接把丰疆 `name,E,N,Z,` 文件作为 `gnss_waypoints.txt`，建议喷涂复测也沿用丰疆原始点名，例如 `task_new` 到 `task_new100`；如果后处理前统一整理为 `pt1` 到 `pt101`，三份文件必须一起改。

## 司南复测喷涂点

每个喷涂点完成后，把司南 RTK 天线放到实际喷涂落点上，停稳后运行一次：

```bash
source /opt/ros/humble/setup.bash
source /home/test/rtk_nav_ws/install/setup.bash

python3 rtk_nav_ws/fj_dynamic/improve/test-100/recore_spary_points.py
```

默认每点采集 20 帧 `/epsg_position` 并取均值，自动追加下一个点号。

如果需要指定点号：

```bash
python3 rtk_nav_ws/fj_dynamic/improve/test-100/recore_spary_points.py --pt-id pt37
```

如果现场 RTK 抖动较大，可以增加帧数：

```bash
python3 rtk_nav_ws/fj_dynamic/improve/test-100/recore_spary_points.py --frames 50
```

## 后续分析使用方式

后续至少可以计算三类误差：

```text
端到端误差：spray_fj_100 - target_fj_100
机器人侧误差：spray_sn_100 - target_fj_100
丰疆/司南同点差：spray_sn_100 - spray_fj_100
```

不要把 A1.1 标定前后的数据混在同一个 100 点统计里。
