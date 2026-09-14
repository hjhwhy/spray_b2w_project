#!/usr/bin/env python3
"""
A1.1 — RTK 天线相对 base_link 实际偏移标定脚本
精度提升路线图 §2.2 A1 步骤 1

原理
----
main.cpp 偏移公式：
    base_E = ant_E + rtk_x_offset * cos(yaw)
    base_N = ant_N + rtk_x_offset * sin(yaw)
推导：
    body_x = (ant_E - P_E)*cos(yaw) + (ant_N - P_N)*sin(yaw)
    rtk_x_offset = -body_x
天线在 base_link 前方 0.4477m → body_x=+0.4477 → rtk_x_offset=-0.4477（与 YAML 默认一致）

标定方法
--------
1. 机器人静止，base_link 中心垂直对准地面物理标记 P
2. 用丰疆测量 P 的 EPSG:2100 坐标，输入本脚本
3. 采集 N 帧 /epsg_position，计算均值 → 得到 rtk_x_offset 建议值
4. 旋转机器人到不同朝向重复，验证一致性
5. 运行结束后自动保存 JSON + 文本报告

现场转向时如果 base_link 无法保持在同一个 P 上，使用：
    --per-round-base-link
每次采集前重新输入当前朝向下丰疆测得的 base_link 垂直投影点 P_i。
数学上只要求每轮的 P_i 和司南 RTK 均值 A_i 属于同一机器人静止姿态，
不要求所有朝向共用同一个 P。

用法
----
python3 2-2-A-step-1.py --base-link-e 481599.199 --base-link-n 4210292.482
python3 2-2-A-step-1.py                   # 交互式输入
python3 2-2-A-step-1.py ... --frames 20   # 自定义采集帧数
python3 2-2-A-step-1.py --per-round-base-link

运行环境
--------
source /opt/ros/humble/setup.bash
source /home/test/rtk_nav_ws/install/setup.bash
（tcp_base_ctl.service 需在运行，ins_parser 节点需在线）
"""

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except ImportError:
    print("❌ 找不到 rclpy，请先 source ROS2 环境：")
    print("   source /opt/ros/humble/setup.bash")
    print("   source /home/test/rtk_nav_ws/install/setup.bash")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────
# 结果保存目录：脚本同级的 ../calibration_results/
# ─────────────────────────────────────────────────────────────
SCRIPT_DIR    = Path(__file__).parent
RESULTS_DIR   = SCRIPT_DIR.parent / "calibration_results"

SEP  = "=" * 64
SEP2 = "-" * 64


# ─────────────────────────────────────────────────────────────
# 数学工具
# ─────────────────────────────────────────────────────────────

def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """ROS 四元数 → 偏航角（rad，东向 0°，逆时针正）"""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def world_to_body(d_e: float, d_n: float, yaw: float) -> Tuple[float, float]:
    """世界系差值 (dE, dN) → 机体系 (body_x 车头, body_y 车左)"""
    body_x =  d_e * math.cos(yaw) + d_n * math.sin(yaw)
    body_y = -d_e * math.sin(yaw) + d_n * math.cos(yaw)
    return body_x, body_y


def circular_mean(angles: List[float]) -> float:
    """yaw 圆周均值，安全处理 ±π 边界"""
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


def stdev(vals: List[float]) -> float:
    if len(vals) < 2:
        return 0.0
    m = sum(vals) / len(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals))


# ─────────────────────────────────────────────────────────────
# ROS2 采集节点
# ─────────────────────────────────────────────────────────────

class RtkCollector(Node):
    """订阅 /epsg_position，同步采集 N 帧，返回原始帧 + 统计量"""

    def __init__(self, topic: str, n_frames: int):
        super().__init__("rtk_calibrator")
        self._n   = n_frames
        self._buf: List[Dict] = []   # 每帧: {e, n, yaw_deg, yaw_rad}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(PoseStamped, topic, self._cb, qos)

    def _cb(self, msg: PoseStamped) -> None:
        if len(self._buf) >= self._n:
            return
        e   = msg.pose.position.x
        n   = msg.pose.position.y
        o   = msg.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self._buf.append({"e": e, "n": n,
                          "yaw_rad": yaw,
                          "yaw_deg": math.degrees(yaw)})
        pct = len(self._buf) / self._n * 100
        bar = "█" * int(pct / 5) + "░" * (20 - int(pct / 5))
        print(f"\r  [{bar}] {len(self._buf):>3}/{self._n}  "
              f"E={e:.4f}  N={n:.4f}  yaw={math.degrees(yaw):+.1f}°",
              end="", flush=True)

    def collect(self, timeout: float = 40.0) -> Optional[Dict]:
        """
        采集 N 帧，返回包含 raw_frames 和统计量的 dict，超时返回 None。
        """
        self._buf.clear()
        deadline = time.time() + timeout
        while len(self._buf) < self._n:
            if time.time() > deadline:
                print()
                return None
            rclpy.spin_once(self, timeout_sec=0.2)
        print()  # 换行

        Es   = [f["e"]       for f in self._buf]
        Ns   = [f["n"]       for f in self._buf]
        yaws = [f["yaw_rad"] for f in self._buf]

        e_mean   = sum(Es)   / self._n
        n_mean   = sum(Ns)   / self._n
        yaw_mean = circular_mean(yaws)
        e_std    = stdev(Es)
        n_std    = stdev(Ns)

        return {
            "raw_frames": list(self._buf),   # 保留完整原始帧
            "stats": {
                "ant_e_mean"  : e_mean,
                "ant_n_mean"  : n_mean,
                "yaw_deg_mean": math.degrees(yaw_mean),
                "yaw_rad_mean": yaw_mean,
                "e_std_mm"    : e_std  * 1000,
                "n_std_mm"    : n_std  * 1000,
            },
        }


# ─────────────────────────────────────────────────────────────
# 交互式输入
# ─────────────────────────────────────────────────────────────

def ask_base_link(round_idx: Optional[int] = None) -> Tuple[float, float]:
    print()
    print("━" * 64)
    if round_idx is None:
        print("  输入丰疆测量的 base_link 中心点 P（EPSG:2100）")
    else:
        print(f"  输入第 {round_idx} 次姿态的 base_link 中心点 P_i（EPSG:2100）")
    print("  （机器狗 base_link 中心垂直投影到地面标记点）")
    print("━" * 64)
    while True:
        try:
            e = float(input("  P_E（Easting） : ").strip())
            n = float(input("  P_N（Northing）: ").strip())
            if 400_000 < e < 600_000 and 4_100_000 < n < 4_400_000:
                return e, n
            print("  ⚠ 坐标超出希腊 EPSG:2100 合理范围，请确认")
        except (ValueError, EOFError):
            print("  ⚠ 请输入数字")


def ask_yes(prompt: str, default_yes: bool = False) -> bool:
    try:
        ans = input(prompt).strip().lower()
        if not ans:
            return default_yes
        return ans in ("y", "yes")
    except EOFError:
        return False


# ─────────────────────────────────────────────────────────────
# 单次测量 + 结果计算
# ─────────────────────────────────────────────────────────────

def measure_one_round(
    collector: RtkCollector,
    p_e: float,
    p_n: float,
    round_idx: int,
) -> Optional[Dict]:
    """
    采集一组 RTK 帧，计算偏移。
    返回包含所有中间量的 dict，采集失败返回 None。
    """
    print(f"\n  ▶ 第 {round_idx} 次采集...")
    data = collector.collect()
    if data is None:
        print("  ❌ 采集超时（40s），请检查 ins_parser 是否在运行")
        return None

    stats   = data["stats"]
    ant_e   = stats["ant_e_mean"]
    ant_n   = stats["ant_n_mean"]
    yaw     = stats["yaw_rad_mean"]
    e_std   = stats["e_std_mm"]
    n_std   = stats["n_std_mm"]

    d_e = ant_e - p_e
    d_n = ant_n - p_n
    body_x, body_y = world_to_body(d_e, d_n, yaw)

    # 符号约定：main.cpp: base = ant + rtk_x_offset*(cos,sin)
    # → rtk_x_offset = -body_x
    suggested_x = -body_x
    suggested_y = -body_y

    # 终端打印
    print()
    print(SEP)
    print(f"  第 {round_idx} 次测量结果")
    print(SEP2)
    print(f"  base_link P（丰疆测）   E={p_e:.4f}  N={p_n:.4f}")
    print(f"  RTK 天线均值 A           E={ant_e:.4f}  N={ant_n:.4f}")
    print(f"  定位散布                 E_std={e_std:.1f} mm  N_std={n_std:.1f} mm")
    print(SEP2)
    print(f"  世界系偏差 A - P         dE={d_e:+.4f} m  dN={d_n:+.4f} m")
    print(f"  车头方向 yaw             {math.degrees(yaw):+.2f}°")
    print(SEP2)
    print(f"  机体系投影：")
    print(f"    body_x（车头 +x）= {body_x:+.4f} m")
    print(f"    body_y（车左 +y）= {body_y:+.4f} m")
    print(SEP2)
    print(f"  本次建议值：")
    print(f"    rtk_x_offset: {suggested_x:+.4f}")
    if abs(suggested_y) > 0.005:
        print(f"    rtk_y_offset: {suggested_y:+.4f}  ⚠ 横向偏移 > 5 mm")
    print(SEP)

    return {
        "round"      : round_idx,
        "raw_frames" : data["raw_frames"],
        "stats"      : stats,
        "base_link"  : {"e": p_e, "n": p_n},
        "diff_world" : {"d_e": d_e, "d_n": d_n},
        "body"       : {"x": body_x, "y": body_y},
        "suggested"  : {"rtk_x_offset": suggested_x,
                        "rtk_y_offset": suggested_y},
    }


# ─────────────────────────────────────────────────────────────
# 汇总 + 质量评估（终端输出）
# ─────────────────────────────────────────────────────────────

def build_summary(rounds: List[Dict], current_yaml: float) -> Dict:
    xs = [r["suggested"]["rtk_x_offset"] for r in rounds]
    ys = [r["suggested"]["rtk_y_offset"] for r in rounds]
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    x_std  = stdev(xs)
    y_std  = stdev(ys)
    delta  = x_mean - current_yaml
    return {
        "n_rounds"            : len(rounds),
        "rtk_x_offset_mean"   : x_mean,
        "rtk_x_offset_std_mm" : x_std * 1000,
        "rtk_y_offset_mean"   : y_mean,
        "rtk_y_offset_std_mm" : y_std * 1000,
        "current_yaml_offset" : current_yaml,
        "delta_m"             : delta,
        "delta_cm"            : delta * 100,
        "recommendation_yaml" : f"rtk_x_offset: {x_mean:.4f}   "
                                 f"# 实测（原 {current_yaml:.4f}，差 {delta*100:+.1f} cm）",
    }


def print_summary(summary: Dict, rounds: List[Dict]) -> None:
    x_mean = summary["rtk_x_offset_mean"]
    y_mean = summary["rtk_y_offset_mean"]
    x_std  = summary["rtk_x_offset_std_mm"]
    y_std  = summary["rtk_y_offset_std_mm"]
    delta  = summary["delta_cm"]

    print()
    print(SEP)
    print("  ■ 汇总（所有朝向综合）")
    print(SEP)
    print(f"  测量次数：{summary['n_rounds']} 次")
    for r in rounds:
        print(f"    朝向 {r['round']}：rtk_x_offset = {r['suggested']['rtk_x_offset']:+.4f} m"
              f"  rtk_y_offset = {r['suggested']['rtk_y_offset']:+.4f} m"
              f"  yaw = {r['stats']['yaw_deg_mean']:+.1f}°")
    print()
    print(f"  均值 rtk_x_offset = {x_mean:+.4f} m    std = {x_std:.1f} mm")
    print(f"  均值 rtk_y_offset = {y_mean:+.4f} m    std = {y_std:.1f} mm")
    print()
    print(f"  当前 YAML rtk_x_offset = {summary['current_yaml_offset']:+.4f} m")
    print(f"  与当前值差异：{summary['delta_m']:+.4f} m  ({delta:+.1f} cm)")
    print()

    # 质量评估
    n = summary["n_rounds"]
    if n > 1:
        if x_std < 10:
            print("  ✅ 多朝向一致性良好（std < 10 mm），结果可信")
        elif x_std < 20:
            print("  ⚠ 多朝向散布 10-20 mm，建议：")
            print("     - 确认每次 base_link 精确对准同一标记点 P")
            print("     - 旋转时避免底盘平移（底盘应原地自转）")
        else:
            print("  ❌ 多朝向散布 > 20 mm，结果不可靠，建议：")
            print("     - 使用三脚架精密对中（不要手持杆）")
            print("     - 在平整坚实地面上测量")
    if abs(y_mean) > 0.01:
        print()
        print(f"  ⚠ 横向偏移 rtk_y_offset = {y_mean:+.4f} m（> 10 mm）")
        print("     当前 YAML 无 rtk_y_offset，如需修正横向误差需改 main.cpp")
    print()
    print("  ──── 建议更新 YAML ────")
    print(f"  {summary['recommendation_yaml']}")
    print()
    print("  文件路径：b2w_navigation_ws/config/b2w_controller_params.yaml")
    print("  修改后执行：")
    print("    colcon build --packages-select b2w_navigation_controller")
    print("    sudo systemctl restart tcp_base_ctl.service")
    print(SEP)


# ─────────────────────────────────────────────────────────────
# 结果持久化（JSON + 文本报告）
# ─────────────────────────────────────────────────────────────

def save_results(
    ts: str,
    config: Dict,
    rounds: List[Dict],
    summary: Optional[Dict],
) -> Tuple[Path, Path]:
    """
    保存两份文件：
    - <ts>_data.json  ：完整 JSON（含原始帧，适合后期脚本分析）
    - <ts>_report.txt ：人可读报告（汇总 + 建议，适合存档）

    返回 (json_path, txt_path)。
    """
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    stem = f"rtk_offset_calib_{ts}"

    # ── JSON ──────────────────────────────────────────────────
    payload = {
        "schema_version": "1.0",
        "timestamp"     : ts,
        "config"        : config,
        "rounds"        : rounds,
        "summary"       : summary,
    }
    json_path = RESULTS_DIR / f"{stem}.json"
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    # ── 文本报告 ───────────────────────────────────────────────
    txt_path = RESULTS_DIR / f"{stem}_report.txt"
    if config.get("base_link_mode") == "per_round":
        base_link_line = "  base_link P   每轮单独输入 P_i"
    else:
        base_link_line = (
            f"  base_link P   E={config['base_link_e']:.4f}  "
            f"N={config['base_link_n']:.4f}"
        )
    lines = [
        f"RTK 天线偏移标定报告",
        f"生成时间  : {ts}",
        f"",
        f"【配置】",
        base_link_line,
        f"  采集帧数      {config['frames_per_round']} 帧/次",
        f"  RTK 话题      {config['topic']}",
        f"  当前 YAML     rtk_x_offset={config['current_yaml_offset']}",
        f"",
        f"【各次测量】",
    ]
    for r in rounds:
        s = r["stats"]
        d = r["diff_world"]
        b = r["body"]
        sg = r["suggested"]
        lines += [
            f"  朝向 {r['round']}：",
            f"    base_link   E={r['base_link']['e']:.4f}  N={r['base_link']['n']:.4f}",
            f"    天线均值   E={s['ant_e_mean']:.4f}  N={s['ant_n_mean']:.4f}  yaw={s['yaw_deg_mean']:+.2f}°",
            f"    定位散布   E_std={s['e_std_mm']:.1f} mm  N_std={s['n_std_mm']:.1f} mm",
            f"    世界差值   dE={d['d_e']:+.4f} m  dN={d['d_n']:+.4f} m",
            f"    机体投影   body_x={b['x']:+.4f} m  body_y={b['y']:+.4f} m",
            f"    本次建议   rtk_x_offset={sg['rtk_x_offset']:+.4f}  rtk_y_offset={sg['rtk_y_offset']:+.4f}",
            f"",
        ]

    if summary:
        lines += [
            f"【汇总】",
            f"  测量次数     {summary['n_rounds']}",
            f"  rtk_x_offset 均值  {summary['rtk_x_offset_mean']:+.4f} m   std={summary['rtk_x_offset_std_mm']:.1f} mm",
            f"  rtk_y_offset 均值  {summary['rtk_y_offset_mean']:+.4f} m   std={summary['rtk_y_offset_std_mm']:.1f} mm",
            f"  当前 YAML          {summary['current_yaml_offset']:+.4f} m",
            f"  差异               {summary['delta_m']:+.4f} m  ({summary['delta_cm']:+.1f} cm)",
            f"",
            f"【建议写入 YAML】",
            f"  {summary['recommendation_yaml']}",
        ]

    with txt_path.open("w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    return json_path, txt_path


# ─────────────────────────────────────────────────────────────
# 主程序
# ─────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="A1.1 RTK 天线相对 base_link 偏移标定",
        epilog="详见精度提升路线图 §2.2 A1 步骤 1",
    )
    parser.add_argument("--base-link-e", type=float, metavar="E",
                        help="丰疆测量的 base_link 中心 Easting（EPSG:2100）")
    parser.add_argument("--base-link-n", type=float, metavar="N",
                        help="丰疆测量的 base_link 中心 Northing（EPSG:2100）")
    parser.add_argument("--frames", type=int, default=10,
                        help="每次采集帧数（默认 10，10Hz RTK 约 1s）")
    parser.add_argument("--topic", default="/epsg_position",
                        help="RTK 话题（默认 /epsg_position）")
    parser.add_argument("--current-offset", type=float, default=-0.4477,
                        metavar="X",
                        help="当前 YAML rtk_x_offset（默认 -0.4477）")
    parser.add_argument("--per-round-base-link", action="store_true",
                        help="每次采集前重新输入当前 base_link 坐标；用于转向后无法保持同一 P 点的现场流程")
    args = parser.parse_args()

    # 时间戳（文件名和 JSON 共用）
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    # ── 获取 base_link 坐标
    fixed_base_link = not args.per_round_base_link
    p_e: Optional[float] = None
    p_n: Optional[float] = None
    if fixed_base_link and args.base_link_e is not None and args.base_link_n is not None:
        p_e, p_n = args.base_link_e, args.base_link_n
        print(f"\n  base_link P（命令行）：E={p_e:.4f}  N={p_n:.4f}")
    elif fixed_base_link:
        p_e, p_n = ask_base_link()
    else:
        if (args.base_link_e is None) != (args.base_link_n is None):
            print("  ⚠ --base-link-e 和 --base-link-n 必须同时提供；per-round 模式下将忽略不完整输入。")

    config = {
        "base_link_mode"    : "per_round" if args.per_round_base_link else "fixed",
        "base_link_e"       : p_e,
        "base_link_n"       : p_n,
        "frames_per_round"  : args.frames,
        "topic"             : args.topic,
        "current_yaml_offset": args.current_offset,
    }

    # ── 初始化 ROS2
    rclpy.init()
    collector = RtkCollector(args.topic, args.frames)

    print()
    print(SEP)
    print("  标定准备")
    print(SEP)
    if fixed_base_link:
        print(f"  base_link P   E={p_e:.4f}  N={p_n:.4f}")
    else:
        print("  base_link P   每次采集前重新输入当前 P_i")
    print(f"  采集帧数      {args.frames} 帧/次（10Hz RTK 约 {args.frames/10:.0f}s）")
    print(f"  话题          {args.topic}")
    print(f"  当前 YAML     rtk_x_offset={args.current_offset}")
    print()
    print("  请确认：")
    print("  1. 机器人完全静止")
    if fixed_base_link:
        print("  2. base_link 中心垂直对准地面物理标记点 P")
    else:
        print("  2. 每次转向停稳后，用丰疆重新测当前 base_link 投影点 P_i")
    print("  3. tcp_base_ctl.service 在运行（ins_parser 节点在线）")
    print(SEP)

    rounds: List[Dict] = []
    round_idx = 1

    try:
        while True:
            if not fixed_base_link:
                p_e, p_n = ask_base_link(round_idx)

            input(f"\n  按 Enter 开始第 {round_idx} 次采集（朝向 {round_idx}）...")
            result = measure_one_round(collector, p_e, p_n, round_idx)

            if result is None:
                if ask_yes("  重试当前朝向？(y/n，默认 y) ", default_yes=True):
                    continue
                break

            rounds.append(result)
            round_idx += 1

            print()
            print("  建议：旋转机器人约 90° 到新朝向验证一致性")
            if not ask_yes("  继续另一朝向测量？(y/n，默认 n) ", default_yes=False):
                break

            print()
            if fixed_base_link:
                print("  请旋转机器人到新朝向，保持 base_link 仍在标记点 P 上。")
            else:
                print("  请旋转机器人到新朝向并停稳；下一轮会重新输入当前 P_i。")

    except KeyboardInterrupt:
        print("\n\n  ⚠ 用户中断，保存已有数据...")

    finally:
        rclpy.shutdown()

        if not rounds:
            print("  无有效数据，退出。")
            return

        # 计算汇总
        summary = build_summary(rounds, args.current_offset)

        # 终端打印汇总
        print_summary(summary, rounds)

        # 保存文件
        try:
            json_path, txt_path = save_results(ts, config, rounds, summary)
            print()
            print("  ■ 结果已保存")
            print(f"  JSON（原始帧 + 全量数据）：{json_path}")
            print(f"  文本报告（摘要 + 建议）  ：{txt_path}")
            print()
            print("  后期分析：")
            print(f"    python3 -c \"import json; d=json.load(open('{json_path}')); ")
            print(f"    print(d['summary'])\"")
        except Exception as exc:
            print(f"\n  ⚠ 保存失败：{exc}")

        print()
        print(SEP)


if __name__ == "__main__":
    main()
