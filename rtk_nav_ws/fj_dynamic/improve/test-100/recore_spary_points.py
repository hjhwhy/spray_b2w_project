#!/usr/bin/env python3
"""
Record Sinan RTK measurements for the 100-point spray validation task.

Default outputs, relative to this script:
  - spray_sn_100.txt: waypoint-like compact rows:
        pt1,E,N,Z,
  - spray_sn_100_detail.csv: averaged sample diagnostics for later analysis

Usage on robot:
  source /opt/ros/humble/setup.bash
  source /home/test/rtk_nav_ws/install/setup.bash
  python3 test-100/recore_spary_points.py

At each sprayed physical point, place the Sinan RTK antenna on the spray mark,
wait for the pole/antenna to settle, then run the command once. The script
appends the next ptNNN row automatically.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_POINT_FILE = SCRIPT_DIR / "spray_sn_100.txt"
DEFAULT_DETAIL_FILE = SCRIPT_DIR / "spray_sn_100_detail.csv"


def format_coord(value: float) -> str:
    return format(value, ".16g")


def format_mm(value_m: float) -> str:
    return format(value_m * 1000.0, ".3f")


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def circular_mean(angles: List[float]) -> float:
    s = sum(math.sin(a) for a in angles)
    c = sum(math.cos(a) for a in angles)
    return math.atan2(s, c)


def stdev(values: List[float]) -> float:
    if len(values) < 2:
        return 0.0
    return statistics.pstdev(values)


def ensure_append_newline(file_path: Path) -> bool:
    if not file_path.exists() or file_path.stat().st_size == 0:
        return False
    with file_path.open("rb") as f:
        f.seek(-1, 2)
        return f.read(1) != b"\n"


def normalize_point_id(raw: str) -> str:
    raw = raw.strip()
    if not raw:
        raise ValueError("empty point id")
    if raw.isdigit():
        return f"pt{int(raw)}"
    match = re.fullmatch(r"pt0*(\d+)", raw, re.IGNORECASE)
    if match:
        return f"pt{int(match.group(1))}"
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", raw):
        raise ValueError("point id may only contain letters, numbers, '_', '.', '-'")
    return raw


def point_sort_value(point_id: str) -> int:
    match = re.fullmatch(r"pt0*(\d+)", point_id, re.IGNORECASE)
    if not match:
        return 0
    return int(match.group(1))


def get_next_point_id(point_file: Path, prefix: str = "pt") -> str:
    if not point_file.exists():
        return f"{prefix}1"

    pattern = re.compile(rf"^\s*{re.escape(prefix)}0*(\d+),", re.IGNORECASE)
    max_index = 0
    with point_file.open("r", encoding="utf-8") as f:
        for line in f:
            match = pattern.match(line)
            if match:
                max_index = max(max_index, int(match.group(1)))
    return f"{prefix}{max_index + 1}"


class SprayPointRecorder(Node):
    def __init__(self, topic_name: str, n_frames: int) -> None:
        super().__init__("spray_sn_point_recorder")
        self.n_frames = n_frames
        self.samples: List[Dict[str, float]] = []

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscription = self.create_subscription(
            PoseStamped,
            topic_name,
            self.pose_callback,
            qos,
        )
        self.get_logger().info(
            f"Waiting for {n_frames} PoseStamped frames on {topic_name} ..."
        )

    def pose_callback(self, msg: PoseStamped) -> None:
        if len(self.samples) >= self.n_frames:
            return
        pose = msg.pose
        yaw = quat_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self.samples.append(
            {
                "e": pose.position.x,
                "n": pose.position.y,
                "z": pose.position.z,
                "yaw_rad": yaw,
            }
        )

        pct = len(self.samples) / self.n_frames * 100.0
        bar = "#" * int(pct / 5.0) + "-" * (20 - int(pct / 5.0))
        print(
            f"\r  [{bar}] {len(self.samples):>3}/{self.n_frames} "
            f"E={pose.position.x:.4f} N={pose.position.y:.4f} Z={pose.position.z:.4f}",
            end="",
            flush=True,
        )

    def collect(self, timeout: float) -> Optional[Dict[str, float]]:
        deadline = time.time() + timeout
        while len(self.samples) < self.n_frames:
            if time.time() > deadline:
                print()
                return None
            rclpy.spin_once(self, timeout_sec=0.2)
        print()

        es = [s["e"] for s in self.samples]
        ns = [s["n"] for s in self.samples]
        zs = [s["z"] for s in self.samples]
        yaws = [s["yaw_rad"] for s in self.samples]
        return {
            "e_mean": sum(es) / len(es),
            "n_mean": sum(ns) / len(ns),
            "z_mean": sum(zs) / len(zs),
            "yaw_deg_mean": math.degrees(circular_mean(yaws)),
            "e_std_m": stdev(es),
            "n_std_m": stdev(ns),
            "z_std_m": stdev(zs),
            "frames": len(self.samples),
        }


def append_point_file(point_file: Path, point_id: str, stats: Dict[str, float]) -> None:
    point_file.parent.mkdir(parents=True, exist_ok=True)
    need_newline = ensure_append_newline(point_file)
    line = (
        f"{point_id},"
        f"{format_coord(stats['e_mean'])},"
        f"{format_coord(stats['n_mean'])},"
        f"{format_coord(stats['z_mean'])},\n"
    )
    with point_file.open("a", encoding="utf-8") as f:
        if need_newline:
            f.write("\n")
        f.write(line)


def append_detail_file(
    detail_file: Path,
    point_id: str,
    stats: Dict[str, float],
    topic: str,
) -> None:
    detail_file.parent.mkdir(parents=True, exist_ok=True)
    exists = detail_file.exists() and detail_file.stat().st_size > 0
    fieldnames = [
        "id",
        "sn_E",
        "sn_N",
        "sn_Z",
        "yaw_deg",
        "e_std_mm",
        "n_std_mm",
        "z_std_mm",
        "frames",
        "topic",
        "timestamp",
    ]
    with detail_file.open("a", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if not exists:
            writer.writeheader()
        writer.writerow(
            {
                "id": point_id,
                "sn_E": format_coord(stats["e_mean"]),
                "sn_N": format_coord(stats["n_mean"]),
                "sn_Z": format_coord(stats["z_mean"]),
                "yaw_deg": format(stats["yaw_deg_mean"], ".6f"),
                "e_std_mm": format_mm(stats["e_std_m"]),
                "n_std_mm": format_mm(stats["n_std_m"]),
                "z_std_mm": format_mm(stats["z_std_m"]),
                "frames": int(stats["frames"]),
                "topic": topic,
                "timestamp": datetime.now().isoformat(timespec="seconds"),
            }
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Record one averaged Sinan RTK spray point and append it to "
            "test-100/spray_sn_100.txt."
        )
    )
    parser.add_argument(
        "--topic",
        default="/epsg_position",
        help="PoseStamped topic to subscribe to. Default: /epsg_position",
    )
    parser.add_argument(
        "--file",
        default=str(DEFAULT_POINT_FILE),
        help=f"Compact output file. Default: {DEFAULT_POINT_FILE}",
    )
    parser.add_argument(
        "--detail-file",
        default=str(DEFAULT_DETAIL_FILE),
        help=f"Detail CSV output file. Default: {DEFAULT_DETAIL_FILE}",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=20,
        help="Number of RTK frames to average. Default: 20",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Timeout in seconds while waiting for frames. Default: 30",
    )
    parser.add_argument(
        "--pt-id",
        help="Explicit point id, e.g. pt001 or 1. Default: next id in output file",
    )
    args = parser.parse_args()

    if args.frames < 1:
        print("frames must be >= 1", file=sys.stderr)
        return 2

    point_file = Path(args.file).expanduser().resolve()
    detail_file = Path(args.detail_file).expanduser().resolve()
    point_id = (
        normalize_point_id(args.pt_id)
        if args.pt_id
        else get_next_point_id(point_file)
    )

    rclpy.init()
    node = SprayPointRecorder(args.topic, args.frames)

    try:
        stats = node.collect(args.timeout)
        if stats is None:
            node.get_logger().error(
                f"No enough messages from {args.topic} within {args.timeout} seconds."
            )
            node.destroy_node()
            rclpy.shutdown()
            return 1

        append_point_file(point_file, point_id, stats)
        append_detail_file(detail_file, point_id, stats, args.topic)

        node.get_logger().info(
            "Saved %s -> E=%s, N=%s, Z=%s, std=(%s,%s,%s) mm"
            % (
                point_id,
                format_coord(stats["e_mean"]),
                format_coord(stats["n_mean"]),
                format_coord(stats["z_mean"]),
                format_mm(stats["e_std_m"]),
                format_mm(stats["n_std_m"]),
                format_mm(stats["z_std_m"]),
            )
        )
        node.destroy_node()
        rclpy.shutdown()
        return 0
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
        node.destroy_node()
        rclpy.shutdown()
        return 130


if __name__ == "__main__":
    sys.exit(main())
