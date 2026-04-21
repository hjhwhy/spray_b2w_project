#!/usr/bin/env python3
import argparse
import re
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def format_coord(value: float) -> str:
    text = f"{value:.3f}"
    text = text.rstrip("0").rstrip(".")
    return text if text else "0"


def get_next_point_index(file_path: Path) -> int:
    if not file_path.exists():
        return 1

    pattern = re.compile(r"^\s*pt(\d+),")
    max_index = 0

    with file_path.open("r", encoding="utf-8") as f:
        for line in f:
            match = pattern.match(line)
            if match:
                max_index = max(max_index, int(match.group(1)))

    return max_index + 1


def ensure_append_newline(file_path: Path) -> bool:
    if not file_path.exists() or file_path.stat().st_size == 0:
        return False

    with file_path.open("rb") as f:
        f.seek(-1, 2)
        return f.read(1) != b"\n"


class EpsgWaypointRecorder(Node):
    def __init__(self, topic_name: str, file_path: Path) -> None:
        super().__init__("epsg_waypoint_recorder")
        self.file_path = file_path
        self.subscription = self.create_subscription(
            PoseStamped,
            topic_name,
            self.pose_callback,
            10,
        )
        self.saved = False
        self.get_logger().info(f"Waiting for latest PoseStamped on {topic_name} ...")

    def pose_callback(self, msg: PoseStamped) -> None:
        if self.saved:
            return

        self.file_path.parent.mkdir(parents=True, exist_ok=True)
        point_index = get_next_point_index(self.file_path)
        line = (
            f"pt{point_index},"
            f"{format_coord(msg.pose.position.x)},"
            f"{format_coord(msg.pose.position.y)},"
            f"{format_coord(msg.pose.position.z)},\n"
        )

        need_newline = ensure_append_newline(self.file_path)
        with self.file_path.open("a", encoding="utf-8") as f:
            if need_newline:
                f.write("\n")
            f.write(line)

        self.saved = True
        self.get_logger().info(
            "Saved %s -> x=%s, y=%s, z=%s"
            % (
                f"pt{point_index}",
                format_coord(msg.pose.position.x),
                format_coord(msg.pose.position.y),
                format_coord(msg.pose.position.z),
            )
        )


def main() -> int:
    default_file = Path(__file__).resolve().parent / "gnss_waypoints.txt"

    parser = argparse.ArgumentParser(
        description="Subscribe to /epsg_position once and append the latest point to gnss_waypoints.txt."
    )
    parser.add_argument(
        "--topic",
        default="/epsg_position",
        help="PoseStamped topic to subscribe to. Default: /epsg_position",
    )
    parser.add_argument(
        "--file",
        default=str(default_file),
        help=f"Waypoint file path. Default: {default_file}",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Timeout in seconds while waiting for a fresh topic message. Default: 10",
    )
    args = parser.parse_args()

    file_path = Path(args.file).expanduser().resolve()

    rclpy.init()
    node = EpsgWaypointRecorder(args.topic, file_path)

    try:
        rclpy.spin_once(node, timeout_sec=args.timeout)
        if not node.saved:
            node.get_logger().error(
                f"No message received from {args.topic} within {args.timeout} seconds."
            )
            node.destroy_node()
            rclpy.shutdown()
            return 1
        node.destroy_node()
        return 0
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
        node.destroy_node()
        rclpy.shutdown()
        return 130


if __name__ == "__main__":
    sys.exit(main())
