#!/usr/bin/env python3

import argparse
import math
import shlex
import subprocess
import sys


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return x, y, z, w


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate or send a /z1_move_to_target service call from XYZ + RPY."
    )
    parser.add_argument("--x", type=float, required=True, help="Target x in meters")
    parser.add_argument("--y", type=float, required=True, help="Target y in meters")
    parser.add_argument("--z", type=float, required=True, help="Target z in meters")
    parser.add_argument("--roll", type=float, default=0.0, help="Roll angle")
    parser.add_argument("--pitch", type=float, default=90.0, help="Pitch angle")
    parser.add_argument("--yaw", type=float, default=0.0, help="Yaw angle")
    parser.add_argument(
        "--degrees",
        action="store_true",
        help="Treat roll/pitch/yaw as degrees. Default input unit.",
    )
    parser.add_argument(
        "--radians",
        action="store_true",
        help="Treat roll/pitch/yaw as radians.",
    )
    parser.add_argument(
        "--send",
        action="store_true",
        help="Send the ros2 service call directly instead of only printing it.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    use_radians = args.radians
    if args.degrees and args.radians:
        print("--degrees and --radians cannot be used together", file=sys.stderr)
        return 2

    roll = args.roll if use_radians else math.radians(args.roll)
    pitch = args.pitch if use_radians else math.radians(args.pitch)
    yaw = args.yaw if use_radians else math.radians(args.yaw)

    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
    payload = (
        '{target_pose: {position: {x: %.4f, y: %.4f, z: %.4f}, '
        'orientation: {x: %.6f, y: %.6f, z: %.6f, w: %.6f}}}'
        % (args.x, args.y, args.z, qx, qy, qz, qw)
    )

    command = [
        "ros2",
        "service",
        "call",
        "/z1_move_to_target",
        "z1_arm_controller_cpp/srv/MoveArm",
        payload,
    ]

    print("RPY radians:")
    print("  roll=%.6f pitch=%.6f yaw=%.6f" % (roll, pitch, yaw))
    print("Quaternion:")
    print("  x=%.6f y=%.6f z=%.6f w=%.6f" % (qx, qy, qz, qw))
    print("ros2 command:")
    print("  %s" % shlex.join(command))

    if not args.send:
        return 0

    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())