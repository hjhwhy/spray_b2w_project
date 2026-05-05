#!/usr/bin/env python3
import argparse
import os
import pty
import re
import sys
import time
from pathlib import Path


DEFAULT_INPUT = "/home/oneko/projects/spray_b2w_robot_project_greek/logs/5-4/sn/test_grid.txt"


def nmea_checksum(sentence_body: str) -> str:
    value = 0
    for ch in sentence_body:
        value ^= ord(ch)
    return f"{value:02X}"


def with_checksum(sentence: str) -> str:
    sentence = sentence.strip()
    if sentence.startswith("$") or sentence.startswith("#"):
        prefix = sentence[0]
        body = sentence[1:].split("*", 1)[0]
        return f"{prefix}{body}*{nmea_checksum(body)}"
    return sentence


def parse_points(path: Path) -> list[tuple[str, str]]:
    text = path.read_text(encoding="utf-8", errors="ignore")
    blocks = re.split(r"(?=^pt\d+[:：])", text, flags=re.MULTILINE)
    points: list[tuple[str, str]] = []

    for block in blocks:
        match = re.search(r"^(pt\d+)[:：]", block, flags=re.MULTILINE)
        if not match:
            continue
        point_id = match.group(1)
        gpgga_match = re.search(r"(\$GPGGA,[^\s]+)", block)
        if gpgga_match:
            points.append((point_id, gpgga_match.group(1).strip()))

    if points:
        return points

    # test_grid.txt only contains raw log lines without ptN headers.
    gpgga_lines = re.findall(r"(\$GPGGA,[^\s]+)", text)
    return [(f"pt{i + 1}", gpgga.strip()) for i, gpgga in enumerate(gpgga_lines)]


def build_headinga(yaw_deg: float) -> str:
    # parse_headinga() searches SOL_COMPUTED and reads yaw at status_idx + 3.
    body = (
        "#HEADINGA,COM1,0,0,FINESTEERING,0,0,0,0,"
        f"SOL_COMPUTED,NARROW_INT,0.010,{yaw_deg:.6f},0.0,0.0,0.0"
    )
    return with_checksum(body)


def send_line(master_fd: int, line: str, dry_run: bool) -> None:
    payload = (line.strip() + "\r\n").encode("ascii", errors="ignore")
    if dry_run:
        print(payload.decode("ascii").rstrip())
        return
    os.write(master_fd, payload)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay five GNSS points from gnss.txt through a pseudo serial port for ins_parser testing."
    )
    parser.add_argument("--input", default=DEFAULT_INPUT, help="Path to gnss.txt")
    parser.add_argument("--interval", type=float, default=0.05, help="Seconds between samples")
    parser.add_argument("--samples-per-point", type=int, default=2, help="Samples sent for each point")
    parser.add_argument("--repeat", type=int, default=1, help="Repeat the five-point sequence, use 0 for infinite")
    parser.add_argument("--yaw", type=float, default=187.864304, help="Simulated HEADINGA yaw in degrees")
    parser.add_argument("--dry-run", action="store_true", help="Only print replay lines, do not create PTY")
    args = parser.parse_args()

    points = parse_points(Path(args.input))
    if not points:
        print(f"No GPGGA points found in {args.input}", file=sys.stderr)
        return 1

    points = points[:5]
    headinga = build_headinga(args.yaw)

    master_fd = -1
    if not args.dry_run:
        master_fd, slave_fd = pty.openpty()
        slave_name = os.ttyname(slave_fd)
        os.close(slave_fd)
        print("Pseudo serial port ready.")
        print(f"Use this port for ins_parser: {slave_name}")
        print("Example:")
        print(f"  ros2 run ins_driver_node ins_parser --ros-args -p port:={slave_name}")
        print(f"Replay plan: {len(points)} points, {args.samples_per_point} samples per point, repeat={args.repeat}")
        print("The script exits automatically after replay unless --repeat 0 is used.")
        sys.stdout.flush()

    try:
        loop = 0
        while args.repeat == 0 or loop < args.repeat:
            loop += 1
            for point_id, gpgga in points:
                gpgga = with_checksum(gpgga)
                print(f"TX {point_id}: {gpgga} x {args.samples_per_point}")
                for _ in range(args.samples_per_point):
                    send_line(master_fd, gpgga, args.dry_run)
                    time.sleep(0.02)
                    send_line(master_fd, headinga, args.dry_run)
                    time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if master_fd >= 0:
            os.close(master_fd)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
