#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Grid:
    width: int
    height: int
    step: float
    origin_x: float
    origin_y: float
    values: list[list[float]]
    x_direction: int = 1
    y_direction: int = -1

    @classmethod
    def load(cls, path: Path) -> "Grid":
        lines = [line.strip() for line in path.read_text(errors="ignore").splitlines() if line.strip()]
        height = int(float(lines[0]))
        width = int(float(lines[1]))
        step = float(lines[2])
        origin_x = float(lines[3])
        origin_y = float(lines[4])
        values = [[float(v) for v in line.split()] for line in lines[5:]]
        if len(values) != height:
            raise ValueError(f"{path}: expected {height} rows, got {len(values)}")
        for idx, row in enumerate(values):
            if len(row) != width:
                raise ValueError(f"{path}: row {idx} expected {width} cols, got {len(row)}")
        return cls(width, height, step, origin_x, origin_y, values)

    def sample(self, x: float, y: float, x_dir: int = 1, y_dir: int = -1) -> float:
        col = (x - self.origin_x) / self.step if x_dir >= 0 else (self.origin_x - x) / self.step
        row = (self.origin_y - y) / self.step if y_dir < 0 else (y - self.origin_y) / self.step
        if col < 0 or row < 0 or col > self.width - 1 or row > self.height - 1:
            raise ValueError(f"point out of grid range: x={x}, y={y}")
        c0 = int(math.floor(col))
        r0 = int(math.floor(row))
        c1 = min(c0 + 1, self.width - 1)
        r1 = min(r0 + 1, self.height - 1)
        dc = col - c0
        dr = row - r0
        v00 = self.values[r0][c0]
        v10 = self.values[r0][c1]
        v01 = self.values[r1][c0]
        v11 = self.values[r1][c1]
        return (
            v00 * (1 - dc) * (1 - dr)
            + v10 * dc * (1 - dr)
            + v01 * (1 - dc) * dr
            + v11 * dc * dr
        )


def dms_to_decimal(deg: int, minute: int, second: float, hemi: str) -> float:
    val = deg + minute / 60.0 + second / 3600.0
    return -val if hemi in {"S", "W"} else val


def parse_latlon(text: str) -> tuple[float, float]:
    text = text.strip().strip('"')
    m = re.search(
        r"(\d+)°(\d+)'([\d.]+)″([NS])\s*/\s*(\d+)°(\d+)'([\d.]+)″([EW])",
        text,
    )
    if not m:
        raise ValueError(f"cannot parse DMS: {text}")
    lat = dms_to_decimal(int(m.group(1)), int(m.group(2)), float(m.group(3)), m.group(4))
    lon = dms_to_decimal(int(m.group(5)), int(m.group(6)), float(m.group(7)), m.group(8))
    return lat, lon


def helmert_7(lat: float, lon: float, h: float) -> tuple[float, float, float]:
    dx = 203.437
    dy = -73.461
    dz = -243.594
    alpha = math.radians(-0.17 / 3600.0)
    beta = math.radians(-0.06 / 3600.0)
    gamma = math.radians(-0.151 / 3600.0)

    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f * f
    lat_r = math.radians(lat)
    lon_r = math.radians(lon)
    n = a / math.sqrt(1 - e2 * math.sin(lat_r) ** 2)
    x = (n + h) * math.cos(lat_r) * math.cos(lon_r)
    y = (n + h) * math.cos(lat_r) * math.sin(lon_r)
    z = (n * (1 - e2) + h) * math.sin(lat_r)

    # coordinate_frame small-angle 7-parameter transform
    x2 = dx + x - gamma * y + beta * z
    y2 = dy + gamma * x + y - alpha * z
    z2 = dz - beta * x + alpha * y + z
    return x2, y2, z2


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    a = 6378137.0
    f = 1 / 298.257222101
    e2 = 2 * f - f * f
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - e2))
    for _ in range(20):
        n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat_next = math.atan2(z, p * (1 - e2 * n / (n + h)))
        if abs(lat_next - lat) < 1e-14:
            lat = lat_next
            break
        lat = lat_next
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    h = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), h


def tm87(lat: float, lon: float) -> tuple[float, float]:
    a = 6378137.0
    f = 1 / 298.257223563
    e2 = 2 * f - f * f
    ep2 = e2 / (1 - e2)
    k0 = 0.9996
    lon0 = math.radians(24.0)

    lat_r = math.radians(lat)
    lon_r = math.radians(lon)
    n = a / math.sqrt(1 - e2 * math.sin(lat_r) ** 2)
    t = math.tan(lat_r) ** 2
    c = ep2 * math.cos(lat_r) ** 2
    a1 = (lon_r - lon0) * math.cos(lat_r)
    m = a * (
        (1 - e2 / 4 - 3 * e2**2 / 64 - 5 * e2**3 / 256) * lat_r
        - (3 * e2 / 8 + 3 * e2**2 / 32 + 45 * e2**3 / 1024) * math.sin(2 * lat_r)
        + (15 * e2**2 / 256 + 45 * e2**3 / 1024) * math.sin(4 * lat_r)
        - (35 * e2**3 / 3072) * math.sin(6 * lat_r)
    )
    x = 500000 + k0 * n * (
        a1
        + (1 - t + c) * a1**3 / 6
        + (5 - 18 * t + t * t + 72 * c - 58 * ep2) * a1**5 / 120
    )
    y = k0 * (
        m
        + n * math.tan(lat_r) * (
            a1 * a1 / 2
            + (5 - t + 9 * c + 4 * c * c) * a1**4 / 24
            + (61 - 58 * t + t * t + 600 * c - 330 * ep2) * a1**6 / 720
        )
    )
    return x, y


def read_points(path: Path):
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("type") != "GPS":
                continue
            yield row["point"], row["latitude"], row["longitude"], row["gps_height"]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="logs/5-4/fj/fj.csv")
    parser.add_argument("--output", default="logs/5-4/fj/fj_recomputed.csv")
    parser.add_argument("--grid-x-dir", type=int, default=1)
    parser.add_argument("--grid-y-dir", type=int, default=-1)
    args = parser.parse_args()

    base = Path("rtk_nav_ws/fj_dynamic")
    ngrid = Grid.load(base / "dN_2km_V1-0.ngrd")
    egrid = Grid.load(base / "dE_2km_V1-0.egrd")

    out_rows = []
    for point, lat_text, lon_text, h_text in read_points(Path(args.input)):
        lat, lon = parse_latlon(f"{lat_text} / {lon_text}")
        h = float(h_text)
        x1, y1, z1 = helmert_7(lat, lon, h)
        lat2, lon2, h2 = ecef_to_geodetic(x1, y1, z1)
        x, y = tm87(lat2, lon2)
        try:
            dx = egrid.sample(x, y, args.grid_x_dir, args.grid_y_dir)
            dy = ngrid.sample(x, y, args.grid_x_dir, args.grid_y_dir)
        except ValueError:
            dx = 0.0
            dy = 0.0
        out_rows.append([point, f"{lat_text}", f"{lon_text}", f"{h:.3f}", f"{x + dx:.3f}", f"{y + dy:.3f}", f"{h2:.3f}"])

    out_path = Path(args.output)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["point", "latitude", "longitude", "gps_height", "easting", "northing", "plane_height"])
        writer.writerows(out_rows)

    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
