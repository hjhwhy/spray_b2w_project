#!/usr/bin/env python3
"""
RTK 录点链路分析脚本（司南 INS / ins_parser_node）—— 独立于实机录点脚本

定位：
  - 实机录入  : record_epsg_waypoint.py，写 gnss_waypoints.txt（每点 1 行）。
  - 分析记录 : 本脚本，写 gnss_waypoints_detail.txt（每点一个 block）。
  两个脚本互不依赖，序号互不影响，同一个物理点既可以用前者落地为
  waypoint，也可以用本脚本生成分析样本。

每次运行：
  1. 等待 N 帧（默认 3）配对的 /gps + /epsg_position。
  2. 把每一帧的完整链路写入 gnss_waypoints_detail.txt（追加），包含：
        - 原始 GGA 经纬度 / 度分秒 / 高度 / NavSatStatus
        - 本地复算 PROJ default（pyproj 可用时）
        - 本地复算 HEPOS dE/dN（读 fj_dynamic/*.egrd/.ngrd 双线性插值）
        - /epsg_position 节点最终输出（HEPOS-correct）
        - 复算 vs 节点输出的一致性残差（正常 mm 级）
  3. 在 block 末尾输出 N 帧的均值/标准差，以及 "等价 waypoint 行"
     （pt{idx},x,y,z,），方便对照 / 直接复制到 gnss_waypoints.txt。

序号：默认从 gnss_waypoints_detail.txt 已有的 `=== record pt{N} ` 扫描自增；
也可用 --pt-id 显式指定（对齐实机刚录的某条 pt）。

依赖：rclpy（必需）；pyproj（可选，装上则附加 PROJ default 复算）。
兼容 Python 3.8+，不使用 PEP 604 联合类型注解。
"""
from __future__ import annotations

import argparse
import math
import re
import statistics
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix


DEFAULT_HEPOS_E_GRID = "/home/test/rtk_nav_ws/fj_dynamic/dE_2km_V1-0.egrd"
DEFAULT_HEPOS_N_GRID = "/home/test/rtk_nav_ws/fj_dynamic/dN_2km_V1-0.ngrd"
DEFAULT_HEPOS_ORIGIN_X = -64400.0
DEFAULT_HEPOS_ORIGIN_Y = 3840000.0

# ins_parser 默认把原始 GPGGA 写入 $HOME/logs/ins_parser_runs/gpgga_raw_*.log
DEFAULT_GPGGA_RAW_DIR = str(Path.home() / "logs" / "ins_parser_runs")
GPGGA_RAW_TS_RE = re.compile(
    r"^\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})\]\s+\[RAW\]\s+#\d+\s+(\$.+)$"
)


def fmt_coord(value: float) -> str:
    return format(value, ".16g")


def fmt_local_time(sec: int, nanosec: int) -> str:
    if sec == 0 and nanosec == 0:
        return "(no stamp)"
    dt = datetime.fromtimestamp(sec + nanosec * 1e-9, tz=timezone.utc).astimezone()
    return dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def deg_to_dms(value_deg: float, positive: str, negative: str) -> str:
    sign = positive if value_deg >= 0 else negative
    v = abs(value_deg)
    d = int(v)
    m_full = (v - d) * 60.0
    m = int(m_full)
    s = (m_full - m) * 60.0
    return f"{d}°{m:02d}'{s:08.5f}\"{sign}"


def quat_to_yaw_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny, cosy))


def get_next_point_index(file_path: Path) -> int:
    """扫描 detail 文件里的 `=== record pt{N} ` 标题，返回最大值 + 1。"""
    if not file_path.exists():
        return 1
    pattern = re.compile(r"=== record pt(\d+)\b")
    max_index = 0
    with file_path.open("r", encoding="utf-8") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                max_index = max(max_index, int(match.group(1)))
    return max_index + 1


def ensure_append_newline(file_path: Path) -> bool:
    if not file_path.exists() or file_path.stat().st_size == 0:
        return False
    with file_path.open("rb") as f:
        f.seek(-1, 2)
        return f.read(1) != b"\n"


# --------------------- HEPOS grid（纯 Python 实现） ---------------------

@dataclass
class HeposGrid:
    rows: int
    cols: int
    grid_size: float
    header4: float
    header5: float
    data: List[List[float]] = field(default_factory=list)

    @classmethod
    def load(cls, path: Path) -> "HeposGrid":
        with path.open("r", encoding="utf-8") as f:
            tokens: List[str] = f.read().split()
        if len(tokens) < 5:
            raise ValueError(f"HEPOS grid {path} 头部不完整")
        rows = int(float(tokens[0]))
        cols = int(float(tokens[1]))
        grid_size = float(tokens[2])
        header4 = float(tokens[3])
        header5 = float(tokens[4])
        body = tokens[5:]
        if len(body) != rows * cols:
            raise ValueError(
                f"HEPOS grid {path} 数据长度不匹配: expect {rows * cols}, got {len(body)}"
            )
        flat = [float(t) for t in body]
        data = [flat[i * cols : (i + 1) * cols] for i in range(rows)]
        return cls(rows, cols, grid_size, header4, header5, data)


@dataclass
class HeposCorrector:
    e_grid: HeposGrid
    n_grid: HeposGrid
    origin_x: float = DEFAULT_HEPOS_ORIGIN_X
    origin_y: float = DEFAULT_HEPOS_ORIGIN_Y

    def index(self, x: float, y: float) -> Tuple[float, float]:
        col = (x - self.origin_x) / self.e_grid.grid_size
        row = (y - self.origin_y) / self.e_grid.grid_size
        return col, row

    def sample(self, grid: HeposGrid, col: float, row: float) -> Optional[float]:
        if col < 0.0 or row < 0.0:
            return None
        if col > grid.cols - 1 or row > grid.rows - 1:
            return None
        c0 = int(math.floor(col))
        r0 = int(math.floor(row))
        c1 = min(c0 + 1, grid.cols - 1)
        r1 = min(r0 + 1, grid.rows - 1)
        tx = col - c0
        ty = row - r0
        v00 = grid.data[r0][c0]
        v10 = grid.data[r0][c1]
        v01 = grid.data[r1][c0]
        v11 = grid.data[r1][c1]
        v0 = v00 * (1.0 - tx) + v10 * tx
        v1 = v01 * (1.0 - tx) + v11 * tx
        return v0 * (1.0 - ty) + v1 * ty

    def correct(self, x_proj: float, y_proj: float) -> dict:
        col, row = self.index(x_proj, y_proj)
        de_cm = self.sample(self.e_grid, col, row)
        dn_cm = self.sample(self.n_grid, col, row)
        if de_cm is None or dn_cm is None:
            return {"applied": False, "col": col, "row": row}
        return {
            "applied": True,
            "col": col,
            "row": row,
            "de_cm": de_cm,
            "dn_cm": dn_cm,
            "de_m": de_cm * 0.01,
            "dn_m": dn_cm * 0.01,
            "x_corrected": x_proj + de_cm * 0.01,
            "y_corrected": y_proj + dn_cm * 0.01,
        }


# --------------------- 原始 GPGGA 解析（从 ins_parser 本地 raw log 读取） ---------------------

class RawGpggaResolver:
    """按 ROS 时间戳去 ins_parser 的 gpgga_raw_*.log 找最近的 $GPGGA 原文。

    日志格式（ins_parser.cpp:435 write_gpgga_log）：
        [YYYY-MM-DD HH:MM:SS] [RAW]    #N $GPGGA,...
        [YYYY-MM-DD HH:MM:SS] [PARSED] #N Qual=...

    时间戳为 ins_parser 收到该报文时的本地时钟，秒级精度。
    """

    def __init__(self, raw_log_dir: Path) -> None:
        self.raw_log_dir = raw_log_dir
        self.log_path: Optional[Path] = None
        self.entries: List[Tuple[float, str]] = []  # (epoch_sec, raw_sentence)
        self.status = "(not loaded)"

    def discover(self) -> None:
        """挑当前目录里 mtime 最新的 gpgga_raw_*.log。"""
        if not self.raw_log_dir.exists():
            self.status = f"目录不存在: {self.raw_log_dir}"
            return
        candidates = sorted(
            self.raw_log_dir.glob("gpgga_raw_*.log"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        if not candidates:
            self.status = f"未找到 gpgga_raw_*.log: {self.raw_log_dir}"
            return
        self.log_path = candidates[0]
        self.status = f"using {self.log_path.name}"

    def reload(self) -> None:
        """重新读全文（采集 3 帧 ~3s，文件不会很大）。"""
        if self.log_path is None:
            return
        try:
            text = self.log_path.read_text(encoding="utf-8", errors="replace")
        except Exception as e:
            self.status = f"读取失败: {e}"
            return
        entries: List[Tuple[float, str]] = []
        for line in text.splitlines():
            m = GPGGA_RAW_TS_RE.match(line)
            if not m:
                continue
            ts_str, sentence = m.group(1), m.group(2)
            try:
                dt = datetime.strptime(ts_str, "%Y-%m-%d %H:%M:%S").astimezone()
                entries.append((dt.timestamp(), sentence))
            except ValueError:
                continue
        self.entries = entries
        self.status = f"using {self.log_path.name} ({len(entries)} RAW lines)"

    def find_nearest(self, ros_stamp_sec: float, max_dt_s: float = 2.0) -> Optional[str]:
        if not self.entries:
            return None
        best = None
        best_dt = float("inf")
        for ts, sent in self.entries:
            dt = abs(ts - ros_stamp_sec)
            if dt < best_dt:
                best = sent
                best_dt = dt
        if best is None or best_dt > max_dt_s:
            return None
        return best


# --------------------- pyproj 可选 ---------------------

def make_proj_default():
    try:
        from pyproj import Transformer

        # 与狗端 use_epsg_crs_datum=true 一致：EPSG:4326 -> EPSG:2100，自动叠 GGRS87 datum
        transformer = Transformer.from_crs("EPSG:4326", "EPSG:2100", always_xy=True)

        def project(lon: float, lat: float) -> Tuple[float, float]:
            x, y = transformer.transform(lon, lat)
            return x, y

        return project, "pyproj EPSG:4326->EPSG:2100 (CRS-to-CRS, datum included)"
    except Exception as e:
        return None, f"pyproj 不可用: {e}"


# --------------------- 帧数据结构 ---------------------

@dataclass
class FramePair:
    gps: NavSatFix
    epsg: PoseStamped
    recv_time: float


# --------------------- ROS 节点 ---------------------

class GpsEpsgRecorder(Node):
    GPS_QUAL_LABEL = {
        0: "no fix",
        1: "GPS fix",
        2: "DGPS fix",
        3: "PPS fix",
        4: "RTK fixed",
        5: "RTK float",
        6: "estimated",
        7: "manual",
        8: "simulation",
    }

    def __init__(
        self,
        gps_topic: str,
        epsg_topic: str,
        frames: int,
        max_stamp_dt: float,
        require_qual: Optional[int],
    ) -> None:
        super().__init__("gps_epsg_point_recorder")
        self.frames_target = frames
        self.max_stamp_dt = max_stamp_dt
        self.require_qual = require_qual

        self.latest_gps: Optional[NavSatFix] = None
        self.latest_epsg: Optional[PoseStamped] = None
        self.last_logged_gps_stamp: Optional[Tuple[int, int]] = None
        self.last_logged_epsg_stamp: Optional[Tuple[int, int]] = None
        self.frames: List[FramePair] = []
        self.start_time = time.monotonic()
        self.last_progress = 0.0
        self.gps_count = 0
        self.epsg_count = 0
        self.skipped_quality = 0
        self.skipped_stamp = 0

        # ins_parser 默认是 RELIABLE，不必特意改 BEST_EFFORT；保持默认 + depth=10
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.gps_sub = self.create_subscription(NavSatFix, gps_topic, self._on_gps, qos)
        self.epsg_sub = self.create_subscription(PoseStamped, epsg_topic, self._on_epsg, qos)

        self.get_logger().info(
            f"等待配对消息: {gps_topic} + {epsg_topic} | 目标帧数={frames} "
            f"质量门限={'gps_qual>=' + str(require_qual) if require_qual is not None else '关闭'}"
        )

    def _on_gps(self, msg: NavSatFix) -> None:
        self.gps_count += 1
        self.latest_gps = msg
        self._maybe_pair()

    def _on_epsg(self, msg: PoseStamped) -> None:
        self.epsg_count += 1
        self.latest_epsg = msg
        self._maybe_pair()

    def _maybe_pair(self) -> None:
        if self.latest_gps is None or self.latest_epsg is None:
            return
        if len(self.frames) >= self.frames_target:
            return

        gps_stamp = (self.latest_gps.header.stamp.sec, self.latest_gps.header.stamp.nanosec)
        epsg_stamp = (self.latest_epsg.header.stamp.sec, self.latest_epsg.header.stamp.nanosec)

        # 不重复使用同一帧
        if gps_stamp == self.last_logged_gps_stamp or epsg_stamp == self.last_logged_epsg_stamp:
            return

        gps_t = gps_stamp[0] + gps_stamp[1] * 1e-9
        epsg_t = epsg_stamp[0] + epsg_stamp[1] * 1e-9
        dt = abs(gps_t - epsg_t)
        if gps_t > 0 and epsg_t > 0 and dt > self.max_stamp_dt:
            self.skipped_stamp += 1
            return

        # 质量门限
        if self.require_qual is not None:
            qual = self._infer_qual(self.latest_gps)
            if qual < self.require_qual:
                self.skipped_quality += 1
                # 只 mark gps_stamp：低质量的是当前 GPS 帧，等下条 GPS 来再判；
                # 当前 EPSG 帧本身没问题，下一条 GPS 来时仍可与它配对。
                self.last_logged_gps_stamp = gps_stamp
                self.get_logger().warn(
                    f"丢弃低质量帧: qual={qual} ({self.GPS_QUAL_LABEL.get(qual, '?')}) < {self.require_qual}"
                )
                return

        self.frames.append(FramePair(self.latest_gps, self.latest_epsg, time.monotonic()))
        self.last_logged_gps_stamp = gps_stamp
        self.last_logged_epsg_stamp = epsg_stamp
        self.get_logger().info(
            f"  收到第 {len(self.frames)}/{self.frames_target} 帧: "
            f"lat={self.latest_gps.latitude:.9f} lon={self.latest_gps.longitude:.9f} "
            f"E={self.latest_epsg.pose.position.x:.4f} N={self.latest_epsg.pose.position.y:.4f}"
        )

    def _infer_qual(self, msg: NavSatFix) -> int:
        # ins_parser 把 GPGGA quality 映射到 NavSatStatus.status：
        #   qual=4 (RTK fixed)        -> STATUS_GBAS_FIX = 2
        #   qual=2/5 (DGPS/RTK float) -> STATUS_SBAS_FIX = 1
        #   qual=1 (GPS fix)          -> STATUS_FIX      = 0
        #   qual=0 (no fix)           -> STATUS_NO_FIX   = -1
        # 这里从 status 反推一个保守的下界（无法区分 qual=4 vs 其它 GBAS 来源）
        s = msg.status.status
        if s == -1:
            return 0
        if s >= 2:
            return 4
        if s >= 1:
            return 2
        return 1

    def done(self) -> bool:
        return len(self.frames) >= self.frames_target


# --------------------- 写入逻辑 ---------------------

def write_detail_block(
    detail_path: Path,
    point_index: int,
    frames: List[FramePair],
    proj_default,
    proj_default_desc: str,
    hepos: Optional[HeposCorrector],
    hepos_desc: str,
    raw_resolver: Optional[RawGpggaResolver],
    e_mean: float,
    n_mean: float,
    z_mean: float,
    e_std: float,
    n_std: float,
    z_std: float,
) -> None:
    detail_path.parent.mkdir(parents=True, exist_ok=True)
    need_newline = ensure_append_newline(detail_path)
    now_local = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    with detail_path.open("a", encoding="utf-8") as f:
        if need_newline:
            f.write("\n")
        f.write(f"=== record pt{point_index} @ {now_local} ===\n")
        f.write(f"proj_default: {proj_default_desc}\n")
        f.write(f"hepos_grid  : {hepos_desc}\n")
        f.write(f"gpgga_raw   : {raw_resolver.status if raw_resolver else '(disabled)'}\n")
        f.write(f"frames      : {len(frames)}\n")

        for i, frame in enumerate(frames, 1):
            gps = frame.gps
            epsg = frame.epsg
            gps_t = fmt_local_time(gps.header.stamp.sec, gps.header.stamp.nanosec)
            epsg_t = fmt_local_time(epsg.header.stamp.sec, epsg.header.stamp.nanosec)
            dt = abs(
                (gps.header.stamp.sec + gps.header.stamp.nanosec * 1e-9)
                - (epsg.header.stamp.sec + epsg.header.stamp.nanosec * 1e-9)
            )

            yaw_deg = quat_to_yaw_deg(
                epsg.pose.orientation.x,
                epsg.pose.orientation.y,
                epsg.pose.orientation.z,
                epsg.pose.orientation.w,
            )

            f.write(f"\n[frame {i}/{len(frames)}]\n")
            f.write(f"  gps_stamp     : {gps_t}\n")
            f.write(f"  epsg_stamp    : {epsg_t}\n")
            f.write(f"  stamp_dt_s    : {dt:.6f}\n")
            f.write(f"  gps_status    : {gps.status.status} (NavSatStatus)\n")
            f.write(f"  service       : 0x{gps.status.service:04X}\n")

            if raw_resolver is not None:
                gps_t_epoch = (
                    gps.header.stamp.sec + gps.header.stamp.nanosec * 1e-9
                )
                raw = raw_resolver.find_nearest(gps_t_epoch)
                f.write(f"  raw_gpgga     : {raw if raw else '(no match within ±2s)'}\n")
            else:
                f.write(f"  raw_gpgga     : (disabled)\n")

            f.write(f"  WGS84\n")
            f.write(f"    lat_deg     : {gps.latitude:.10f}\n")
            f.write(f"    lon_deg     : {gps.longitude:.10f}\n")
            f.write(f"    alt_m       : {gps.altitude:.4f}\n")
            f.write(f"    lat_dms     : {deg_to_dms(gps.latitude, 'N', 'S')}\n")
            f.write(f"    lon_dms     : {deg_to_dms(gps.longitude, 'E', 'W')}\n")

            if proj_default is not None:
                try:
                    e_proj, n_proj = proj_default(gps.longitude, gps.latitude)
                    f.write(f"  PROJ default (本地复算)\n")
                    f.write(f"    E_m         : {e_proj:.6f}\n")
                    f.write(f"    N_m         : {n_proj:.6f}\n")

                    if hepos is not None:
                        result = hepos.correct(e_proj, n_proj)
                        f.write(f"  HEPOS grid 修正 (本地复算)\n")
                        if result["applied"]:
                            f.write(f"    grid_col    : {result['col']:.4f}\n")
                            f.write(f"    grid_row    : {result['row']:.4f}\n")
                            f.write(f"    dE_cm       : {result['de_cm']:+.4f}\n")
                            f.write(f"    dN_cm       : {result['dn_cm']:+.4f}\n")
                            f.write(f"    dE_m        : {result['de_m']:+.6f}\n")
                            f.write(f"    dN_m        : {result['dn_m']:+.6f}\n")
                            x_recompute = result["x_corrected"]
                            y_recompute = result["y_corrected"]
                            f.write(f"    E_corrected : {x_recompute:.6f}\n")
                            f.write(f"    N_corrected : {y_recompute:.6f}\n")

                            d_e = epsg.pose.position.x - x_recompute
                            d_n = epsg.pose.position.y - y_recompute
                            f.write(f"  一致性 (/epsg_position - 复算 HEPOS-correct)\n")
                            f.write(f"    dE_m        : {d_e:+.6f}\n")
                            f.write(f"    dN_m        : {d_n:+.6f}\n")
                            horiz = math.hypot(d_e, d_n)
                            f.write(f"    horiz_m     : {horiz:.6f}\n")
                        else:
                            f.write(
                                f"    out of grid (col={result['col']:.4f}, row={result['row']:.4f})\n"
                            )
                except Exception as e:
                    f.write(f"  PROJ default 复算失败: {e}\n")
            else:
                f.write(f"  PROJ default (skip): pyproj 未安装\n")

            f.write(f"  /epsg_position (HEPOS-correct, 节点最终输出)\n")
            f.write(f"    E_m         : {epsg.pose.position.x:.6f}\n")
            f.write(f"    N_m         : {epsg.pose.position.y:.6f}\n")
            f.write(f"    Z_m         : {epsg.pose.position.z:.4f}\n")
            f.write(f"    yaw_deg     : {yaw_deg:.4f}\n")

        f.write(f"\n[summary pt{point_index}]\n")
        f.write(f"  frames_used    : {len(frames)}\n")
        f.write(f"  E_mean         : {e_mean:.6f}  std={e_std:.4f} m\n")
        f.write(f"  N_mean         : {n_mean:.6f}  std={n_std:.4f} m\n")
        f.write(f"  Z_mean         : {z_mean:.4f}  std={z_std:.4f} m\n")
        f.write(f"  waypoint_line  : pt{point_index},{fmt_coord(e_mean)},"
                f"{fmt_coord(n_mean)},{fmt_coord(z_mean)},\n")
        f.write(f"=== end pt{point_index} ===\n")


# --------------------- main ---------------------

def main() -> int:
    here = Path(__file__).resolve().parent
    default_detail = here / "gnss_waypoints_detail.txt"

    parser = argparse.ArgumentParser(
        description="RTK 点位链路分析（追加写入 gnss_waypoints_detail.txt，独立于 record_epsg_waypoint.py）"
    )
    parser.add_argument("--gps-topic", default="/gps")
    parser.add_argument("--epsg-topic", default="/epsg_position")
    parser.add_argument(
        "--detail",
        default=str(default_detail),
        help="分析日志文件，追加写入。默认 ./gnss_waypoints_detail.txt",
    )
    parser.add_argument(
        "--pt-id",
        type=int,
        default=0,
        help="显式指定点号（对齐实机刚录的某条 pt）。0 表示从 detail 文件已有最大序号自增",
    )
    parser.add_argument("--frames", type=int, default=3, help="采集帧数（默认 3）")
    parser.add_argument(
        "--timeout",
        type=float,
        default=20.0,
        help="等待 N 帧的超时秒（默认 20）",
    )
    parser.add_argument(
        "--max-stamp-dt",
        type=float,
        default=1.0,
        help="GPS 与 EPSG 时间戳最大差（秒）。/gps 由 GPGGA 触发，/epsg_position "
             "由 HEADINGA 触发，两者天然差几十~几百 ms。默认 1.0",
    )
    parser.add_argument(
        "--min-qual",
        type=int,
        default=4,
        help="最低 GPS 质量（4=RTK fixed）。设为 0 关闭门限。默认 4",
    )
    parser.add_argument(
        "--hepos-e-grid",
        default=DEFAULT_HEPOS_E_GRID,
        help="HEPOS dE grid 文件（本地复算用，找不到就跳过）",
    )
    parser.add_argument(
        "--hepos-n-grid",
        default=DEFAULT_HEPOS_N_GRID,
        help="HEPOS dN grid 文件",
    )
    parser.add_argument(
        "--hepos-origin-x", type=float, default=DEFAULT_HEPOS_ORIGIN_X
    )
    parser.add_argument(
        "--hepos-origin-y", type=float, default=DEFAULT_HEPOS_ORIGIN_Y
    )
    parser.add_argument(
        "--gpgga-raw-dir",
        default=DEFAULT_GPGGA_RAW_DIR,
        help="ins_parser 的 gpgga_raw_*.log 目录（按时间戳匹配每帧的 $GPGGA 原文）"
             f"。默认 {DEFAULT_GPGGA_RAW_DIR}。设为空字符串可禁用。",
    )
    args = parser.parse_args()

    detail_path = Path(args.detail).expanduser().resolve()

    # 加载 HEPOS（可选）
    hepos: Optional[HeposCorrector] = None
    hepos_desc: str
    e_grid_path = Path(args.hepos_e_grid)
    n_grid_path = Path(args.hepos_n_grid)
    if e_grid_path.exists() and n_grid_path.exists():
        try:
            e_grid = HeposGrid.load(e_grid_path)
            n_grid = HeposGrid.load(n_grid_path)
            hepos = HeposCorrector(
                e_grid, n_grid, args.hepos_origin_x, args.hepos_origin_y
            )
            hepos_desc = (
                f"loaded {e_grid_path.name} ({e_grid.rows}x{e_grid.cols} @ "
                f"{e_grid.grid_size:.1f}m) origin=({args.hepos_origin_x},{args.hepos_origin_y})"
            )
        except Exception as e:
            hepos_desc = f"加载失败: {e}"
            hepos = None
    else:
        missing = [str(p) for p in [e_grid_path, n_grid_path] if not p.exists()]
        hepos_desc = f"未找到 grid 文件: {missing}"

    # pyproj（可选）
    proj_default, proj_desc = make_proj_default()

    # 原始 GPGGA 解析器（可选，启动时只记下日志路径，写 detail 前 reload 一次）
    raw_resolver: Optional[RawGpggaResolver] = None
    if args.gpgga_raw_dir:
        raw_resolver = RawGpggaResolver(Path(args.gpgga_raw_dir).expanduser().resolve())
        raw_resolver.discover()

    require_qual = args.min_qual if args.min_qual > 0 else None

    rclpy.init()
    node = GpsEpsgRecorder(
        gps_topic=args.gps_topic,
        epsg_topic=args.epsg_topic,
        frames=args.frames,
        max_stamp_dt=args.max_stamp_dt,
        require_qual=require_qual,
    )

    # 启动自检：等最多 2 秒让 ROS 完成 discovery，然后报告 publisher 数量
    discovery_deadline = time.monotonic() + 2.0
    while time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.frames or (
            node.count_publishers(args.gps_topic) > 0
            and node.count_publishers(args.epsg_topic) > 0
        ):
            break
    n_gps_pub = node.count_publishers(args.gps_topic)
    n_epsg_pub = node.count_publishers(args.epsg_topic)
    node.get_logger().info(
        f"自检: {args.gps_topic} publisher={n_gps_pub} | "
        f"{args.epsg_topic} publisher={n_epsg_pub}"
    )
    if n_gps_pub == 0 and n_epsg_pub == 0:
        node.get_logger().error(
            f"两个话题均无 publisher。检查 ins_parser_node 是否运行："
            f" ros2 node list | grep ins ；ros2 topic list"
        )

    deadline = time.monotonic() + args.timeout
    last_progress = 0.0
    try:
        while rclpy.ok() and not node.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_progress > 2.0:
                node.get_logger().info(
                    f"  进度: 已采 {len(node.frames)}/{node.frames_target} | "
                    f"gps 收到 {node.gps_count} | epsg 收到 {node.epsg_count} | "
                    f"丢弃: stamp_dt={node.skipped_stamp} qual={node.skipped_quality}"
                )
                last_progress = now
    except KeyboardInterrupt:
        node.get_logger().warn("用户中断")
        node.destroy_node()
        rclpy.shutdown()
        return 130

    if not node.done():
        node.get_logger().error(
            f"超时未采到 {node.frames_target} 帧（实际 {len(node.frames)} 帧）。"
            f" gps={node.gps_count} epsg={node.epsg_count}"
            f" 丢弃 stamp_dt={node.skipped_stamp} qual={node.skipped_quality}"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # 计算均值/标准差
    es = [fp.epsg.pose.position.x for fp in node.frames]
    ns = [fp.epsg.pose.position.y for fp in node.frames]
    zs = [fp.epsg.pose.position.z for fp in node.frames]
    e_mean = statistics.fmean(es)
    n_mean = statistics.fmean(ns)
    z_mean = statistics.fmean(zs)
    e_std = statistics.pstdev(es) if len(es) > 1 else 0.0
    n_std = statistics.pstdev(ns) if len(ns) > 1 else 0.0
    z_std = statistics.pstdev(zs) if len(zs) > 1 else 0.0

    point_index = args.pt_id if args.pt_id > 0 else get_next_point_index(detail_path)

    # 写 block 前重新读 raw log，覆盖刚采的 3 帧时间窗
    if raw_resolver is not None:
        raw_resolver.reload()

    write_detail_block(
        detail_path=detail_path,
        point_index=point_index,
        frames=node.frames,
        proj_default=proj_default,
        proj_default_desc=proj_desc,
        hepos=hepos,
        hepos_desc=hepos_desc,
        raw_resolver=raw_resolver,
        e_mean=e_mean,
        n_mean=n_mean,
        z_mean=z_mean,
        e_std=e_std,
        n_std=n_std,
        z_std=z_std,
    )
    waypoint_line = (
        f"pt{point_index},{fmt_coord(e_mean)},{fmt_coord(n_mean)},{fmt_coord(z_mean)},"
    )
    node.get_logger().info(
        f"已记录 pt{point_index} 链路分析（std E={e_std*100:.2f}cm "
        f"N={n_std*100:.2f}cm）"
    )
    node.get_logger().info(f"等价 waypoint 行: {waypoint_line}")
    node.get_logger().info(f"detail 文件   : {detail_path}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
