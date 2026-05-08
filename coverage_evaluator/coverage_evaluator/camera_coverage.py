#!/usr/bin/env python3
"""
camera_coverage.py — 基于俯视摄像的覆盖真值评估核心模块。

使用流程:
    1. 加载 PS 蒙版 PNG（白色=可通行，黑色=障碍/不可通行）
    2. 首帧检测 4 个角点 ArUco → 计算单应矩阵
    3. 蒙版通过单应矩阵映射到论文坐标网格 → passable_mask
    4. 逐帧追踪车顶 ArUco → 论文坐标轨迹 → 累积覆盖网格
    5. 计算区域覆盖率、重复覆盖率、覆盖效率、覆盖率-时间曲线

依赖: numpy, cv2 (opencv-python)
"""

import math
import time
import csv
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
import cv2


# ═══════════════════════════════════════════════════════════════════════════
# 数据结构
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class CameraCoverageConfig:
    """分析参数配置。"""
    # 论文坐标系区域
    paper_width: float = 1.8          # m
    paper_height: float = 2.4         # m

    # 网格分辨率（与 coverage_evaluator 一致）
    resolution: float = 0.005          # m (5mm)

    # 覆盖半径（小车有效作业宽度的一半）
    coverage_radius: float = 0.12      # m

    # ArUco 参数
    aruco_dict: int = cv2.aruco.DICT_4X4_50
    corner_ids: Tuple[int, ...] = (0, 1, 2, 3)
    robot_id: int = 4

    # 角点论文坐标（按 corner_ids 顺序）
    corner_paper_xy: List[Tuple[float, float]] = field(default_factory=lambda: [
        (0.0, 0.0),      # ID=0: 左下
        (1.8, 0.0),      # ID=1: 右下
        (1.8, 2.4),      # ID=2: 右上
        (0.0, 2.4),      # ID=3: 左上
    ])

    # 视频帧率（用于时间轴，0=从视频元数据读取）
    video_fps: float = 0.0

    # 处理帧间隔（每 N 帧处理一次）
    frame_skip: int = 1

    # 时间曲线采样间隔（帧数）
    time_series_interval: int = 30

    # ArUco 丢失容忍（连续丢失此帧数后报 warning）
    max_lost_frames: int = 5

    # 蒙版参数
    mask_threshold: int = 128          # 0-255，蒙版二值化阈值

    @property
    def grid_w(self) -> int:
        return int(math.ceil(self.paper_width / self.resolution))

    @property
    def grid_h(self) -> int:
        return int(math.ceil(self.paper_height / self.resolution))


# ═══════════════════════════════════════════════════════════════════════════
# ArUco 检测
# ═══════════════════════════════════════════════════════════════════════════

def detect_aruco_markers(frame: np.ndarray,
                         dict_type: int) -> Tuple[Dict[int, np.ndarray], np.ndarray]:
    """检测帧中所有 ArUco 标记。

    Returns:
        corners_dict: {marker_id: corner_pts (4,2) np.ndarray}
        frame: 标注后的帧（原地修改的拷贝）
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)
    corners, ids, _ = detector.detectMarkers(frame)

    result: Dict[int, np.ndarray] = {}
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            result[int(marker_id)] = corners[i][0]  # (4,2) corners
    return result, frame


def get_marker_center(corner_pts: np.ndarray) -> Tuple[float, float]:
    """从 (4,2) 角点数组计算中心坐标。"""
    cx = float(np.mean(corner_pts[:, 0]))
    cy = float(np.mean(corner_pts[:, 1]))
    return (cx, cy)


def detect_corner_markers(frame: np.ndarray,
                          config: CameraCoverageConfig) -> Optional[Dict[int, Tuple[float, float]]]:
    """检测 4 个角点 ArUco 标记，返回 {id: (cx, cy)}。如果未全检测到返回 None。"""
    markers, _ = detect_aruco_markers(frame, config.aruco_dict)
    result = {}
    for cid in config.corner_ids:
        if cid not in markers:
            return None
        result[cid] = get_marker_center(markers[cid])
    return result


def detect_robot_marker(frame: np.ndarray,
                        config: CameraCoverageConfig) -> Optional[Tuple[float, float]]:
    """检测车顶 ArUco 标记，返回 (cx, cy) 或 None。"""
    markers, _ = detect_aruco_markers(frame, config.aruco_dict)
    if config.robot_id not in markers:
        return None
    return get_marker_center(markers[config.robot_id])


# ═══════════════════════════════════════════════════════════════════════════
# 单应矩阵与坐标转换
# ═══════════════════════════════════════════════════════════════════════════

def compute_homography(image_corners: List[Tuple[float, float]],
                       paper_corners: List[Tuple[float, float]]) -> np.ndarray:
    """从 4 组图像→论文坐标对应点计算单应矩阵。

    Args:
        image_corners: 图像中的 4 个点 [(x, y), ...]
        paper_corners: 论文坐标中的 4 个点 [(x, y), ...]
    Returns:
        (3, 3) 单应矩阵 H: 图像→论文
    """
    src = np.array(image_corners, dtype=np.float32).reshape(-1, 1, 2)
    dst = np.array(paper_corners, dtype=np.float32).reshape(-1, 1, 2)
    H, _ = cv2.findHomography(src, dst, cv2.RANSAC)
    return H


def image_to_paper(pt: Tuple[float, float], H: np.ndarray) -> Tuple[float, float]:
    """图像坐标 → 论文坐标。"""
    p = np.array([[pt[0], pt[1]]], dtype=np.float32).reshape(-1, 1, 2)
    tp = cv2.perspectiveTransform(p, H)
    return (float(tp[0, 0, 0]), float(tp[0, 0, 1]))


def paper_to_image(pt: Tuple[float, float], H: np.ndarray) -> Tuple[float, float]:
    """论文坐标 → 图像坐标（使用 H 的逆）。"""
    H_inv = np.linalg.inv(H)
    p = np.array([[pt[0], pt[1]]], dtype=np.float32).reshape(-1, 1, 2)
    tp = cv2.perspectiveTransform(p, H_inv)
    return (float(tp[0, 0, 0]), float(tp[0, 0, 1]))


# ═══════════════════════════════════════════════════════════════════════════
# 蒙版处理
# ═══════════════════════════════════════════════════════════════════════════

def load_mask(mask_path: str, threshold: int = 128) -> np.ndarray:
    """加载蒙版 PNG，返回二值图 (H, W) uint8 (0/255)。"""
    img = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"无法读取蒙版: {mask_path}")
    _, binary = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)
    return binary


def mask_to_passable_grid(mask_img: np.ndarray,
                          H: np.ndarray,
                          config: CameraCoverageConfig) -> np.ndarray:
    """将蒙版图像通过单应矩阵映射到论文坐标网格。

    对每个 grid cell 中心，用 H 逆矩阵反投到蒙版图像坐标，
    采样蒙版值。蒙版中白色像素 = 可通行。

    Returns:
        passable_mask: (grid_h, grid_w) bool 数组，True=可通行
    """
    gw = config.grid_w
    gh = config.grid_h
    res = config.resolution
    mask_h, mask_w = mask_img.shape
    H_inv = np.linalg.inv(H)

    passable = np.zeros((gh, gw), dtype=bool)

    # 逐行处理以避免内存爆炸（360×480=172800 cells，完全可行）
    for row in range(gh):
        py = (row + 0.5) * res
        paper_pts = np.zeros((gw, 1, 2), dtype=np.float32)
        paper_pts[:, 0, 0] = (np.arange(gw, dtype=np.float32) + 0.5) * res
        paper_pts[:, 0, 1] = py
        img_pts = cv2.perspectiveTransform(paper_pts, H_inv)

        for col in range(gw):
            ix = int(round(img_pts[col, 0, 0]))
            iy = int(round(img_pts[col, 0, 1]))
            if 0 <= ix < mask_w and 0 <= iy < mask_h:
                passable[row, col] = (mask_img[iy, ix] >= 128)

    return passable


# ═══════════════════════════════════════════════════════════════════════════
# 覆盖计算
# ═══════════════════════════════════════════════════════════════════════════

def accumulate_coverage(trajectory: List[Tuple[float, float, float]],
                        passable_mask: np.ndarray,
                        config: CameraCoverageConfig) -> np.ndarray:
    """逐帧累积覆盖计数。

    Args:
        trajectory: [(t, x_paper, y_paper), ...]
        passable_mask: (grid_h, grid_w) bool
        config: 参数配置

    Returns:
        covered_count: (grid_h, grid_w) int 数组，每格被覆盖的次数
    """
    covered_count = np.zeros(passable_mask.shape, dtype=np.int32)
    rad_cells = int(math.ceil(config.coverage_radius / config.resolution))

    for _, px, py in trajectory:
        cx = int(px / config.resolution)
        cy = int(py / config.resolution)

        x0 = max(cx - rad_cells, 0)
        x1 = min(cx + rad_cells + 1, config.grid_w)
        y0 = max(cy - rad_cells, 0)
        y1 = min(cy + rad_cells + 1, config.grid_h)

        if x0 >= x1 or y0 >= y1:
            continue

        xs = (np.arange(x0, x1, dtype=np.float64) + 0.5) * config.resolution
        ys = (np.arange(y0, y1, dtype=np.float64) + 0.5) * config.resolution
        X, Y = np.meshgrid(xs, ys)
        circle = (X - px) ** 2 + (Y - py) ** 2 <= (config.coverage_radius ** 2)
        covered_count[y0:y1, x0:x1] += circle.astype(np.int32)

    return covered_count


def compute_metrics(covered_count: np.ndarray,
                    passable_mask: np.ndarray,
                    trajectory_len: float) -> Dict[str, float]:
    """从覆盖计数和可通行蒙版计算四项指标。

    Args:
        covered_count: (H, W) int: 每格覆盖次数
        passable_mask: (H, W) bool: 可通行区域
        trajectory_len: 轨迹总长度 (m)

    Returns:
        {
            "area_coverage": 区域覆盖率,
            "repeat_coverage": 重复覆盖率 (被覆盖≥2次的占比),
            "coverage_efficiency": 覆盖效率 (覆盖率/轨迹长度),
            "total_passable_cells": 可通行总格数,
            "trajectory_length_m": 轨迹长度,
        }
    """
    total_passable = int(np.sum(passable_mask))
    if total_passable == 0:
        return {
            "area_coverage": 0.0,
            "repeat_coverage": 0.0,
            "coverage_efficiency": 0.0,
            "total_passable_cells": 0,
            "trajectory_length_m": trajectory_len,
        }

    covered = (covered_count > 0) & passable_mask
    covered_passable = int(np.sum(covered))
    area_cov = covered_passable / total_passable

    repeat_covered = (covered_count >= 2) & passable_mask
    repeat_cov = int(np.sum(repeat_covered)) / total_passable

    efficiency = area_cov / max(trajectory_len, 0.001)

    return {
        "area_coverage": area_cov,
        "repeat_coverage": repeat_cov,
        "coverage_efficiency": efficiency,
        "total_passable_cells": total_passable,
        "trajectory_length_m": trajectory_len,
    }


def compute_time_series(trajectory: List[Tuple[float, float, float]],
                        passable_mask: np.ndarray,
                        config: CameraCoverageConfig) -> List[Tuple[float, float, float]]:
    """计算覆盖率-时间曲线。

    Returns:
        [(t_sec, area_coverage, repeat_coverage), ...]
    """
    covered_count = np.zeros(passable_mask.shape, dtype=np.int32)
    rad_cells = int(math.ceil(config.coverage_radius / config.resolution))
    total_passable = max(int(np.sum(passable_mask)), 1)
    series = []

    for idx, (t, px, py) in enumerate(trajectory):
        cx = int(px / config.resolution)
        cy = int(py / config.resolution)
        x0 = max(cx - rad_cells, 0)
        x1 = min(cx + rad_cells + 1, config.grid_w)
        y0 = max(cy - rad_cells, 0)
        y1 = min(cy + rad_cells + 1, config.grid_h)
        if x0 < x1 and y0 < y1:
            xs = (np.arange(x0, x1, dtype=np.float64) + 0.5) * config.resolution
            ys = (np.arange(y0, y1, dtype=np.float64) + 0.5) * config.resolution
            X, Y = np.meshgrid(xs, ys)
            circle = (X - px) ** 2 + (Y - py) ** 2 <= (config.coverage_radius ** 2)
            covered_count[y0:y1, x0:x1] += circle.astype(np.int32)

        if (idx + 1) % config.time_series_interval == 0:
            cov = int(np.sum((covered_count > 0) & passable_mask))
            rep = int(np.sum((covered_count >= 2) & passable_mask))
            series.append((t, cov / total_passable, rep / total_passable))

    # 保证最后一帧也记录
    if len(trajectory) > 0 and (len(trajectory) % config.time_series_interval != 0):
        t = trajectory[-1][0]
        cov = int(np.sum((covered_count > 0) & passable_mask))
        rep = int(np.sum((covered_count >= 2) & passable_mask))
        series.append((t, cov / total_passable, rep / total_passable))

    return series


# ═══════════════════════════════════════════════════════════════════════════
# 轨迹工具
# ═══════════════════════════════════════════════════════════════════════════

def compute_trajectory_length(trajectory: List[Tuple[float, float, float]]) -> float:
    """计算轨迹总长度 (m)。"""
    total = 0.0
    for i in range(1, len(trajectory)):
        _, x1, y1 = trajectory[i - 1]
        _, x2, y2 = trajectory[i]
        total += math.hypot(x2 - x1, y2 - y1)
    return total


# ═══════════════════════════════════════════════════════════════════════════
# 主分析器
# ═══════════════════════════════════════════════════════════════════════════

class CameraCoverageAnalyzer:
    """离线俯视视频覆盖分析器。

    Usage:
        analyzer = CameraCoverageAnalyzer(config)
        analyzer.analyze("test.mp4", "mask.png")
        metrics = analyzer.get_metrics()
        analyzer.save_results("/output/dir/")
    """

    def __init__(self, config: Optional[CameraCoverageConfig] = None):
        self.config = config or CameraCoverageConfig()
        self._homography: Optional[np.ndarray] = None
        self._passable_mask: Optional[np.ndarray] = None
        self._trajectory: List[Tuple[float, float, float]] = []
        self._covered_count: Optional[np.ndarray] = None
        self._metrics: Dict[str, float] = {}
        self._time_series: List[Tuple[float, float, float]] = []
        self._lost_frames: int = 0
        self._total_frames: int = 0
        self._mask_img: Optional[np.ndarray] = None

    # ── 属性 ──────────────────────────────────────────────────────────

    @property
    def homography(self) -> Optional[np.ndarray]:
        return self._homography

    @property
    def passable_mask(self) -> Optional[np.ndarray]:
        return self._passable_mask

    @property
    def trajectory(self) -> List[Tuple[float, float, float]]:
        return self._trajectory

    @property
    def covered_count(self) -> Optional[np.ndarray]:
        return self._covered_count

    @property
    def metrics(self) -> Dict[str, float]:
        return self._metrics

    @property
    def time_series(self) -> List[Tuple[float, float, float]]:
        return self._time_series

    # ── 主流程 ────────────────────────────────────────────────────────

    def analyze(self, video_path: str, mask_path: str):
        """执行完整分析流程。"""
        cfg = self.config

        # 1. 加载蒙版
        print(f"[1/5] 加载蒙版: {mask_path}")
        self._mask_img = load_mask(mask_path, cfg.mask_threshold)
        print(f"      蒙版尺寸: {self._mask_img.shape[1]}×{self._mask_img.shape[0]}")

        # 2. 打开视频
        print(f"[2/5] 打开视频: {video_path}")
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            raise RuntimeError(f"无法打开视频: {video_path}")

        if cfg.video_fps <= 0:
            cfg.video_fps = cap.get(cv2.CAP_PROP_FPS)
            if cfg.video_fps <= 0:
                cfg.video_fps = 30.0
        print(f"      帧率: {cfg.video_fps:.1f} fps")

        # 3. 首帧检测 ArUco 角点 → 计算单应矩阵
        print(f"[3/5] 检测 ArUco 标记并计算单应矩阵...")
        ret, first_frame = cap.read()
        if not ret:
            raise RuntimeError("无法读取视频首帧")

        # 重置到开头
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

        self._homography = self._calibrate_homography(first_frame)
        print(f"      单应矩阵:\n{self._homography}")

        # 4. 蒙版 → 可通行网格
        print(f"[4/5] 蒙版映射到论文坐标网格 ({cfg.grid_w}×{cfg.grid_h}, "
              f"res={cfg.resolution}m)...")
        t_map = time.time()
        self._passable_mask = mask_to_passable_grid(
            self._mask_img, self._homography, cfg)
        passable_cells = int(np.sum(self._passable_mask))
        passable_area = passable_cells * cfg.resolution ** 2
        print(f"      可通行格数: {passable_cells} "
              f"(≈ {passable_area:.3f} m²) "
              f"耗时 {time.time()-t_map:.1f}s")

        # 5. 逐帧追踪
        print(f"[5/5] 逐帧追踪机器人轨迹...")
        self._trajectory, self._lost_frames, self._total_frames = \
            self._track_robot(cap, cfg)

        cap.release()

        # 6. 计算覆盖
        print("计算覆盖网格与指标...")
        traj_len = compute_trajectory_length(self._trajectory)
        self._covered_count = accumulate_coverage(
            self._trajectory, self._passable_mask, cfg)
        self._metrics = compute_metrics(
            self._covered_count, self._passable_mask, traj_len)
        self._time_series = compute_time_series(
            self._trajectory, self._passable_mask, cfg)

        # 7. 打印摘要
        self._print_summary()

    # ── 单应标定 ──────────────────────────────────────────────────────

    def _calibrate_homography(self, frame: np.ndarray) -> np.ndarray:
        """从首帧检测角点 ArUco 并计算单应矩阵。"""
        corners = detect_corner_markers(frame, self.config)
        if corners is None:
            raise RuntimeError(
                f"首帧未检测到全部 {len(self.config.corner_ids)} 个角点 ArUco "
                f"(IDs={list(self.config.corner_ids)})。"
                f" 请检查 ArUco 标记是否完整入画且清晰可见。")

        img_pts = []
        paper_pts = []
        for i, cid in enumerate(self.config.corner_ids):
            img_pts.append(corners[cid])
            paper_pts.append(self.config.corner_paper_xy[i])
            print(f"      角点 ID={cid}: 图像 {corners[cid]} → "
                  f"论文 {self.config.corner_paper_xy[i]}")

        # 检查：确保检测到的 ID 和 config 中的顺序一致
        # ArUco 检测返回的坐标顺序由 detectMarkers 决定，我们需要按 config.corner_ids 排序
        return compute_homography(img_pts, paper_pts)

    # ── 轨迹追踪 ──────────────────────────────────────────────────────

    def _track_robot(self, cap: cv2.VideoCapture,
                     cfg: CameraCoverageConfig) -> Tuple[List, int, int]:
        """逐帧追踪机器人 ArUco。

        Returns:
            (trajectory, lost_frames, total_processed_frames)
        """
        import cv2.aruco as aruco
        aruco_dict = aruco.getPredefinedDictionary(cfg.aruco_dict)
        params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, params)

        trajectory: List[Tuple[float, float, float]] = []
        lost_count = 0
        total_processed = 0
        frame_idx = 0
        last_valid_pt: Optional[Tuple[float, float]] = None
        consecutive_lost = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            if frame_idx % cfg.frame_skip != 0:
                frame_idx += 1
                continue

            t_sec = frame_idx / cfg.video_fps
            corners, ids, _ = detector.detectMarkers(frame)
            pt = None

            if ids is not None:
                for i, mid in enumerate(ids.flatten()):
                    if int(mid) == cfg.robot_id:
                        c = corners[i][0]
                        pt = (float(np.mean(c[:, 0])), float(np.mean(c[:, 1])))
                        break

            if pt is not None and self._homography is not None:
                paper_pt = image_to_paper(pt, self._homography)
                # 检查是否在区域范围内（允许一定容差）
                if (-0.1 <= paper_pt[0] <= cfg.paper_width + 0.1 and
                        -0.1 <= paper_pt[1] <= cfg.paper_height + 0.1):
                    trajectory.append((t_sec, paper_pt[0], paper_pt[1]))
                    last_valid_pt = paper_pt
                    consecutive_lost = 0
                else:
                    # 超出范围，可能是误检测
                    lost_count += 1
                    consecutive_lost += 1
            else:
                # 丢失帧: 尝试线性插值
                lost_count += 1
                consecutive_lost += 1
                if (consecutive_lost >= cfg.max_lost_frames and
                        consecutive_lost == cfg.max_lost_frames):
                    print(f"  ⚠ t={t_sec:.1f}s: ArUco 已连续丢失 "
                          f"{cfg.max_lost_frames} 帧")

            total_processed += 1
            frame_idx += 1

            if total_processed % 300 == 0:
                print(f"  ... 已处理 {total_processed} 帧 "
                      f"(t={t_sec:.1f}s, {len(trajectory)} 有效轨迹点, "
                      f"{lost_count} 丢失)")

        print(f"  轨迹追踪完成: {total_processed} 帧, "
              f"{len(trajectory)} 有效点, {lost_count} 丢失 "
              f"({100*lost_count/max(total_processed,1):.1f}%)")

        return trajectory, lost_count, total_processed

    # ── 摘要 ──────────────────────────────────────────────────────────

    def _print_summary(self):
        m = self._metrics
        print(f"\n{'='*55}")
        print(f"  覆盖作业指标汇总")
        print(f"{'='*55}")
        print(f"  区域覆盖率:     {m['area_coverage']*100:6.2f} %")
        print(f"  重复覆盖率:     {m['repeat_coverage']*100:6.2f} %")
        print(f"  覆盖效率:       {m['coverage_efficiency']:6.4f} m⁻¹")
        print(f"  轨迹总长度:     {m['trajectory_length_m']:6.2f} m")
        print(f"  可通行总面积:   {m['total_passable_cells']*self.config.resolution**2:6.3f} m²")
        print(f"  有效/总帧:      {len(self._trajectory)}/{self._total_frames}")
        lost_pct = 100 * self._lost_frames / max(self._total_frames, 1)
        if lost_pct > 5:
            print(f"  ⚠ ArUco 丢失率 {lost_pct:.1f}% > 5%，请检查视频质量！")
        print(f"{'='*55}")

    # ── 输出 ──────────────────────────────────────────────────────────

    def save_results(self, output_dir: str, prefix: Optional[str] = None):
        """保存分析结果到 CSV 文件。

        生成:
            <prefix>_coverage_summary.csv     — 最终指标
            <prefix>_coverage_time_series.csv — 覆盖率-时间曲线
            <prefix>_trajectory_camera.csv     — 原始轨迹
        """
        os.makedirs(output_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        pfx = prefix or "camera_coverage"

        # 汇总
        summary_path = os.path.join(output_dir, f"{pfx}_summary_{ts}.csv")
        with open(summary_path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(["metric", "value"])
            for k, v in self._metrics.items():
                w.writerow([k, f"{v:.6f}"])
            w.writerow(["lost_frames", self._lost_frames])
            w.writerow(["total_frames", self._total_frames])
            w.writerow(["valid_trajectory_points", len(self._trajectory)])
        print(f"  → {summary_path}")

        # 时间曲线
        ts_path = os.path.join(output_dir, f"{pfx}_time_series_{ts}.csv")
        with open(ts_path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(["t_sec", "area_coverage", "repeat_coverage"])
            for row in self._time_series:
                w.writerow([f"{row[0]:.2f}", f"{row[1]:.6f}", f"{row[2]:.6f}"])
        print(f"  → {ts_path}")

        # 轨迹
        traj_path = os.path.join(output_dir, f"{pfx}_trajectory_{ts}.csv")
        with open(traj_path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(["t_sec", "x_paper", "y_paper"])
            for t, x, y in self._trajectory:
                w.writerow([f"{t:.3f}", f"{x:.4f}", f"{y:.4f}"])
        print(f"  → {traj_path}")

    def generate_visualizations(self, output_dir: str,
                                prefix: Optional[str] = None):
        """生成可视化图像。

        生成:
            <prefix>_coverage_overlay.png — 覆盖叠加图
            <prefix>_coverage_curve.png   — 覆盖率-时间曲线
        """
        os.makedirs(output_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        pfx = prefix or "camera_coverage"

        self._generate_overlay(output_dir, f"{pfx}_overlay_{ts}.png")
        self._generate_curve(output_dir, f"{pfx}_curve_{ts}.png")

    def _generate_overlay(self, output_dir: str, filename: str):
        """生成覆盖叠加图。"""
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        if self._passable_mask is None or self._covered_count is None:
            print("无数据可用于可视化")
            return

        fig, ax = plt.subplots(figsize=(8, 10))
        gw = self.config.grid_w
        gh = self.config.grid_h
        extent = [0, self.config.paper_width, 0, self.config.paper_height]

        # 背景: 浅灰=可通行
        bg = np.zeros((gh, gw, 3), dtype=np.float32)
        bg[self._passable_mask] = [0.9, 0.9, 0.9]  # 浅灰
        ax.imshow(bg, origin='lower', extent=extent, aspect='equal')

        # 已覆盖: 绿色
        covered = (self._covered_count > 0) & self._passable_mask
        overlay_g = np.zeros((gh, gw, 4), dtype=np.float32)
        overlay_g[covered, 1] = 0.6   # 绿色通道
        overlay_g[covered, 3] = 0.5   # alpha
        ax.imshow(overlay_g, origin='lower', extent=extent, aspect='equal')

        # 未覆盖: 红色
        uncovered = (~(self._covered_count > 0)) & self._passable_mask
        overlay_r = np.zeros((gh, gw, 4), dtype=np.float32)
        overlay_r[uncovered, 0] = 0.8   # 红色通道
        overlay_r[uncovered, 3] = 0.5   # alpha
        ax.imshow(overlay_r, origin='lower', extent=extent, aspect='equal')

        # 轨迹线: 蓝色
        if len(self._trajectory) > 1:
            xs = [p[1] for p in self._trajectory]
            ys = [p[2] for p in self._trajectory]
            ax.plot(xs, ys, 'b-', linewidth=0.5, alpha=0.7, label='trajectory')

        ax.set_xlabel("x_paper (m)")
        ax.set_ylabel("y_paper (m)")
        ax.set_title(
            f"Coverage Overlay\n"
            f"Area: {self._metrics.get('area_coverage',0)*100:.1f}%  "
            f"Repeat: {self._metrics.get('repeat_coverage',0)*100:.1f}%"
        )
        ax.legend(loc='upper right')
        ax.set_xlim(0, self.config.paper_width)
        ax.set_ylim(0, self.config.paper_height)

        filepath = os.path.join(output_dir, filename)
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"  → {filepath}")

    def _generate_curve(self, output_dir: str, filename: str):
        """生成覆盖率-时间曲线。"""
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        if not self._time_series:
            print("无时间序列数据")
            return

        fig, ax = plt.subplots(figsize=(10, 5))
        ts_list = [r[0] for r in self._time_series]
        area_list = [r[1] * 100 for r in self._time_series]
        repeat_list = [r[2] * 100 for r in self._time_series]

        ax.plot(ts_list, area_list, 'g-', linewidth=1.5, label='Area Coverage')
        ax.plot(ts_list, repeat_list, 'orange', linewidth=1.5,
                label='Repeat Coverage (≥2×)')
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Coverage (%)")
        ax.set_title("Coverage vs Time")
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 105)

        filepath = os.path.join(output_dir, filename)
        fig.savefig(filepath, dpi=150, bbox_inches='tight')
        plt.close(fig)
        print(f"  → {filepath}")
