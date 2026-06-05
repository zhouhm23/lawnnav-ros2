#!/usr/bin/env python3
"""
paper_figures.py — 论文覆盖路径插图生成器。

从 coverage_evaluator 生成的 trajectory CSV 和 costmap NPZ 生成两张插图：
  fig1_planned_on_costmap.png — 规划路径在真实 costmap 上的效果
  fig2_actual_coverage.png   — 实际覆盖结果

用法:
    python3 tools/paper_figures.py  --csv logs/trajectory/trajectory_20260510_124716.csv --costmap logs/costmap/costmap_20260510_124716.npz --output ./docs/paper_figures

    python3 tools/paper_figures.py \\
        --csv logs/trajectory/trajectory_xxx.csv \\
        --costmap logs/costmap/costmap_xxx.npz \\
        --polygon "0,0.4;2.4,0.4;2.4,-1.4;0,-1.4" \\
        --robot-width 0.171
"""

import argparse
import math
import os
import sys
from typing import List, Tuple

import numpy as np

# ── 智能导入 path_coverage 模块 ───────────────────────────────────────
_script_dir = os.path.dirname(os.path.abspath(__file__))
_ws_src = os.path.dirname(_script_dir)  # tools/../ = src/
_path_coverage_dir = os.path.join(_ws_src, 'path_coverage_ros2')
if _path_coverage_dir not in sys.path:
    sys.path.insert(0, _path_coverage_dir)

try:
    from path_coverage.trapezoidal_coverage import calc_path # type: ignore
    from path_coverage.list_helper import ( # type: ignore
        rotate_points, rotate_polygon,
        get_angle_of_longest_side_to_horizontal,
    )
    _has_path_coverage = True
except ImportError:
    print("[WARN] 无法导入 path_coverage 模块，图1将跳过规划路径。")
    print("       请确保已安装 shapely: pip3 install shapely")
    calc_path = None
    rotate_points = None
    rotate_polygon = None
    get_angle_of_longest_side_to_horizontal = None
    _has_path_coverage = False

# 尝试导入 matplotlib
try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon
except ImportError:
    print("请安装 matplotlib: pip3 install matplotlib")
    sys.exit(1)

# ── 常量 ───────────────────────────────────────────────────────────────

# costmap 值到颜色的映射
# 0=Free, 1-99=Inflation(gray gradient), 100=Unknown, 255=Lethal
COSTMAP_COLORS = {
    'free':      np.array([0.95, 0.95, 0.95]),  # 浅灰白
    'unknown':   np.array([0.65, 0.65, 0.65]),  # 中灰
    'lethal':    np.array([0.15, 0.15, 0.15]),  # 深灰/黑
}

# 默认多边形: 对照实验的 test_180x240 区域
DEFAULT_POLYGON = [(0.0, 0.4), (2.4, 0.4), (2.4, -1.4), (0.0, -1.4)]


# ═══════════════════════════════════════════════════════════════════════════
# 数据加载
# ═══════════════════════════════════════════════════════════════════════════

def load_costmap(npz_path: str) -> dict:
    """加载 costmap NPZ，返回 {data_2d, origin_x, origin_y, resolution, width, height}."""
    arr = np.load(npz_path)
    return {
        'data': arr['data'],
        'origin_x': float(arr['origin_x']),
        'origin_y': float(arr['origin_y']),
        'resolution': float(arr['resolution']),
        'width': int(arr['width']),
        'height': int(arr['height']),
    }


def load_trajectory(csv_path: str) -> np.ndarray:
    """加载 trajectory CSV，返回 (N,4) array: [t, x, y, yaw]."""
    data = np.loadtxt(csv_path, delimiter=',', skiprows=1, dtype=np.float64)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data


def parse_polygon(s: str) -> List[Tuple[float, float]]:
    """解析多边形字符串 'x1,y1;x2,y2;...' -> [(x1,y1), ...]."""
    pts = []
    for tok in s.split(';'):
        tok = tok.strip()
        if not tok:
            continue
        parts = tok.split(',')
        if len(parts) != 2:
            continue
        pts.append((float(parts[0]), float(parts[1])))
    return pts


# ═══════════════════════════════════════════════════════════════════════════
# Costmap 渲染
# ═══════════════════════════════════════════════════════════════════════════

def costmap_to_rgb(costmap: dict) -> np.ndarray:
    """将 costmap data (H,W) 转为 RGB (H,W,3) float [0,1] 图像."""
    d = costmap['data'].astype(np.float64)
    h, w = d.shape
    rgb = np.zeros((h, w, 3), dtype=np.float64)

    # Free: 0
    mask_free = (d == 0)
    rgb[mask_free] = COSTMAP_COLORS['free']

    # Lethal: 254, 255
    mask_lethal = (d >= 254)
    rgb[mask_lethal] = COSTMAP_COLORS['lethal']

    # Unknown: 100
    mask_unknown = (d == 100) | ((d > 99) & (d < 254))
    rgb[mask_unknown] = COSTMAP_COLORS['unknown']

    # Inflation: 1-99 (gray gradient: lighter near free, darker near obstacle)
    mask_infl = (d >= 1) & (d <= 99)
    if np.any(mask_infl):
        # 值越大越接近障碍物 → 颜色越深
        intensity = 1.0 - (d[mask_infl] / 100.0)
        # 映射到 [0.45, 0.85] 的灰色范围
        gray = 0.45 + intensity * 0.40
        rgb[mask_infl, 0] = gray
        rgb[mask_infl, 1] = gray
        rgb[mask_infl, 2] = gray

    return np.clip(rgb, 0, 1)


def costmap_extent(costmap: dict) -> Tuple[float, float, float, float]:
    """返回 costmap 的 matplotlib extent: (left, right, bottom, top)."""
    ox = costmap['origin_x']
    oy = costmap['origin_y']
    res = costmap['resolution']
    w = costmap['width']
    h = costmap['height']
    return (ox, ox + w * res, oy, oy + h * res)


# ═══════════════════════════════════════════════════════════════════════════
# 覆盖区域计算
# ═══════════════════════════════════════════════════════════════════════════

def compute_coverage_grid(
    trajectory: np.ndarray,
    costmap: dict,
    coverage_radius: float = 0.12,
) -> np.ndarray:
    """沿轨迹计算覆盖网格（与 coverage_evaluator 算法一致）。

    Args:
        trajectory: (N,4) [t, x, y, yaw]
        costmap: costmap dict
        coverage_radius: 覆盖半径 (m)

    Returns:
        covered_mask: (H, W) bool
    """
    res = costmap['resolution']
    ox = costmap['origin_x']
    oy = costmap['origin_y']
    w = costmap['width']
    h = costmap['height']

    covered_count = np.zeros((h, w), dtype=np.int32)
    rad_cells = int(math.ceil(coverage_radius / res))

    for i in range(len(trajectory)):
        px = trajectory[i, 1]
        py = trajectory[i, 2]

        cx = int((px - ox) / res)
        cy = int((py - oy) / res)

        x0 = max(cx - rad_cells, 0)
        x1 = min(cx + rad_cells + 1, w)
        y0 = max(cy - rad_cells, 0)
        y1 = min(cy + rad_cells + 1, h)

        if x0 >= x1 or y0 >= y1:
            continue

        xs = ox + (np.arange(x0, x1, dtype=np.float64) + 0.5) * res
        ys = oy + (np.arange(y0, y1, dtype=np.float64) + 0.5) * res
        X, Y = np.meshgrid(xs, ys)
        circle = (X - px) ** 2 + (Y - py) ** 2 <= (coverage_radius ** 2)
        covered_count[y0:y1, x0:x1] += circle.astype(np.int32)

    return covered_count > 0


# ═══════════════════════════════════════════════════════════════════════════
# 图1: 规划路径在真实 costmap 上的效果（RViz 风格，复用 path_coverage 算法）
# ═══════════════════════════════════════════════════════════════════════════

def _costmap_aware_decompose(
    costmap: dict,
    polygon_pts: List[Tuple[float, float]],
    robot_width: float,
    costmap_max_non_lethal: int = 70,
    polygon_expand: float = 0.05,
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """复用 path_coverage_node 的 do_boustrophedon 管线：
    costmap提取 → BFS可通行区域 → Ruby分解 → 坐标转换 → calc_path。

    Returns:
        cells: 每个 cell 的顶点列表 [(N,2), ...]  (地图坐标)
        paths: 每个 cell 的规划路径 [(M,2), ...]  (地图坐标)
    """
    if not _has_path_coverage:
        print("[WARN] path_coverage 模块未导入，跳过 costmap 分解。")
        return [], []

    from shapely.geometry import Polygon as ShapelyPolygon, Point
    from collections import deque
    from math import floor, ceil, pi
    import subprocess
    import tempfile
    import json

    res = costmap['resolution']
    ox = costmap['origin_x']
    oy = costmap['origin_y']
    cw = costmap['width']
    ch = costmap['height']
    cm_data = costmap['data']

    # ── 1. 构建 shapely 多边形 ──────────────────────────────────────
    poly = ShapelyPolygon(polygon_pts)
    try:
        poly_core = poly.buffer(0)
        if hasattr(poly_core, "geoms") and poly_core.geom_type == "MultiPolygon":
            poly_core = max(poly_core.geoms, key=lambda g: g.area)
    except Exception:
        poly_core = poly

    poly_mask = poly_core
    if polygon_expand > 0.0:
        try:
            poly_mask = poly_mask.buffer(polygon_expand, join_style=2)
        except Exception:
            pass

    # ── 2. costmap 索引范围 ────────────────────────────────────────
    minx, miny, maxx, maxy = poly_mask.bounds
    minx_idx = max(0, floor((minx - ox) / res))
    maxx_idx = min(cw - 1, ceil((maxx - ox) / res))
    miny_idx = max(0, floor((miny - oy) / res))
    maxy_idx = min(ch - 1, ceil((maxy - oy) / res))

    if maxx_idx < minx_idx or maxy_idx < miny_idx:
        print("[WARN] costmap 范围为空，无法分解。")
        return [], []

    w = maxx_idx - minx_idx + 1
    h = maxy_idx - miny_idx + 1

    # ── 3. 构建 cell_meta: (in_mask, core_ok, expand_ok) ────────────
    cell_meta = []
    for ix in range(minx_idx, maxx_idx + 1):
        col = []
        for iy in range(miny_idx, maxy_idx + 1):
            x = (ix + 0.5) * res + ox
            y = (iy + 0.5) * res + oy
            pt = Point([x, y])

            if not poly_mask.covers(pt):
                col.append((False, False, False))
                continue

            data = cm_data[iy, ix]
            if data == -1:
                col.append((True, False, False))
                continue

            in_core = poly_core.covers(pt)
            core_ok = in_core and (data <= costmap_max_non_lethal)
            # expand: 只允许值为 0 的完全自由区域
            expand_ok = (not in_core) and (data == 0)
            col.append((True, core_ok, expand_ok))
        cell_meta.append(col)

    # ── 4. BFS 从 core 种子生长 ────────────────────────────────────
    visited = [[False for _ in range(h)] for _ in range(w)]
    q = deque()
    seed_count = 0
    for x in range(w):
        for y in range(h):
            _, core_ok, _ = cell_meta[x][y]
            if core_ok:
                visited[x][y] = True
                q.append((x, y))
                seed_count += 1

    if seed_count == 0:
        print("[WARN] 无可通行 core 种子，跳过分解。")
        return [], []

    dirs = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    while q:
        cx, cy = q.popleft()
        for dx, dy in dirs:
            nx, ny = cx + dx, cy + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            if visited[nx][ny]:
                continue
            in_mask, core_ok, expand_ok = cell_meta[nx][ny]
            if (not in_mask) or (not (core_ok or expand_ok)):
                continue
            visited[nx][ny] = True
            q.append((nx, ny))

    # ── 5. 构建 Ruby 输入网格 (-1=可通行, 0=障碍) ─────────────────
    rows = []
    for x in range(w):
        col = []
        for y in range(h):
            col.append(-1 if visited[x][y] else 0)
        rows.append(col)

    # ── 6. 调用 Ruby 分解脚本 ──────────────────────────────────────
    ruby_script = os.path.join(
        _path_coverage_dir, 'scripts', 'boustrophedon_decomposition.rb')
    if not os.path.exists(ruby_script):
        print(f"[WARN] Ruby 脚本不存在: {ruby_script}")
        return [], []

    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as ftmp:
        ftmp.write(json.dumps(rows))
        tmp_name = ftmp.name

    try:
        result = subprocess.run(
            ['ruby', ruby_script, tmp_name],
            capture_output=True, text=True, timeout=30)
        cell_polygons_grid = json.loads(result.stdout)
    except Exception as e:
        print(f"[WARN] Ruby 分解失败: {e}")
        if hasattr(e, 'stderr'):
            print(f"  stderr: {e.stderr}")
        return [], []
    finally:
        try:
            os.unlink(tmp_name)
        except Exception:
            pass

    if not cell_polygons_grid:
        print("[WARN] Ruby 分解返回空结果。")
        return [], []

    # ── 7. 坐标转换: 网格坐标 → 地图坐标 ──────────────────────────
    cells = []
    paths = []

    for cell_grid in cell_polygons_grid:
        # 转换到地图坐标
        map_pts = []
        for pt in cell_grid:
            mx = (pt[0] + minx_idx) * res + ox
            my = (pt[1] + miny_idx) * res + oy
            map_pts.append((mx, my))

        if len(map_pts) < 3:
            continue

        try:
            cell_poly = ShapelyPolygon(map_pts)
            cell_poly = cell_poly.buffer(0)  # 修复自交
            if cell_poly.is_empty:
                continue
            # 取最大部分
            if hasattr(cell_poly, "geoms") and cell_poly.geom_type == "MultiPolygon":
                cell_poly = max(cell_poly.geoms, key=lambda g: g.area)
            if cell_poly.geom_type != "Polygon" or cell_poly.is_empty:
                continue
            coords = list(cell_poly.exterior.coords)
            cells.append(np.array(coords))
        except Exception:
            continue

        # ── 8. 对每个 cell 生成规划路径（复用 drive_polygon 逻辑） ──
        try:
            angle = get_angle_of_longest_side_to_horizontal(cell_poly)
            if angle is None:
                continue
            angle += pi / 2  # up/down instead of left/right
            poly_rotated = rotate_polygon(cell_poly, angle)
            path_rotated = calc_path(poly_rotated, robot_width)
            if path_rotated:
                path = rotate_points(path_rotated, -angle)
                paths.append(np.array(path))
        except Exception as e:
            print(f"[WARN] cell 路径生成失败: {e}")
            continue

    return cells, paths


def fig1_planned_on_costmap(
    costmap: dict,
    polygon: List[Tuple[float, float]],
    robot_width: float,
    output_path: str,
) -> None:
    """生成图1：RViz 风格完整规划路径图。

    底图=代价地图，叠加：
      - 红色多边形边界 (visualize_area)
      - 蓝色 cell 分解 (visualize_trapezoid)
      - 绿色完整规划路径 (visualize_path)
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    # ── 底图: costmap ────────────────────────────────────────────────
    rgb = costmap_to_rgb(costmap)
    extent = costmap_extent(costmap)
    ax.imshow(rgb, extent=extent, origin='lower', interpolation='nearest')

    # ── Costmap 感知分解 + 规划路径 ─────────────────────────────────
    cells, paths = _costmap_aware_decompose(
        costmap, polygon, robot_width)

    # 蓝色 cell（先画填充再画边框）
    for cell_pts in cells:
        if len(cell_pts) >= 3:
            cell_patch = MplPolygon(cell_pts, fill=True, closed=True,
                                    facecolor=(0.3, 0.5, 1.0, 0.12),
                                    edgecolor=(0.0, 0.3, 1.0),
                                    linewidth=1.2, linestyle='-')
            ax.add_patch(cell_patch)

    # 绿色规划路径（每个 cell 一条路径）
    total_waypoints = 0
    for path in paths:
        if len(path) >= 2:
            ax.plot(path[:, 0], path[:, 1], color=(0.1, 0.85, 0.1),
                    linewidth=1.5, alpha=0.95, zorder=5)
            total_waypoints += len(path)
            # 箭头标记方向
            step = max(1, len(path) // 6)
            for i in range(0, len(path) - 1, step):
                dx = path[i + 1, 0] - path[i, 0]
                dy = path[i + 1, 1] - path[i, 1]
                ax.arrow(path[i, 0], path[i, 1], dx * 0.45, dy * 0.45,
                         head_width=0.04, head_length=0.06, fc='darkgreen',
                         ec='darkgreen', alpha=0.8, zorder=6)

    if total_waypoints > 0:
        ax.plot([], [], color=(0.1, 0.85, 0.1), linewidth=1.5,
                label='Planned path')

    # ── 红色多边形边框（最上层） ─────────────────────────────────────
    poly_patch = MplPolygon(polygon, fill=False, closed=True,
                            edgecolor=(1.0, 0.15, 0.15),
                            linewidth=2.0, linestyle='-', alpha=0.9,
                            zorder=7)
    ax.add_patch(poly_patch)

    # ── 标注 ─────────────────────────────────────────────────────────
    n_cells = len(cells)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Boustrophedon Coverage Path Planning\n'
                 f'({n_cells} cells, {total_waypoints} waypoints, '
                 f'robot width={robot_width:.3f}m)',
                 fontsize=13)
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    _add_scale_bar(ax, costmap['resolution'])

    plt.tight_layout()
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"[OK] 图1已保存: {output_path}")


# ═══════════════════════════════════════════════════════════════════════════
# 图2: 实际覆盖结果
# ═══════════════════════════════════════════════════════════════════════════

def fig2_actual_coverage(
    costmap: dict,
    trajectory: np.ndarray,
    polygon: List[Tuple[float, float]],
    coverage_radius: float,
    output_path: str,
) -> None:
    """生成图2：实际覆盖结果。

    底图=代价地图，叠加实际轨迹（绿线）和覆盖区域（红色半透明叠加）。
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    # ── 底图: costmap ────────────────────────────────────────────────
    rgb = costmap_to_rgb(costmap)
    extent = costmap_extent(costmap)
    ax.imshow(rgb, extent=extent, origin='lower', interpolation='nearest')

    # ── 多边形边框 ───────────────────────────────────────────────────
    poly_patch = MplPolygon(polygon, fill=False, edgecolor='white',
                            linewidth=1.5, linestyle='--', alpha=0.8)
    ax.add_patch(poly_patch)

    # ── 实际轨迹 ─────────────────────────────────────────────────────
    tx = trajectory[:, 1]
    ty = trajectory[:, 2]
    ax.plot(tx, ty, color='lime', linewidth=0.8, alpha=0.85,
            label='Actual trajectory')

    # ── 覆盖区域（红色半透明 mask） ─────────────────────────────────
    covered = compute_coverage_grid(trajectory, costmap, coverage_radius)

    # 绘制覆盖区域为红色半透明 overlay
    covered_rgba = np.zeros((covered.shape[0], covered.shape[1], 4), dtype=np.float32)
    covered_rgba[covered, 0] = 1.0   # 红色通道
    covered_rgba[covered, 3] = 0.35  # alpha
    ax.imshow(covered_rgba, extent=extent, origin='lower',
              interpolation='nearest', zorder=5)

    # ── 覆盖率计算 ──────────────────────────────────────────────────
    # 只统计可通行区域（costmap 值为 0）内的覆盖率
    passable_mask = (costmap['data'] == 0)
    total_passable = int(np.sum(passable_mask))
    if total_passable > 0:
        covered_in_passable = int(np.sum(covered & passable_mask))
        ratio = covered_in_passable / total_passable
    else:
        ratio = 0.0
        covered_in_passable = 0

    cov_text = (f'Coverage: {ratio * 100:.1f}%\n'
                f'({covered_in_passable}/{total_passable} passable cells)')
    ax.text(0.02, 0.98, cov_text, transform=ax.transAxes,
            fontsize=11, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.85))

    # ── 标注 ─────────────────────────────────────────────────────────
    traj_len = np.sum(np.sqrt(np.diff(tx)**2 + np.diff(ty)**2))
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Actual Coverage Result\n'
                 f'(trajectory: {len(trajectory)} pts, '
                 f'{traj_len:.2f}m, radius={coverage_radius:.3f}m)',
                 fontsize=13)
    ax.set_aspect('equal')
    ax.legend(loc='upper right')
    _add_scale_bar(ax, costmap['resolution'])

    plt.tight_layout()
    fig.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close(fig)
    print(f"[OK] 图2已保存: {output_path}")


# ═══════════════════════════════════════════════════════════════════════════
# 辅助
# ═══════════════════════════════════════════════════════════════════════════

def _add_scale_bar(ax, resolution: float) -> None:
    """添加一个简单的比例尺标注（文字形式）。"""
    ax.text(0.98, 0.02, f'Grid res: {resolution:.3f}m',
            transform=ax.transAxes, fontsize=8, ha='right',
            color='gray', alpha=0.7)


# ═══════════════════════════════════════════════════════════════════════════
# 主入口
# ═══════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description='论文覆盖路径插图生成器')
    parser.add_argument('--csv', required=True,
                        help='trajectory CSV 路径 (trajectory_*.csv)')
    parser.add_argument('--costmap', required=True,
                        help='costmap NPZ 路径 (costmap_*.npz)')
    parser.add_argument('--output', default='./paper_figures/',
                        help='输出目录 (默认: ./paper_figures/)')
    parser.add_argument('--polygon',
                        default='0,0.4;2.4,0.4;2.4,-1.4;0,-1.4',
                        help='作业区域多边形 (格式: x1,y1;x2,y2;...)')
    parser.add_argument('--robot-width', type=float, default=0.171,
                        help='机器人/割草宽度 (m, 默认: 0.171)')
    parser.add_argument('--coverage-radius', type=float, default=0.086,
                        help='覆盖半径 (m) = 割草宽度/2, 默认: 0.086 = 0.171/2')
    args = parser.parse_args()

    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)

    # 解析多边形
    polygon = parse_polygon(args.polygon)
    if len(polygon) < 3:
        print(f"错误: 多边形至少需要3个顶点，当前有 {len(polygon)} 个")
        sys.exit(1)
    print(f"多边形: {len(polygon)} 顶点, "
          f"robot_width={args.robot_width:.3f}m, "
          f"coverage_radius={args.coverage_radius:.3f}m")

    # 加载数据
    print(f"加载 costmap: {args.costmap}")
    costmap = load_costmap(args.costmap)
    print(f"  costmap: {costmap['width']}x{costmap['height']}, "
          f"res={costmap['resolution']:.3f}m, "
          f"origin=({costmap['origin_x']:.3f}, {costmap['origin_y']:.3f})")

    print(f"加载 trajectory: {args.csv}")
    traj = load_trajectory(args.csv)
    print(f"  trajectory: {len(traj)} 点, "
          f"t=[{traj[0,0]:.1f}, {traj[-1,0]:.1f}]s")

    # 生成图1
    fig1_path = os.path.join(args.output, 'fig1_planned_on_costmap.png')
    fig1_planned_on_costmap(costmap, polygon, args.robot_width, fig1_path)

    # 生成图2
    fig2_path = os.path.join(args.output, 'fig2_actual_coverage.png')
    fig2_actual_coverage(costmap, traj, polygon, args.coverage_radius, fig2_path)

    print(f"\n完成！图片输出到: {args.output}/")


if __name__ == '__main__':
    main()
