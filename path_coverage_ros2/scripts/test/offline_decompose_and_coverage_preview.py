#!/usr/bin/env python3

import argparse
import ast
import json
import math
import os
import subprocess
import sys
import tempfile

import matplotlib.pyplot as plt
import numpy as np
import yaml
from shapely.geometry import Polygon

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

from path_coverage.list_helper import (  # noqa: E402
    get_angle_of_longest_side_to_horizontal,
    rotate_points,
    rotate_polygon,
)
from path_coverage.trapezoidal_coverage import calc_path as trapezoid_calc_path  # noqa: E402


def run_boustrophedon(ruby_script, rows_json_path):
    result = subprocess.run(
        ["ruby", ruby_script, rows_json_path],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "Ruby decomposition failed\n"
            f"code={result.returncode}\n"
            f"stdout={result.stdout}\n"
            f"stderr={result.stderr}"
        )

    try:
        polygons = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            "Failed to parse decomposition output as JSON\n"
            f"stdout={result.stdout}\n"
            f"stderr={result.stderr}"
        ) from exc

    return polygons


def read_pgm_image(pgm_path):
    """Read P2/P5 PGM into uint8 ndarray with shape (height, width)."""
    with open(pgm_path, "rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P2", b"P5"):
            raise ValueError(f"Unsupported PGM format: {magic!r}")

        def _next_token():
            while True:
                line = f.readline()
                if not line:
                    return None
                line = line.strip()
                if not line or line.startswith(b"#"):
                    continue
                parts = line.split()
                if parts:
                    return parts

        dims = _next_token()
        while dims is not None and len(dims) < 2:
            nxt = _next_token()
            if nxt is None:
                break
            dims += nxt
        if dims is None or len(dims) < 2:
            raise ValueError("Invalid PGM: missing width/height")
        width, height = int(dims[0]), int(dims[1])

        maxval_tokens = _next_token()
        if maxval_tokens is None:
            raise ValueError("Invalid PGM: missing maxval")
        maxval = int(maxval_tokens[0])
        if maxval <= 0:
            raise ValueError("Invalid PGM: non-positive maxval")

        if magic == b"P2":
            data = []
            while len(data) < width * height:
                parts = _next_token()
                if parts is None:
                    break
                data.extend([int(p) for p in parts])
            if len(data) != width * height:
                raise ValueError("Invalid P2 PGM: pixel count mismatch")
            arr = np.array(data, dtype=np.float32).reshape((height, width))
        else:
            raw = f.read(width * height)
            if len(raw) != width * height:
                raise ValueError("Invalid P5 PGM: pixel count mismatch")
            arr = np.frombuffer(raw, dtype=np.uint8).astype(np.float32).reshape((height, width))

        if maxval != 255:
            arr = arr * (255.0 / float(maxval))

    return arr.clip(0, 255).astype(np.uint8)


def rows_from_map_yaml(map_yaml_path):
    """Convert ROS map (pgm+yaml) to decomposition rows[x][y] with free=-1, blocked=0."""
    with open(map_yaml_path, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)

    image_rel = meta.get("image")
    if not image_rel:
        raise ValueError("Map yaml missing 'image' field")

    base_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    image_path = image_rel if os.path.isabs(image_rel) else os.path.join(base_dir, image_rel)

    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))
    negate = int(meta.get("negate", 0))
    map_mode = str(meta.get("mode", "trinary")).strip().lower()

    pgm = read_pgm_image(image_path)
    h, w = pgm.shape

    rows = []
    for x in range(w):
        col = []
        for y in range(h):
            # Convert top-left image coordinates to bottom-left map-like y axis.
            pixel = float(pgm[h - 1 - y, x])
            if map_mode == "trinary":
                # In trinary maps, grayscale values are discrete:
                # occupied(near 0), free(near 254/255), unknown(mid-tone, usually 205).
                if negate == 0:
                    if pixel <= 50:
                        col.append(0)  # occupied
                    elif pixel >= 250:
                        col.append(-1)  # free
                    else:
                        col.append(0)  # unknown -> blocked for planning safety
                else:
                    if pixel >= 205:
                        col.append(0)  # occupied after negation
                    elif pixel <= 5:
                        col.append(-1)  # free after negation
                    else:
                        col.append(0)  # unknown
            else:
                if negate:
                    occ = pixel / 255.0
                else:
                    occ = (255.0 - pixel) / 255.0

                if occ > occupied_thresh:
                    col.append(0)  # occupied => blocked
                elif occ < free_thresh:
                    col.append(-1)  # free
                else:
                    col.append(0)  # unknown treated as blocked for decomposition safety
        rows.append(col)

    meta_out = {
        "resolution": float(meta.get("resolution", 0.05)),
        "origin_x": float(meta.get("origin", [0.0, 0.0, 0.0])[0]),
        "origin_y": float(meta.get("origin", [0.0, 0.0, 0.0])[1]),
        "width": w,
        "height": h,
        "occupied_thresh": occupied_thresh,
        "free_thresh": free_thresh,
        "negate": negate,
        "mode": map_mode,
    }

    map_image_for_plot = np.flipud(pgm)
    return rows, meta_out, map_image_for_plot


def world_to_grid(wx, wy, meta):
    gx = (float(wx) - meta["origin_x"]) / meta["resolution"]
    gy = (float(wy) - meta["origin_y"]) / meta["resolution"]
    return gx, gy


def parse_polygon_world(polygon_world_text):
    pts = ast.literal_eval(polygon_world_text)
    if not isinstance(pts, (list, tuple)) or len(pts) < 3:
        raise ValueError("polygon-world must be a list/tuple with at least 3 points")
    norm = []
    for p in pts:
        if not isinstance(p, (list, tuple)) or len(p) != 2:
            raise ValueError("each polygon point must be (x, y)")
        norm.append((float(p[0]), float(p[1])))
    return norm


def parse_obstacles_world(obstacles_world_text):
    """Parse one or multiple obstacle polygons in world coordinates.

    Supported formats:
    - Single polygon: '[(x1,y1), (x2,y2), ...]'
    - Multiple polygons: '[[(...), (...), ...], [(...), ...]]'
    """
    value = ast.literal_eval(obstacles_world_text)

    def _norm_poly(poly):
        if not isinstance(poly, (list, tuple)) or len(poly) < 3:
            raise ValueError("each obstacle polygon must have at least 3 points")
        out = []
        for p in poly:
            if not isinstance(p, (list, tuple)) or len(p) != 2:
                raise ValueError("obstacle polygon point must be (x, y)")
            out.append((float(p[0]), float(p[1])))
        return out

    if not isinstance(value, (list, tuple)) or len(value) < 3:
        raise ValueError("obstacles-world must contain at least one polygon")

    first = value[0]
    if isinstance(first, (list, tuple)) and len(first) == 2 and not isinstance(first[0], (list, tuple)):
        return [_norm_poly(value)]

    return [_norm_poly(poly) for poly in value]


def apply_polygon_mask(rows, polygon_world_pts, meta):
    """Keep cells only inside polygon (map world coords); outside becomes blocked (0)."""
    poly_grid = Polygon([world_to_grid(x, y, meta) for x, y in polygon_world_pts])

    width = len(rows)
    height = len(rows[0]) if width > 0 else 0
    out = []
    for x in range(width):
        col = []
        for y in range(height):
            pt = Polygon(
                [
                    (x + 0.49, y + 0.49),
                    (x + 0.51, y + 0.49),
                    (x + 0.51, y + 0.51),
                    (x + 0.49, y + 0.51),
                ]
            ).centroid
            if poly_grid.covers(pt):
                col.append(rows[x][y])
            else:
                col.append(0)
        out.append(col)

    return out, list(poly_grid.exterior.coords)


def apply_obstacle_masks(rows, obstacles_world, meta):
    """Mark obstacle polygons as blocked inside current rows map (0=blocked)."""
    if not obstacles_world:
        return rows

    obstacle_grids = [Polygon([world_to_grid(x, y, meta) for x, y in poly]) for poly in obstacles_world]
    width = len(rows)
    height = len(rows[0]) if width > 0 else 0

    out = [col[:] for col in rows]
    for x in range(width):
        for y in range(height):
            pt = Polygon(
                [
                    (x + 0.49, y + 0.49),
                    (x + 0.51, y + 0.49),
                    (x + 0.51, y + 0.51),
                    (x + 0.49, y + 0.51),
                ]
            ).centroid
            for obs in obstacle_grids:
                if obs.covers(pt):
                    out[x][y] = 0
                    break

    return out


def polygons_from_rows(rows, ruby_script):
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False, encoding="utf-8") as tf:
        json.dump(rows, tf)
        tmp_json = tf.name

    try:
        polygons = run_boustrophedon(ruby_script, tmp_json)
    finally:
        try:
            os.remove(tmp_json)
        except OSError:
            pass

    return polygons


def rows_to_image(rows):
    width = len(rows)
    height = len(rows[0]) if width > 0 else 0
    img = np.ones((height, width), dtype=np.float32)
    for x in range(width):
        for y in range(height):
            # rows[x][y] == -1 means free in this project
            img[y, x] = 0.0 if rows[x][y] == -1 else 1.0
    return img


def generate_cell_path(cell_points, robot_width, clearance):
    poly = Polygon(cell_points)
    angle = get_angle_of_longest_side_to_horizontal(poly)
    if angle is None:
        return []

    angle += math.pi / 2.0
    poly_rot = rotate_polygon(poly, angle)
    path_rot = trapezoid_calc_path(poly_rot, robot_width, clearance=clearance)
    path = rotate_points(path_rot, -angle)
    return path


def preview(rows, ruby_script, robot_width, clearance, save_path, meta=None, polygon_grid_pts=None, map_image=None):
    polygons = polygons_from_rows(rows, ruby_script)
    print(f"decompose cells: {len(polygons)}")

    img = rows_to_image(rows)
    h, w = img.shape

    if meta is not None:
        x0 = meta["origin_x"]
        y0 = meta["origin_y"]
        x1 = x0 + w * meta["resolution"]
        y1 = y0 + h * meta["resolution"]
        extent = (x0, x1, y0, y1)
    else:
        extent = (0, w, 0, h)

    fig, ax = plt.subplots(figsize=(10, 7))
    if map_image is not None:
        # Show the real pgm map as background for visual consistency with RViz maps.
        ax.imshow(
            map_image,
            cmap="gray",
            origin="lower",
            interpolation="nearest",
            extent=extent,
            alpha=1.0,
            vmin=0,
            vmax=255,
        )
    else:
        ax.imshow(
            img,
            cmap="gray_r",
            origin="lower",
            interpolation="nearest",
            extent=extent,
            alpha=0.65,
        )

    def to_plot_xy(p):
        if meta is None:
            return p[0], p[1]
        return (
            meta["origin_x"] + p[0] * meta["resolution"],
            meta["origin_y"] + p[1] * meta["resolution"],
        )

    if polygon_grid_pts:
        poly_plot = [to_plot_xy(p) for p in polygon_grid_pts]
        ax.plot([p[0] for p in poly_plot], [p[1] for p in poly_plot], color="#ff3030", linewidth=2.0)

    for i, cell in enumerate(polygons, start=1):
        if len(cell) < 3:
            continue

        # Cell boundary
        cell_plot = [to_plot_xy(p) for p in cell]
        px = [p[0] for p in cell_plot] + [cell_plot[0][0]]
        py = [p[1] for p in cell_plot] + [cell_plot[0][1]]
        ax.plot(px, py, color="#0d5fff", linewidth=2.0)

        # Cell index label
        c = Polygon(cell_plot).centroid
        ax.text(c.x, c.y, str(i), color="#0d5fff", fontsize=10)

        # Coverage path: sparse direction arrows, consistent with RViz arrow intent.
        path = generate_cell_path(cell, robot_width, clearance)
        if len(path) < 2:
            continue

        # Draw arrows sparsely to avoid covering the whole cell as a green block.
        step = max(1, len(path) // 24)
        arrow_count = 0
        for idx in range(0, max(0, len(path) - 1), step):
            p0 = path[idx]
            p1 = path[idx + 1]
            p0p = to_plot_xy(p0)
            p1p = to_plot_xy(p1)
            ax.annotate(
                "",
                xy=p1p,
                xytext=p0p,
                arrowprops={
                    "arrowstyle": "->",
                    "color": "#12a150",
                    "lw": 1.4,
                    "shrinkA": 0,
                    "shrinkB": 0,
                    "mutation_scale": 10,
                    "alpha": 0.95,
                },
            )
            arrow_count += 1
        print(f"cell {i}: path_points={len(path)}, arrows_drawn={arrow_count}")

    ax.set_title(
        f"Cells: {len(polygons)} | robot_width={robot_width:.3f} | clearance={clearance:.3f}"
    )
    ax.set_xlabel("map x (m)" if meta else "grid x")
    ax.set_ylabel("map y (m)" if meta else "grid y")
    ax.set_aspect("equal", adjustable="box")

    if save_path:
        fig.savefig(save_path, dpi=160, bbox_inches="tight")
        print(f"saved: {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Offline preview for boustrophedon decomposition and coverage path generation"
    )
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument(
        "--rows-json",
        help="Path to decomposition input JSON (rows[x][y], free=-1, blocked=0)",
    )
    src.add_argument(
        "--map-yaml",
        help="Path to ROS map yaml (with pgm image), e.g. map.yaml",
    )
    parser.add_argument(
        "--polygon-world",
        default="",
        help="Polygon points in map coordinates, e.g. '[(2.0,0.0),(1.7,-0.9),(0.2,-0.9),(0.3,0.0),(2.0,0.0)]'",
    )
    parser.add_argument(
        "--obstacles-world",
        default="",
        help="Optional world-coordinate obstacle polygon(s) to force blocked, single polygon '[(x,y),...]' or multi-polygons '[[(x,y),...],[(x,y),...]]'",
    )
    parser.add_argument(
        "--ruby-script",
        default=os.path.abspath(os.path.join(SCRIPT_DIR, "..", "boustrophedon_decomposition.rb")),
        help="Path to boustrophedon_decomposition.rb",
    )
    parser.add_argument("--robot-width", type=float, default=0.15)
    parser.add_argument("--clearance", type=float, default=0.0)
    parser.add_argument("--save", default="", help="Optional output image path")
    parser.add_argument(
        "--export-rows-json",
        default="",
        help="Optional path to export converted rows JSON (useful when input is --map-yaml)",
    )

    args = parser.parse_args()

    meta = None
    map_image = None
    polygon_grid_pts = None

    base_rows = None
    if args.rows_json:
        with open(args.rows_json, "r", encoding="utf-8") as f:
            rows = json.load(f)
    else:
        rows, meta, map_image = rows_from_map_yaml(args.map_yaml)
        base_rows = [col[:] for col in rows]

    if args.polygon_world.strip():
        if meta is None:
            raise ValueError("--polygon-world requires --map-yaml input (needs origin/resolution)")
        polygon_world_pts = parse_polygon_world(args.polygon_world.strip())
        rows, polygon_grid_pts = apply_polygon_mask(rows, polygon_world_pts, meta)

        if base_rows is not None:
            poly_grid = Polygon([world_to_grid(x, y, meta) for x, y in polygon_world_pts])
            blocked_inside = 0
            free_inside = 0
            for x in range(len(base_rows)):
                for y in range(len(base_rows[0])):
                    if not poly_grid.covers(Polygon([(x + 0.49, y + 0.49), (x + 0.51, y + 0.49), (x + 0.51, y + 0.51), (x + 0.49, y + 0.51)]).centroid):
                        continue
                    if base_rows[x][y] == -1:
                        free_inside += 1
                    else:
                        blocked_inside += 1
            print(f"polygon mask stats: free_inside={free_inside}, blocked_inside={blocked_inside}")

    if args.obstacles_world.strip():
        if meta is None:
            raise ValueError("--obstacles-world requires --map-yaml input")
        obstacles_world = parse_obstacles_world(args.obstacles_world.strip())
        rows = apply_obstacle_masks(rows, obstacles_world, meta)
        print(f"applied obstacle polygons: {len(obstacles_world)}")

    if args.export_rows_json.strip():
        with open(args.export_rows_json.strip(), "w", encoding="utf-8") as f:
            json.dump(rows, f)
        print(f"exported rows json: {args.export_rows_json.strip()}")

    preview(
        rows=rows,
        ruby_script=args.ruby_script,
        robot_width=args.robot_width,
        clearance=max(0.0, args.clearance),
        save_path=args.save.strip(),
        meta=meta,
        polygon_grid_pts=polygon_grid_pts,
        map_image=map_image,
    )


if __name__ == "__main__":
    main()
