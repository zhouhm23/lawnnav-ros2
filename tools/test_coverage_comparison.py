#!/usr/bin/env python3
"""
test_coverage_comparison.py — 全覆盖性能对照实验（论文表1）。

支持参数化:
    --sensor camera|lidar|vslam  --algo ours|baseline
    --runs N  (默认3次)

实验矩阵 (6组):
    单深度相机 × 边缘外扩 BCD (★ 本文方法)
    单深度相机 × 标准 BCD      (消融)
    单2D雷达   × 边缘外扩 BCD   (泛化, 有空再测)
    单2D雷达   × 标准 BCD       (传统基线)
    视觉+雷达  × 边缘外扩 BCD   (融合上限, 有空再测)
    视觉+雷达  × 标准 BCD       (融合基线, 有空再测)

用法:
    python3 tools/test_coverage_comparison.py --sensor camera --algo ours
    python3 tools/test_coverage_comparison.py --core    # 核心4组
    python3 tools/test_coverage_comparison.py --all     # 全部6组
"""
import argparse, os, shlex, shutil, signal, subprocess, sys, time
from pathlib import Path

WS_ROOT = Path(__file__).resolve().parent.parent
LAUNCHER_DIR = WS_ROOT / "launcher"
MAP_BACKUP_DIR = str(Path.home() / ".ros" / "maps")
SLAM_MAPS_DIR = str(WS_ROOT / "slam" / "maps")
LOG_DIR = str(WS_ROOT / "logs" / "coverage")
RESULTS_DIR = str(Path(__file__).resolve().parent / "results")
REGION_FILE = str(LAUNCHER_DIR / "regions" / "test_180x240.yaml")

SENSOR_MAP = {"camera": "camera_map", "lidar": "radar_map", "vslam": "vslam_map"}
NAV_CMD = {
    "camera": "ros2 launch navigation rtabmap_camera_nav.launch.py localization:=true",
    "lidar":  "ros2 launch navigation slam_toolbox_lidar_nav.launch.py",
    "vslam":  "ros2 launch navigation rtabmap_vslam_nav.launch.py localization:=true",
}
ALGO_CMD = {
    "ours":     "ros2 launch path_coverage path_coverage.launch.py",
    "baseline": "ros2 launch path_coverage path_coverage_baseline.launch.py",
}
COVERAGE_TIMEOUT = 1200  # 20 分钟
POST_RUN_WAIT = 5

os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(LOG_DIR, exist_ok=True)

def _source_cmd():
    parts = ["source /opt/ros/humble/setup.sh"]
    ws = WS_ROOT / "install" / "setup.bash"
    if ws.exists(): parts.append(f"source {shlex.quote(str(ws))}")
    return " && ".join(parts)

def _ros_env():
    env = os.environ.copy()
    env["RCUTILS_LOGGING_SEVERITY_THRESHOLD"] = "ERROR"
    env["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "[{severity}] {message}"
    return env

def _info(m): print(f"\033[36m[INFO]\033[0m {m}")
def _ok(m): print(f"\033[32m[OK]\033[0m {m}")
def _warn(m): print(f"\033[33m[WARN]\033[0m {m}")

def _stop_ros():
    try:
        subprocess.run(f'{_source_cmd()} && ros2 topic pub --once /cmd_vel '
                       f'geometry_msgs/msg/Twist "{{linear: {{x:0,y:0,z:0}}, angular: {{x:0,y:0,z:0}}}}"',
                       shell=True, executable="/bin/bash",
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
    except Exception: pass
    s = Path.home() / ".stop_ros.sh"
    if s.exists():
        subprocess.call(["sudo", "bash", str(s)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.5)
    else:
        subprocess.call(["pkill", "-f", "ros2"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.5)

def _cleanup(*procs):
    for p in procs:
        if p and p.poll() is None: p.terminate()
        try:
            if p: p.wait(timeout=3)
        except subprocess.TimeoutExpired: pass

def _restore_rtabmap_db(map_name):
    src = os.path.join(MAP_BACKUP_DIR, f"{map_name}.db")
    dst = str(Path.home() / ".ros" / "rtabmap.db")
    if not os.path.exists(src):
        _warn(f"rtabmap.db 不存在: {src}，请先 start.py → mapping → save {map_name}")
        return False
    shutil.copy2(src, dst)
    _ok(f"rtabmap.db 已恢复: {map_name}.db")
    return True

def _ensure_lidar_map(map_name):
    """确保雷达 pgm+yaml 在 slam/maps/ 目录下（导航 launch 从此处读取）。

    优先检查 slam/maps/（slam_toolbox 默认保存位置），
    否则从 ~/.ros/maps/ 复制。
    """
    yaml_dst = os.path.join(SLAM_MAPS_DIR, f"{map_name}.yaml")
    if os.path.exists(yaml_dst):
        _ok(f"雷达地图已存在: {yaml_dst}")
        return True

    # 尝试从 ~/.ros/maps/ 复制
    yaml_src = os.path.join(MAP_BACKUP_DIR, f"{map_name}.yaml")
    if os.path.exists(yaml_src):
        os.makedirs(SLAM_MAPS_DIR, exist_ok=True)
        shutil.copy2(yaml_src, yaml_dst)
        pgm_src = os.path.join(MAP_BACKUP_DIR, f"{map_name}.pgm")
        if os.path.exists(pgm_src):
            shutil.copy2(pgm_src, os.path.join(SLAM_MAPS_DIR, f"{map_name}.pgm"))
        _ok(f"雷达地图已复制: {map_name}")
        return True

    _warn(f"雷达地图不存在。请先建图保存到 ~/.ros/maps/{map_name}.yaml")
    return False

def _launch_mapserver(map_name):
    yaml_path = os.path.join(MAP_BACKUP_DIR, f"{map_name}.yaml")
    if not os.path.exists(yaml_path):
        _warn(f"地图不存在: {yaml_path}")
        return None
    mp = subprocess.Popen(
        ["bash", "-lc", f"{_source_cmd()} && ros2 run nav2_map_server map_server "
         f"--ros-args -p yaml_filename:={shlex.quote(yaml_path)}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=_ros_env())
    time.sleep(3)
    subprocess.run(f"{_source_cmd()} && ros2 lifecycle set /map_server configure && "
                   f"ros2 lifecycle set /map_server activate",
                   shell=True, executable="/bin/bash",
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20)
    _ok("map_server 已激活")
    return mp

def _publish_rtabmap_map():
    _info("触发 RTAB-Map 发布 grid_map...")
    subprocess.run(f"{_source_cmd()} && ros2 service call /rtabmap/publish_map "
                   f'rtabmap_msgs/srv/PublishMap "{{global_map: true, optimized: true, graph_only: false}}"',
                   shell=True, executable="/bin/bash",
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)

def run_one(sensor, algo, run_id):
    label = f"{sensor}_{algo}_run{run_id}"
    map_name = SENSOR_MAP[sensor]
    print(f"\n{'='*60}")
    print(f"  {label}  传感器:{sensor}  算法:{algo}  第{run_id}次")
    print(f"{'='*60}")
    _info("请在 PC 上启动录像！按 Enter 继续..."); input()
    _stop_ros()

    nav = subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && {NAV_CMD[sensor]}"],
                           stdout=open(os.path.join(LOG_DIR, f"{label}_nav.log"), "w"),
                           stderr=subprocess.STDOUT, env=_ros_env())
    _info(f"navigation 已启动 ({sensor})"); time.sleep(10)

    mapserver = None
    if sensor in ("camera", "vslam"):
        if not _restore_rtabmap_db(map_name): _cleanup(nav); return False
        time.sleep(10); _publish_rtabmap_map()
    elif sensor == "lidar":
        if not _ensure_lidar_map(map_name): _cleanup(nav); return False
        mapserver = _launch_mapserver(map_name); time.sleep(2)

    _info("等待 costmap 稳定 (5s)..."); time.sleep(5)

    pc = subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && {ALGO_CMD[algo]}"],
                          stdout=open(os.path.join(LOG_DIR, f"{label}_pathcoverage.log"), "w"),
                          stderr=subprocess.STDOUT, env=_ros_env())
    ev = subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && ros2 launch coverage_evaluator coverage_evaluator.launch.py"],
                          stdout=open(os.path.join(LOG_DIR, f"{label}_evaluator.log"), "w"),
                          stderr=subprocess.STDOUT, env=_ros_env())
    _info("等待节点就绪 (5s)..."); time.sleep(5)

    pub = str(LAUNCHER_DIR / "publish_region.py")
    rc = subprocess.run(["python3", pub, "--file", REGION_FILE, "--wait", "5"], timeout=30)
    if rc.returncode != 0:
        _warn("区域发布失败"); _cleanup(pc, ev, nav, mapserver); return False

    _ok(f"{label} 覆盖已开始")
    start = time.time()
    try:
        while time.time() - start < COVERAGE_TIMEOUT:
            if pc.poll() is not None: _info("path_coverage 已退出"); break
            time.sleep(5)
        else:
            _warn(f"超时 {COVERAGE_TIMEOUT}s，强制结束")
            _cleanup(pc, ev, nav, mapserver); return False
    except KeyboardInterrupt:
        _warn("用户中断"); _cleanup(pc, ev, nav, mapserver); raise

    _info(f"等待 evaluator 写出最后数据 ({POST_RUN_WAIT}s)..."); time.sleep(POST_RUN_WAIT)
    _cleanup(pc, ev, nav, mapserver)
    _ok(f"{label} 完成"); return True

def main():
    p = argparse.ArgumentParser(description="全覆盖性能对照实验")
    p.add_argument("--sensor", choices=["camera","lidar","vslam"], help="传感器")
    p.add_argument("--algo", choices=["ours","baseline"], help="覆盖算法")
    p.add_argument("--runs", type=int, default=3, help="每组重复次数(默认3)")
    p.add_argument("--all", action="store_true", help="全部6组")
    p.add_argument("--core", action="store_true", help="核心4组")
    args = p.parse_args()
    signal.signal(signal.SIGINT, lambda s,f: sys.exit(130))

    if args.sensor and args.algo:
        combos = [(args.sensor, args.algo)]
    elif args.all:
        combos = [("camera","ours"),("camera","baseline"),
                  ("lidar","ours"),("lidar","baseline"),
                  ("vslam","ours"),("vslam","baseline")]
    elif args.core:
        combos = [("camera","ours"),("camera","baseline"),
                  ("lidar","baseline"),("vslam","baseline")]
    else:
        _warn("请指定 --sensor --algo 或 --all 或 --core"); sys.exit(1)

    total_runs = len(combos) * args.runs
    print(f"\n{'='*60}\n  全覆盖对照实验\n  组数:{len(combos)} × {args.runs}次 = {total_runs}次\n{'='*60}\n")

    count = ok = 0
    for sensor, algo in combos:
        for rid in range(1, args.runs+1):
            count += 1
            _info(f"[{count}/{total_runs}] {sensor}×{algo} 第{rid}次")
            try:
                if run_one(sensor, algo, rid): ok += 1
            except KeyboardInterrupt: _warn("实验中断"); break
            time.sleep(3)

    print(f"\n{'='*60}\n  完成: {ok}/{total_runs} 成功\n{'='*60}")
    print(f"日志: {LOG_DIR}/")
    print(f"运行 python3 tools/paper_figures.py 生成论文插图")

if __name__ == "__main__": main()
