#!/usr/bin/env python3
"""
test_coverage_comparison.py — 覆盖算法三组消融对照实验。

Group A (传统基准):  LiDAR 静态地图 + 原始 path_coverage
Group B (消融组):    RTAB-Map 视觉 SLAM + 原始 path_coverage
Group C (创新组):    RTAB-Map 视觉 SLAM + 改进 path_coverage

用法:
    python3 tools/test_coverage_comparison.py --mode a|b|c|all
"""
import argparse, os, shlex, shutil, signal, subprocess, sys, time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
LAUNCHER_DIR = SCRIPT_DIR / ".." / "launcher"
WS_ROOT = SCRIPT_DIR / ".."
MAP_BACKUP_DIR = str(Path.home() / ".ros" / "maps")
LOG_DIR = str(Path.home() / "ros2_ws" / "src" / "logs" / "comparison")
REGION_FILE = str(LAUNCHER_DIR / "regions" / "test_180x240.yaml")
SLAM_MAPS_DIR = str(WS_ROOT / "slam" / "maps")
DEFAULT_MAP = "test_map"
PREFIX = {"a": "group_a_baseline", "b": "group_b_ablation", "c": "group_c_innovation"}
GROUP_LABEL = {"a": "Group A (LiDAR+原始)", "b": "Group B (RTAB-Map+原始)", "c": "Group C (RTAB-Map+改进)"}

def _source_cmd():
    parts = [f"source /opt/ros/humble/setup.sh"]
    ws = WS_ROOT / "install" / "setup.bash"
    if ws.exists(): parts.append(f"source {shlex.quote(str(ws))}")
    return " && ".join(parts)

def _info(m): print(f"\033[36m[INFO]\033[0m {m}")
def _ok(m): print(f"\033[32m[OK]\033[0m {m}")
def _warn(m): print(f"\033[33m[WARN]\033[0m {m}")

def _stop_ros():
    s = Path.home() / ".stop_ros.sh"
    if s.exists(): subprocess.call(["bash", str(s)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); time.sleep(1.0)

def _ensure_test_map():
    yd, pd = os.path.join(SLAM_MAPS_DIR, f"{DEFAULT_MAP}.yaml"), os.path.join(SLAM_MAPS_DIR, f"{DEFAULT_MAP}.pgm")
    if os.path.exists(yd): return True
    ys = os.path.join(MAP_BACKUP_DIR, f"{DEFAULT_MAP}.yaml")
    if not os.path.exists(ys): _warn(f"未找到 {ys}"); return False
    os.makedirs(SLAM_MAPS_DIR, exist_ok=True)
    shutil.copy2(ys, yd)
    if os.path.exists(os.path.join(MAP_BACKUP_DIR, f"{DEFAULT_MAP}.pgm")): shutil.copy2(os.path.join(MAP_BACKUP_DIR, f"{DEFAULT_MAP}.pgm"), pd)
    _ok(f"地图已复制到 {SLAM_MAPS_DIR}"); return True

def _restore_rtabmap_db() -> bool:
    """恢复保存的 rtabmap.db — 与 launcher restore_map 行为一致。

    localization:=true 模式必须使用建图时保存的数据库，
    否则 RTAB-Map 无法正确定位。
    """
    src_db = os.path.join(MAP_BACKUP_DIR, f"{DEFAULT_MAP}.db")
    if not os.path.exists(src_db):
        _warn(f"rtabmap.db 备份不存在: {src_db}")
        _warn("请确保已在 launcher 中执行过 save test_map")
        return False
    dst_db = str(Path.home() / ".ros" / "rtabmap.db")
    shutil.copy2(src_db, dst_db)
    _ok(f"rtabmap.db 已恢复: {DEFAULT_MAP}.db → ~/.ros/rtabmap.db")
    return True

def _check_ld19():
    try:
        r = subprocess.run(["bash", "-lc", f"{_source_cmd()} && ros2 topic hz /scan --timeout 3 2>&1"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=8)
        return "average rate" in r.stdout.decode(errors="replace").lower()
    except: return False

def _launch_mapserver(prefix):
    """启动 map_server 并返回进程句柄（用于 cleanup）。"""
    gy = os.path.join(MAP_BACKUP_DIR, f"{DEFAULT_MAP}.yaml")
    if not os.path.exists(gy): _warn(f"地图不存在: {gy}"); return None
    _info("启动 map_server...")
    ms = subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={shlex.quote(gy)}"], stdout=open(os.path.join(LOG_DIR, f"{prefix}_mapserver.log"), "w"), stderr=subprocess.STDOUT)
    time.sleep(5.0)
    subprocess.run(f"{_source_cmd()} && ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate", shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
    _ok("map_server 已激活")
    return ms

def _cleanup(*ps):
    for p in ps:
        if p and p.poll() is None: p.terminate()
        try:
            if p: p.wait(timeout=3.0)
        except: pass

def _run_common(label, nav_cmd, rviz_cmd, pathcov_cmd, use_mapserver, costmap_wait):
    """统一启动流程 — 时序与 launcher coverage 模式对齐，RPi 适配 x1.5。

    时序: nav→3s→rviz→8s→mapserver→costmap_wait→pc+eval→23s→publish
    """
    _stop_ros(); os.makedirs(LOG_DIR, exist_ok=True)
    # RTAB-Map 模式：恢复保存的数据库（localization 定位必需）
    if use_mapserver and not _restore_rtabmap_db():
        _warn("无法恢复 rtabmap.db，定位可能失败！继续？"); input()
    nav = subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && {nav_cmd}"], stdout=open(os.path.join(LOG_DIR, f"{PREFIX[label]}_nav.log"), "w"), stderr=subprocess.STDOUT)
    time.sleep(1.0)
    rviz = subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && {rviz_cmd}"], stdout=open(os.path.join(LOG_DIR, f"{PREFIX[label]}_rviz.log"), "w"), stderr=subprocess.STDOUT)
    time.sleep(5.0)
    ms = _launch_mapserver(PREFIX[label]) if use_mapserver else None
    _info(f"等待 costmap 稳定 ({costmap_wait}s)..."); time.sleep(costmap_wait)
    pc = subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && {pathcov_cmd}"], stdout=open(os.path.join(LOG_DIR, f"{PREFIX[label]}_pathcoverage.log"), "w"), stderr=subprocess.STDOUT)
    ev = subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && ros2 launch coverage_evaluator coverage_evaluator.launch.py"], stdout=open(os.path.join(LOG_DIR, f"{PREFIX[label]}_evaluator.log"), "w"), stderr=subprocess.STDOUT)
    _info("等待节点就绪 + DDS 发现 (23s)..."); time.sleep(23.0)
    # LD19 check for Group A — must be after nav has fully started
    if label == "a" and not _check_ld19():
        _warn("LD19 /scan 无数据！请确认雷达已连接。继续？"); input()
    pub = str(LAUNCHER_DIR / "publish_region.py")
    rc = subprocess.run(["python3", pub, "--file", REGION_FILE, "--wait", "8"], timeout=30)
    if rc.returncode != 0: _warn(f"区域发布失败"); _cleanup(pc, ev, rviz, nav, ms); return
    _ok(f"{GROUP_LABEL[label]} 就绪 —— 观察 RViz 确认覆盖开始"); print()
    try: nav.wait()
    except KeyboardInterrupt: pass
    finally: _cleanup(pc, ev, rviz, nav, ms)
    _ok(f"{GROUP_LABEL[label]} 完成")

def _prompt_origin(label: str) -> None:
    input("车已放好？按 Enter 继续...")

def run_group_a():
    print("\n\033[1;34m=== Group A: LiDAR + 原始算法 ===\033[0m\n")
    if not _ensure_test_map(): return
    _prompt_origin("a")
    _run_common("a", f"ros2 launch navigation navigation.launch.py map:={DEFAULT_MAP}", "ros2 launch navigation rviz_navigation.launch.py", "ros2 launch path_coverage path_coverage_baseline.launch.py", False, 30)

def run_group_b():
    print("\n\033[1;33m=== Group B: RTAB-Map + 原始算法 (消融) ===\033[0m\n")
    _prompt_origin("b")
    _run_common("b", "ros2 launch navigation rtabmap_navigation.launch.py localization:=true", "ros2 launch navigation rviz_rtabmap_navigation.launch.py", "ros2 launch path_coverage path_coverage_baseline.launch.py", True, 45)

def run_group_c():
    print("\n\033[1;32m=== Group C: RTAB-Map + 改进算法 (创新) ===\033[0m\n")
    _prompt_origin("c")
    _run_common("c", "ros2 launch navigation rtabmap_navigation.launch.py localization:=true", "ros2 launch navigation rviz_rtabmap_navigation.launch.py", "ros2 launch path_coverage path_coverage.launch.py", True, 45)

def main():
    p = argparse.ArgumentParser(description="三组消融对照实验")
    p.add_argument("--mode", choices=["a","b","c","all"], default="all")
    args = p.parse_args()
    signal.signal(signal.SIGINT, lambda s,f: sys.exit(130))
    groups = ["a","b","c"] if args.mode == "all" else [args.mode]
    for g in groups:
        if len(groups) > 1 and g != groups[0]:
            print("\n" + "="*50 + "\n  组间切换：请将车手动推回地图原点...\n" + "="*50)
        {"a": run_group_a, "b": run_group_b, "c": run_group_c}[g]()

if __name__ == "__main__": main()
