#!/usr/bin/env python3
"""
test_coverage_comparison.py — 覆盖算法三组消融对照实验。

Group A (传统基准):  LiDAR 静态地图 + 原始 path_coverage
Group B (消融组):    RTAB-Map 视觉 SLAM + 原始 path_coverage
Group C (创新组):    RTAB-Map 视觉 SLAM + 改进 path_coverage

用法:
    python3 tools/test_coverage_comparison.py --mode a|b|c|all
"""
import argparse, os, resource, shlex, shutil, signal, subprocess, sys, time
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

# 子进程内存上限 (bytes)，总计≤4GB
_MEM_LIMITS = {
    "navigation": 2.0 * 1024**3,
    "pathcov":    0.5 * 1024**3,
    "evaluator":  0.4 * 1024**3,
    "mapserver":  0.3 * 1024**3,
}

def _preexec_fn(name: str):
    """Best-effort 内存限制。RLIMIT_AS 在子进程 VM 超限时可能失败，静默跳过。"""
    limit = _MEM_LIMITS.get(name)
    if limit:
        try:
            resource.setrlimit(resource.RLIMIT_AS, (limit, limit))
        except Exception:
            pass  # 非关键，子进程已有大 VM 映射时不阻塞启动

def _ros_env():
    """返回附加了日志静默环境变量的 os.environ 拷贝。"""
    env = os.environ.copy()
    env['RCUTILS_LOGGING_SEVERITY_THRESHOLD'] = 'ERROR'
    env['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] {message}'
    return env

def _source_cmd():
    parts = [f"source /opt/ros/humble/setup.sh"]
    ws = WS_ROOT / "install" / "setup.bash"
    if ws.exists(): parts.append(f"source {shlex.quote(str(ws))}")
    return " && ".join(parts)

def _info(m): print(f"\033[36m[INFO]\033[0m {m}")
def _ok(m): print(f"\033[32m[OK]\033[0m {m}")
def _warn(m): print(f"\033[33m[WARN]\033[0m {m}")

def _stop_ros():
    """停止所有 ROS 进程：先归零速度，再 sudo 系统级清理。"""
    try:
        subprocess.run(
            f'{_source_cmd()} && '
            f'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
            f'"{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}"',
            shell=True, executable='/bin/bash',
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=5,
        )
    except Exception:
        pass
    s = Path.home() / ".stop_ros.sh"
    if s.exists():
        subprocess.call(['sudo', 'bash', str(s)],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.5)
    else:
        subprocess.call(['pkill', '-f', 'ros2'],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.5)

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

def _check_ld19():
    try:
        r = subprocess.run(["bash", "-lc", f"{_source_cmd()} && ros2 topic hz /scan --timeout 3 2>&1"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=8)
        return "average rate" in r.stdout.decode(errors="replace").lower()
    except: return False

def _launch_mapserver(prefix):
    gy = os.path.join(MAP_BACKUP_DIR, f"{DEFAULT_MAP}.yaml")
    if not os.path.exists(gy): _warn(f"地图不存在: {gy}"); return
    _info("启动 map_server...")
    subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={shlex.quote(gy)}"],
                     stdout=open(os.path.join(LOG_DIR, f"{prefix}_mapserver.log"), "w"), stderr=subprocess.STDOUT,
                     preexec_fn=lambda: _preexec_fn("mapserver"), env=_ros_env())
    time.sleep(3.0)
    subprocess.run(f"{_source_cmd()} && ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate", shell=True, executable="/bin/bash", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=20)
    _ok("map_server 已激活")

def _cleanup(*ps):
    for p in ps:
        if p and p.poll() is None: p.terminate()
        try:
            if p: p.wait(timeout=3.0)
        except: pass

def _run_common(label, nav_cmd, pathcov_cmd, use_mapserver, costmap_wait):
    _stop_ros(); os.makedirs(LOG_DIR, exist_ok=True)
    nav = subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && {nav_cmd}"],
                           stdout=open(os.path.join(LOG_DIR, f"{PREFIX[label]}_nav.log"), "w"), stderr=subprocess.STDOUT,
                           preexec_fn=lambda: _preexec_fn("navigation"), env=_ros_env())
    time.sleep(10.0)
    if use_mapserver: _launch_mapserver(PREFIX[label])
    _info(f"等待 costmap 稳定 ({costmap_wait}s)..."); time.sleep(costmap_wait)
    _restore_rtabmap_db();_info(f"rtabmap_db已恢复")
    pc = subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && {pathcov_cmd}"],
                           stdout=open(os.path.join(LOG_DIR, f"{PREFIX[label]}_pathcoverage.log"), "w"), stderr=subprocess.STDOUT,
                           preexec_fn=lambda: _preexec_fn("pathcov"), env=_ros_env())
    ev = subprocess.Popen(["bash", "-lc", f"{_source_cmd()} && ros2 launch coverage_evaluator coverage_evaluator.launch.py"],
                           stdout=open(os.path.join(LOG_DIR, f"{PREFIX[label]}_evaluator.log"), "w"), stderr=subprocess.STDOUT,
                           preexec_fn=lambda: _preexec_fn("evaluator"), env=_ros_env())
    _info("等待节点就绪 (5s)..."); time.sleep(5.0)
    # Quick LD19 check for Group A (after nav started)
    if label == "a" and not _check_ld19():
        _warn("LD19 未检测到 /scan，覆盖可能失败！继续？"); input()
    pub = str(LAUNCHER_DIR / "publish_region.py")
    rc = subprocess.run(["python3", pub, "--file", REGION_FILE, "--wait", "5"], timeout=30)
    if rc.returncode != 0: _warn(f"区域发布失败"); _cleanup(pc, ev, nav); return
    _ok(f"{GROUP_LABEL[label]} 就绪"); print()
    try: nav.wait()
    except KeyboardInterrupt: pass
    finally: _cleanup(pc, ev, nav)
    _ok(f"{GROUP_LABEL[label]} 完成")

def run_group_a():
    print("\n\033[1;34m=== Group A: LiDAR + 原始算法 ===\033[0m\n")
    if not _ensure_test_map(): return
    _run_common("a", f"ros2 launch navigation navigation.launch.py map:={DEFAULT_MAP}",  "ros2 launch path_coverage path_coverage_baseline.launch.py", False, 5)

def run_group_b():
    print("\n\033[1;33m=== Group B: RTAB-Map + 原始算法 (消融) ===\033[0m\n")
    _run_common("b", "ros2 launch navigation rtabmap_navigation.launch.py localization:=true",  "ros2 launch path_coverage path_coverage_baseline.launch.py", True, 5)

def run_group_c():
    print("\n\033[1;32m=== Group C: RTAB-Map + 改进算法 (创新) ===\033[0m\n")
    _run_common("c", "ros2 launch navigation rtabmap_navigation.launch.py localization:=true",  "ros2 launch path_coverage path_coverage.launch.py", True, 5)

def main():
    p = argparse.ArgumentParser(description="三组消融对照实验")
    p.add_argument("--mode", choices=["a","b","c","all"], default="all")
    args = p.parse_args()
    signal.signal(signal.SIGINT, lambda s,f: sys.exit(130))
    groups = ["a","b","c"] if args.mode == "all" else [args.mode]
    for g in groups:
        {"a": run_group_a, "b": run_group_b, "c": run_group_c}[g]()
        if len(groups) > 1 and g != groups[-1]:
            print("\n" + "="*50 + "\n  请将车推回起点后按 Enter...\n" + "="*50)
            try: input()
            except KeyboardInterrupt: break

if __name__ == "__main__": main()
