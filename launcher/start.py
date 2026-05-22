#!/usr/bin/env python3
"""
start.py — 割草机器人系统一键启动器（交互式）。

用法:
    python3 launcher/start.py           # 默认安静模式
    python3 launcher/start.py --debug   # 调试模式

交互命令:
    mapping             — 启动 SLAM 建图模式 (navigation + RViz)
    live    [区域]       — 建图模式下直接开始覆盖（不切换定位，保留 mapping）
    coverage [地图] [区域] — 纯定位覆盖（适用 localization 稳定时）
    region  <名称>       — 在 RViz 中 Publish Point 圈多边形并保存
    save    [名称]       — 保存当前地图到 ~/.ros/maps/
    load    <名称>       — 从备份恢复地图
    list                 — 列出所有地图备份 + 覆盖区域
    log     [进程名]      — 查看进程日志
    status               — 查看各进程运行状态
    stop                 — 停止所有子进程
    quit                 — 退出
"""

import argparse
import os
import re
import resource
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

MAP_BACKUP_DIR = str(Path.home() / ".ros" / "maps")
REGION_DIR = str(Path.home() / ".ros" / "regions")
RTABMAP_DB = str(Path.home() / ".ros" / "rtabmap.db")
LOG_DIR = "/home/ubuntu/ros2_ws/src/logs/start_logs"

# 各子进程内存上限 (bytes)，总计≤4GB（总8GB，待机占用3.5GB）
MEMORY_LIMITS = {
    "navigation":   2.0 * 1024**3,   # RTAB-Map + Nav2（内存大户）
    "rviz":         0.6 * 1024**3,   # RViz 可视化
    "path_coverage":0.5 * 1024**3,   # Python 覆盖节点
    "evaluator":    0.4 * 1024**3,   # Python 评估节点
    "map_server":   0.3 * 1024**3,   # 栅格地图服务
}

# ═══════════════════════════════════════════════════════════════════════════
# Process manager
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class ManagedProcess:
    name: str
    proc: subprocess.Popen
    command: str


class Launcher:
    """交互式系统启动器。

    管理多个后台 ROS2 进程，支持 mapping/coverage 模式切换。
    """

    LOG_KEEP_PATTERN = re.compile(
        r'(warn|warning|error|fatal|exception|traceback)', re.IGNORECASE
    )

    def __init__(self, debug: bool = False) -> None:
        self.debug = debug
        self.script_dir = Path(__file__).resolve().parent
        self.ws_root = self.script_dir.parent
        self._procs: Dict[str, ManagedProcess] = {}
        self._running = True
        self._current_mode: str = "none"
        self._first_time = True
        self._non_interactive = False  # set by --non-interactive, skip sudo stop

    # ── 输出 ──────────────────────────────────────────────────────────

    def _info(self, message: str) -> None:
        print(f"\033[36m[INFO]\033[0m {message}")

    def _warn(self, message: str) -> None:
        print(f"\033[33m[WARN]\033[0m {message}")

    def _ok(self, message: str) -> None:
        print(f"\033[32m[OK]\033[0m {message}")

    def _show_output(self, pipe, name: str) -> None:
        last_print = {}  # {name: last_timestamp}
        for line in iter(pipe.readline, ''):
            clean = line.rstrip()
            if not clean:
                continue
            if not self.debug:
                if not self.LOG_KEEP_PATTERN.search(clean):
                    continue
                # 安静模式节流: 同一来源每秒最多打印 1 行
                now = time.monotonic()
                prev = last_print.get(name, 0)
                if now - prev < 1.0:
                    continue
                last_print[name] = now
            print(f"[{name}] {clean}")
        pipe.close()

    # ── 进程管理 ──────────────────────────────────────────────────────

    def _source_cmd(self) -> str:
        ros_setup = Path('/opt/ros/humble/local_setup.sh')
        if not ros_setup.exists():
            ros_setup = Path('/opt/ros/humble/setup.sh')
        parts = [f'source {shlex.quote(str(ros_setup))}']
        ws_setup = self.ws_root / 'install' / 'setup.bash'
        if ws_setup.exists():
            parts.append(f'source {shlex.quote(str(ws_setup))}')
        # workspace setup 不存在时不报错 — 源码开发模式无需编译
        return ' && '.join(parts)

    def _spawn(self, name: str, command: str) -> subprocess.Popen:
        full_cmd = f'{self._source_cmd()} && {command}'
        self._info(f'启动 {name}' if not self.debug else f'启动 {name}: {command}')

        # 内存限制（best-effort，子进程 VM 超限时静默跳过）
        mem_limit = MEMORY_LIMITS.get(name)
        def _preexec():
            if mem_limit:
                try:
                    resource.setrlimit(resource.RLIMIT_AS, (mem_limit, mem_limit))
                except Exception:
                    pass

        # 日志静默：只保留 ERROR/FATAL，降低磁盘 IO
        env = os.environ.copy()
        env['RCUTILS_LOGGING_SEVERITY_THRESHOLD'] = 'ERROR'
        env['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] {message}'

        if self.debug:
            proc = subprocess.Popen(
                ['bash', '-lc', full_cmd],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, bufsize=1,
                preexec_fn=_preexec, env=env,
            )
            threading.Thread(target=self._show_output, args=(proc.stdout, name),
                             daemon=True).start()
            threading.Thread(target=self._show_output, args=(proc.stderr, name),
                             daemon=True).start()
        else:
            os.makedirs(LOG_DIR, exist_ok=True)
            log_path = os.path.join(LOG_DIR, f"{name}.log")
            fh = open(log_path, 'w')
            proc = subprocess.Popen(
                ['bash', '-lc', full_cmd],
                stdout=fh, stderr=fh, text=True,
                preexec_fn=_preexec, env=env,
            )
            threading.Thread(target=self._check_startup,
                             args=(name, proc), daemon=True).start()

        self._procs[name] = ManagedProcess(name=name, proc=proc, command=command)
        return proc

    def _check_startup(self, name: str, proc: subprocess.Popen) -> None:
        time.sleep(2.0)
        rc = proc.poll()
        if rc is not None:
            if rc == -11:
                self._warn(f'{name} 内存超限被系统终止 (SIGSEGV)，退出码={rc}')
            else:
                self._warn(f'{name} 启动后立即退出 (exit={rc})，查看: log {name}')

    def _kill(self, name: str) -> None:
        mp = self._procs.pop(name, None)
        if mp is None:
            return
        if mp.proc.poll() is None:
            mp.proc.terminate()
            try:
                mp.proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                mp.proc.kill()
                try:
                    mp.proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    pass
            self._info(f'已停止 {name}')
        else:
            self._info(f'{name} 已退出')

    def _kill_all(self) -> None:
        for name in list(self._procs.keys()):
            self._kill(name)
        self._run_stop_ros()
        self._current_mode = "none"

    def _is_running(self, name: str) -> bool:
        mp = self._procs.get(name)
        return mp is not None and mp.proc.poll() is None

    # ── 阶段操作 ──────────────────────────────────────────────────────

    def _run_stop_ros(self) -> None:
        """停止所有 ROS 进程：先归零速度，再执行系统级清理。"""
        # 1. 速度归零，防止机器人继续运动
        try:
            subprocess.run(
                f'{self._source_cmd()} && '
                f'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
                f'"{{linear: {{x: 0.0, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}"',
                shell=True, executable='/bin/bash',
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=5,
            )
        except Exception:
            pass  # 速度归零失败不阻塞

        # 2. sudo 执行系统级 stop 脚本（需要无密码 sudo 权限）
        #    非交互模式下跳过 — 调用方（如 test 脚本）负责系统级清理
        if self._non_interactive:
            self._info('非交互模式，跳过 sudo stop_ros（由调用方负责清理）')
            return
        stop_script = Path.home() / '.stop_ros.sh'
        if stop_script.exists():
            self._info('执行 sudo ~/.stop_ros.sh ...')
            subprocess.call(['sudo', 'bash', str(stop_script)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.5)
        else:
            # 兜底：直接 killall ros2 进程
            self._info('~/.stop_ros.sh 不存在，执行 killall 兜底...')
            subprocess.call(['pkill', '-f', 'ros2'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.5)

    def _cleanup_db(self) -> None:
        db = Path('/home/ubuntu/.ros/rtabmap.db')
        if db.exists():
            db.unlink()
            self._info(f'已删除旧地图 {db}')

    def start_mapping(self) -> None:
        """启动 mapping 模式：SLAM 建图 + RViz。"""
        if self._current_mode == "mapping":
            self._warn("已在 mapping 模式")
            return

        for name in list(self._procs.keys()):
            if name != "rviz":
                self._kill(name)

        if self._current_mode == "none":
            self._run_stop_ros()
            self._cleanup_db()

        self._info("=== 启动 SLAM 建图模式 (localization:=false) ===")
        self._spawn("navigation",
                    "ros2 launch navigation rtabmap_navigation.launch.py localization:=false")
        time.sleep(5.0)
        self._ensure_rviz()
        self._current_mode = "mapping"
        self._ok("建图模式就绪 — 在 RViz 中用 Publish Point 圈选区域开始建图")

    def start_coverage(self, map_name: str = "", region_name: str = "") -> None:
        """纯定位覆盖: localization:=true。

        RViz 先启动确保订阅者就绪 → 10s 后启动 navigation → 10s 后触发 RTAB-Map 发布 grid_map。
        """
        if self._current_mode == "coverage":
            self._warn("已在 coverage 模式，请先 stop")
            return

        self._save_and_prepare(map_name)
        region_file = self._find_region(region_name)
        if not region_file:
            self._warn("未找到覆盖区域文件")
            return
        self._info(f"地图: {map_name or '(当前)'}, 区域: {os.path.basename(region_file)}")

        # RViz 先启动，确保静态地图订阅者就绪（避免错过 RTAB-Map 一次性发布）
        # 非交互模式（测试脚本调用）跳过 RViz，无需等待
        if not self._non_interactive:
            self._ensure_rviz()
            self._info("等待 RViz 就绪 (10s)...")
            time.sleep(10.0)

        # 启动 navigation（RTAB-Map localization + Nav2）
        self._info("=== 纯定位覆盖 (localization:=true) ===")
        self._spawn("navigation",
                    "ros2 launch navigation rtabmap_navigation.launch.py localization:=true")
        self._info("等待 navigation 初始化 (10s)...")
        time.sleep(10.0)

        # 触发 RTAB-Map 从 .db 发布完整 grid_map
        self._publish_rtabmap_map()

        self._current_mode = "coverage"
        self._launch_coverage_tools(region_file)

    def _publish_rtabmap_map(self) -> None:
        """调用 /rtabmap/publish_map 服务，从 .db 发布完整全局 grid_map。"""
        self._info("触发 RTAB-Map 发布 grid_map...")
        subprocess.run(
            f"{self._source_cmd()} && "
            f"ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap "
            f'"{{global_map: true, optimized: true, graph_only: false}}"',
            shell=True, executable='/bin/bash',
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=10,
        )

    def start_live(self, region_name: str = "") -> None:
        """建图模式直接覆盖: localization:=false，边建图边覆盖。

        适用场景: localization 不稳定时的保底方案。
        需要先执行 `mapping` 建好基础地图。
        """
        if self._current_mode == "coverage":
            self._warn("已在 coverage 模式，请先 stop")
            return
        if self._current_mode != "mapping":
            self._warn("请先执行 mapping 建图，再执行 live")
            return

        self._save_map_auto()
        region_file = self._find_region(region_name)
        if not region_file:
            self._warn("未找到覆盖区域文件")
            return
        self._info(f"区域: {os.path.basename(region_file)}, 模式: mapping (边建图边覆盖)")

        # 等待地图积累（匹配用户手动流程: 建图后等 10s）
        self._info("等待地图积累 10s...")
        for i in range(6):
            time.sleep(5.0)
            self._info(f"  ... {25 - i*5}s")

        self._current_mode = "mapping"  # 保持 mapping 模式
        self._launch_coverage_tools(region_file)

    def _save_and_prepare(self, map_name: str) -> None:
        """保存当前地图 + 清理 navigation 进程（保留 rviz）。"""
        if self._current_mode == "mapping":
            self._save_map_auto()
        self._kill_all()
        if map_name:
            self.restore_map(map_name)
        elif not os.path.exists(RTABMAP_DB):
            backups = self._list_backups()
            if backups:
                self._warn(f"使用最新备份: {backups[0]}")
                self.restore_map(backups[0])
            else:
                self._warn("无可用地图，请先 mapping 建图")

    def _launch_coverage_tools(self, region_file: str) -> None:
        """启动 path_coverage + evaluator → 等待节点就绪 → 发布区域。"""
        self._spawn("path_coverage",
                    "ros2 launch path_coverage path_coverage.launch.py")
        self._spawn("evaluator",
                    "ros2 launch coverage_evaluator coverage_evaluator.launch.py")

        # 等节点启动 + DDS 发现 + 订阅建立
        self._info("等待节点就绪 (10s)...")
        time.sleep(10.0)

        # 验证 path_coverage 仍在运行（未被 OOM 杀掉或启动崩溃）
        if not self._is_running("path_coverage"):
            self._warn("path_coverage 未运行！检查日志: log path_coverage")
            return
        self._info("path_coverage 运行中 ✓")

        self._info("发布覆盖区域...")
        pub_script = str(self.script_dir / "publish_region.py")
        rc = subprocess.run(
            ["python3", pub_script, "--file", region_file, "--wait", "8"],
            timeout=30,
        )
        if rc.returncode != 0:
            self._warn(f"区域发布失败 (exit={rc.returncode})")
            return
        self._ok("覆盖模式就绪 — 区域已发布，开始覆盖作业")

    # ── 区域管理 ──────────────────────────────────────────────────────

    def start_region_mode(self, name: str = "") -> None:
        """进入区域定义模式：用户在 RViz 中用 Publish Point 圈多边形。"""
        if not name:
            name = time.strftime("region_%Y%m%d_%H%M%S")
        self._ensure_rviz()
        self._info(f"=== 区域定义模式: {name} ===")
        print()
        print("  \033[1m在 RViz 中:\033[0m")
        print("  1. 点击顶部工具栏 \033[1mPublish Point\033[0m")
        print("  2. 在地图上 \033[1m按序点击 ≥3 个顶点\033[0m 圈出覆盖区域")
        print("  3. 最后一个点 \033[1m靠近第一个点\033[0m 即自动闭合")
        print()
        cap_script = str(self.script_dir / "region_capture.py")
        rc = subprocess.call(
            ["python3", cap_script, "--name", name, "--output", REGION_DIR],
        )
        if rc == 0:
            self._ok(f"区域已保存: ~/.ros/regions/{name}.yaml")
        else:
            self._warn(f"区域定义未完成 (exit={rc})")

    def _find_region(self, region_name: str = "") -> str:
        """查找区域 YAML 文件路径。空名称=返回最新。
        搜索顺序: ~/.ros/regions/ → launcher/regions/ (内置)"""
        os.makedirs(REGION_DIR, exist_ok=True)
        builtin_dir = str(self.script_dir / "regions")

        search_dirs = [REGION_DIR, builtin_dir]
        if region_name:
            fname = region_name if region_name.endswith('.yaml') else f"{region_name}.yaml"
            for d in search_dirs:
                path = os.path.join(d, fname)
                if os.path.exists(path):
                    return path
            return ""
        # 无名称=返回用户目录最新，否则内置
        for d in search_dirs:
            if os.path.isdir(d):
                files = sorted([f for f in os.listdir(d) if f.endswith('.yaml')],
                               reverse=True)
                if files:
                    return os.path.join(d, files[0])
        return ""

    def list_regions(self) -> None:
        """列出所有区域文件（用户 + 内置）。"""
        builtin_dir = str(self.script_dir / "regions")
        print()
        for label, d in [("用户 (~/.ros/regions/)", REGION_DIR),
                          ("内置 (launcher/regions/)", builtin_dir)]:
            os.makedirs(d, exist_ok=True)
            files = sorted([f for f in os.listdir(d) if f.endswith('.yaml')],
                           reverse=True) if os.path.isdir(d) else []
            if files:
                print(f"  {label}:")
                for f in files:
                    path = os.path.join(d, f)
                    try:
                        import yaml
                        with open(path) as fh:
                            data = yaml.safe_load(fh)
                        nv = len(data.get("vertices", []))
                        print(f"    {f}  ({nv} 顶点)")
                    except Exception:
                        print(f"    {f}")
        print()

    # ── 地图管理 ──────────────────────────────────────────────────────

    def _save_map_auto(self) -> None:
        """从 mapping 切换时自动保存 rtabmap.db + grid map。"""
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._do_save_map(f"auto_{ts}")

    def save_map(self, name: str = "") -> None:
        """保存 rtabmap.db + grid map 到 ~/.ros/maps/<name>。"""
        if not name:
            name = time.strftime("save_%Y%m%d_%H%M%S")
        self._do_save_map(name)

    def _do_save_map(self, name: str) -> None:
        """保存 rtabmap.db + 栅格地图 (pgm+yaml)。"""
        os.makedirs(MAP_BACKUP_DIR, exist_ok=True)

        # 1. 保存 rtabmap.db
        if os.path.exists(RTABMAP_DB):
            db_dst = os.path.join(MAP_BACKUP_DIR, f"{name}.db")
            shutil.copy2(RTABMAP_DB, db_dst)
            self._ok(f"rtabmap.db → ~/.ros/maps/{name}.db "
                     f"({os.path.getsize(db_dst)/1024/1024:.1f} MB)")
        else:
            self._warn(f"{RTABMAP_DB} 不存在，跳过数据库保存")

        # 2. 保存栅格地图
        grid_path = os.path.join(MAP_BACKUP_DIR, name)
        self._info("保存栅格地图 (调用 map_saver_cli)...")
        try:
            subprocess.run(
                f"{self._source_cmd()} && "
                f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(grid_path)}",
                shell=True, executable='/bin/bash',
                stdout=subprocess.DEVNULL if not self.debug else None,
                stderr=subprocess.DEVNULL if not self.debug else None,
                timeout=30,
            )
            rc = 0
        except subprocess.TimeoutExpired:
            rc = 1
        except Exception:
            rc = 1
        yaml_file = f"{grid_path}.yaml"
        if rc == 0 and os.path.exists(yaml_file):
            self._ok(f"栅格地图 → {yaml_file}")
        else:
            self._warn(f"栅格地图保存失败 (exit={rc})，"
                       f"可能 /map topic 暂无数据。"
                       f" 请确保 RTAB-Map 已积累足够地图数据后再 save。")

    def restore_map(self, name: str) -> None:
        """从备份恢复 rtabmap.db。"""
        fname = name if name.endswith('.db') else f"{name}.db"
        src = os.path.join(MAP_BACKUP_DIR, fname)
        if not os.path.exists(src):
            self._warn(f"备份不存在: {src}")
            return
        shutil.copy2(src, RTABMAP_DB)
        self._ok(f"地图已恢复: {name}")

    def _list_backups(self) -> list:
        if not os.path.isdir(MAP_BACKUP_DIR):
            return []
        return sorted([f for f in os.listdir(MAP_BACKUP_DIR)
                       if f.endswith('.db')], reverse=True)

    def list_maps(self) -> None:
        """列出所有地图备份（含栅格地图状态）。"""
        backups = self._list_backups()
        if not backups:
            print("  无备份")
            return
        print(f"\n 地图备份 (~/.ros/maps/):") 
        for b in backups:
            name = b.replace('.db', '')
            sz = os.path.getsize(os.path.join(MAP_BACKUP_DIR, b)) / 1024 / 1024
            yaml_file = os.path.join(MAP_BACKUP_DIR, f"{name}.yaml")
            grid_status = "\033[32m✓ grid\033[0m" if os.path.exists(yaml_file) else "\033[33m✗ no grid\033[0m"
            print(f"    {b}  ({sz:.1f} MB)  {grid_status}")
        print()

    def _ensure_rviz(self) -> None:
        """确保 RViz 在运行（如果已运行则跳过）。非交互模式跳过。"""
        if self._non_interactive:
            return
        if self._is_running("rviz"):
            self._info("RViz 已在运行，跳过")
            return
        self._spawn("rviz",
                    "ros2 launch navigation rviz_rtabmap_navigation.launch.py")
        time.sleep(2.0)

    # ── 状态 ──────────────────────────────────────────────────────────

    def print_status(self) -> None:
        print(f"\n当前模式: \033[1m{self._current_mode}\033[0m")
        print(f"{'进程':<20} {'状态':<10}")
        print("-" * 30)
        for name, mp in sorted(self._procs.items()):
            running = mp.proc.poll() is None
            status = "\033[32m运行中\033[0m" if running else f"\033[31m已退出({mp.proc.returncode})\033[0m"
            print(f"{name:<20} {status}")

    def show_log(self, name: str = "") -> None:
        """查看子进程日志。无参数列出所有日志文件，有参数 tail 指定进程的最后 20 行。"""
        if not os.path.isdir(LOG_DIR):
            print("  无日志")
            return
        if not name:
            files = sorted(os.listdir(LOG_DIR))
            if not files:
                print("  无日志文件")
                return
            print(f"\n 日志文件 ({LOG_DIR}/):")
            for f in files:
                path = os.path.join(LOG_DIR, f)
                sz = os.path.getsize(path)
                print(f"    {f}  ({sz} bytes)")
            print("  用法: log <进程名>  查看最后 20 行")
            return

        log_path = os.path.join(LOG_DIR, f"{name}.log")
        if not os.path.exists(log_path):
            self._warn(f"日志不存在: {log_path}")
            return
        # 打印最后 20 行
        with open(log_path) as fh:
            lines = fh.readlines()
        recent = lines[-20:] if len(lines) > 20 else lines
        print(f"\n --- {name}.log (最后 {len(recent)}/{len(lines)} 行) ---")
        for line in recent:
            print(f"  {line.rstrip()}")
        print()

    # ── 交互循环 ──────────────────────────────────────────────────────

    def run(self) -> None:
        print("\033[1;35m╔══════════════════════════════════════╗\033[0m")
        print("\033[1;35m║   割草机器人 系统控制台             ║\033[0m")
        print("\033[1;35m╚══════════════════════════════════════╝\033[0m")
        self._info(f"工作空间: {self.ws_root}")
        self._info("提示: 首次使用请先执行 mapping，建图完成后执行 coverage")
        print()

        try:
            self._interactive_loop()
        except KeyboardInterrupt:
            print()
            self._info("收到中断信号")
        finally:
            self._kill_all()
            self._info("已退出，所有服务已停止")

    def _interactive_loop(self) -> None:
        while self._running:
            try:
                cmd = input("\033[1m>\033[0m ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not cmd:
                continue

            if cmd in ("q", "quit", "exit"):
                break
            elif cmd == "mapping":
                self._first_time = False
                self.start_mapping()
            elif cmd.startswith("live"):
                parts = cmd.split(maxsplit=1)
                region = parts[1].strip() if len(parts) > 1 else ""
                self.start_live(region)
            elif cmd.startswith("region"):
                parts = cmd.split(maxsplit=1)
                name = parts[1].strip() if len(parts) > 1 else ""
                self.start_region_mode(name)
            elif cmd.startswith("coverage"):
                parts = cmd.split()
                map_name = parts[1] if len(parts) > 1 else ""
                region_name = parts[2] if len(parts) > 2 else ""
                self._first_time = False
                self.start_coverage(map_name, region_name)
            elif cmd.startswith("save"):
                name = cmd[5:].strip() if len(cmd) > 4 else ""
                self.save_map(name)
            elif cmd.startswith("load "):
                self.restore_map(cmd[5:].strip())
            elif cmd.startswith("log"):
                parts = cmd.split(maxsplit=1)
                name = parts[1].strip() if len(parts) > 1 else ""
                self.show_log(name)
            elif cmd in ("list", "ls"):
                self.list_maps()
                self.list_regions()
            elif cmd in ("s", "status"):
                self.print_status()
            elif cmd == "stop":
                self._kill_all()
                self._info("所有进程已停止")
            elif cmd in ("h", "help", "?"):
                self._print_help()
            else:
                self._warn(f"未知命令: {cmd}（输入 help 查看可用命令）")

    def _print_help(self) -> None:
        print("""
\033[1m可用命令:\033[0m
  mapping               — 启动 SLAM 建图 + RViz
  live    [区域]         — ⭐ 建图模式下直接覆盖 (不切换定位，最稳定)
  coverage [地图] [区域] — 纯定位覆盖 (适用 localization 稳定时)
  region  <名称>         — 在 RViz 中圈选覆盖区域并保存
  save    [名称]         — 保存当前地图
  load    <名称>         — 恢复地图备份
  list                  — 列出地图 + 区域
  log     [进程名]       — 查看进程日志
  status                — 查看进程状态
  stop                  — 停止所有子进程
  quit                  — 退出

\033[1m推荐流程 (最稳定):\033[0m
  > mapping              ← 建图，手动控车遍历区域
  > save test_map        ← 保存地图
  > live test_180x240    ← 直接覆盖 (mapping 模式，不等定位)

\033[1m定位稳定时:\033[0m
  > coverage test_map test_180x240  ← 纯定位覆盖 (更精确)
""")


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='割草机器人系统一键启动器（交互式）')
    parser.add_argument('--debug', action='store_true',
                        help='调试模式，输出全部 ROS2 日志（默认仅警告/错误）')
    parser.add_argument('--legacy', nargs='?', default=None,
                        help=argparse.SUPPRESS)  # 兼容旧参数，无实际作用
    parser.add_argument('--non-interactive', action='store_true',
                        help='非交互模式：直接执行指定命令后退出')
    parser.add_argument('command', nargs='*', default=[],
                        help='非交互模式下的命令 (如: coverage test_map test_180x240)')
    return parser.parse_args()


def _non_interactive_run(launcher: Launcher, args: argparse.Namespace) -> None:
    """Execute a single command non-interactively and exit."""
    if not args.command:
        print("非交互模式需要指定命令，如: python3 launcher/start.py --non-interactive coverage test_map test_180x240")
        sys.exit(1)

    # Mark non-interactive so _run_stop_ros skips sudo
    launcher._non_interactive = True

    cmd = args.command[0].lower()
    cmd_args = args.command[1:] if len(args.command) > 1 else []

    if cmd == "mapping":
        launcher.start_mapping()
    elif cmd == "coverage":
        map_name = cmd_args[0] if len(cmd_args) > 0 else ""
        region_name = cmd_args[1] if len(cmd_args) > 1 else ""
        launcher.start_coverage(map_name, region_name)
    elif cmd == "live":
        region_name = cmd_args[0] if len(cmd_args) > 0 else ""
        launcher.start_live(region_name)
    else:
        print(f"非交互模式不支持命令: {cmd}")
        print("支持: mapping, coverage, live")
        sys.exit(1)

    # Wait for path_coverage to finish (or user Ctrl+C)
    launcher._info("非交互模式：等待覆盖任务完成...")
    try:
        while launcher._is_running("path_coverage"):
            time.sleep(2.0)
    except KeyboardInterrupt:
        launcher._info("收到中断信号")
    finally:
        launcher._kill_all()
        launcher._info("非交互模式结束")


def main() -> None:
    args = parse_args()
    launcher = Launcher(debug=args.debug)

    def on_signal(sig, _frame):
        print(f'\n\033[33m[INFO]\033[0m 收到信号 {sig}，正在退出...')
        launcher._kill_all()
        sys.exit(130)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    if args.non_interactive:
        _non_interactive_run(launcher, args)
    else:
        launcher.run()


if __name__ == '__main__':
    main()
