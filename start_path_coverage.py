#!/usr/bin/env python3
import argparse
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path


class Launcher:
    LOG_KEEP_PATTERN = re.compile(
        r'(warn|warning|error|fatal|exception|traceback)', re.IGNORECASE
    )

    def __init__(self, quiet: bool = False) -> None:
        self.quiet = quiet
        self.script_dir = Path(__file__).resolve().parent
        self.ws_root = self.script_dir.parent
        self.children: list[subprocess.Popen] = []

    def _info(self, message: str) -> None:
        if not self.quiet:
            print(f'[INFO] {message}')

    def _warn(self, message: str) -> None:
        print(f'[WARN] {message}')

    def _stream_pipe(self, pipe, stream_name: str) -> None:
        for line in iter(pipe.readline, ''):
            clean_line = line.rstrip()
            if not clean_line:
                continue
            if self.quiet and not self.LOG_KEEP_PATTERN.search(clean_line):
                continue
            print(f'[{stream_name}] {clean_line}')
        pipe.close()

    def _source_prefix(self) -> str:
        ros_setup = Path('/opt/ros/humble/setup.bash')
        if not ros_setup.exists():
            print('[ERROR] /opt/ros/humble/setup.bash not found')
            sys.exit(1)

        parts = [f'source {shlex.quote(str(ros_setup))}']
        ws_setup = self.ws_root / 'install' / 'setup.bash'
        if ws_setup.exists():
            parts.append(f'source {shlex.quote(str(ws_setup))}')
        else:
            self._warn(f'{ws_setup} not found, trying to continue')
        return ' && '.join(parts)

    def _launch_bg(self, command: str, name: str) -> subprocess.Popen:
        full_cmd = f'{self._source_prefix()} && {command}'
        self._info(f'Start {name}: {command}')
        proc = subprocess.Popen(
            ['bash', '-lc', full_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.children.append(proc)
        threading.Thread(
            target=self._stream_pipe,
            args=(proc.stdout, f'{name}:stdout'),
            daemon=True,
        ).start()
        threading.Thread(
            target=self._stream_pipe,
            args=(proc.stderr, f'{name}:stderr'),
            daemon=True,
        ).start()
        return proc

    def _run_fg(self, command: str, name: str) -> int:
        full_cmd = f'{self._source_prefix()} && {command}'
        self._info(f'Start {name} (foreground): {command}')
        self._info('Press Ctrl+C to stop all child processes')
        proc = subprocess.Popen(
            ['bash', '-lc', full_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        threading.Thread(
            target=self._stream_pipe,
            args=(proc.stdout, f'{name}:stdout'),
            daemon=True,
        ).start()
        threading.Thread(
            target=self._stream_pipe,
            args=(proc.stderr, f'{name}:stderr'),
            daemon=True,
        ).start()
        return proc.wait()

    def _run_stop_script(self) -> None:
        stop_script = Path.home() / '.stop_ros.sh'
        if not stop_script.exists():
            self._warn(f'{stop_script} not found, skipping')
            return

        self._info(f'Running {stop_script} (non-fatal if it fails)')
        rc = subprocess.call(['bash', str(stop_script)])
        if rc != 0:
            self._warn(f'~/.stop_ros.sh failed with code {rc}, continuing')

    def _cleanup_rtabmap_db(self) -> None:
        db_path = Path('/home/ubuntu/.ros/rtabmap.db')
        if db_path.exists():
            db_path.unlink()
            self._info(f'Removed {db_path}')
        else:
            self._info(f'{db_path} not found, skip removal')

    def cleanup(self) -> None:
        self._info('Stopping child processes...')
        for proc in reversed(self.children):
            if proc.poll() is None:
                proc.terminate()
        end_time = time.time() + 5.0
        for proc in reversed(self.children):
            if proc.poll() is None:
                timeout = max(0.0, end_time - time.time())
                try:
                    proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    proc.kill()

    def run(self) -> int:
        self._info(f'Workspace: {self.ws_root}')
        self._info('Launch mode: RTAB-Map navigation + RViz')

        self._run_stop_script()
        self._cleanup_rtabmap_db()

        self._launch_bg(
            'ros2 launch navigation rtabmap_navigation.launch.py localization:=false',
            'RTAB-Map navigation',
        )
        time.sleep(4)

        return self._run_fg(
            'ros2 launch navigation rviz_rtabmap_navigation.launch.py',
            'RViz (RTAB-Map navigation)',
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='One-click launcher for RTAB-Map navigation + RViz.'
    )
    parser.add_argument(
        'legacy_map_name',
        nargs='?',
        default=None,
        help='Legacy argument kept for compatibility; ignored in current workflow.',
    )
    parser.add_argument(
        '--with-stop',
        action='store_true',
        help='Legacy flag kept for compatibility; stop script now runs by default.',
    )
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Simplify output: only show warnings and errors from launched processes.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.legacy_map_name:
        print(
            f'[WARN] Legacy map argument "{args.legacy_map_name}" is ignored '
            'in current workflow.'
        )
    if args.with_stop and not args.quiet:
        print('[INFO] --with-stop is now default behavior; continuing')

    launcher = Launcher(quiet=args.quiet)

    def on_signal(signum, _frame):
        print(f'\n[INFO] Received signal {signum}, shutting down...')
        launcher.cleanup()
        raise SystemExit(130)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    exit_code = 1
    try:
        exit_code = launcher.run()
    finally:
        launcher.cleanup()

    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
