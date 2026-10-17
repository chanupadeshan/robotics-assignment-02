#!/usr/bin/python3
"""Stop this workspace's maze launches and leftover corrected-world Gazebo server."""
import argparse
import os
from pathlib import Path
import signal
import time

WORKSPACE = Path(__file__).resolve().parents[1]
ROS_LOG_DIR = str(WORKSPACE / 'log/ros')
WORLD_PATHS = {
    str(WORKSPACE / 'src/maze_world/worlds/maze_fixed.sdf'),
    str(WORKSPACE / 'install/maze_world/share/maze_world/worlds/maze_fixed.sdf'),
}


def processes():
    """Snapshot processes owned by the current user, tolerating concurrent exits."""
    result = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            args = entry.joinpath('cmdline').read_bytes().split(b'\0')
            args = [arg.decode(errors='replace') for arg in args if arg]
            stat = entry.joinpath('stat').read_text().rsplit(')', 1)[1].split()
            environ = entry.joinpath('environ').read_bytes().split(b'\0')
            environment = dict(item.split(b'=', 1) for item in environ if b'=' in item)
            result[int(entry.name)] = {
                'args': args, 'parent': int(stat[1]), 'start': stat[19], 'state': stat[0],
                'environment': environment,
            }
        except (OSError, ValueError, IndexError):
            continue
    return result


def project_processes(snapshot):
    """Select exact maze world paths and project launches with a matching environment."""
    result = {}
    for pid, info in snapshot.items():
        args = info['args']
        if not args:
            continue
        is_world = args[:2] == ['gz', 'sim'] and any(path in args for path in WORLD_PATHS)
        project_environment = info['environment'].get(b'ROS_LOG_DIR') == ROS_LOG_DIR.encode()
        launch_specs = [
            ['launch', 'maze_world', 'simulation.launch.py'],
            ['launch', 'maze_navigation', 'nav2.launch.py'],
            ['launch', 'maze_navigation', 'slam.launch.py'],
        ]
        is_launch = any(Path(arg).name == 'ros2' and args[i+1:i+4] in launch_specs
                        for i, arg in enumerate(args))
        if is_world or (project_environment and is_launch):
            result[pid] = info
    # Include GUI/wrappers while their confirmed launch parent still exists.
    changed = True
    while changed:
        changed = False
        for pid, info in snapshot.items():
            if info['parent'] in result and pid not in result:
                result[pid] = info
                changed = True
    return result


def still_same(pid, info):
    current = processes().get(pid)
    return current is not None and current['start'] == info['start'] and current['state'] != 'Z'


def signal_selected(selected, sig):
    for pid, info in selected.items():
        if not still_same(pid, info):
            continue
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass


def wait_for_exit(selected, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not any(still_same(pid, info) for pid, info in selected.items()):
            return
        time.sleep(.2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='List matching processes only')
    args = parser.parse_args()
    selected = project_processes(processes())
    if not selected:
        print('No matching maze project processes are running.')
        return 0
    for pid, info in selected.items():
        print(f'{pid}: {" ".join(info["args"])}', flush=True)
    if args.dry_run:
        return 0
    # Let launch own shutdown first; only signal its remaining children afterward.
    parents = {pid: info for pid, info in selected.items() if info['parent'] not in selected}
    signal_selected(parents, signal.SIGINT)
    wait_for_exit(selected, 12)
    remaining = {pid: info for pid, info in selected.items() if still_same(pid, info)}
    signal_selected(remaining, signal.SIGTERM)
    wait_for_exit(remaining, 5)
    remaining = [pid for pid, info in selected.items() if still_same(pid, info)]
    if remaining:
        print(f'Still running: {remaining}; no SIGKILL was sent.')
        return 1
    print('Maze project processes stopped. You can launch the simulation again.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
