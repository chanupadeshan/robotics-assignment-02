"""Runtime single-robot guards, shared by the project launch files."""
import atexit
import fcntl
import os
from pathlib import Path
import time


def guard(role, forbidden):
    """Hold a per-domain role lock and refuse existing conflicting ROS nodes."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor

    path = Path('/tmp') / f'maze_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID", "0")}_{role}.lock'
    stream = path.open('a+')
    try:
        fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        stream.close()
        raise RuntimeError(f'Another project {role} launch holds {path}') from exc
    atexit.register(stream.close)  # Keep the lock for the launch process lifetime.
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('maze_launch_guard', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        names = {name for name, namespace in node.get_node_names_and_namespaces()}
        conflicts = names.intersection(forbidden)
        if conflicts:
            raise RuntimeError(f'Refusing duplicate/conflicting nodes: {sorted(conflicts)}. '
                               'Stop their owning launch with Ctrl+C first.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
