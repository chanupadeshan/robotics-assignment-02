"""Preview a learned route, or send it directly to Nav2's FollowPath local controller."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from .agent import Agent
from .environment import Maze, rollout


def angle(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def angle_error(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def route_vertices(maze, cells, actual_start=None):
    """Connect the live pose to the untraversed portion of the learned route."""
    if not cells:
        raise ValueError('Empty learned route')
    vertices = [maze.xy(c) for c in cells]
    if actual_start is not None and math.dist(actual_start, vertices[0]) > .001:
        if len(vertices) > 1:
            a, b = vertices[:2]
            dx, dy = b[0]-a[0], b[1]-a[1]
            t = ((actual_start[0]-a[0])*dx+(actual_start[1]-a[1])*dy)/(dx*dx+dy*dy)
            projected = (a[0]+t*dx, a[1]+t*dy)
            if 0 <= t <= 1 and math.dist(actual_start, projected) <= .10:
                # The live robot has already traversed part of the first learned edge.
                vertices[0] = tuple(actual_start)
            else:
                vertices.insert(0, tuple(actual_start))
        else:
            vertices.insert(0, tuple(actual_start))
    return vertices


def checked_points(maze, vertices, resolution=.025):
    points = [vertices[0]]
    for a, b in zip(vertices, vertices[1:]):
        count = max(1, math.ceil(math.dist(a, b)/resolution))
        points += [(a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count)
                   for i in range(1, count+1)]
    if min(maze.clearance(*p) for p in points) < .015:
        raise ValueError('Learned route or start connector has insufficient clearance')
    return points


def route_points(maze, cells, actual_start=None, resolution=.025):
    """Densify the learned adjacent-cell path, retaining its cell order."""
    return checked_points(maze, route_vertices(maze, cells, actual_start), resolution)


class Execution(Node):
    def __init__(self, maze):
        super().__init__('maze_dqn_execution',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.maze, self.goal_handle = maze, None
        self.stop_requested = False
        # Create clients before TF warm-up so endpoint discovery cannot age its cache.
        self.client = ActionClient(self, FollowPath, '/follow_path')
        self.state_clients = {name: self.create_client(GetState, f'/{name}/get_state')
                              for name in ['amcl', 'controller_server', 'velocity_smoother',
                                           'collision_monitor']}
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            NavPath, '/dqn_path', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.samples, self.commands, self.status = {}, 0, {}
        self.last_command = None
        self.map_message = None
        self.subs = [self.create_subscription(LaserScan, '/scan',
                     lambda m: self.record('/scan', m), qos_profile_sensor_data),
                     self.create_subscription(Odometry, '/odom',
                     lambda m: self.record('/odom', m), qos_profile_sensor_data),
                     self.create_subscription(TwistStamped, '/cmd_vel', self.command, 1)]
        self.subs.append(self.create_subscription(
            OccupancyGrid, '/map', lambda m: setattr(self, 'map_message', m),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        for name in ['navigate_to_pose', 'follow_path']:
            self.subs.append(self.create_subscription(
                GoalStatusArray, f'/{name}/_action/status',
                lambda m, name=name: self.status.update({name: m.status_list}),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.truth_queue = queue.SimpleQueue()
        self.truth, self.truth_time, self.truth_stamp = None, 0., None
        self.minimum_clearance = math.inf

    def record(self, name, msg):
        self.samples[name] = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def command(self, msg):
        self.last_command = {'linear_x': msg.twist.linear.x, 'angular_z': msg.twist.angular.z}
        if abs(msg.twist.linear.x) > .001 or abs(msg.twist.angular.z) > .001:
            self.commands += 1

    def read_truth(self, stream):
        for line in stream:
            try:
                self.truth_queue.put(json.loads(line))
            except ValueError:
                pass

    def spin(self):
        rclpy.spin_once(self, timeout_sec=.05)
        while not self.truth_queue.empty():
            msg = self.truth_queue.get()
            for pose in msg.get('pose', []):
                if pose.get('name') != 'burger':
                    continue
                p, q = pose['position'], pose['orientation']
                self.truth = {'x': p.get('x', 0.), 'y': p.get('y', 0.),
                              'yaw': math.atan2(2*(q.get('w', 1.)*q.get('z', 0.) +
                                                    q.get('x', 0.)*q.get('y', 0.)),
                                                1-2*(q.get('y', 0.)**2+q.get('z', 0.)**2))}
                self.truth_time = time.monotonic()
                stamp = msg['header']['stamp']
                self.truth_stamp = float(stamp.get('sec', 0))+float(stamp.get('nsec', 0))/1e9
                self.minimum_clearance = min(self.minimum_clearance,
                                             self.maze.clearance(p.get('x', 0.), p.get('y', 0.)))

    def spin_for(self, seconds):
        end = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic() < end:
            self.spin()

    def wait(self, future, timeout=20):
        end = time.monotonic()+timeout
        while not future.done() and rclpy.ok() and time.monotonic() < end:
            self.spin()
        if not future.done():
            raise RuntimeError('ROS response timed out')
        return future.result()

    def pose(self):
        t = self.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        return {'x': t.translation.x, 'y': t.translation.y, 'yaw': angle(t.rotation)}

    def readiness(self, execute):
        states = {}
        if execute:
            for name, client in self.state_clients.items():
                end = time.monotonic()+5
                while not client.service_is_ready() and time.monotonic() < end:
                    self.spin()
                if not client.service_is_ready():
                    raise RuntimeError(f'Missing {name} lifecycle service')
                states[name] = self.wait(client.call_async(GetState.Request())).current_state.label
            if any(value != 'active' for value in states.values()):
                raise RuntimeError(f'Nav2 command chain is inactive: {states}')
        self.spin_for(8)
        now = self.get_clock().now().nanoseconds/1e9
        for topic in ['/scan', '/odom']:
            if topic not in self.samples or not -1 < now-self.samples[topic] < 2:
                raise RuntimeError(f'No fresh {topic}; no goal sent')
        pose = self.pose()
        if execute:
            if self.map_message is None or not self.maze.matches_map(
                    self.map_message.info, self.map_message.data):
                raise RuntimeError('Active Nav2 map differs from the trained reference maze')
            if dict(self.get_topic_names_and_types()).get('/cmd_vel') != ['geometry_msgs/msg/TwistStamped']:
                raise RuntimeError('Expected project TwistStamped command bridge')
            graph = self.get_node_names_and_namespaces()
            for name in self.state_clients:
                if sum(n == name and ns == '/' for n, ns in graph) != 1:
                    raise RuntimeError(f'Missing or duplicate project node {name}')
            if any(s.status in (1, 2, 3) for statuses in self.status.values() for s in statuses):
                raise RuntimeError('Another navigation/path goal is active; finish or cancel it first')
            if self.truth is None or abs(now-self.truth_stamp) > 2:
                raise RuntimeError('No fresh named Gazebo ground truth; no goal sent')
            if math.dist((pose['x'], pose['y']), (self.truth['x'], self.truth['y'])) > .12:
                raise RuntimeError('AMCL and Gazebo positions disagree; correct localization first')
            if angle_error(pose['yaw'], self.truth['yaw']) > .3:
                raise RuntimeError('AMCL and Gazebo headings disagree; correct localization first')
            if not self.client.server_is_ready():
                raise RuntimeError('FollowPath action unavailable')
        return pose, states

    def path_message(self, points, final_yaw):
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(points):
            yaw = (math.atan2(points[i+1][1]-y, points[i+1][0]-x)
                   if i+1 < len(points) else final_yaw)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            path.poses.append(pose)
        return path

    def execute(self, path, timeout, goal_xy, final_yaw):
        start, start_commands = time.monotonic(), self.commands
        initial_truth = self.truth.copy()
        self.minimum_clearance = math.inf
        goal = FollowPath.Goal(path=path, controller_id='FollowPath',
                               goal_checker_id='goal_checker', progress_checker_id='progress_checker')
        self.goal_handle = self.wait(self.client.send_goal_async(goal))
        if not self.goal_handle.accepted:
            self.goal_handle = None
            raise RuntimeError('FollowPath goal rejected')
        print('FollowPath goal accepted (learned DQN path; NavFn bypassed).', flush=True)
        future, reason, divergence = self.goal_handle.get_result_async(), None, None
        trajectory, last_sample, last_print = [], 0., 0.
        while not future.done() and time.monotonic()-start < timeout:
            self.spin()
            now = time.monotonic()
            sim_now = self.get_clock().now().nanoseconds/1e9
            if self.stop_requested:
                reason = 'Interrupted; goal canceled before ROS shutdown'
            elif now-self.truth_time > 2 or abs(sim_now-self.truth_stamp) > 1:
                reason = 'Gazebo truth stale'
            elif self.minimum_clearance < 0:
                reason = '0.13 m wall-clearance envelope crossed'
            try:
                pose = self.pose()
                difference = math.dist((pose['x'], pose['y']), (self.truth['x'], self.truth['y']))
                bad = difference > .15 or angle_error(pose['yaw'], self.truth['yaw']) > .35
                divergence = (divergence or now) if bad else None
                if divergence is not None and now-divergence > 3:
                    reason = 'Localization diverged from simulator'
                if now-last_sample > .5:
                    last_sample = now
                    sample = {'seconds': now-start, 'map': pose, 'gazebo': self.truth.copy(),
                              'clearance': self.maze.clearance(self.truth['x'], self.truth['y'])}
                    trajectory.append(sample)
                    if now-last_print > 10:
                        last_print = now
                        print(json.dumps(sample), flush=True)
            except Exception as exc:
                reason = f'TF unavailable during motion: {exc}'
            if reason:
                break
        if not future.done():
            reason = reason or 'Navigation timeout'
            self.wait(self.goal_handle.cancel_goal_async())
        result = self.wait(future)
        self.goal_handle = None
        self.spin_for(1)
        final = self.pose()
        error = math.dist(goal_xy, (self.truth['x'], self.truth['y']))
        map_error = math.dist(goal_xy, (final['x'], final['y']))
        heading_error = angle_error(final_yaw, self.truth['yaw'])
        return {'passed': result.status == 4 and result.result.error_code == 0 and
                error < .12 and map_error < .09 and heading_error < .2 and
                self.minimum_clearance >= 0 and self.commands > start_commands and reason is None,
                'status': result.status, 'error_code': result.result.error_code,
                'error_msg': result.result.error_msg, 'watchdog': reason,
                'wall_seconds': time.monotonic()-start, 'start_truth': initial_truth,
                'final_truth': self.truth.copy(), 'final_map_pose': final,
                'gazebo_position_error': error, 'map_position_error': map_error,
                'gazebo_yaw_error': heading_error, 'moving_commands': self.commands-start_commands,
                'final_command': self.last_command,
                'minimum_clearance': self.minimum_clearance, 'trajectory': trajectory,
                'collision_check': 'Sampled 0.13 m circle against SDF boxes; no contact sensor',
                'execution': 'One FollowPath action to DWB; no NavFn/ComputePathToPose calls'}

    def execute_route(self, cells, timeout, final_yaw):
        """Visit learned waypoints in order; DWB has one adjacent target per action."""
        start = time.monotonic()
        pose = self.pose()
        vertices = route_vertices(self.maze, cells, (pose['x'], pose['y']))
        records, trajectory = [], []
        for i, target in enumerate(vertices[1:], 1):
            if self.stop_requested or time.monotonic()-start >= timeout:
                break
            pose = self.pose()
            points = checked_points(self.maze, [(pose['x'], pose['y']), target])
            yaw = (math.atan2(vertices[i+1][1]-target[1], vertices[i+1][0]-target[0])
                   if i+1 < len(vertices) else final_yaw)
            print(f'DQN waypoint {i}/{len(vertices)-1}: {target}', flush=True)
            offset = time.monotonic()-start
            result = self.execute(self.path_message(points, yaw), timeout-offset, target, yaw)
            records.append(result)
            trajectory.extend(dict(sample, seconds=sample['seconds']+offset, waypoint=i)
                              for sample in result['trajectory'])
            if not result['passed']:
                break
        if not records:
            raise RuntimeError('Route interrupted or timed out before the first waypoint')
        final = {k: v for k, v in records[-1].items() if k != 'trajectory'}
        final.update({'passed': len(records) == len(vertices)-1 and all(r['passed'] for r in records),
                      'wall_seconds': time.monotonic()-start,
                      'start_truth': records[0]['start_truth'],
                      'minimum_clearance': min(r['minimum_clearance'] for r in records),
                      'moving_commands': sum(r['moving_commands'] for r in records),
                      'waypoints_completed': sum(r['passed'] for r in records),
                      'waypoints_requested': len(vertices)-1,
                      'segments': records, 'trajectory': trajectory,
                      'execution': 'Learned waypoint sequence; FollowPath/DWB per leg; no NavFn calls'})
        if len(records) != len(vertices)-1 and all(r['passed'] for r in records):
            final['watchdog'] = ('Interrupted between waypoints' if self.stop_requested
                                 else 'Route timeout between waypoints')
        return final


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--goal-cell', type=int, default=35)
    parser.add_argument('--start-cell', type=int, help='Offline preview only; not allowed with --execute')
    parser.add_argument('--yaw', type=float, help='Arrival yaw in radians; defaults to last segment')
    parser.add_argument('--execute', action='store_true', help='Actually drive in the running simulator')
    parser.add_argument('--timeout', type=float, default=360)
    parser.add_argument('--output', type=Path, default=Path('reports/dqn-route.json'))
    args = parser.parse_args()
    if args.execute and args.start_cell is not None:
        parser.error('Execution always reads the current pose; omit --start-cell')
    maze, report, node, process, lock = Maze(), {'passed': False}, None, None, None

    def request_stop(signum, frame):
        if node is not None:
            node.stop_requested = True
        else:
            raise KeyboardInterrupt('Interrupted before navigation started')

    previous_signals = {s: signal.signal(s, request_stop) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        agent, metadata = Agent.load(args.model)
        if metadata['maze_signature'] != maze.signature:
            raise ValueError('Checkpoint geometry differs from the current corrected maze')
        if args.goal_cell not in metadata['goals']:
            raise ValueError('Goal was not included in this checkpoint training configuration')
        actual = None
        if args.start_cell is None:
            lock = open(f'/tmp/maze_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID", "0")}_dqn.lock', 'a')
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('Another DQN route is active; wait for it to finish') from None
            # Keep the ROS context alive long enough to cancel on Ctrl+C/SIGTERM.
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            node = Execution(maze)
            if args.execute:
                process = subprocess.Popen(['gz', 'topic', '-e', '-t',
                                            '/world/maze/dynamic_pose/info', '--json-output'],
                                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                           text=True, bufsize=1)
                threading.Thread(target=node.read_truth, args=(process.stdout,), daemon=True).start()
            actual, states = node.readiness(args.execute)
            if node.stop_requested:
                raise RuntimeError('Interrupted before navigation started')
            report['readiness'] = {'pose': actual, 'lifecycle': states}
            start = maze.cell_at(actual['x'], actual['y'])
        else:
            start = args.start_cell
            maze.xy(start)
        learned = rollout(maze, agent, start, args.goal_cell)
        report.update({'policy': learned, 'maze_signature': maze.signature,
                       'model': str(args.model), 'executed': False})
        if not learned['success'] or learned['wall_attempts'] or len(set(learned['path'])) != len(learned['path']):
            raise RuntimeError('Greedy DQN failed or looped; no search fallback and no motion')
        points = route_points(maze, learned['path'],
                              (actual['x'], actual['y']) if actual else None)
        if args.yaw is not None:
            final_yaw = args.yaw
        elif len(points) > 1:
            final_yaw = math.atan2(points[-1][1]-points[-2][1], points[-1][0]-points[-2][0])
        else:
            final_yaw = actual['yaw'] if actual else 0.
        report.update({'route_xy': [maze.xy(c) for c in learned['path']],
                       'dense_path_points': len(points),
                       'planned_minimum_clearance': min(maze.clearance(*p) for p in points),
                       'goal_yaw': final_yaw})
        print('Learned cell route: '+str(learned['path']), flush=True)
        if node is not None:
            path = node.path_message(points, final_yaw)
            node.publisher.publish(path)
            if args.execute and len(learned['path']) > 1:
                report['executed'] = True
                report['navigation'] = node.execute_route(learned['path'], args.timeout, final_yaw)
                report['passed'] = report['navigation']['passed']
            else:
                node.spin_for(2)
                report['passed'] = True
        else:
            report['passed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc)
        print(f'DQN route failed: {exc}', flush=True)
    finally:
        if node is not None and node.goal_handle is not None:
            try:
                node.wait(node.goal_handle.cancel_goal_async())
                node.wait(node.goal_handle.get_result_async())
                node.goal_handle = None
            except Exception as exc:
                report['cancel_error'] = str(exc)
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        summary = {k: v for k, v in report.items() if k != 'navigation'}
        if 'navigation' in report:
            summary['navigation'] = {k: v for k, v in report['navigation'].items()
                                     if k not in ('trajectory', 'segments')}
        print(json.dumps(summary, indent=2), flush=True)
        # The command lock covers active motion, not potentially slow DDS teardown.
        if lock is not None and (node is None or node.goal_handle is None):
            lock.close()
            lock = None
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        if lock is not None:
            lock.close()
        for signum, handler in previous_signals.items():
            signal.signal(signum, handler)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
