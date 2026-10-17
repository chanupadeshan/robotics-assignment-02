#!/usr/bin/python3
"""Readiness checks and opt-in sequential Nav2 trials with JSON evidence."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import queue
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


def position(t):
    return {'x': t.translation.x, 'y': t.translation.y, 'yaw': yaw(t.rotation)}


class Verification(Node):
    def __init__(self):
        super().__init__('maze_verification',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.latest = {}
        self.times = defaultdict(list)
        self.commands = 0
        self.active_goal = None
        self.truth = None
        self.min_clearance = math.inf
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.subscriptions_ = []
        for topic, kind, qos in [
            ('/scan', LaserScan, qos_profile_sensor_data),
            ('/odom', Odometry, qos_profile_sensor_data),
            ('/map', OccupancyGrid,
             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            ('/cmd_vel', TwistStamped, 10),
        ]:
            self.subscriptions_.append(self.create_subscription(
                kind, topic, lambda msg, topic=topic: self.record(topic, msg), qos))
        self.truth_queue = queue.SimpleQueue()
        self.truth_stamp = None
        self.truth_samples = 0
        sdf_path = Path(get_package_share_directory('maze_world')) / 'worlds/maze_fixed.sdf'
        sdf = ET.parse(str(sdf_path))
        self.walls = []
        for link in sdf.findall(".//model[@name='maze_walls']/link"):
            p = list(map(float, link.findtext('pose').split()))
            s = list(map(float, link.findtext('collision/geometry/box/size').split()))
            self.walls.append((p[0], p[1], s[0]/2, s[1]/2))

    def record(self, topic, msg):
        self.latest[topic] = msg
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.times[topic].append((time.monotonic(), stamp))
        if topic == '/cmd_vel' and (abs(msg.twist.linear.x) > .001
                                    or abs(msg.twist.angular.z) > .001):
            self.commands += 1

    def clearance(self, x, y):
        # Conservative 0.13 m enclosing circle, sampled against actual SDF boxes.
        return min(math.hypot(max(abs(x-wx)-hx, 0), max(abs(y-wy)-hy, 0))-.13
                   for wx, wy, hx, hy in self.walls)

    def read_truth(self, stream):
        for line in stream:
            try:
                self.truth_queue.put(json.loads(line))
            except json.JSONDecodeError:
                continue

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=.1)
        while not self.truth_queue.empty():
            msg = self.truth_queue.get()
            for pose in msg.get('pose', []):
                if pose.get('name') != 'burger':
                    continue
                p, q = pose.get('position', {}), pose.get('orientation', {})
                x, y, z, w = (q.get(k, 0.) for k in ('x', 'y', 'z', 'w'))
                self.truth = {'x': p.get('x', 0.), 'y': p.get('y', 0.),
                              'yaw': math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))}
                stamp = msg['header']['stamp']
                self.truth_stamp = float(stamp.get('sec', 0))+float(stamp.get('nsec', 0))*1e-9
                self.truth_samples += 1
                clearance = self.clearance(self.truth['x'], self.truth['y'])
                self.min_clearance = min(self.min_clearance, clearance)

    def spin_for(self, seconds):
        deadline = time.monotonic()+seconds
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin_once()

    def wait(self, future, seconds=20):
        deadline = time.monotonic()+seconds
        while not future.done() and rclpy.ok() and time.monotonic() < deadline:
            self.spin_once()
        if not future.done():
            raise RuntimeError('Timed out waiting for ROS response')
        return future.result()

    def pose(self):
        transform = self.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        return position(transform.transform)

    def readiness(self):
        self.spin_for(8)
        nodes = ['map_server', 'amcl', 'controller_server', 'smoother_server', 'planner_server',
                 'behavior_server', 'velocity_smoother', 'collision_monitor', 'bt_navigator',
                 'local_costmap/local_costmap', 'global_costmap/global_costmap']
        futures, clients = {}, []
        for name in nodes:
            client = self.create_client(GetState, '/'+name+'/get_state')
            clients.append(client)
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError(f'{name}: missing lifecycle service')
            futures[name] = client.call_async(GetState.Request())
        states = {name: self.wait(f).current_state.label for name, f in futures.items()}
        if any(state != 'active' for state in states.values()):
            raise RuntimeError(f'Inactive lifecycle nodes: {states}')
        sim_time = {}
        for name in nodes+['robot_state_publisher']:
            client = self.create_client(GetParameters, '/'+name+'/get_parameters')
            clients.append(client)
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError(f'{name}: missing parameter service')
            request = GetParameters.Request(names=['use_sim_time'])
            sim_time[name] = self.wait(client.call_async(request)).values[0].bool_value
        if not all(sim_time.values()):
            raise RuntimeError(f'Inconsistent simulation clock: {sim_time}')
        topics = dict(self.get_topic_names_and_types())
        rates = {}
        for name in ['/scan', '/odom', '/map']:
            if name not in self.latest:
                raise RuntimeError(f'No messages received on {name}')
            samples = self.times[name]
            if len(samples) > 1:
                wall_delta, sim_delta = samples[-1][0]-samples[0][0], samples[-1][1]-samples[0][1]
                rates[name] = {'wall_hz': (len(samples)-1)/wall_delta,
                               'sim_hz': (len(samples)-1)/sim_delta if sim_delta > 0 else None}
        now = self.get_clock().now().nanoseconds/1e9
        for name in ['/scan', '/odom']:
            age = now-self.times[name][-1][1]
            if not -1.0 < age < 2.0:
                raise RuntimeError(f'{name} is stale: {age:.3f} sim seconds')
        if self.truth is None or self.truth_stamp is None or abs(now-self.truth_stamp) > 2.0:
            raise RuntimeError('No fresh named Gazebo burger ground truth; no goals sent')
        chain = [('map', 'odom'), ('odom', 'base_footprint'), ('base_footprint', 'base_scan')]
        for parent, child in chain:
            self.buffer.lookup_transform(parent, child, rclpy.time.Time())
        if topics.get('/cmd_vel') != ['geometry_msgs/msg/TwistStamped']:
            raise RuntimeError(f'Wrong /cmd_vel type: {topics.get("/cmd_vel")}')
        if self.latest['/scan'].header.frame_id != 'base_scan':
            raise RuntimeError('Expected base_scan laser frame')
        actions = [(NavigateToPose, 'navigate_to_pose'),
                   (ComputePathToPose, 'compute_path_to_pose')]
        for kind, name in actions:
            client = ActionClient(self, kind, name)
            if not client.wait_for_server(timeout_sec=5):
                raise RuntimeError(f'Missing action server {name}')
            client.destroy()
        graph = self.get_node_names_and_namespaces()
        duplicates = []
        for name in nodes:
            namespace, _, basename = ('/'+name).rpartition('/')
            namespace = namespace or '/'
            if sum(n == basename and ns == namespace for n, ns in graph) > 1:
                duplicates.append(name)
        if duplicates:
            raise RuntimeError(f'Duplicate nodes: {duplicates}')
        map_info = self.latest['/map'].info
        return {'lifecycle': states, 'use_sim_time': sim_time, 'rates': rates,
                'scan_frame': self.latest['/scan'].header.frame_id,
                'map': {'width': map_info.width, 'height': map_info.height,
                        'resolution': map_info.resolution},
                'cmd_vel_type': topics['/cmd_vel'],
                'map_pose': self.pose(), 'gazebo_pose': self.truth}

    def goal(self, x, y, angle, timeout):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
        planner = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        plan = ComputePathToPose.Goal(goal=pose, planner_id='GridBased', use_start=False)
        ph = self.wait(planner.send_goal_async(plan))
        if not ph.accepted:
            raise RuntimeError('Planning goal rejected')
        path_result = self.wait(ph.get_result_async())
        if path_result.status != GoalStatus.STATUS_SUCCEEDED or not path_result.result.path.poses:
            raise RuntimeError(f'No valid path: {path_result.result}')
        path_clearance = min(self.clearance(p.pose.position.x, p.pose.position.y)
                             for p in path_result.result.path.poses)
        if path_clearance < .005:
            raise RuntimeError(
                f'Planned path has insufficient wall clearance: {path_clearance:.3f}')
        client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        start_pose, start_truth = self.pose(), self.truth.copy() if self.truth else None
        start_commands, start = self.commands, time.monotonic()
        start_samples = self.truth_samples
        self.min_clearance = math.inf
        handle = self.wait(client.send_goal_async(NavigateToPose.Goal(pose=pose)))
        if not handle.accepted:
            raise RuntimeError('Navigation goal rejected')
        self.active_goal = handle
        future = handle.get_result_async()
        watchdog = None
        while not future.done() and time.monotonic() < start+timeout:
            self.spin_once()
            if self.min_clearance < 0:
                watchdog = 'Wall clearance envelope crossed'
                break
            sim_now = self.get_clock().now().nanoseconds/1e9
            if self.truth_stamp is None or abs(sim_now-self.truth_stamp) > 1.0:
                watchdog = 'Gazebo ground truth became stale'
                break
        if not future.done():
            self.wait(handle.cancel_goal_async(), seconds=10)
            result = self.wait(future, seconds=10)
        else:
            result = future.result()
        self.active_goal = None
        self.spin_for(1)
        final, truth = self.pose(), self.truth.copy() if self.truth else None
        error = math.hypot(final['x']-x, final['y']-y)
        truth_error = math.hypot(truth['x']-x, truth['y']-y) if truth else None
        yaw_error = abs(math.atan2(math.sin(final['yaw']-angle), math.cos(final['yaw']-angle)))
        displacement = (math.hypot(truth['x']-start_truth['x'], truth['y']-start_truth['y'])
                        if truth and start_truth else None)
        clearance = self.min_clearance if math.isfinite(self.min_clearance) else None
        record = {'goal': [x, y, angle], 'action_status': result.status,
                  'result': str(result.result), 'wall_seconds': time.monotonic()-start,
                  'start_pose': start_pose, 'final_pose': final, 'position_error': error,
                  'yaw_error': yaw_error, 'gazebo_start': start_truth, 'gazebo_final': truth,
                  'gazebo_position_error': truth_error,
                  'moving_commands': self.commands-start_commands,
                  'gazebo_samples': self.truth_samples-start_samples,
                  'watchdog': watchdog,
                  'gazebo_displacement': displacement,
                  'planned_min_clearance': path_clearance,
                  'sampled_min_clearance': clearance,
                  'collision_check': ('0.13 m circle vs SDF walls at Gazebo pose samples; '
                                      'no contact sensor')}
        record['passed'] = (
            result.status == GoalStatus.STATUS_SUCCEEDED and error < .09 and yaw_error < .2
            and truth_error is not None and truth_error < .12
            and self.commands > start_commands and self.min_clearance >= 0
            and displacement is not None and displacement > .05 and watchdog is None)
        planner.destroy()
        client.destroy()
        print(json.dumps(record, indent=2), flush=True)
        return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goals', choices=['none', 'short', 'three', 'all'], default='none')
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--output', type=Path, default=Path('/tmp/maze_verification.json'))
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Verification()
    bridge = subprocess.Popen(['gz', 'topic', '-e', '-t', '/world/maze/dynamic_pose/info',
                               '--json-output'], stdout=subprocess.PIPE,
                              stderr=subprocess.DEVNULL, text=True, bufsize=1)
    reader = threading.Thread(target=node.read_truth, args=(bridge.stdout,), daemon=True)
    reader.start()
    report = {'passed': False, 'goals': []}
    try:
        report['readiness'] = node.readiness()
        print(json.dumps(report['readiness'], indent=2), flush=True)
        goals = []
        if args.goals in ('short', 'all'):
            goals.append((-.45, -1.25, 0.0))
        if args.goals in ('three', 'all'):
            goals += [(.75, -1.25, 0.0), (1.25, -1.25, math.pi/2), (1.25, -.75, math.pi/2)]
        for goal in goals:
            trial = node.goal(*goal, timeout=args.timeout)
            report['goals'].append(trial)
            if not trial['passed']:
                raise RuntimeError('Goal verification failed; subsequent goals were not sent')
        report['passed'] = True
    except Exception as exc:
        report['error'] = str(exc)
        print(f'VERIFICATION FAILED: {exc}', flush=True)
    finally:
        if node.active_goal is not None:
            try:
                node.wait(node.active_goal.cancel_goal_async(), seconds=10)
                node.wait(node.active_goal.get_result_async(), seconds=10)
            except Exception as exc:
                report['cancellation_error'] = str(exc)
        bridge.terminate()
        try:
            bridge.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bridge.kill()
            bridge.wait()
        reader.join(timeout=2)
        bridge.stdout.close()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
