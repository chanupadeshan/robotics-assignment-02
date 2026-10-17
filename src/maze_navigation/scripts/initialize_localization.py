#!/usr/bin/python3
"""Initialize AMCL using the simulation clock, then gate navigation on real TF."""
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('initialize_localization', parameter_overrides=[])
    for key, value in [('x', -0.764), ('y', -1.250), ('yaw', 0.0), ('timeout', 90.0)]:
        node.declare_parameter(key, value)
    received = {}
    node.subscriptions_ = [
        node.create_subscription(OccupancyGrid, '/map', lambda m: received.update(map=m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(LaserScan, '/scan', lambda m: received.update(scan=m),
                                 qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odom', lambda m: received.update(odom=m),
                                 qos_profile_sensor_data),
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 lambda m: received.update(amcl=m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    buffer = Buffer(node=node)
    node.tf_listener_ = TransformListener(buffer, node)
    client = node.create_client(GetState, '/amcl/get_state')
    future = None
    active = False
    published = False
    last_publish = 0.0
    deadline = time.monotonic() + node.get_parameter('timeout').value
    result = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if not active and client.service_is_ready():
                if future is None:
                    future = client.call_async(GetState.Request())
                elif future.done():
                    active = future.result().current_state.id == 3
                    future = None
            clock = node.get_clock().now()
            ready = (active and clock.nanoseconds > 0 and
                     all(k in received for k in ('map', 'scan', 'odom')) and
                     buffer.can_transform('odom', 'base_scan', rclpy.time.Time()))
            if not ready:
                continue
            if published and 'amcl' in received and buffer.can_transform(
                    'map', 'base_footprint', rclpy.time.Time()):
                node.get_logger().info(
                    'Localization ready: active AMCL, map, scan, odom, '
                    'map -> odom -> base_scan.')
                result = 0
                break
            if now - last_publish >= 1.0:
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = clock.to_msg()
                pose.pose.pose.position.x = node.get_parameter('x').value
                pose.pose.pose.position.y = node.get_parameter('y').value
                yaw = node.get_parameter('yaw').value
                pose.pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.pose.orientation.w = math.cos(yaw / 2)
                pose.pose.covariance[0] = 0.0025
                pose.pose.covariance[7] = 0.0025
                pose.pose.covariance[35] = 0.0076
                publisher.publish(pose)
                last_publish = now
                published = True
                node.get_logger().info(
                    'Published simulation-stamped initial pose with nonzero covariance.')
        if result:
            node.get_logger().error(
                f'Localization timeout; active={active}, received={list(received)}. '
                'Navigation was not started.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
