#!/usr/bin/env python3
"""
DerpBot autonomous explorer — Phase 1: Drive & Survive.

Subscribes to LiDAR + odom + IMU.
Uses VFF reactive obstacle avoidance to wander without crashing.
Publishes Twist to /derpbot_0/cmd_vel.
"""

import rclpy
from pathlib import Path

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry

from obstacle_avoider import ObstacleAvoider
from occupancy_grid import OccupancyGrid


import math

ROBOT = "derpbot_0"


def _yaw_from_quaternion(q) -> float:
    """Extract yaw from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AgentNode(Node):
    def __init__(self):
        super().__init__("agent_node", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        ])

        self.avoider = ObstacleAvoider()
        self.grid = OccupancyGrid()

        # latest sensor data
        self._scan: LaserScan | None = None
        self._odom: Odometry | None = None

        # QoS profiles
        reliable_qos = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscribers
        self.create_subscription(LaserScan, f"/{ROBOT}/scan", self._scan_cb, reliable_qos)
        self.create_subscription(Odometry, f"/{ROBOT}/odom", self._odom_cb, reliable_qos)
        self.create_subscription(Imu, f"/{ROBOT}/imu", self._imu_cb, best_effort_qos)

        # Publisher
        self._cmd_pub = self.create_publisher(Twist, f"/{ROBOT}/cmd_vel", reliable_qos)

        # Control loop at 10 Hz
        self.create_timer(0.1, self._control_loop)

        self._debug_dir = Path(__file__).parent.parent
        self.get_logger().info("AgentNode Phase 2 started — Mapping")

    # ------------------------------------------------------------------ #
    # Callbacks                                                            #
    # ------------------------------------------------------------------ #

    def _scan_cb(self, msg: LaserScan) -> None:
        self._scan = msg
        if self._odom is not None:
            p = self._odom.pose.pose
            rx = p.position.x
            ry = p.position.y
            ryaw = _yaw_from_quaternion(p.orientation)
            self.grid.update(rx, ry, ryaw, msg.ranges, msg.angle_min, msg.angle_increment)

    def _odom_cb(self, msg: Odometry) -> None:
        self._odom = msg

    def _imu_cb(self, msg: Imu) -> None:
        pass  # available for future phases

    # ------------------------------------------------------------------ #
    # Control loop                                                         #
    # ------------------------------------------------------------------ #

    def _control_loop(self) -> None:
        if self._scan is None:
            # no data yet — stay still
            return

        scan = self._scan
        linear_x, angular_z = self.avoider.compute(
            ranges=list(scan.ranges),
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
        )

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # save debug grid
        try:
            grid_path = node._debug_dir / "grid.png"
            node.grid.save_png(grid_path)
            node.get_logger().info(f"Grid saved to {grid_path}")
        except Exception as e:
            node.get_logger().warn(f"Grid save failed: {e}")
        # stop robot on exit (best-effort — context may already be gone)
        try:
            node._cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
