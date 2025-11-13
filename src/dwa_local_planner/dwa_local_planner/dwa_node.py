#!/usr/bin/env python3
"""
DWA Local Planner node rewritten from `dwa_base.py` logic.

This node implements a simple Dynamic Window Approach style planner inspired by
`dwa_base.py`. It samples random (speed, turn) candidates, forward-simulates
their trajectories from the current odometry, scores them (goal distance,
heading, collision, smoothness) and publishes the selected velocity as
`/cmd_vel`. It also publishes a `visualization_msgs/Marker` containing the
sampled trajectories for RViz.

All tunable values are exposed as ROS2 parameters (inline comments show units
and guidance). Parameters mirror the variables in `dwa_base.py` where applicable.
"""

import math
import random
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion


class DWAPlannerNode(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')

        # ----------- Declare parameters (inline comments explain each) -----------
        # maximum forward speed (m/s)
        # used as upper bound when sampling candidate forward velocities
        self.declare_parameter('max_speed', 0.15)
        # maximum absolute turn rate (rad/s)
        # used as bound for sampled angular velocities
        self.declare_parameter('max_turn', 2.5)
        # control loop timestep (s) - also used as integration step
        self.declare_parameter('step_time', 0.1)
        # number of candidate (speed,turn) pairs to sample each cycle
        self.declare_parameter('num_samples', 200)
        # safety margin used when checking collisions (m)
        self.declare_parameter('safety_margin', 0.3)
        # weights for scoring function (larger -> stronger influence)
        self.declare_parameter('goal_weight', 5.0)
        self.declare_parameter('heading_weight', 2.0)
        self.declare_parameter('smoothness_weight', 0.1)
        self.declare_parameter('obstacle_weight', 1.0)
        # visualization topic frame
        self.declare_parameter('visual_frame', 'odom')
        # goal parameters
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 1.0)
        self.declare_parameter('goal_tolerance', 0.05)
        # number of lookahead steps when simulating each candidate
        self.declare_parameter('lookahead_steps', 100)

        # ----------- Read parameters into attributes (typed) -------------------
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_turn = float(self.get_parameter('max_turn').value)
        self.step_time = float(self.get_parameter('step_time').value)
        self.num_samples = int(self.get_parameter('num_samples').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.goal_weight = float(self.get_parameter('goal_weight').value)
        self.heading_weight = float(self.get_parameter('heading_weight').value)
        self.smoothness_weight = float(self.get_parameter('smoothness_weight').value)
        self.obstacle_weight = float(self.get_parameter('obstacle_weight').value)
        self.visual_frame = str(self.get_parameter('visual_frame').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.lookahead_steps = int(self.get_parameter('lookahead_steps').value)

        # ----------- Internal state --------------------------------------------
        self.odom_msg = None     # latest Odometry message
        self.scan_msg = None     # latest LaserScan message
        self.goal_reached = False

        # ----------- ROS interfaces -------------------------------------------
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_pub = self.create_publisher(Marker, '/visual_paths', 10)

        # Timer driven control loop
        # Use `step_time` as the control loop period for parity with base logic
        self.create_timer(self.step_time, self.control_loop)

        self.get_logger().info('DWA planner node started')

    # -------------------- Callbacks -----------------------------------------
    def odom_cb(self, msg: Odometry):
        """Store latest odometry for use in simulation and scoring."""
        self.odom_msg = msg

    def scan_cb(self, msg: LaserScan):
        """Store latest scan for collision checking."""
        self.scan_msg = msg

    # -------------------- Core algorithms ----------------------------------
    def predict_motion(self, speed: float, turn_rate: float) -> List[Tuple[float, float]]:
        """
        Forward-simulate a trajectory starting from current odometry using
        simple unicycle kinematics. Returns a list of (x, y) world coordinates.
        """
        if self.odom_msg is None:
            return []

        x = float(self.odom_msg.pose.pose.position.x)
        y = float(self.odom_msg.pose.pose.position.y)
        orient = self.odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

        path = []
        for _ in range(self.lookahead_steps):
            # integrate
            x += speed * math.cos(yaw) * self.step_time
            y += speed * math.sin(yaw) * self.step_time
            yaw += turn_rate * self.step_time
            path.append((x, y))
        return path

    def check_for_collisions(self, path: List[Tuple[float, float]]) -> float:
        """
        Return an obstacle-related penalty (lower is better). If any point in
        the path is too close to an obstacle, return a large negative penalty.

        The method maps world coordinates to robot-relative coordinates using
        current odometry then looks up the laser range for the heading angle.
        """
        if self.scan_msg is None or self.odom_msg is None:
            # no scan or odom -> cannot reason about obstacles; return neutral small penalty
            return 0.0

        # laser scan metadata
        angle_min = float(self.scan_msg.angle_min)
        angle_inc = float(self.scan_msg.angle_increment)
        ranges = list(self.scan_msg.ranges)
        max_range = float(self.scan_msg.range_max)

        rx = float(self.odom_msg.pose.pose.position.x)
        ry = float(self.odom_msg.pose.pose.position.y)
        orient = self.odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

        min_dist = float('inf')

        for (px, py) in path:
            # transform to robot frame
            dx = px - rx
            dy = py - ry
            # rotate into robot frame by -yaw
            rel_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
            rel_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
            dist = math.hypot(rel_x, rel_y)
            angle = math.atan2(rel_y, rel_x)
            idx = int((angle - angle_min) / angle_inc)
            if idx < 0 or idx >= len(ranges):
                # out of scan range -> be conservative and treat as collision
                return -1e6
            scan_dist = ranges[idx]
            if math.isinf(scan_dist) or math.isnan(scan_dist):
                scan_dist = max_range
            # if obstacle closer than path point minus safety margin -> collision
            if scan_dist <= dist - self.safety_margin:
                return -1e6
            if dist < min_dist:
                min_dist = dist

        # safe clearance -> return inverse clearance as penalty (smaller is better)
        if min_dist == float('inf'):
            return 0.0
        return -1.0 / max(min_dist - self.safety_margin, 1e-6)

    def score_trajectory(self, path: List[Tuple[float, float]], turn_rate: float) -> float:
        """
        Combine multiple heuristics into a single score (higher = better):
          - goal proximity (prefer smaller final distance)
          - heading alignment (prefer facing goal)
          - obstacle penalty (from check_for_collisions)
          - smoothness penalty (prefer small angular rates)
        """
        if not path:
            return -float('inf')

        # goal distance score (smaller distance -> higher score)
        last_x, last_y = path[-1]
        goal_dist = math.hypot(self.goal_x - last_x, self.goal_y - last_y)
        goal_score = -self.goal_weight * goal_dist

        # heading score based on current pose
        if self.odom_msg is None:
            heading_score = 0.0
        else:
            orient = self.odom_msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
            desired_yaw = math.atan2(self.goal_y - float(self.odom_msg.pose.pose.position.y),
                                     self.goal_x - float(self.odom_msg.pose.pose.position.x))
            angle_diff = abs(math.atan2(math.sin(desired_yaw - yaw), math.cos(desired_yaw - yaw)))
            heading_score = -self.heading_weight * angle_diff

        # obstacle penalty (negative number, lower is worse)
        obstacle_penalty = self.obstacle_weight * self.check_for_collisions(path)

        # smoothness (penalize larger turn rates)
        smoothness = -self.smoothness_weight * abs(turn_rate)

        total = goal_score + heading_score + obstacle_penalty + smoothness
        return total

    def choose_best_path(self, candidates: List[Tuple[float, float, List[Tuple[float, float]]]]) -> Tuple[float, float, List[Tuple[float, float]]]:
        """
        Evaluate a list of (speed, turn, path) and return the best triple.
        """
        best_score = -float('inf')
        best = (0.0, 0.0, [])

        for speed, turn, path in candidates:
            score = self.score_trajectory(path, turn)
            if score > best_score:
                best_score = score
                best = (speed, turn, path)
        return best

    # -------------------- Control loop ------------------------------------
    def control_loop(self):
        """Main periodic callback: sample, simulate, score, publish."""
        # If no sensors present or goal reached, do nothing
        if self.odom_msg is None or self.scan_msg is None or self.goal_reached:
            return

        # Check goal reached first
        cur_x = float(self.odom_msg.pose.pose.position.x)
        cur_y = float(self.odom_msg.pose.pose.position.y)
        if math.hypot(self.goal_x - cur_x, self.goal_y - cur_y) <= self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info(f'Goal reached: ({self.goal_x:.2f}, {self.goal_y:.2f})')
                self.goal_reached = True
            # stop the robot
            self.cmd_pub.publish(Twist())
            return

        # Sample candidate controls and predict their trajectories
        candidates = []
        for _ in range(self.num_samples):
            v = random.uniform(0.0, self.max_speed)
            w = random.uniform(-self.max_turn, self.max_turn)
            path = self.predict_motion(v, w)
            candidates.append((v, w, path))

        # Choose best candidate
        best_v, best_w, best_path = self.choose_best_path(candidates)

        # Publish chosen velocity
        move = Twist()
        move.linear.x = float(best_v)
        move.angular.z = float(best_w)
        self.cmd_pub.publish(move)

        # Publish visualization marker of all candidate paths (faint) and chosen path (bright)
        marker = Marker()
        marker.header.frame_id = self.visual_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'dwa_paths'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.color.r = 0.6
        marker.color.g = 0.6
        marker.color.b = 0.6
        marker.color.a = 0.25

        # append segments for all candidates (sparse to avoid too many points)
        for _, _, path in candidates:
            if len(path) < 2:
                continue
            for i in range(len(path) - 1):
                p1 = Point(x=path[i][0], y=path[i][1], z=0.02)
                p2 = Point(x=path[i + 1][0], y=path[i + 1][1], z=0.02)
                marker.points.append(p1)
                marker.points.append(p2)

        # publish candidates marker
        self.traj_pub.publish(marker)

        # publish chosen path as a separate (brighter) marker
        if best_path:
            best_marker = Marker()
            best_marker.header.frame_id = self.visual_frame
            best_marker.header.stamp = self.get_clock().now().to_msg()
            best_marker.ns = 'dwa_best'
            best_marker.id = 1
            best_marker.type = Marker.LINE_STRIP
            best_marker.action = Marker.ADD
            best_marker.scale.x = 0.04
            best_marker.color.r = 0.0
            best_marker.color.g = 1.0
            best_marker.color.b = 0.0
            best_marker.color.a = 0.9
            for (px, py) in best_path:
                best_marker.points.append(Point(x=px, y=py, z=0.03))
            self.traj_pub.publish(best_marker)


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ensure robot stops
        if node.cmd_pub is not None:
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
