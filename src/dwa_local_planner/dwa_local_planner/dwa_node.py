#!/usr/bin/env python3
"""
DWA Local Planner node rewritten from `dwa_base.py` logic.

This node implements a simple Dynamic Window Approach style planner. It samples random (speed, turn) candidates, forward-simulates
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
        # robot physical radius (m) - used for collision checking
        self.declare_parameter('robot_radius', 0.105)

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
        self.scan_pts_world = []  # cached scan points in world frame (x,y)
        self.goal_reached = False
        # State for 1 Hz consolidated logging
        self.last_status = {}

        # ----------- ROS interfaces -------------------------------------------
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_pub = self.create_publisher(Marker, '/visual_paths', 10)
        self.best_traj_pub = self.create_publisher(Marker, '/dwa/best_trajectory', 10)

        # Timer driven control loop
        # Use `step_time` as the control loop period for parity with base logic
        self.create_timer(self.step_time, self.control_loop)
        # 1 Hz consolidated status logging
        self.create_timer(1.0, self.log_status)

        self.get_logger().info(
            f'DWA planner node started. Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}). '
            f'Max speed: {self.max_speed} m/s, Max turn: {self.max_turn} rad/s, '
            f'Samples: {self.num_samples}, Lookahead: {self.lookahead_steps} steps'
        )

    # -------------------- Callbacks -----------------------------------------
    def odom_cb(self, msg: Odometry):
        """Store latest odometry for use in simulation and scoring."""
        self.odom_msg = msg

    def scan_cb(self, msg: LaserScan):
        """Store latest scan and precompute scan points in world frame for collision checking."""
        self.scan_msg = msg
        self.scan_pts_world = []
        # need odometry to transform scan points into world frame
        if self.odom_msg is None:
            return
        rx = float(self.odom_msg.pose.pose.position.x)
        ry = float(self.odom_msg.pose.pose.position.y)
        orient = self.odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

        angle = float(msg.angle_min)
        for r in msg.ranges:
            if r is None or math.isinf(r) or math.isnan(r) or r <= 0.0:
                angle += float(msg.angle_increment)
                continue
            # point in robot frame
            px_r = r * math.cos(angle)
            py_r = r * math.sin(angle)
            # transform to world (odom) frame using current robot pose
            px_w = rx + math.cos(yaw) * px_r - math.sin(yaw) * py_r
            py_w = ry + math.sin(yaw) * px_r + math.cos(yaw) * py_r
            self.scan_pts_world.append((px_w, py_w))
            angle += float(msg.angle_increment)

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
        Check a candidate path against precomputed scan points (world frame).

        Returns:
          - `None` if the path collides (too close to an obstacle considering robot radius)
          - a positive obstacle cost (higher = worse) otherwise. The cost is
            inverse to the minimum clearance along the path: 1 / clearance.
        """
        if not path:
            return 0.0
        if not self.scan_pts_world:
            # no scan data available -> neutral cost
            return 0.0

        inflation = float(self.get_parameter('robot_radius').value) + float(self.get_parameter('safety_margin').value)
        inflation_sq = inflation * inflation
        min_dist_sq = float('inf')
        collision_point = None

        # For each path point, check distance to each obstacle point (scan point in world)
        for (px, py) in path:
            for (ox, oy) in self.scan_pts_world:
                dx = ox - px
                dy = oy - py
                d2 = dx * dx + dy * dy
                if d2 < inflation_sq:
                    # collision detected
                    collision_dist = math.sqrt(d2)
                    self.get_logger().debug(
                        f'Collision detected: path point ({px:.3f}, {py:.3f}) '
                        f'too close to obstacle ({ox:.3f}, {oy:.3f}), '
                        f'distance={collision_dist:.3f}m < inflation={inflation:.3f}m'
                    )
                    return None
                if d2 < min_dist_sq:
                    min_dist_sq = d2
                    collision_point = (ox, oy)

        if min_dist_sq == float('inf'):
            return 0.0

        min_dist = math.sqrt(min_dist_sq)
        # cost increases as clearance decreases
        clearance = max(min_dist - float(self.get_parameter('safety_margin').value), 1e-6)
        cost = 1.0 / clearance
        self.get_logger().debug(
            f'Path safety: min_dist={min_dist:.3f}m, clearance={clearance:.3f}m, obstacle_cost={cost:.3f}'
        )
        return cost

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

        # obstacle cost: larger -> worse. If the path collides, reject by returning -inf
        ob_cost = self.check_for_collisions(path)
        if ob_cost is None:
            self.get_logger().debug('Trajectory rejected: COLLISION')
            return -float('inf')
        obstacle_penalty = -self.obstacle_weight * ob_cost

        # smoothness (penalize larger turn rates)
        smoothness = -self.smoothness_weight * abs(turn_rate)

        total = goal_score + heading_score + obstacle_penalty + smoothness
        self.get_logger().debug(
            f'Score breakdown: goal_score={goal_score:.3f} (dist={goal_dist:.3f}m), '
            f'heading_score={heading_score:.3f} (angle_diff={angle_diff:.3f}rad), '
            f'obstacle_penalty={obstacle_penalty:.3f}, smoothness={smoothness:.3f}, '
            f'TOTAL={total:.3f}'
        )
        return total

    def choose_best_path(self, candidates: List[Tuple[float, float, List[Tuple[float, float]]]]) -> Tuple[float, float, List[Tuple[float, float]]]:
        """
        Evaluate a list of (speed, turn, path) and return the best triple.
        """
        best_score = -float('inf')
        best = (0.0, 0.0, [])
        valid_count = 0
        collision_count = 0

        for speed, turn, path in candidates:
            score = self.score_trajectory(path, turn)
            if score == -float('inf'):
                collision_count += 1
            else:
                valid_count += 1
                if score > best_score:
                    best_score = score
                    best = (speed, turn, path)
        
        best_v, best_w, best_path = best
        # Store for 1 Hz consolidated logging (not logged here)
        self.last_status['valid_count'] = valid_count
        self.last_status['collision_count'] = collision_count
        self.last_status['best_v'] = best_v
        self.last_status['best_w'] = best_w
        self.last_status['best_score'] = best_score
        return best

    def log_status(self):
        """Consolidated 1 Hz status log combining robot state and trajectory evaluation."""
        if self.goal_reached:
            self.get_logger().info('✓ Goal reached, robot stopped')
            return
        if not self.last_status:
            self.get_logger().debug('No trajectory evaluated yet')
            return
        
        # Consolidated message with all relevant status
        status_msg = (
            f'[POS] ({self.last_status.get("pos_x", 0):.2f}, {self.last_status.get("pos_y", 0):.2f}) '
            f'[DIST] {self.last_status.get("dist_to_goal", 0):.3f}m '
            f'[SCANS] {self.last_status.get("scan_pts", 0)} '
            f'[EVAL] {self.last_status.get("valid_count", 0)} valid, {self.last_status.get("collision_count", 0)} collisions '
            f'[CMD] v={self.last_status.get("best_v", 0):.3f}m/s ω={self.last_status.get("best_w", 0):.3f}rad/s '
            f'[SCORE] {self.last_status.get("best_score", 0):.2f}'
        )
        self.get_logger().info(status_msg)

    # -------------------- Control loop ------------------------------------
    def control_loop(self):
        """Main periodic callback: sample, simulate, score, publish."""
        # If no sensors present or goal reached, do nothing
        if self.odom_msg is None:
            self.get_logger().debug('⏳ Waiting for odometry message...')
            return
        if self.scan_msg is None:
            self.get_logger().debug('⏳ Waiting for laser scan message...')
            return
        if self.goal_reached:
            self.get_logger().debug('✓ Goal already reached, maintaining stop command')
            self.cmd_pub.publish(Twist())
            return

        # Check goal reached first
        cur_x = float(self.odom_msg.pose.pose.position.x)
        cur_y = float(self.odom_msg.pose.pose.position.y)
        dist_to_goal = math.hypot(self.goal_x - cur_x, self.goal_y - cur_y)
        if dist_to_goal <= self.goal_tolerance:
            if not self.goal_reached:
                self.get_logger().info(
                    f'✓ Goal REACHED at ({cur_x:.2f}, {cur_y:.2f}). '
                    f'Target was ({self.goal_x:.2f}, {self.goal_y:.2f})'
                )
                self.goal_reached = True
            # stop the robot
            self.cmd_pub.publish(Twist())
            return
        
        # Update status cache for 1 Hz logging
        self.last_status['pos_x'] = cur_x
        self.last_status['pos_y'] = cur_y
        self.last_status['dist_to_goal'] = dist_to_goal
        self.last_status['scan_pts'] = len(self.scan_pts_world)

        # Sample candidate controls and predict their trajectories
        self.get_logger().debug(f'Sampling {self.num_samples} candidate trajectories...')
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

        # publish chosen path as a separate (brighter) marker on dedicated topic
        if best_path:
            best_marker = Marker()
            best_marker.header.frame_id = self.visual_frame
            best_marker.header.stamp = self.get_clock().now().to_msg()
            best_marker.ns = 'dwa_best'
            best_marker.id = 0
            best_marker.type = Marker.LINE_STRIP
            best_marker.action = Marker.ADD
            best_marker.scale.x = 0.05
            best_marker.color.r = 0.0
            best_marker.color.g = 1.0
            best_marker.color.b = 0.0
            best_marker.color.a = 0.95
            for (px, py) in best_path:
                best_marker.points.append(Point(x=px, y=py, z=0.03))
            self.best_traj_pub.publish(best_marker)


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlannerNode()
    node.get_logger().info('DWA planner node spinning... (Ctrl+C to stop)')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user (Ctrl+C)')
    finally:
        node.get_logger().info('Shutting down DWA planner node; stopping robot')
        # ensure robot stops
        if node.cmd_pub is not None:
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        print('\n[DWA] Node fully shut down')


if __name__ == '__main__':
    main()
