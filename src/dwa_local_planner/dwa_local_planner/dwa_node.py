#!/usr/bin/env python3
"""Dynamic Window Approach (DWA) Local Planner for ROS 2.

This module implements a DWA-based local planner for differential drive robots.
It computes collision-free velocity commands by:
1. Computing a dynamic window of feasible velocities
2. Sampling candidate trajectories within this window
3. Evaluating trajectories based on goal heading, clearance, and velocity
4. Selecting and publishing the optimal velocity command
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import numpy as np
import math


def angle_normalize(x):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(x), math.cos(x))


class DWAConfig:
    """Configuration parameters for DWA planner.
    
    Tuning Guide:
    -------------
    Robot Kinematic Limits:
    - max_speed: Reduce if robot overshoots or is unstable (default: 0.22 m/s)
    - min_speed: Allow small negative for backing up, but keep small (default: -0.1 m/s)
    - max_yaw_rate: Higher allows sharper turns but may cause instability (default: 2.0 rad/s)
    - max_accel/max_delta_yaw_rate: Lower for smoother motion (default: 0.5, 1.5)
    
    Sampling Resolution:
    - v_res: Smaller = more samples, slower computation (default: 0.01 m/s)
    - yaw_res: Smaller = finer angular resolution (default: 0.1 rad/s)
    
    Prediction:
    - predict_time: Longer horizon = more foresight but slower (default: 2.0 s)
    - dt: Simulation time step, match control loop rate (default: 0.1 s)
    
    Cost Function Weights (most important for tuning!):
    - heading_weight: Higher = more aggressive toward goal (default: 1.0)
    - clearance_weight: Higher = more cautious obstacle avoidance (default: 1.5)
    - velocity_weight: Higher = prefers faster motion (default: 0.8)
    
    Safety:
    - robot_radius: Should match or slightly exceed actual robot size (default: 0.22 m)
    - safety_margin: Extra clearance for obstacles (default: 0.1 m)
    - min_obstacle_dist: Stop if obstacle closer than this (default: 0.25 m)
    """
    def __init__(self):
        # Robot kinematic limits (adjust based on your robot specs)
        self.max_speed = 0.22           # Maximum linear velocity (m/s)
        self.min_speed = -0.1           # Minimum linear velocity (m/s) - allow slight reverse
        self.max_yaw_rate = 2.0         # Maximum angular velocity (rad/s)
        self.max_accel = 2.0            # Maximum linear acceleration (m/s^2) - increased for responsiveness
        self.max_delta_yaw_rate = 3.0   # Maximum angular acceleration (rad/s^2) - increased for sharper turns
        
        # Sampling resolution (finer = more accurate but slower)
        self.v_res = 0.01               # Linear velocity sampling resolution (m/s)
        self.yaw_res = 0.1              # Angular velocity sampling resolution (rad/s)
        
        # Prediction parameters
        self.predict_time = 2.0         # Trajectory prediction horizon (seconds)
        self.dt = 0.1                   # Time step for trajectory prediction (seconds)
        
        # Cost function weights - TUNE THESE FOR BEST PERFORMANCE
        self.heading_weight = 1.5       # Weight for heading toward goal - increased
        self.clearance_weight = 2.5     # Weight for obstacle clearance (higher = more cautious)
        self.velocity_weight = 0.5      # Weight for forward velocity - reduced to allow more rotation
        self.rotation_weight = 4.0      # Weight for rotation when path is blocked
        
        # Safety parameters
        self.robot_radius = 0.22        # Robot radius for collision checking (meters)
        self.safety_margin = 0.05       # Additional safety margin (meters) - reduced for better mobility
        self.min_obstacle_dist = 0.15   # Emergency stop distance (meters) - very close only


class DWALocalPlanner(Node):
    """DWA Local Planner Node.
    
    Subscribes to:
    - /odom: Robot odometry (position and velocity)
    - /scan: Laser scan data for obstacle detection
    
    Publishes to:
    - /cmd_vel: Velocity commands (Twist)
    """
    
    def __init__(self):
        super().__init__('dwa_local_planner')
        
        # Load configuration
        self.config = DWAConfig()
        
        # Goal and path following
        self.final_goal = np.array([2.0, 0.0])  # Final goal position
        self.global_path = None       # Global path from planner
        self.local_goal = None        # Current local waypoint to follow
        self.lookahead_dist = 1.0     # Distance to look ahead on path
        
        # Recovery/stuck detection
        self.stuck_counter = 0        # Counts control loops with low velocity
        self.stuck_threshold = 20     # Loops before considering stuck (2 seconds at 10Hz)
        self.recovery_mode = False    # Whether in recovery mode
        self.last_position = None     # Track position for stuck detection
        self.position_history = []    # Recent positions for stuck detection
        self.no_path_counter = 0      # Counts loops without valid path
        self.no_path_threshold = 30   # Loops before recovery (3 seconds)
        
        # Robot state
        self.pose = None              # Current pose [x, y, yaw]
        self.vel = [0.0, 0.0]         # Current velocity [v, w]
        self.scan = None              # Latest laser scan data
        self.scan_angle_min = None
        self.scan_angle_increment = None
        
        # ROS 2 interfaces
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Path, '/plan', self.path_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Control loop timer
        self.create_timer(self.config.dt, self.control_loop)
        
        self.get_logger().info('DWA Local Planner initialized')
        self.get_logger().info(f'Final goal: [{self.final_goal[0]:.2f}, {self.final_goal[1]:.2f}]')
        self.get_logger().info('Waiting for global path on /plan topic...')
        self.get_logger().info('Or send goal to /goal_pose to trigger planning')

    def odom_cb(self, msg):
        """Odometry callback - updates robot pose and velocity."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Extract velocities
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        
        self.pose = np.array([x, y, yaw])
        self.vel = [v, w]

    def scan_cb(self, msg):
        """Laser scan callback - updates obstacle data."""
        # Convert to numpy and handle invalid values
        ranges = np.array(msg.ranges)
        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max
        
        self.scan = ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment
        self.scan_range_max = msg.range_max
    
    def path_cb(self, msg):
        """Path callback - receives global path from planner."""
        if len(msg.poses) > 0:
            self.global_path = msg
            self.get_logger().info(
                f'Received global path with {len(msg.poses)} waypoints',
                once=True
            )
    
    def goal_cb(self, msg):
        """Goal callback - receives new goal pose."""
        self.final_goal = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])
        self.get_logger().info(f'New goal received: [{self.final_goal[0]:.2f}, {self.final_goal[1]:.2f}]')

    def control_loop(self):
        """Main control loop - runs at configured rate (default 10 Hz)."""
        # Wait for valid sensor data
        if self.pose is None or self.scan is None:
            self.get_logger().warn('Waiting for sensor data...', throttle_duration_sec=2.0)
            return
        
        # Update local goal from path or use final goal
        if self.global_path is not None and len(self.global_path.poses) > 0:
            self.local_goal = self.get_local_goal_from_path()
            self.no_path_counter = 0  # Reset counter
        else:
            # No path available
            self.no_path_counter += 1
            
            if self.no_path_counter > self.no_path_threshold:
                # No path for too long, trigger recovery
                self.get_logger().warn(
                    'No global path for 3 seconds, triggering recovery',
                    throttle_duration_sec=3.0
                )
                self.recovery_mode = True
                return
            
            # Use final goal temporarily
            self.local_goal = self.final_goal
            self.get_logger().warn(
                'No global path available, heading directly to goal',
                throttle_duration_sec=5.0
            )
        
        # Check if we've reached the final goal
        dist_to_final_goal = np.linalg.norm(self.final_goal - self.pose[:2])
        if dist_to_final_goal < 0.2:  # Goal tolerance
            self.get_logger().info('Final goal reached!', once=True)
            self.publish_velocity(0.0, 0.0)
            return
        
        # Distance to current local goal (for logging)
        dist_to_goal = np.linalg.norm(self.local_goal - self.pose[:2])
        
        # Detect if stuck (not moving much)
        self.detect_stuck()
        
        # If in recovery mode, execute recovery behavior
        if self.recovery_mode:
            self.get_logger().warn('Executing recovery behavior...', throttle_duration_sec=1.0)
            best_v, best_w = self.recovery_behavior()
            self.publish_velocity(best_v, best_w)
            return
        
        # Emergency stop if obstacle too close
        min_scan_dist = np.min(self.scan)
        if min_scan_dist < self.config.min_obstacle_dist:
            self.get_logger().warn(
                f'Emergency stop! Obstacle at {min_scan_dist:.2f}m',
                throttle_duration_sec=1.0
            )
            # Try turning in place to find a way out
            best_v, best_w = self.find_escape_maneuver()
            self.publish_velocity(best_v, best_w)
            return
        
        # Compute dynamic window based on current velocity and acceleration limits
        dw = self.calc_dynamic_window()
        
        # Find best velocity command using DWA
        best_v, best_w = self.dwa_planning(dw)
        
        # Check if forward is blocked for logging
        forward_blocked = self.is_forward_blocked()
        min_front_dist = np.min(self.scan[len(self.scan)//3:2*len(self.scan)//3]) if len(self.scan) > 0 else 999
        
        # Log the command periodically for debugging
        self.get_logger().info(
            f'Cmd: v={best_v:.2f}, w={best_w:.2f}, dist={dist_to_goal:.2f}m, front_blocked={forward_blocked}, front_dist={min_front_dist:.2f}m',
            throttle_duration_sec=1.0
        )
        
        # Publish the optimal velocity command
        self.publish_velocity(best_v, best_w)
    
    def get_local_goal_from_path(self):
        """Extract local goal from global path using lookahead distance.
        
        Returns:
            numpy array [x, y] of local goal position
        """
        if self.global_path is None or len(self.global_path.poses) == 0:
            return self.final_goal
        
        # Find the point on the path that is lookahead_dist away
        min_dist_to_path = float('inf')
        closest_idx = 0
        
        # First, find closest point on path
        for i, pose_stamped in enumerate(self.global_path.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dist = math.hypot(px - self.pose[0], py - self.pose[1])
            
            if dist < min_dist_to_path:
                min_dist_to_path = dist
                closest_idx = i
        
        # Look ahead from closest point
        for i in range(closest_idx, len(self.global_path.poses)):
            px = self.global_path.poses[i].pose.position.x
            py = self.global_path.poses[i].pose.position.y
            dist = math.hypot(px - self.pose[0], py - self.pose[1])
            
            if dist >= self.lookahead_dist:
                return np.array([px, py])
        
        # If no point is far enough, use the last point
        last_pose = self.global_path.poses[-1]
        return np.array([
            last_pose.pose.position.x,
            last_pose.pose.position.y
        ])
    
    def detect_stuck(self):
        """Detect if robot is stuck (not making progress)."""
        current_pos = self.pose[:2]
        
        # Add current position to history
        self.position_history.append(current_pos.copy())
        
        # Keep only recent history (last 2 seconds = 20 samples at 10Hz)
        if len(self.position_history) > 20:
            self.position_history.pop(0)
        
        # Check if we have enough history
        if len(self.position_history) < 20:
            return
        
        # Calculate distance traveled in last 2 seconds
        start_pos = self.position_history[0]
        end_pos = self.position_history[-1]
        distance_traveled = np.linalg.norm(end_pos - start_pos)
        
        # If moved less than 10cm in 2 seconds, consider stuck
        if distance_traveled < 0.1:
            self.stuck_counter += 1
            
            if self.stuck_counter >= self.stuck_threshold:
                self.recovery_mode = True
                self.stuck_counter = 0
                self.get_logger().warn(
                    f'STUCK DETECTED! Moved only {distance_traveled:.3f}m in 2s. Starting recovery...',
                    throttle_duration_sec=5.0
                )
        else:
            # Making progress, reset counter
            self.stuck_counter = 0
            if self.recovery_mode:
                # Check if we've moved enough to exit recovery
                if distance_traveled > 0.3:
                    self.recovery_mode = False
                    self.get_logger().info('Recovery successful! Resuming normal operation.')
    
    def recovery_behavior(self):
        """Execute recovery behavior when stuck.
        
        Strategy:
        1. Back up slowly while rotating
        2. Find direction with most clearance
        3. Move toward clearance
        
        Returns:
            Tuple of (v, w) for recovery maneuver
        """
        # Find direction with maximum clearance
        best_angle_idx = np.argmax(self.scan)
        best_clearance = self.scan[best_angle_idx]
        
        # Convert scan index to angle
        best_angle = self.scan_angle_min + best_angle_idx * self.scan_angle_increment
        
        # Check rear clearance
        rear_idx = int((math.pi - self.scan_angle_min) / self.scan_angle_increment)
        if rear_idx >= len(self.scan):
            rear_idx = len(self.scan) - 1
        rear_clearance = self.scan[rear_idx] if rear_idx >= 0 else 0.5
        
        # Strategy based on clearance
        if rear_clearance > 0.5:
            # Safe to back up
            v = -0.1  # Slow reverse
            # Rotate toward best clearance while backing
            w = np.clip(best_angle * 0.5, -1.0, 1.0)
            self.get_logger().info(
                f'Recovery: Backing up (rear clear: {rear_clearance:.2f}m)',
                throttle_duration_sec=2.0
            )
        elif best_clearance > 0.6:
            # Rotate in place toward best clearance
            v = 0.0
            w = np.clip(best_angle * 2.0, -self.config.max_yaw_rate, self.config.max_yaw_rate)
            self.get_logger().info(
                f'Recovery: Rotating toward clearance (angle: {best_angle:.2f}rad)',
                throttle_duration_sec=2.0
            )
        else:
            # Very limited clearance, just rotate slowly
            v = 0.0
            w = 0.5 if best_angle > 0 else -0.5
            self.get_logger().warn(
                'Recovery: Limited clearance, rotating slowly',
                throttle_duration_sec=2.0
            )
        
        return v, w
    
    def publish_velocity(self, v, w):
        """Publish velocity command to cmd_vel topic."""
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)
    
    def is_forward_blocked(self, check_angle=math.pi/6, check_dist=0.05):
        """Check if the forward path is blocked by obstacles.
        
        Args:
            check_angle: Angular range to check (radians, centered on front)
            check_dist: Distance threshold (meters)
            
        Returns:
            True if forward path is blocked, False otherwise
        """
        # Check laser scan readings in front of robot
        # Front is at angle 0 in robot frame
        center_idx = int((0 - self.scan_angle_min) / self.scan_angle_increment)
        angle_indices = int(check_angle / self.scan_angle_increment)
        
        start_idx = max(0, center_idx - angle_indices)
        end_idx = min(len(self.scan), center_idx + angle_indices)
        
        if start_idx < len(self.scan) and end_idx > 0:
            front_ranges = self.scan[start_idx:end_idx]
            min_front_dist = np.min(front_ranges)
            return min_front_dist < check_dist
        
        return False
    
    def find_escape_maneuver(self):
        """Find best rotation to escape from very close obstacle.
        
        Returns:
            Tuple of (v, w) for escape maneuver
        """
        # Find the direction with maximum clearance
        best_angle_idx = np.argmax(self.scan)
        best_clearance = self.scan[best_angle_idx]
        
        # Convert scan index to angle
        best_angle = self.scan_angle_min + best_angle_idx * self.scan_angle_increment
        
        # Goal angle in robot frame (use local goal if available)
        goal_to_use = self.local_goal if self.local_goal is not None else self.final_goal
        goal_angle = math.atan2(
            goal_to_use[1] - self.pose[1],
            goal_to_use[0] - self.pose[0]
        ) - self.pose[2]
        goal_angle = angle_normalize(goal_angle)
        
        # Choose between clearance direction and goal direction
        if best_clearance > 0.5:
            # If there's good clearance somewhere, turn toward it
            target_angle = best_angle
        else:
            # Otherwise try to turn toward goal
            target_angle = goal_angle
        
        # Slow forward motion + rotation toward best direction
        v = 0.05 if abs(target_angle) < math.pi / 2 else 0.0
        w = np.clip(2.0 * target_angle, -self.config.max_yaw_rate, self.config.max_yaw_rate)
        
        return v, w

    def dwa_planning(self, dw):
        """Core DWA algorithm - finds optimal velocity command.
        
        Args:
            dw: Dynamic window [v_min, v_max, w_min, w_max]
            
        Returns:
            Tuple of (best_v, best_w) - optimal velocity command
        """
        best_v, best_w = 0.0, 0.0
        best_score = -float('inf')
        collision_count = 0
        total_count = 0
        
        # Sample velocities within dynamic window
        v_min, v_max, w_min, w_max = dw
        v_samples = np.arange(v_min, v_max + 1e-6, self.config.v_res)
        w_samples = np.arange(w_min, w_max + 1e-6, self.config.yaw_res)
        
        # Evaluate each velocity combination
        for v in v_samples:
            for w in w_samples:
                total_count += 1
                
                # Predict trajectory for this velocity
                trajectory = self.predict_trajectory(v, w)
                
                # Check for collisions
                if self.is_collision(trajectory):
                    collision_count += 1
                    continue
                
                # Compute costs
                heading_cost = self.calc_heading_cost(trajectory)
                clearance_cost = self.calc_clearance_cost(trajectory)
                velocity_cost = self.calc_velocity_cost(v)
                
                # Check if forward path is blocked
                forward_blocked = self.is_forward_blocked()
                
                # If forward is blocked, strongly favor rotation
                if forward_blocked:
                    # Penalize forward motion, reward rotation
                    rotation_bonus = abs(w) / self.config.max_yaw_rate
                    forward_penalty = max(0, v) / self.config.max_speed
                    
                    total_score = (
                        self.config.heading_weight * heading_cost +
                        self.config.clearance_weight * clearance_cost +
                        self.config.rotation_weight * rotation_bonus -
                        0.5 * forward_penalty
                    )
                else:
                    # Normal operation: prefer forward motion
                    total_score = (
                        self.config.heading_weight * heading_cost +
                        self.config.clearance_weight * clearance_cost +
                        self.config.velocity_weight * velocity_cost
                    )
                
                # Update best trajectory
                if total_score > best_score:
                    best_score = total_score
                    best_v = v
                    best_w = w
        
        # Debug logging
        if best_score == -float('inf'):
            self.get_logger().warn(
                f'No valid trajectory! All {total_count} samples collided. DW: v=[{v_min:.2f}, {v_max:.2f}], w=[{w_min:.2f}, {w_max:.2f}]',
                throttle_duration_sec=2.0
            )
        
        return best_v, best_w
    
    def calc_dynamic_window(self):
        """Calculate dynamic window of feasible velocities.
        
        The dynamic window considers:
        1. Robot's kinematic limits (max speed, max yaw rate)
        2. Current velocity and acceleration constraints
        
        Returns:
            List [v_min, v_max, w_min, w_max]
        """
        v_current, w_current = self.vel
        
        # Velocity limits based on acceleration constraints
        v_min = max(self.config.min_speed, v_current - self.config.max_accel * self.config.dt)
        v_max = min(self.config.max_speed, v_current + self.config.max_accel * self.config.dt)
        w_min = max(-self.config.max_yaw_rate, w_current - self.config.max_delta_yaw_rate * self.config.dt)
        w_max = min(self.config.max_yaw_rate, w_current + self.config.max_delta_yaw_rate * self.config.dt)
        
        return [v_min, v_max, w_min, w_max]

    def predict_trajectory(self, v, w):
        """Predict robot trajectory for given velocities.
        
        Uses simple Euler integration for differential drive kinematics.
        
        Args:
            v: Linear velocity (m/s)
            w: Angular velocity (rad/s)
            
        Returns:
            numpy array of shape (n_steps, 3) containing [x, y, yaw] at each step
        """
        trajectory = []
        x, y, yaw = self.pose
        t = 0.0
        
        while t <= self.config.predict_time:
            # Differential drive kinematics (Euler integration)
            x += v * math.cos(yaw) * self.config.dt
            y += v * math.sin(yaw) * self.config.dt
            yaw += w * self.config.dt
            yaw = angle_normalize(yaw)  # Keep yaw in [-pi, pi]
            
            trajectory.append([x, y, yaw])
            t += self.config.dt
        
        return np.array(trajectory)
    
    def is_collision(self, trajectory):
        """Check if trajectory collides with obstacles.
        
        For each point in the trajectory, checks multiple points around the robot's
        footprint against laser scan data.
        
        Args:
            trajectory: Array of shape (n_steps, 3) containing [x, y, yaw]
            
        Returns:
            True if collision detected, False otherwise
        """
        robot_radius = self.config.robot_radius + self.config.safety_margin
        
        # Sample fewer points for performance (every 3rd step)
        step = max(1, len(trajectory) // 5)
        
        # Check multiple points along the trajectory
        for i in range(0, len(trajectory), step):
            pose = trajectory[i]
            
            # Check center and front of robot
            check_points = [
                pose[:2],  # Center
                pose[:2] + robot_radius * np.array([math.cos(pose[2]), math.sin(pose[2])]),  # Front
            ]
            
            for point in check_points:
                # Transform point to robot's current frame
                dx = point[0] - self.pose[0]
                dy = point[1] - self.pose[1]
                
                # Distance and angle from robot to this point
                dist = math.hypot(dx, dy)
                
                # Skip if checking the robot's current position
                if dist < 0.01:
                    continue
                    
                angle = math.atan2(dy, dx) - self.pose[2]  # Relative to robot's heading
                angle = angle_normalize(angle)
                
                # Find corresponding laser scan index
                scan_idx = int((angle - self.scan_angle_min) / self.scan_angle_increment)
                
                # Only check if within scan range
                if 0 <= scan_idx < len(self.scan):
                    # Check if obstacle is closer than predicted position
                    if self.scan[scan_idx] < dist:
                        return True
        
        return False
    
    def calc_heading_cost(self, trajectory):
        """Calculate cost based on heading toward goal.
        
        Rewards trajectories that end closer to goal and point toward it.
        Also considers current position for better immediate goal alignment.
        
        Args:
            trajectory: Array of shape (n_steps, 3) containing [x, y, yaw]
            
        Returns:
            Normalized cost value (higher is better)
        """
        # Get final position from trajectory
        final_pos = trajectory[-1, :2]
        final_yaw = trajectory[-1, 2]
        
        # Use local goal for heading calculation
        goal_to_use = self.local_goal if self.local_goal is not None else self.final_goal
        
        # Distance from final position to goal
        to_goal = goal_to_use - final_pos
        dist_to_goal = np.linalg.norm(to_goal)
        
        # Angle to goal from final position
        angle_to_goal = math.atan2(to_goal[1], to_goal[0])
        heading_error = abs(angle_normalize(angle_to_goal - final_yaw))
        
        # Also check progress from current position
        current_dist_to_goal = np.linalg.norm(goal_to_use - self.pose[:2])
        progress = max(0, current_dist_to_goal - dist_to_goal)
        
        # Cost components
        dist_cost = 1.0 - min(dist_to_goal / 10.0, 1.0)  # Closer is better
        heading_cost = 1.0 - (heading_error / math.pi)   # Better alignment is better
        progress_cost = min(progress * 2.0, 1.0)         # Making progress is good
        
        return dist_cost + heading_cost + progress_cost
    
    def calc_clearance_cost(self, trajectory):
        """Calculate cost based on clearance from obstacles.
        
        Rewards trajectories that maintain distance from obstacles.
        Uses an exponential cost to strongly penalize close approaches.
        
        Args:
            trajectory: Array of shape (n_steps, 3) containing [x, y, yaw]
            
        Returns:
            Normalized cost value (higher is better)
        """
        min_clearance = float('inf')
        
        # Sample points along trajectory to check clearance
        step = max(1, len(trajectory) // 10)
        
        for i in range(0, len(trajectory), step):
            pose = trajectory[i]
            
            # Check multiple angles around the robot at this pose
            for angle_offset in [-math.pi/4, 0, math.pi/4]:  # Check left, front, right
                check_angle = pose[2] + angle_offset
                
                # Point to check
                check_x = pose[0] + self.config.robot_radius * math.cos(check_angle)
                check_y = pose[1] + self.config.robot_radius * math.sin(check_angle)
                
                # Transform to robot's current frame
                dx = check_x - self.pose[0]
                dy = check_y - self.pose[1]
                
                dist = math.hypot(dx, dy)
                if dist < 0.01:
                    continue
                    
                angle = math.atan2(dy, dx) - self.pose[2]
                angle = angle_normalize(angle)
                
                # Find laser scan reading
                scan_idx = int((angle - self.scan_angle_min) / self.scan_angle_increment)
                
                if 0 <= scan_idx < len(self.scan):
                    # Clearance is obstacle distance minus trajectory distance
                    clearance = self.scan[scan_idx] - dist
                    min_clearance = min(min_clearance, clearance)
        
        # If no valid clearance found, return high clearance (no obstacles seen)
        if min_clearance == float('inf'):
            return 1.0
        
        # Exponential cost: strongly rewards larger clearance near obstacles
        # 0.3m clearance = 0.5 cost, 0.6m+ = 1.0 cost
        if min_clearance < 0:
            return 0.0  # Collision
        else:
            # Exponential reward for clearance
            return min(1.0, 1.0 - math.exp(-2.0 * min_clearance))
    
    def calc_velocity_cost(self, v):
        """Calculate cost based on velocity.
        
        Rewards faster forward motion.
        
        Args:
            v: Linear velocity (m/s)
            
        Returns:
            Normalized cost value (higher is better)
        """
        # Normalize by max speed
        return v / self.config.max_speed

def main(args=None):
    rclpy.init(args=args)
    node = DWALocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
