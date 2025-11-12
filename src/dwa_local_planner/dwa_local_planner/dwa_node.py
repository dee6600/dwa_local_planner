#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import time

def angle_normalize(x):
    return math.atan2(math.sin(x), math.cos(x))

class DWALocalPlanner(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')
        # robot limits
        self.max_speed = 0.26        # m/s
        self.min_speed = -0.05
        self.max_yaw_rate = 1.0     # rad/s
        self.max_accel = 0.5        # m/s^2
        self.max_delta_yaw_rate = 1.0

        # DWA params
        self.dt = 0.1               # controller period (s)
        self.predict_time = 1.5     # how far to simulate (s)
        self.v_res = 0.02
        self.yaw_res = 0.1

        # goal (hardcoded for demo; change as needed)
        self.goal = np.array([2.0, 0.0])

        # state
        self.pose = None  # [x,y,yaw]
        self.vel = [0.0, 0.0]  # [v, w]
        self.scan = None

        # subs / pubs
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('DWA local planner started')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # velocity (linear.x, angular.z)
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.pose = np.array([x, y, yaw])
        self.vel = [v, w]

    def scan_cb(self, msg):
        # convert to numpy and handle inf
        ranges = np.array(msg.ranges)
        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max
        self.scan = ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment
        self.scan_range_max = msg.range_max

    def control_loop(self):
        if self.pose is None or self.scan is None:
            return
        # compute dynamic window from current velocity
        dw = self.calc_dynamic_window()
        best_u = [0.0, 0.0]
        best_score = -1e9

        # sample velocities in dynamic window
        v_min, v_max, w_min, w_max = dw
        v_samples = np.arange(v_min, v_max + 1e-6, self.v_res)
        w_samples = np.arange(w_min, w_max + 1e-6, self.yaw_res)

        for v in v_samples:
            for w in w_samples:
                traj = self.predict_trajectory(self.pose, v, w)
                if not self.check_collision(traj):
                    to_goal_cost = self.calc_to_goal_cost(traj)
                    speed_cost = v  # prefer faster
                    score = (1.0 * to_goal_cost) + (0.1 * speed_cost)
                    if score > best_score:
                        best_score = score
                        best_u = [v, w]

        # publish best command
        twist = Twist()
        twist.linear.x = float(best_u[0])
        twist.angular.z = float(best_u[1])
        self.cmd_pub.publish(twist)

    def calc_dynamic_window(self):
        # [v_min, v_max, w_min, w_max]
        v = self.vel[0]
        w = self.vel[1]
        v_min = max(self.min_speed, v - self.max_accel * self.dt)
        v_max = min(self.max_speed, v + self.max_accel * self.dt)
        w_min = max(-self.max_yaw_rate, w - self.max_delta_yaw_rate * self.dt)
        w_max = min(self.max_yaw_rate, w + self.max_delta_yaw_rate * self.dt)
        return [v_min, v_max, w_min, w_max]

    def predict_trajectory(self, init_pose, v, w):
        t = 0.0
        pose = np.array(init_pose)
        traj = []
        while t <= self.predict_time:
            # simple differential drive motion integration
            x, y, yaw = pose
            if abs(w) < 1e-5:
                x += v * math.cos(yaw) * self.dt
                y += v * math.sin(yaw) * self.dt
            else:
                x += v * math.cos(yaw) * self.dt
                y += v * math.sin(yaw) * self.dt
                yaw += w * self.dt
            pose = np.array([x, y, yaw])
            traj.append(pose.copy())
            t += self.dt
        return np.array(traj)

    def check_collision(self, traj):
        # For each pose in trajectory, check the nearest laser range in that direction
        # Simple conservative collision check: if robot's x,y is within some range of an obstacle measured by scan
        robot_radius = 0.18  # turtlebot3 radius (approx)
        # For each step in predicted trajectory
        for p in traj:
            x, y, yaw = p
            # compute angle and distance from robot's current pose to this point
            dx = x - self.pose[0]
            dy = y - self.pose[1]
            dist = math.hypot(dx, dy)
            # find scan index corresponding to angle
            angle = angle_normalize(math.atan2(dy, dx))
            idx = int((angle - self.scan_angle_min) / self.scan_angle_increment)
            if idx < 0 or idx >= len(self.scan):
                # out of range of scan -> be conservative and say collision
                return True
            if self.scan[idx] <= (dist - robot_radius):
                # obstacle too close along this direction
                return True
        return False

    def calc_to_goal_cost(self, traj):
        # smaller is better: negative distance so we maximize
        last = traj[-1]
        dx = self.goal[0] - last[0]
        dy = self.goal[1] - last[1]
        dist = math.hypot(dx, dy)
        # invert to make higher score -> better, but keep scaling
        return -dist

def main(args=None):
    rclpy.init(args=args)
    node = DWALocalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
