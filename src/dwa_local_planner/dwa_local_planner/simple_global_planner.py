#!/usr/bin/env python3
"""Simple A* Global Planner for DWA.

This node creates a global path from robot position to goal using A* algorithm
on an occupancy grid built from laser scans.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from collections import deque


class SimpleGlobalPlanner(Node):
    """Simple A* global planner node."""
    
    def __init__(self):
        super().__init__('simple_global_planner')
        
        # State
        self.robot_pose = None
        self.goal_pose = None
        self.scan = None
        self.scan_angle_min = None
        self.scan_angle_increment = None
        
        # Occupancy grid parameters
        self.grid_resolution = 0.05  # meters per cell
        self.grid_size = 200  # 200x200 cells = 10x10 meters
        self.grid = np.zeros((self.grid_size, self.grid_size))
        self.grid_origin = [-5.0, -5.0]  # Bottom-left corner in world frame
        
        # Robot dimensions (TurtleBot3 Waffle Pi)
        self.robot_radius = 0.22  # meters
        self.safety_distance = 0.35  # Additional safe distance from obstacles
        self.inflation_radius = self.robot_radius + self.safety_distance  # Total inflation
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/costmap', 10)
        
        # Timer to replan
        self.create_timer(5.0, self.plan_path)  # Replan global path
        
        self.get_logger().info('Simple Global Planner started')
        self.get_logger().info('Send goal to /goal_pose to trigger planning')
    
    def odom_cb(self, msg):
        """Odometry callback."""
        self.robot_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quat_to_yaw(msg.pose.pose.orientation)
        ]
    
    def scan_cb(self, msg):
        """Laser scan callback."""
        ranges = np.array(msg.ranges)
        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max
        
        self.scan = ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment
        
        # Update occupancy grid
        self.update_grid()
    
    def goal_cb(self, msg):
        """Goal pose callback."""
        self.goal_pose = [
            msg.pose.position.x,
            msg.pose.position.y
        ]
        self.get_logger().info(f'Goal received: [{self.goal_pose[0]:.2f}, {self.goal_pose[1]:.2f}]')
        
        # Plan immediately
        self.plan_path()
    
    def quat_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices."""
        gx = int((x - self.grid_origin[0]) / self.grid_resolution)
        gy = int((y - self.grid_origin[1]) / self.grid_resolution)
        return gx, gy
    
    def grid_to_world(self, gx, gy):
        """Convert grid indices to world coordinates."""
        x = self.grid_origin[0] + (gx + 0.5) * self.grid_resolution
        y = self.grid_origin[1] + (gy + 0.5) * self.grid_resolution
        return x, y
    
    def update_grid(self):
        """Update occupancy grid from laser scan."""
        if self.robot_pose is None or self.scan is None:
            return
        
        # Clear old grid (with decay)
        self.grid *= 0.9
        
        # Add obstacles from scan
        rx, ry, ryaw = self.robot_pose
        
        for i, range_val in enumerate(self.scan):
            if range_val < 0.1 or range_val > 5.0:
                continue
            
            # Obstacle position in world frame
            angle = self.scan_angle_min + i * self.scan_angle_increment + ryaw
            ox = rx + range_val * math.cos(angle)
            oy = ry + range_val * math.sin(angle)
            
            # Convert to grid
            gx, gy = self.world_to_grid(ox, oy)
            
            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                # Mark obstacle and inflate based on robot dimensions
                inflation_cells = int(self.inflation_radius / self.grid_resolution)
                
                for dx in range(-inflation_cells, inflation_cells + 1):
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                            dist = math.hypot(dx, dy) * self.grid_resolution  # Distance in meters
                            
                            if dist <= self.robot_radius:
                                # Lethal obstacle (robot footprint)
                                self.grid[ny, nx] = 100
                            elif dist <= self.inflation_radius:
                                # Inflated cost (proportional to distance)
                                cost = 99 * (1.0 - (dist - self.robot_radius) / self.safety_distance)
                                self.grid[ny, nx] = max(self.grid[ny, nx], cost)
    
    def plan_path(self):
        """Plan path using A* algorithm."""
        if self.robot_pose is None or self.goal_pose is None:
            return
        
        # Convert to grid coordinates
        start_gx, start_gy = self.world_to_grid(self.robot_pose[0], self.robot_pose[1])
        goal_gx, goal_gy = self.world_to_grid(self.goal_pose[0], self.goal_pose[1])
        
        # Check bounds
        if not (0 <= start_gx < self.grid_size and 0 <= start_gy < self.grid_size):
            self.get_logger().warn('Start position out of grid bounds')
            return
        if not (0 <= goal_gx < self.grid_size and 0 <= goal_gy < self.grid_size):
            self.get_logger().warn('Goal position out of grid bounds')
            return
        
        # A* search
        path_grid = self.astar(start_gx, start_gy, goal_gx, goal_gy)
        
        if path_grid is None:
            self.get_logger().warn('No path found!')
            return
        
        # Convert to Path message
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for gx, gy in path_grid:
            x, y = self.grid_to_world(gx, gy)
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} waypoints')
    
    def astar(self, start_x, start_y, goal_x, goal_y):
        """A* path planning algorithm."""
        # Priority queue: (f_cost, g_cost, x, y)
        from heapq import heappush, heappop
        
        open_set = []
        heappush(open_set, (0, 0, start_x, start_y))
        
        came_from = {}
        g_score = {(start_x, start_y): 0}
        
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0),
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        while open_set:
            _, g, x, y = heappop(open_set)
            
            if x == goal_x and y == goal_y:
                # Reconstruct path
                path = []
                current = (x, y)
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((start_x, start_y))
                return path[::-1]
            
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                
                # Check bounds
                if not (0 <= nx < self.grid_size and 0 <= ny < self.grid_size):
                    continue
                
                # Check obstacle (lethal)
                if self.grid[ny, nx] >= 99:
                    continue
                
                # Calculate cost with strong penalty for high-cost areas
                move_cost = math.hypot(dx, dy)
                # Exponential cost increase for inflated areas
                obstacle_cost = self.grid[ny, nx] * 0.1  # Strongly penalize obstacle proximity
                tentative_g = g + move_cost + obstacle_cost
                
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = tentative_g
                    h = math.hypot(nx - goal_x, ny - goal_y)
                    f = tentative_g + h
                    heappush(open_set, (f, tentative_g, nx, ny))
                    came_from[(nx, ny)] = (x, y)
        
        return None


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGlobalPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
