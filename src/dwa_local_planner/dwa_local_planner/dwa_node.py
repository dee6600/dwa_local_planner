#!/usr/bin/env python3
"""
Simple DWA local planner (ROS 2, rclpy)

- Subscribes:
    /odom  (nav_msgs/Odometry)
    /scan  (sensor_msgs/LaserScan)
- Publishes:
    /cmd_vel            (geometry_msgs/Twist)
    /dwa/trajectories   (visualization_msgs/Marker)  -> all sampled trajectories (faint)
    /dwa/best_trajectory (visualization_msgs/Marker) -> chosen trajectory (bright)
Parameters (declared - tune with ros2 param set):
    max_vel_x, min_vel_x, max_yaw_rate, max_acc_x, max_acc_yaw,
    v_resolution, omega_resolution, predict_time, dt,
    robot_radius, obstacle_inflation, min_obstacle_dist,
    to_goal_cost_gain, speed_cost_gain, obstacle_cost_gain,
    control_rate, goal_x, goal_y, goal_tolerance
"""
import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion

# ---------- Utility state container ----------
class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.omega = omega

# ---------- DWA Node ----------
class SimpleDWA(Node):
    def __init__(self):
        super().__init__('simple_dwa')

        # subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_pub = self.create_publisher(Marker, '/dwa/trajectories', 10)
        self.best_pub = self.create_publisher(Marker, '/dwa/best_trajectory', 10)

        # declare tunable parameters (defaults chosen comparable to TurtleBot)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_vel_x', 0.18),                    # maximum forward speed (m/s)
                ('min_vel_x', -0.05),                   # minimum speed / reverse limit (m/s)
                ('max_yaw_rate', 1.0),                  # maximum rotation rate (rad/s)
                ('max_acc_x', 0.2),                     # maximum linear acceleration (m/s²)
                ('max_acc_yaw', 0.5),                   # maximum angular acceleration (rad/s²)
                ('v_resolution', 0.02),                 # linear velocity sampling step (m/s)
                ('omega_resolution', 0.1),              # angular velocity sampling step (rad/s)
                ('predict_time', 1.8),                  # trajectory lookahead horizon (s)
                ('dt', 0.1),                            # integration timestep (s)
                ('robot_radius', 0.22),                 # robot body radius (m)
                ('obstacle_inflation', 0.05),           # safety margin around obstacles (m)
                ('min_obstacle_dist', 0.05),            # minimum safe clearance distance (m)
                ('to_goal_cost_gain', 0.5),             # goal attraction weight (higher = prioritize goal)
                ('speed_cost_gain', 0.5),               # speed preference weight (higher = prefer faster)
                ('obstacle_cost_gain', 2.0),            # obstacle avoidance weight (higher = stronger avoidance)
                ('control_rate', 10.0),                 # control loop frequency (Hz)
                ('goal_x', 2.0),                        # goal x-coordinate (m, in odom frame)
                ('goal_y', 1.0),                        # goal y-coordinate (m, in odom frame)
                ('goal_tolerance', 0.12),               # distance to goal considered reached (m)
            ])

        # read params into locals
        self.read_params()

        # internal state and sensor data
        self.state = State()                    # current robot pose (x, y, yaw) and velocities (v, omega)
        self.scan_msg = None                    # latest LaserScan message from /scan subscription
        self.scan_pts_world = []                # precomputed scan points in world (odom) frame; transformed each time /scan arrives
        self.goal = (self.goal_x, self.goal_y) # target goal position (x, y) updated when goal_x/goal_y params change
        self.goal_reached = False               # flag indicating whether robot has reached the goal

        # control timer
        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Simple DWA node started.')

    def read_params(self):
        """
        Fetch all ROS2 parameters and convert to instance variables.
        Called at startup and in each control loop to support dynamic reconfiguration.
        """
        p = self.get_parameters([
            'max_vel_x', 'min_vel_x', 'max_yaw_rate', 'max_acc_x', 'max_acc_yaw',
            'v_resolution', 'omega_resolution', 'predict_time', 'dt', 'robot_radius',
            'obstacle_inflation', 'min_obstacle_dist', 'to_goal_cost_gain', 'speed_cost_gain',
            'obstacle_cost_gain', 'control_rate', 'goal_x', 'goal_y', 'goal_tolerance'
        ])
        d = {x.name: x.value for x in p}

        self.max_vel_x = float(d['max_vel_x'])              # maximum forward speed (m/s)
        self.min_vel_x = float(d['min_vel_x'])              # minimum speed / reverse limit (m/s)
        self.max_yaw_rate = float(d['max_yaw_rate'])        # maximum rotation rate (rad/s)
        self.max_acc_x = float(d['max_acc_x'])              # maximum linear acceleration (m/s²)
        self.max_acc_yaw = float(d['max_acc_yaw'])          # maximum angular acceleration (rad/s²)
        self.v_resolution = float(d['v_resolution'])        # linear velocity sampling step (m/s)
        self.omega_resolution = float(d['omega_resolution']) # angular velocity sampling step (rad/s)
        self.predict_time = float(d['predict_time'])        # trajectory lookahead horizon (s)
        self.dt = float(d['dt'])                            # integration timestep (s)
        self.robot_radius = float(d['robot_radius'])        # robot body radius (m)
        self.obstacle_inflation = float(d['obstacle_inflation']) # safety margin around obstacles (m)
        self.min_obstacle_dist = float(d['min_obstacle_dist']) # minimum safe clearance distance (m)
        self.to_goal_cost_gain = float(d['to_goal_cost_gain'])       # goal attraction weight
        self.speed_cost_gain = float(d['speed_cost_gain'])           # speed preference weight
        self.obstacle_cost_gain = float(d['obstacle_cost_gain'])     # obstacle avoidance weight
        self.control_rate = float(d['control_rate'])        # control loop frequency (Hz)
        self.goal_x = float(d['goal_x'])                    # goal x-coordinate (m, in odom frame)
        self.goal_y = float(d['goal_y'])                    # goal y-coordinate (m, in odom frame)
        self.goal_tolerance = float(d['goal_tolerance'])    # distance to goal considered reached (m)

    # ---------- callbacks ----------
    def odom_cb(self, msg: Odometry):
        # update robot pose & velocities
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        self.state.x = p.x
        self.state.y = p.y
        self.state.yaw = yaw
        self.state.v = msg.twist.twist.linear.x
        self.state.omega = msg.twist.twist.angular.z

    def scan_cb(self, msg: LaserScan):
        # store scan and precompute scan points in world coordinates using current odom (important!)
        self.scan_msg = msg
        self.scan_pts_world = []  # reset
        if self.state is None:
            return

        angle = msg.angle_min
        sx = self.state.x
        sy = self.state.y
        syaw = self.state.yaw

        for r in msg.ranges:
            # skip invalid ranges
            if r is None or math.isinf(r) or math.isnan(r) or r <= 0.0:
                angle += msg.angle_increment
                continue
            # point in robot frame
            px_r = r * math.cos(angle)
            py_r = r * math.sin(angle)
            # transform to world (odom) frame using current robot pose
            px_w = sx + math.cos(syaw) * px_r - math.sin(syaw) * py_r
            py_w = sy + math.sin(syaw) * px_r + math.cos(syaw) * py_r
            self.scan_pts_world.append((px_w, py_w, r))
            angle += msg.angle_increment

    # ---------- control loop ----------
    def control_loop(self):
        # refresh parameters in-case changed at runtime
        self.read_params()

        # if data missing or goal reached, stop
        if self.scan_msg is None or self.state is None:
            return

        if self.is_goal_reached():
            if not self.goal_reached:
                self.get_logger().info(f'Goal reached: ({self.goal_x:.2f}, {self.goal_y:.2f})')
                self.goal_reached = True
            self.publish_cmd(0.0, 0.0)
            return

        # compute dynamic window
        dw = self.calc_dynamic_window()

        # search best trajectory
        best_u, all_trajs = self.search_best_trajectory(dw)

        # visualize sampled trajectories and chosen trajectory
        self.publish_trajectories_marker(all_trajs, best_u)

        if best_u is None:
            # no valid trajectory -> stop (could add recovery later)
            self.get_logger().warn('No valid trajectory found; stopping.')
            self.publish_cmd(0.0, 0.0)
            return

        v, omega = best_u
        self.publish_cmd(v, omega)

    def is_goal_reached(self):
        dx = self.goal_x - self.state.x
        dy = self.goal_y - self.state.y
        return math.hypot(dx, dy) <= self.goal_tolerance

    # ---------- DWA supporting functions ----------
    def calc_dynamic_window(self):
        # base limits
        min_v = self.min_vel_x
        max_v = self.max_vel_x
        min_omega = -self.max_yaw_rate
        max_omega = self.max_yaw_rate

        # dynamic limits based on current speed & accelerations over dt (control timestep)
        max_v_dyn = self.state.v + self.max_acc_x * self.dt
        min_v_dyn = self.state.v - self.max_acc_x * self.dt
        max_omega_dyn = self.state.omega + self.max_acc_yaw * self.dt
        min_omega_dyn = self.state.omega - self.max_acc_yaw * self.dt

        dw_min_v = max(min_v, min_v_dyn)
        dw_max_v = min(max_v, max_v_dyn)
        dw_min_omega = max(min_omega, min_omega_dyn)
        dw_max_omega = min(max_omega, max_omega_dyn)

        # guard
        if dw_min_v > dw_max_v:
            dw_min_v, dw_max_v = min_v, max_v
        if dw_min_omega > dw_max_omega:
            dw_min_omega, dw_max_omega = -self.max_yaw_rate, self.max_yaw_rate

        return (dw_min_v, dw_max_v, dw_min_omega, dw_max_omega)

    def generate_trajectory(self, v, omega):
        # forward-simulate simple unicycle for predict_time using fixed (v,omega)
        x = self.state.x
        y = self.state.y
        yaw = self.state.yaw
        traj = []
        t = 0.0
        while t <= self.predict_time:
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += omega * self.dt
            traj.append((x, y, yaw))
            t += self.dt
        return traj

    def calc_obstacle_cost(self, traj):
        """
        Return an obstacle cost (lower is better). If any pose collides (within inflated radius),
        return None to mark trajectory invalid.
        """
        if not self.scan_pts_world:
            # no scan data: be conservative but treat it valid with small penalty
            return 1.0

        inflation = self.robot_radius + self.obstacle_inflation
        inflation_sq = inflation * inflation
        min_dist_sq = float('inf')

        # For each pose in trajectory, check distance to each scan point (in world coords)
        for (x_t, y_t, _) in traj:
            for (px_w, py_w, _) in self.scan_pts_world:
                dx = px_w - x_t
                dy = py_w - y_t
                d2 = dx * dx + dy * dy
                if d2 < inflation_sq:
                    return None  # collision
                if d2 < min_dist_sq:
                    min_dist_sq = d2

        if min_dist_sq == float('inf'):
            return 0.0
        min_dist = math.sqrt(min_dist_sq)
        # obstacle cost as inverse of clearance (bigger clearance -> smaller cost)
        return 1.0 / (max(min_dist - self.obstacle_inflation, 1e-6))

    def calc_to_goal_cost(self, traj):
        # distance from last pose to goal
        if not traj:
            return float('inf')
        last = traj[-1]
        dx = self.goal_x - last[0]
        dy = self.goal_y - last[1]
        return math.hypot(dx, dy)

    def search_best_trajectory(self, dw):
        (v_min, v_max, om_min, om_max) = dw

        # sampling counts
        v_steps = max(1, int(math.ceil((v_max - v_min) / self.v_resolution)))
        om_steps = max(1, int(math.ceil((om_max - om_min) / self.omega_resolution)))

        best_score = -float('inf')
        best_u = None
        all_trajs = []  # list of tuples (v, omega, traj, valid, cost)

        for i in range(v_steps + 1):
            v = v_min + i * (v_max - v_min) / max(1, v_steps)
            for j in range(om_steps + 1):
                omega = om_min + j * (om_max - om_min) / max(1, om_steps)
                traj = self.generate_trajectory(v, omega)

                # obstacle check
                ob_cost = self.calc_obstacle_cost(traj)
                if ob_cost is None:
                    all_trajs.append((v, omega, traj, False, float('inf')))
                    continue

                to_goal = self.calc_to_goal_cost(traj)
                speed_score = v  # prefer higher speed
                # combine into score (higher better)
                score = (self.to_goal_cost_gain * (1.0 / (1.0 + to_goal))
                         + self.speed_cost_gain * speed_score
                         - self.obstacle_cost_gain * ob_cost)

                all_trajs.append((v, omega, traj, True, score))

                if score > best_score:
                    best_score = score
                    best_u = (v, omega)

        return best_u, all_trajs

    # ---------- visualization ----------
    def publish_trajectories_marker(self, all_trajs, best_u):
        # Publish all sampled trajectories as faint lines (frame: odom)
        # and best trajectory as brighter line.
        # All markers use frame_id 'odom' so they line up with odometry.
        header_frame = 'odom'

        # All-samples marker (LINE_LIST): each small segment is added consecutively
        all_marker = Marker()
        all_marker.header.frame_id = header_frame
        all_marker.header.stamp = self.get_clock().now().to_msg()
        all_marker.ns = 'dwa_all'
        all_marker.id = 0
        all_marker.type = Marker.LINE_LIST
        all_marker.action = Marker.ADD
        all_marker.scale.x = 0.02  # line width
        all_marker.color.r = 0.6
        all_marker.color.g = 0.6
        all_marker.color.b = 0.6
        all_marker.color.a = 0.35

        # Best trajectory marker (LINE_STRIP)
        best_marker = Marker()
        best_marker.header.frame_id = header_frame
        best_marker.header.stamp = self.get_clock().now().to_msg()
        best_marker.ns = 'dwa_best'
        best_marker.id = 1
        best_marker.type = Marker.LINE_STRIP
        best_marker.action = Marker.ADD
        best_marker.scale.x = 0.05
        best_marker.color.r = 0.0
        best_marker.color.g = 1.0
        best_marker.color.b = 0.0
        best_marker.color.a = 0.9

        # optional: endpoint of best trajectory (sphere)
        endpoint_marker = Marker()
        endpoint_marker.header.frame_id = header_frame
        endpoint_marker.header.stamp = self.get_clock().now().to_msg()
        endpoint_marker.ns = 'dwa_endpoint'
        endpoint_marker.id = 2
        endpoint_marker.type = Marker.SPHERE
        endpoint_marker.action = Marker.ADD
        endpoint_marker.scale.x = 0.08
        endpoint_marker.scale.y = 0.08
        endpoint_marker.scale.z = 0.08
        endpoint_marker.color.r = 1.0
        endpoint_marker.color.g = 0.4
        endpoint_marker.color.b = 0.0
        endpoint_marker.color.a = 0.9

        # Build markers
        best_traj_points = None
        for (v, omega, traj, valid, score) in all_trajs:
            # Draw each trajectory as many small segments in LINE_LIST
            if len(traj) < 2:
                continue
            # choose color brightness based on validity (we use alpha only through marker)
            for k in range(len(traj) - 1):
                p1 = traj[k]
                p2 = traj[k + 1]
                pt1 = Point(x=p1[0], y=p1[1], z=0.02)
                pt2 = Point(x=p2[0], y=p2[1], z=0.02)
                all_marker.points.append(pt1)
                all_marker.points.append(pt2)

            # if this is best trajectory, store for drawing a LINE_STRIP
            if best_u is not None and math.isclose(v, best_u[0], rel_tol=1e-6) and math.isclose(omega, best_u[1], rel_tol=1e-6):
                best_traj_points = traj

        # Fill best_marker if found
        if best_traj_points is not None:
            for p in best_traj_points:
                best_marker.points.append(Point(x=p[0], y=p[1], z=0.03))
            # endpoint
            end = best_traj_points[-1]
            endpoint_marker.pose.position = Point(x=end[0], y=end[1], z=0.03)
        else:
            # empty markers (no valid best) - give a tiny empty point to avoid RViz warnings
            all_marker.points = all_marker.points[:2000]  # safety trim

        # publish
        self.traj_pub.publish(all_marker)
        self.best_pub.publish(best_marker)
        # endpoint as separate marker (helps identify where it's heading)
        self.best_pub.publish(endpoint_marker)

    # ---------- I/O ----------
    def publish_cmd(self, v, omega):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(omega)
        self.cmd_pub.publish(t)


# ---------- main ----------
def main(args=None):
    rclpy.init(args=args)
    node = SimpleDWA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down DWA node')
    finally:
        # stop robot before exit
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
