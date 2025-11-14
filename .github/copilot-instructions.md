## DWA Local Planner — Copilot Instructions

**Purpose:** Minimal, actionable guidance for AI coding agents to be productive in this ROS2 ament_python project.

### Repo Overview
- **Type:** ROS2 workspace with single ament_python package at `src/dwa_local_planner`.
- **Language:** Python 3 (rclpy, geometry_msgs, nav_msgs, sensor_msgs).
- **Architecture:** Dynamic Window Approach (DWA) local planner as a single-file node (`dwa_node.py`).

### Quick Commands (run from workspace root)
```bash
colcon build --packages-select dwa_local_planner
source install/setup.bash  # or install/local_setup.bash for session-local
ros2 run dwa_local_planner dwa_node
colcon test --packages-select dwa_local_planner && colcon test-result --verbose
```

### Big Picture Architecture

**Data Flow:**
1. **Sensors:** Subscribe to `/odom` (Odometry) and `/scan` (LaserScan) in robot frame
2. **Planning:** Sample candidate (speed, turn_rate) pairs; forward-simulate trajectories using unicycle kinematics (Euler integration)
3. **Scoring:** Evaluate each trajectory on goal distance, heading, clearance from obstacles, and smoothness
4. **Output:** Publish best velocity command to `/cmd_vel` (Twist)

**Design Rationale:** Single-file implementation for clarity and experimentation. All tunable parameters are exposed as ROS2 parameters (declared and read in `__init__`).

### Where to Look First

**`src/dwa_local_planner/dwa_local_planner/dwa_node.py`** — Core planner (only ~450 lines):
- **`class DWAPlannerNode(Node)`:** Main entry point; uses `create_subscription`, `create_publisher`, `create_timer`.
- **`scan_cb` method:** Transforms laser scans (robot frame) to world coordinates. Replaces NaN/Inf with valid range data.
- **`predict_motion` method:** Unicycle kinematics + Euler integration over `lookahead_steps` to simulate candidate trajectories.
- **`check_for_collisions` method:** Compares trajectory path to precomputed world-frame scan points; returns infinity cost on collision (or inverse clearance otherwise).
- **`control_loop` method:** Main callback; samples velocities, simulates, scores, and publishes best command.

**Key Parameters** (all ROS2 params, tunable at runtime or via launch file):
- `max_speed`, `max_turn`: Robot kinematic limits
- `num_samples`: How many (v, ω) pairs to try per cycle (default: 200)
- `goal_x`, `goal_y`: Target position
- `goal_weight`, `heading_weight`, `smoothness_weight`, `obstacle_weight`: Scoring function coefficients
- `robot_radius`, `safety_margin`: Inflation for collision checks

### Developer Workflows & Debugging

**Fast iteration:**
```bash
colcon build --packages-select dwa_local_planner --symlink-install
```

**Inspect sensor data:**
```bash
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /cmd_vel
```

**Publish synthetic test messages:**
```bash
ros2 topic pub --once /scan sensor_msgs/msg/LaserScan "{ranges: [1.0, 1.0, ...]}"
```

**Run tests & linting:** Tests live in `src/dwa_local_planner/test/` (flake8, pep257 stubs). Status logged to stdout at 10 Hz (`control_loop`) and 1 Hz (`log_status`).

### Conventions & Patterns (Project-Specific)

1. **ROS2 Parameters:** All tunable values are declared via `declare_parameter(name, default)` and read via `get_parameter(name).value`. Avoid hardcoded constants in methods.
2. **Sensor Preprocessing:** Always check for NaN/Inf in laser scans; transform to world frame using current odometry pose and `tf_transformations.euler_from_quaternion`.
3. **Collision Safety:** `check_for_collisions` returns `None` (or large cost) if any trajectory point enters the inflation zone (robot_radius + safety_margin). Conservative by design.
4. **Trajectory Representation:** Paths are lists of (x, y) tuples in world frame; collision checks run against precomputed world-frame scan points for efficiency.
5. **Dependency Management:** Runtime deps go in `package.xml` (ament/ROS); `setup.py` only exposes console script entrypoints (`dwa_node`, `global_planner`).

### Safe Changes & Editing Tips

- **Add parameters:** Declare in `__init__`, read in relevant methods, document units and guidance in inline comments.
- **Modify scoring function:** Adjust weights in `declare_parameter` calls; logic is in `control_loop`. Higher weight = higher priority.
- **Change control loop rate:** Modify `self.create_timer(self.step_time, ...)` and ensure `step_time` parameter matches.
- **Extend collision model:** Enhance `check_for_collisions` to handle additional geometry (e.g., costmaps) but preserve the "return None on collision" pattern.

### Integration Points

- **Topics:** Subscribes to `/odom` (Odometry), `/scan` (LaserScan); publishes `/cmd_vel` (Twist), `/visual_paths` (Marker).
- **Launch file:** `src/dwa_local_planner/launch/dwa_gazebo.launch.py` wraps the node with Gazebo + RViz; see inline comments for tuning notes.
- **Expected input format:** Scans must have `angle_min`, `angle_increment`, `ranges` fields; odometry must have valid pose (x, y, quaternion).

### Key Files
- `dwa_node.py` — Main planner and algorithm
- `setup.py` — Console script: `dwa_node = dwa_local_planner.dwa_node:main`
- `package.xml` — Runtime and test dependencies
- `launch/dwa_gazebo.launch.py` — Integration with Gazebo
- `test/` — Linting/style checks (flake8, pep257, copyright)

### Documentation References
- **Tuning guide:** `DWA_TUNING_GUIDE.md` — Detailed parameter tuning and common issues
- **Usage guide:** `USAGE_GUIDE.md` — Two-layer navigation (global + local planner) workflow
- **Launch file:** `LAUNCH.md` — Launch file configuration details
