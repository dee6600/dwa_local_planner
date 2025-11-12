## DWA Local Planner — Copilot Instructions

Purpose: give AI coding agents the minimal, actionable context to be productive in this ROS2 ament_python package.

- Repo type: ROS2 workspace (ament_python package) located under `src/dwa_local_planner`.
- Primary language: Python 3. Uses `rclpy`, ROS message types and numpy for numeric code.

Quick commands (run from workspace root):

- Build package: `colcon build --packages-select dwa_local_planner`
- Source install overlay: `source install/setup.bash` (or `local_setup.bash` for session-local)
- Run node: `ros2 run dwa_local_planner dwa_node` (exposed via `console_scripts` in `setup.py`)
- Run tests: `colcon test --packages-select dwa_local_planner` then `colcon test-result --verbose`

Where to look first

- Node implementation: `src/dwa_local_planner/dwa_local_planner/dwa_node.py` — this is the single-file demo planner. Key items:
  - `class DWALocalPlanner(Node)` — rclpy Node using `create_subscription`, `create_publisher`, `create_timer`.
  - Hardcoded goal: `self.goal = np.array([2.0, 0.0])` in `__init__` — change here for quick demos.
  - Entry point: `main()` at bottom; packaged as `dwa_node` in `setup.py`.
  - Laser handling: `scan_cb` normalizes NaN/Inf with `msg.range_max` — follow this pattern when working with scans.
  - Collision check maps angles to scan indices using `scan_angle_min` and `scan_angle_increment`.

- Packaging and metadata: `src/dwa_local_planner/setup.py` and `package.xml`
  - `package.xml` declares runtime deps (`rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`) and `build_type` = `ament_python`.
  - `setup.py` registers the console script: `'dwa_node = dwa_local_planner.dwa_node:main'`.
  - Note: dependencies are declared in `package.xml` (ROS/ament) rather than `setup.py`.

Conventions and patterns to follow (project-specific)

- Use `rclpy.Node` idioms already in the node: `create_subscription(...)`, `create_publisher(...)`, and `create_timer(...)`.
- Sensor preprocessing: convert `msg.ranges` to numpy, replace NaN/Inf with `msg.range_max` (see `scan_cb`). Keep this when adding algorithms that read scans.
- Trajectory prediction uses a simple Euler integration loop (`predict_trajectory`). Prefer clarity and small, well-tested helper functions when refactoring.
- Collision check is conservative: if angle index is out-of-range, the code treats it as collision — keep that safety assumption when modifying behavior.

Tests and CI

- Tests live under `src/dwa_local_planner/test/` and are ament/pytest helpers: `test_flake8.py`, `test_pep257.py`, etc. Use `colcon test` to run them in CI.
- Linting and style: the project uses standard ament checks (flake8/pep257) via the test stubs. If you change imports or style, run the linters locally.

Editing tips and safe changes

- To change runtime parameters (goal, limits, dt): either edit the hardcoded values in `DWALocalPlanner.__init__` or (preferred) add ROS2 parameters and read them in `__init__` — the codebase currently uses hardcoded values.
- When adding dependencies, add them to `package.xml` for ROS to resolve; only add to `setup.py` if you intend to publish a PyPI package.
- Keep the node name stable: `super().__init__('dwa_local_planner')` — changing it affects topic names and logs.

Integration points

- Publishes to `/cmd_vel` (Twist) and subscribes to `/odom` (Odometry) and `/scan` (LaserScan). Any change to topic names should be reflected in launch files or tests.
- The code expects scans in robot frame with `angle_min`, `angle_increment` set; when integrating other sensors, ensure these fields are populated.

If you change behavior

- Update or add unit tests under `src/dwa_local_planner/test/` and run `colcon test`.
- Keep messages in the same message types (Twist/Odometry/LaserScan) to avoid breaking integration tests.

Files worth referencing in edits

- `src/dwa_local_planner/dwa_local_planner/dwa_node.py` — main implementation
- `src/dwa_local_planner/setup.py` — console script entry
- `src/dwa_local_planner/package.xml` — ament and deps
- `src/dwa_local_planner/test/` — test stubs and lint guards

If anything here is unclear or you want more examples (e.g., how to add ROS2 parameters, a launch file, or a small unit test for `predict_trajectory`), tell me which area to expand and I will update this file.
