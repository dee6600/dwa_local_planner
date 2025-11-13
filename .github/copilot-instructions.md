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

## DWA Local Planner — Copilot Instructions

Purpose: concise, repo-specific guidance for AI coding agents to be productive quickly.

- Repo layout: a ROS2 workspace with a single ament_python package at `src/dwa_local_planner`.
- Language: Python 3 (uses `rclpy`, ROS message types, `numpy`).

Quick commands (run from workspace root):

- Build package: `colcon build --packages-select dwa_local_planner`
- Source overlay after a successful build: `source install/setup.bash` (or `source install/local_setup.bash` for session-local)
- Run node (once built and sourced): `ros2 run dwa_local_planner dwa_node`
- Run tests: `colcon test --packages-select dwa_local_planner` && `colcon test-result --verbose`

Big picture

- This repo is a compact demo of a Dynamic Window Approach local planner implemented as a single node: `DWALocalPlanner` in `dwa_node.py`.
- Data flow: sensor `LaserScan` (`/scan`) -> planner (`predict_trajectory`, collision check) -> outputs `Twist` on `/cmd_vel`. The node also subscribes to `/odom` for current state.
- Design rationale: single-file demo for clarity and easy experimentation — expect hardcoded params and simple Euler integration for trajectory prediction.

Where to look first

- `src/dwa_local_planner/dwa_local_planner/dwa_node.py`: the entire planner lives here. Key symbols:
  - `class DWALocalPlanner(Node)` — subscriptions/publishers/timer and main loop.
  - `self.goal = np.array([2.0, 0.0])` — quick demo goal; change here or convert to ROS parameters.
  - `scan_cb` — converts `msg.ranges` to numpy and replaces NaN/Inf with `msg.range_max`.
  - `predict_trajectory` — Euler integration to forward-simulate candidate velocities.
  - collision-check logic maps scan angles to indices using `angle_min`/`angle_increment` and treats out-of-range as collision (conservative).

Developer workflows & debugging

- Build: `colcon build --packages-select dwa_local_planner` (use `--symlink-install` locally for faster iteration).
- Source overlay: `source install/setup.bash` (or `source install/local_setup.bash`).
- Run node: `ros2 run dwa_local_planner dwa_node`.
- Debugging tips: use `ros2 topic echo /scan` and `ros2 topic echo /odom` to inspect inputs; `ros2 topic echo /cmd_vel` to verify outputs. Use `ros2 topic pub` to publish synthetic messages for unit tests.
- Tests & lint: tests are in `src/dwa_local_planner/test/` (ament/pytest stubs include `test_flake8.py`, `test_pep257.py`). Run `colcon test` and check `colcon test-result --verbose`.

Conventions & repo-specific patterns

- Keep `rclpy.Node` idioms: use `create_subscription`, `create_publisher`, `create_timer` patterns as present in `dwa_node.py`.
- Sensor preprocessing: always normalize `msg.ranges` to a numpy array and replace NaN/Inf with `msg.range_max` (see `scan_cb`).
- Safety-first collision model: out-of-range or missing scan indices are treated as collisions — preserve this when refactoring.
- Dependency management: add runtime deps to `package.xml` (ROS/ament). `setup.py` only exposes the console script entrypoint.

When changing behavior

- Prefer adding ROS2 parameters (read in `__init__`) instead of editing hardcoded constants in-place.
- Update or add unit tests in `src/dwa_local_planner/test/` when altering planner logic.

Key files

- `src/dwa_local_planner/dwa_local_planner/dwa_node.py` — main node and algorithm.
- `src/dwa_local_planner/package.xml` — ament/ROS dependencies.
- `src/dwa_local_planner/setup.py` — console script entry: `dwa_node = dwa_local_planner.dwa_node:main`.
- `src/dwa_local_planner/test/` — test stubs (linting + style checks).

If any section is missing details you'd like (examples: turning hardcoded values into ROS parameters, adding a launch file, or a unit test example for `predict_trajectory`), tell me which area to expand and I'll iterate.
