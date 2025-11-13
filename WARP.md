# WARP.md

This file provides guidance to WARP (warp.dev) when working with code in this repository.

## Commands

Run these from the workspace root (`ros2_ws`).

- Build (package-select):
  - `colcon build --packages-select dwa_local_planner`
- Source the overlay (new shell/session):
  - `source install/setup.bash`  
    (Use `source install/local_setup.bash` for session-local overlays.)
- Run the node:
  - `ros2 run dwa_local_planner dwa_node`
- Test all (then view results):
  - `colcon test --packages-select dwa_local_planner`
  - `colcon test-result --verbose`
- Run a single/filtered pytest test (within colcon):
  - `colcon test --packages-select dwa_local_planner --pytest-args "-k <pattern> -vv"`
    - Examples: only flake8 checks → `-k flake8`; only pep257 → `-k pep257`
- Lint via the provided ament tests:
  - `colcon test --packages-select dwa_local_planner --pytest-args "-k 'flake8 or pep257'"`

## Architecture and structure

- Workspace type: ROS 2 (ament) with a single Python package at `src/dwa_local_planner` (build type: `ament_python`).
- Entry points and packaging:
  - `src/dwa_local_planner/setup.py` registers `console_scripts` → `dwa_node = dwa_local_planner.dwa_node:main`
  - `src/dwa_local_planner/package.xml` declares runtime deps: `rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs` and test deps for ament linters; `<build_type>ament_python</build_type>`.
- Node implementation (big picture): `src/dwa_local_planner/dwa_local_planner/dwa_node.py`
  - Class `DWALocalPlanner(Node)` sets up:
    - Subscriptions: `/odom` (Odometry), `/scan` (LaserScan)
    - Publisher: `/cmd_vel` (Twist)
    - Timer loop at `dt=0.1s` → `control_loop()` implements a simplified Dynamic Window Approach (DWA):
      1) Compute dynamic window from current `v, w` and robot limits
      2) Sample velocities, predict short trajectories, conservative collision check against LaserScan
      3) Score trajectories (goal progress + speed) and publish the best `(v, w)` as `/cmd_vel`
  - Sensor preprocessing (`scan_cb`): converts ranges to numpy and replaces NaN/Inf with `range_max`.
  - Collision check maps world-space direction to LaserScan index using `angle_min`/`angle_increment`; out-of-range indices are treated as collision (conservative).
  - A demo goal is hardcoded: `self.goal = np.array([2.0, 0.0])`.

## Tests and lint

- Tests live in `src/dwa_local_planner/test/` and are ament/pytest wrappers for linters:
  - `test_flake8.py` (ament_flake8), `test_pep257.py` (ament_pep257), etc.
- Use `colcon test` (with optional `--pytest-args`) to run/target specific checks; review with `colcon test-result --verbose`.

## Copilot rules (from .github/copilot-instructions.md) — key points for Warp

- Use standard `rclpy.Node` patterns already present (`create_subscription`, `create_publisher`, `create_timer`).
- Keep the conservative collision assumption (out-of-range scan index → treat as collision) unless you explicitly change the safety model.
- If changing runtime parameters (e.g., goal, limits, dt), prefer adding ROS 2 parameters read in `__init__`; current code uses hardcoded values.
- When adding dependencies, declare them in `package.xml` so ament/rosdep can resolve them; only use `setup.py install_requires` for PyPI usage.
- Stable node name `dwa_local_planner` is assumed by logs/topics; changing it will affect integration.
