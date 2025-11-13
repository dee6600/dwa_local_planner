# DWA Local Planner for TurtleBot3

A custom implementation of the Dynamic Window Approach (DWA) local planner for ROS 2, designed for TurtleBot3 simulation in Gazebo.

## Overview

This project implements a simplified DWA local planner that generates collision-free velocity commands for mobile robot navigation. The planner:

- Subscribes to `/odom` and `/scan` topics
- Computes dynamic velocity windows based on robot constraints
- Samples and scores candidate trajectories
- Publishes optimal velocity commands to `/cmd_vel`

## Prerequisites

- ROS 2 (Humble/Foxy or later)
- TurtleBot3 packages
- Gazebo
- Python 3

## Build

From the workspace root:

```bash
colcon build --packages-select dwa_local_planner
```

## Usage

1. Source the overlay:
   ```bash
   source install/setup.bash
   ```

2. Launch TurtleBot3 in Gazebo (in a separate terminal):
   ```bash
   export TURTLEBOT3_MODEL=waffle_pi
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

3. Run the DWA planner node:
   ```bash
   ros2 run dwa_local_planner dwa_node
   ```

## Testing

Run all tests:
```bash
colcon test --packages-select dwa_local_planner
colcon test-result --verbose
```

Run specific linter checks:
```bash
colcon test --packages-select dwa_local_planner --pytest-args "-k 'flake8 or pep257'"
```

## Architecture

- **Node**: `dwa_local_planner`
- **Subscriptions**: 
  - `/odom` (nav_msgs/Odometry)
  - `/scan` (sensor_msgs/LaserScan)
- **Publisher**: `/cmd_vel` (geometry_msgs/Twist)
- **Control loop**: 10 Hz (0.1s period)

### Algorithm

1. Compute dynamic window from current velocity and acceleration limits
2. Sample candidate (v, ω) pairs within the window
3. Predict short-horizon trajectories for each candidate
4. Perform conservative collision checking against laser scan data
5. Score collision-free trajectories (goal progress + forward speed)
6. Publish the best velocity command

## Configuration

Current implementation uses hardcoded parameters in `dwa_node.py`:
- Goal position: `[2.0, 0.0]`
- Max linear velocity: 0.22 m/s
- Max angular velocity: 2.84 rad/s
- Acceleration limits and prediction horizon defined in code

## License

[Add your license here]
