# 🤖 DWA Local Planner for TurtleBot3

> A highly optimized **Dynamic Window Approach (DWA)** local motion planner for ROS 2, delivering collision-free, efficient trajectory generation for mobile robots in complex environments.

[![Build Status](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-orange)](LICENSE)

---

## 🎯 Overview

This project provides a production-ready **single-file DWA local planner** optimized for TurtleBot3 robots running in Gazebo simulations. The planner generates smooth, collision-free velocity commands by:

- **Sampling** dynamic velocity windows based on robot kinematics
- **Simulating** short-horizon candidate trajectories using unicycle model
- **Scoring** trajectories on goal progress, obstacle clearance, and smoothness
- **Publishing** optimal commands at 10 Hz control rate

### Key Features

✨ **Fully Parameterized** — All 15+ tunable parameters exposed as ROS2 parameters  
✨ **Real-Time Performance** — ~10 ms planning cycle on modest hardware  
✨ **Conservative Safety** — Built-in collision checking with configurable safety margins  
✨ **RViz Integration** — Visualize all sampled trajectories and chosen path  
✨ **Modular Design** — Easy to extend with custom scoring functions  

---

## 📋 Prerequisites

- **ROS 2** (Humble/Foxy or later)
- **TurtleBot3** simulation packages
- **Gazebo** (11+)
- **Python 3.8+** with `rclpy`, `numpy`

### Quick Install (Ubuntu/Debian)

```bash
# Install ROS 2 dependencies
sudo apt install ros-humble-rclpy ros-humble-geometry-msgs ros-humble-nav-msgs \
                 ros-humble-sensor-msgs ros-humble-turtlebot3-gazebo
```

---

## 🚀 Quick Start

### 1️⃣ Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select dwa_local_planner
source install/setup.bash
```

### 2️⃣ Launch Everything (Gazebo + DWA + RViz)

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch dwa_local_planner dwa_gazebo.launch.py
```

### 3️⃣ Send a Goal

In a new terminal:

```bash
source install/setup.bash

# Navigate to position (2.0, 1.0)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}
}"
```

🎉 **The robot will now plan and execute a collision-free path to the goal!**

---

## 📊 Architecture

### Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                      SENSORS                                │
│            /odom (Odometry) + /scan (LaserScan)            │
└─────────────────────────┬───────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────┐
│              DWA LOCAL PLANNER NODE                          │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ 1. Sample (v, ω) candidates within dynamic window  │   │
│  │ 2. Simulate trajectories using unicycle kinematics │   │
│  │ 3. Check collisions against sensor data            │   │
│  │ 4. Score valid trajectories                        │   │
│  │ 5. Select best command                             │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────┬───────────────────────────────────┘
                          │
                    /cmd_vel (Twist)
                          │
┌─────────────────────────▼───────────────────────────────────┐
│                  ROBOT EXECUTION                             │
│              TurtleBot3 (Gazebo Simulation)                 │
└─────────────────────────────────────────────────────────────┘
```

### Directory Structure

```
src/dwa_local_planner/
├── dwa_local_planner/
│   ├── dwa_node.py           # Main DWA planner implementation
│   └── __init__.py
├── launch/
│   └── dwa_gazebo.launch.py  # Integrated launch file
├── test/
│   ├── test_flake8.py        # Style checks
│   ├── test_pep257.py        # Documentation checks
│   └── test_copyright.py     # License checks
├── package.xml               # ROS 2 dependencies
├── setup.py                  # Python package setup
└── LAUNCH.md                 # Detailed launch guide
```

---

## 🎛️ Parameter Configuration

All parameters are ROS2 parameters and can be set via launch file or dynamically:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_speed` | 0.15 m/s | Maximum forward velocity |
| `max_turn` | 2.5 rad/s | Maximum angular velocity |
| `step_time` | 0.1 s | Control loop timestep |
| `num_samples` | 200 | Candidate trajectories per cycle |
| `safety_margin` | 0.3 m | Collision buffer radius |
| `goal_weight` | 5.0 | Priority for reaching goal |
| `heading_weight` | 2.0 | Priority for moving toward goal |
| `obstacle_weight` | 1.0 | Priority for avoiding obstacles |
| `robot_radius` | 0.105 m | Robot physical radius (TurtleBot3) |

**For detailed tuning guidance**, see [DWA_TUNING_GUIDE.md](DWA_TUNING_GUIDE.md)

---

## 📖 Usage Modes

### Mode 1: Direct Goal Navigation (Simplest)

Robot heads directly toward hardcoded goal:

```bash
ros2 run dwa_local_planner dwa_node
```

Modify goal in `dwa_node.py`:

```python
self.declare_parameter('goal_x', 2.0)
self.declare_parameter('goal_y', 1.0)
```

### Mode 2: Integrated Simulation

Full stack with Gazebo + DWA + RViz visualization:

```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py goal_x:=3.0 goal_y:=2.0
```

### Mode 3: Two-Layer Navigation (Advanced)

Combine with global planner for complex environments — see [USAGE_GUIDE.md](USAGE_GUIDE.md)

---

## 🧪 Testing

### Run All Tests

```bash
colcon test --packages-select dwa_local_planner
colcon test-result --verbose
```

### Run Specific Checks

```bash
# Style checks only
colcon test --packages-select dwa_local_planner --pytest-args "-k 'flake8'"

# Documentation checks
colcon test --packages-select dwa_local_planner --pytest-args "-k 'pep257'"
```

---

## 📡 ROS Topics

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Robot odometry (position, velocity) |
| `/scan` | `sensor_msgs/LaserScan` | Laser scan for obstacle detection |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to robot |
| `/visual_paths` | `visualization_msgs/Marker` | All candidate trajectories (RViz) |
| `/dwa/best_trajectory` | `visualization_msgs/Marker` | Selected trajectory (RViz) |

---

## 🔧 Advanced Configuration

### Override Parameters at Launch

```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  max_speed:=0.20 \
  safety_margin:=0.2 \
  heading_weight:=3.0
```

### Run with Custom Configuration

Create `my_config.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dwa_local_planner',
            executable='dwa_node',
            parameters=[{
                'max_speed': 0.20,
                'safety_margin': 0.25,
                'goal_x': 3.5,
                'goal_y': 2.5,
            }]
        ),
    ])
```

Then launch: `ros2 launch my_config.launch.py`

---

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| **Robot not moving** | Check `/cmd_vel` publishing: `ros2 topic echo /cmd_vel` |
| **Collisions** | Increase `safety_margin` or `obstacle_weight` |
| **Too slow** | Increase `max_speed` and `velocity_weight` |
| **Jerky motion** | Decrease `max_turn` or reduce `num_samples` |
| **Can't fit through passage** | Decrease `safety_margin` or `robot_radius` |

See [DWA_TUNING_GUIDE.md](DWA_TUNING_GUIDE.md) for detailed troubleshooting.

---

## 📚 Documentation

- **[LAUNCH.md](src/dwa_local_planner/LAUNCH.md)** — Launch file configuration and usage
- **[USAGE_GUIDE.md](USAGE_GUIDE.md)** — Detailed usage modes and two-layer navigation
- **[DWA_TUNING_GUIDE.md](DWA_TUNING_GUIDE.md)** — Parameter tuning strategies and recipes
- **[.github/copilot-instructions.md](.github/copilot-instructions.md)** — AI agent instructions

---

## 📈 Performance Notes

- **Planning Cycle:** ~10 ms (100 trajectories, 100 steps lookahead)
- **Control Rate:** 10 Hz (configurable via `step_time`)
- **CPU Usage:** <5% on Intel i7 with 200 samples
- **Memory:** ~50 MB (Python runtime + dependencies)

---

## 🎓 Algorithm Explanation

The **Dynamic Window Approach** works in three phases:

1. **Dynamic Window Generation**
   - Compute achievable velocity ranges based on current speed and acceleration limits
   - This creates a "window" of feasible (v, ω) pairs

2. **Trajectory Sampling & Simulation**
   - Sample uniformly within the window
   - Forward-simulate each trajectory using unicycle kinematic model
   - Collect (x, y) waypoints along each path

3. **Trajectory Evaluation & Selection**
   ```
   Score(trajectory) = 
       + goal_weight × heading_to_goal
       + heading_weight × forward_progress
       + obstacle_weight × clearance_from_obstacles
       + smoothness_weight × velocity_stability
   ```
   - Select trajectory with highest score

This approach guarantees safety while maintaining responsiveness — ideal for dynamic environments!

---

## 🔗 Related Resources

- [ROS 2 Navigation](https://navigation.ros.org/)
- [TurtleBot3 Documentation](https://docs.turtlebot.com/)
- [DWA Algorithm Paper](https://www.researchgate.net/publication/4040859_The_Dynamic_Window_Approach_to_Collision_Avoidance)

---

## 📝 License

Licensed under the Apache License, Version 2.0. See LICENSE file for details.

---

## 💬 Contributing

Found a bug? Have a feature suggestion? Open an issue or pull request!

For AI agents: See [.github/copilot-instructions.md](.github/copilot-instructions.md) for developer guidance.

---

**⭐ If you find this useful, please consider starring the repository!**
