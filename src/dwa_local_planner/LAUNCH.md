# 🚀 DWA Local Planner Launch Guide

> Comprehensive guide to launching and configuring the DWA local planner with Gazebo, RViz, and TurtleBot3.

---

## 📋 Table of Contents

- [Quick Start](#quick-start)
- [Launch Files](#launch-files)
- [Parameter Configuration](#parameter-configuration)
- [RViz Setup](#rviz-setup)
- [Advanced Usage](#advanced-usage)
- [Troubleshooting](#troubleshooting)

---

## ⚡ Quick Start

### 30-Second Setup

```bash
# Terminal 1: Build
cd ~/ros2_ws
colcon build --packages-select dwa_local_planner
source install/setup.bash

# Terminal 1: Launch Everything
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch dwa_local_planner dwa_gazebo.launch.py

# Terminal 2: Monitor (optional)
source install/setup.bash
ros2 topic echo /cmd_vel
```

**Expected Output:**
- Gazebo window opens with TurtleBot3
- RViz window displays robot and sensor data
- Terminal shows DWA node starting

🎉 **You're now running the complete DWA planner!**

---

## 📁 Launch Files

### Primary Launch File: `dwa_gazebo.launch.py`

**Location:** `src/dwa_local_planner/launch/dwa_gazebo.launch.py`

**Purpose:** Integrated launch combining Gazebo, DWA planner, and RViz

**What it starts:**

1. **Gazebo Simulator**
   - TurtleBot3 robot model
   - World environment with obstacles
   - Physics simulation

2. **DWA Local Planner Node**
   - Processes sensor data
   - Computes velocity commands
   - Publishes to `/cmd_vel`

3. **RViz Visualizer**
   - Displays robot and sensors
   - Shows trajectory predictions
   - Provides interactive visualization

### Launch Command Syntax

**Basic:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py
```

**With custom goal:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py goal_x:=3.0 goal_y:=2.0
```

**With multiple parameters:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  goal_x:=3.0 \
  goal_y:=2.0 \
  max_speed:=0.20 \
  safety_margin:=0.25
```

---

## 🎛️ Parameter Configuration

### Launch-Time Parameters

Override parameters directly from command line:

```bash
# Goal position
ros2 launch dwa_local_planner dwa_gazebo.launch.py goal_x:=2.5 goal_y:=1.5

# Robot constraints
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  max_speed:=0.20 \
  max_turn:=2.0

# Safety settings
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  safety_margin:=0.25 \
  robot_radius:=0.105

# Sampling & performance
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  num_samples:=150 \
  lookahead_steps:=80

# Cost weights
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  goal_weight:=5.0 \
  heading_weight:=2.0 \
  obstacle_weight:=1.0
```

### All Available Parameters

| Parameter | Default | Type | Description |
|-----------|---------|------|-------------|
| `goal_x` | 2.0 | float | Target X position (m) |
| `goal_y` | 1.0 | float | Target Y position (m) |
| `goal_tolerance` | 0.1 | float | Distance threshold to declare goal reached (m) |
| `max_speed` | 0.15 | float | Maximum forward velocity (m/s) |
| `max_turn` | 2.5 | float | Maximum angular velocity (rad/s) |
| `step_time` | 0.1 | float | Control loop period (s) — also integration timestep |
| `num_samples` | 200 | int | Number of trajectory candidates per cycle |
| `lookahead_steps` | 100 | int | Prediction horizon (steps × step_time = lookahead duration) |
| `safety_margin` | 0.3 | float | Collision buffer radius (m) |
| `robot_radius` | 0.105 | float | Physical robot radius (m) |
| `goal_weight` | 5.0 | float | Cost weight for reaching goal |
| `heading_weight` | 2.0 | float | Cost weight for heading toward goal |
| `obstacle_weight` | 1.0 | float | Cost weight for obstacle avoidance |
| `smoothness_weight` | 0.1 | float | Cost weight for motion smoothness |
| `near_goal_distance` | 0.3 | float | Distance threshold to enter "near goal" mode (m) |
| `near_goal_heading_boost` | 5.0 | float | Heading weight boost when near goal |
| `visual_frame` | 'odom' | string | Frame for visualization markers |

### Tuning Strategies

**For narrow passages:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  safety_margin:=0.2 \
  robot_radius:=0.10 \
  heading_weight:=3.0
```

**For open spaces:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  max_speed:=0.20 \
  num_samples:=250 \
  heading_weight:=1.5
```

**For obstacle-heavy environments:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  obstacle_weight:=2.0 \
  lookahead_steps:=150 \
  safety_margin:=0.35
```

**For performance (reduce CPU usage):**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  num_samples:=100 \
  lookahead_steps:=50 \
  step_time:=0.2
```

---

## 🎨 RViz Setup

### Pre-Configured Configuration File

**Location:** `config/dwa_rviz.rviz` (auto-loaded by launch file)

**Included Displays:**

| Display | Topic | Purpose |
|---------|-------|---------|
| **Robot Model** | TF | Shows robot geometry and orientation |
| **LaserScan** | `/scan` | Obstacle points in red |
| **Grid** | Base frame | Reference grid (XY plane) |
| **TF Tree** | TF | Coordinate frames (odom → base_link → etc) |
| **Sampled Trajectories** | `/visual_paths` | All candidate paths (faint) |
| **Best Trajectory** | `/dwa/best_trajectory` | Selected path (bright green) |

### Manual RViz Setup

If pre-configured file doesn't load:

```bash
# Launch RViz
rviz2

# Add displays:
# 1. RobotModel (default TF)
# 2. LaserScan: Topic=/scan, Color Transformer=Intensity
# 3. Marker: Topic=/visual_paths
# 4. Marker: Topic=/dwa/best_trajectory
# 5. Grid: Frame=odom
# 6. TF: Show all frames
```

### Interpreting RViz Display

| Element | Meaning | Action |
|---------|---------|--------|
| **Gray paths** | Colliding trajectories | May appear if safety too aggressive |
| **Green paths** | Valid trajectories | Normal operation |
| **Bright green path** | Selected trajectory | Currently executing this path |
| **Red dots** | Laser scan obstacles | Detected by sensor |
| **Orange sphere** | Trajectory endpoint | Where robot will reach in lookahead window |

---

## 🔧 Advanced Usage

### Custom Launch File

Create `my_dwa_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    dwa_pkg = FindPackageShare('dwa_local_planner')
    
    return LaunchDescription([
        # Include Gazebo
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch', 'turtlebot3_world.launch.py'
            ]),
            launch_arguments={'model': 'waffle_pi'}.items()
        ),
        
        # DWA Node with custom parameters
        Node(
            package='dwa_local_planner',
            executable='dwa_node',
            name='dwa_planner',
            output='screen',
            parameters=[{
                'goal_x': 3.0,
                'goal_y': 2.0,
                'max_speed': 0.18,
                'safety_margin': 0.25,
                'heading_weight': 2.5,
            }]
        ),
    ])
```

Then launch:
```bash
ros2 launch my_dwa_launch.py
```

### Running Components Separately

**Only Gazebo:**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Only DWA Planner:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run dwa_local_planner dwa_node
```

**Only RViz (with pre-configured display):**
```bash
source install/setup.bash
rviz2 -d install/dwa_local_planner/share/dwa_local_planner/config/dwa_rviz.rviz
```

### Remote/Distributed Launch

**Machine A (Gazebo server):**
```bash
export ROS_DOMAIN_ID=42
export ROS_MASTER_URI=http://machine_a:11311
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Machine B (Planner):**
```bash
export ROS_DOMAIN_ID=42
export ROS_MASTER_URI=http://machine_a:11311
ros2 run dwa_local_planner dwa_node
```

### Launch with Custom World

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world_model:=turtlebot3_stage4.world
```

---

## 📊 Monitoring Launch

### Check Launch Status

```bash
# List running nodes
ros2 node list
# Expected output:
# /gazebo
# /dwa_local_planner
# /rviz2

# Inspect node details
ros2 node info /dwa_local_planner

# Monitor topics being published
ros2 topic list

# Watch logs in real-time
ros2 run dwa_local_planner dwa_node 2>&1 | grep -i "INFO\|ERROR"
```

### Performance Monitoring

```bash
# Check CPU/memory usage
ros2 top

# Monitor trajectory planning frequency
ros2 topic hz /cmd_vel
# Expected: ~10 Hz (10 commands per second)

# Check sensor data update rate
ros2 topic hz /scan
# Expected: ~4 Hz (typical lidar)

# Monitor odometry
ros2 topic hz /odom
# Expected: ~30-50 Hz
```

---

## 🐛 Troubleshooting

### Issue: "Launch file not found"

**Solution:**
```bash
# Ensure package is built
colcon build --packages-select dwa_local_planner

# Source the install space
source install/setup.bash

# Verify package is discoverable
ros2 pkg prefix dwa_local_planner
```

### Issue: "Gazebo crashes on launch"

**Solutions:**
```bash
# Option 1: Set Gazebo verbose mode
GAZEBO_DEBUG=1 ros2 launch dwa_local_planner dwa_gazebo.launch.py

# Option 2: Check graphics compatibility
glxinfo | grep "OpenGL"

# Option 3: Use headless Gazebo
export GAZEBO_VERBOSE=1
export DISPLAY=:0  # or appropriate display number
```

### Issue: "RViz not loading configuration"

**Solutions:**
```bash
# Verify config file exists
ls install/dwa_local_planner/share/dwa_local_planner/config/

# If missing, manually configure RViz
rviz2
# Then File → Save Config As → dwa_rviz.rviz
```

### Issue: "DWA node exits with error"

**Debug:**
```bash
# Run with verbose output
RCL_LOG_LEVEL=DEBUG ros2 run dwa_local_planner dwa_node

# Check for parameter errors
ros2 param list /dwa_local_planner

# Verify sensor topics
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

### Issue: "No robot movement"

**Checklist:**
```bash
# 1. Verify Gazebo is running
ps aux | grep gazebo

# 2. Check velocity commands
ros2 topic echo /cmd_vel

# 3. Monitor planner logs
ros2 node info /dwa_local_planner

# 4. Check for collision
ros2 topic echo /scan | head -20

# 5. Ensure goal is reachable
ros2 launch dwa_local_planner dwa_gazebo.launch.py goal_x:=0.5 goal_y:=0.0
```

---

## 💻 Build & Deploy

### Build Optimized

```bash
# Build with optimization flags
colcon build --packages-select dwa_local_planner \
  -DCMAKE_BUILD_TYPE=Release
```

### Deploy to Real Robot

```bash
# Build for robot hardware
colcon build --packages-select dwa_local_planner

# Transfer to robot
scp -r install/dwa_local_planner robot@turtlebot:~/ros2_ws/install/

# SSH to robot
ssh robot@turtlebot

# Run on real hardware (replace /scan, /odom with actual topics)
source ~/ros2_ws/install/setup.bash
ros2 run dwa_local_planner dwa_node
```

---

## 📈 Performance Tuning

### Reduce Latency

```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  num_samples:=100 \
  lookahead_steps:=50 \
  step_time:=0.05
```

### Improve Accuracy

```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  num_samples:=300 \
  lookahead_steps:=150 \
  step_time:=0.05
```

### Balance Quality vs Speed

```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  num_samples:=200 \
  lookahead_steps:=100 \
  step_time:=0.1
```

---

## 🔗 Related Documentation

- **[README.md](../README.md)** — Project overview
- **[USAGE_GUIDE.md](../USAGE_GUIDE.md)** — Usage modes and workflows
- **[DWA_TUNING_GUIDE.md](../DWA_TUNING_GUIDE.md)** — Parameter tuning guide

---

## 💡 Quick Reference

```bash
# Launch everything
ros2 launch dwa_local_planner dwa_gazebo.launch.py

# With custom goal
ros2 launch dwa_local_planner dwa_gazebo.launch.py goal_x:=3.0 goal_y:=2.0

# Monitor velocity
ros2 topic echo /cmd_vel

# Monitor planning rate
ros2 topic hz /cmd_vel

# Run just planner (no Gazebo/RViz)
ros2 run dwa_local_planner dwa_node

# View all published topics
ros2 topic list
```

---

**Ready to launch? Start with `ros2 launch dwa_local_planner dwa_gazebo.launch.py` 🚀**

## Notes

- Default goal is set to (2.0, 1.0) m in the world frame
- The robot starts at origin (0, 0) facing forward
- Laser scan obstacles and planning visualization update at 10 Hz (see `dt` and `control_rate` parameters)
