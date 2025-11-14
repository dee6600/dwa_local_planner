# 📘 DWA Local Planner Usage Guide

> Comprehensive guide to using the Dynamic Window Approach (DWA) local planner with ROS 2 and TurtleBot3.

---

## 🎯 Quick Navigation

- **New to DWA?** → Start with [System Overview](#-system-overview)
- **Just want to run it?** → Jump to [Quick Start](#-quick-start)
- **Need troubleshooting?** → See [Troubleshooting](#-troubleshooting)
- **Tuning parameters?** → Check [DWA_TUNING_GUIDE.md](../DWA_TUNING_GUIDE.md)

---

## 📊 System Overview

### Architecture: Two-Layer Navigation

This project implements a **hierarchical navigation system** for complex environments:

```
┌──────────────────────────────────────────────────────────────────┐
│                       GLOBAL PLANNING LAYER                       │
│  Computes long-term collision-free paths around static obstacles │
│  (A* on occupancy grid, replans every 2 seconds)                 │
└──────────────────────┬───────────────────────────────────────────┘
                       │ /plan (nav_msgs/Path)
                       │
┌──────────────────────▼───────────────────────────────────────────┐
│                       LOCAL PLANNING LAYER                        │
│  Generates real-time trajectory commands following global path   │
│  (DWA, 10 Hz, reactive to dynamic obstacles)                    │
└──────────────────────┬───────────────────────────────────────────┘
                       │ /cmd_vel (geometry_msgs/Twist)
                       │
┌──────────────────────▼───────────────────────────────────────────┐
│                    ROBOT EXECUTION                                │
│  TurtleBot3 physical or simulated robot                          │
└──────────────────────────────────────────────────────────────────┘
```

### Component Responsibilities

| Component | Purpose | Input | Output |
|-----------|---------|-------|--------|
| **Global Planner** | Long-term path planning | `/goal_pose`, `/scan` | `/plan` |
| **DWA Local Planner** | Real-time trajectory execution | `/plan`, `/odom`, `/scan` | `/cmd_vel` |
| **Gazebo** | Physics simulation | — | `/odom`, `/scan` |

---

## 🚀 Quick Start

### The 30-Second Startup

**Terminal 1 — Launch Gazebo:**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2 — Run DWA Planner:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run dwa_local_planner dwa_node
```

**Terminal 3 — Send a Goal:**
```bash
source install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}
}"
```

**Result:** Robot navigates to (2.0, 1.0) avoiding obstacles!

---

## 🎛️ Three Usage Modes

### Mode A: Simple Direct Goal (Default)

**Best for:** Getting started, simple environments

Robot moves directly toward hardcoded goal with obstacle avoidance.

**Setup:**
```bash
# Terminal 1
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2
cd ~/ros2_ws && source install/setup.bash
ros2 run dwa_local_planner dwa_node
```

**Modify goal in `dwa_node.py`:**
```python
self.declare_parameter('goal_x', 2.0)
self.declare_parameter('goal_y', 1.0)
```

**Output:**
- Robot will autonomously navigate to goal while avoiding obstacles
- View `/cmd_vel` to see velocity commands
- View `/visual_paths` in RViz for trajectory visualization

---

### Mode B: Integrated Simulation (Full Stack)

**Best for:** Development, visualization, parameter tuning

All-in-one launch: Gazebo + DWA + RViz with pre-configured displays.

**Setup:**
```bash
cd ~/ros2_ws
colcon build --packages-select dwa_local_planner
source install/setup.bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py
```

**Optional Parameters:**
```bash
# Override default goal
ros2 launch dwa_local_planner dwa_gazebo.launch.py goal_x:=3.0 goal_y:=2.5

# Override multiple parameters
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  goal_x:=3.0 goal_y:=2.0 \
  max_speed:=0.18 \
  safety_margin:=0.25
```

**What You'll See:**
- Gazebo showing TurtleBot3 in world
- RViz displaying:
  - Robot model and laser scan
  - All sampled trajectories (faint lines)
  - Best selected trajectory (bright green)

**RViz Config:** `install/dwa_local_planner/share/dwa_local_planner/config/dwa_rviz.rviz`

---

### Mode C: Advanced Two-Layer Navigation

**Best for:** Complex environments, true global planning

Combines A* global planner with DWA local planner for robust navigation.

**Setup:**

```bash
# Terminal 1 — Gazebo
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 — Global Planner (if available)
source install/setup.bash
ros2 run dwa_local_planner global_planner  # Optional, if implemented

# Terminal 3 — DWA Local Planner
source install/setup.bash
ros2 run dwa_local_planner dwa_node

# Terminal 4 — Send Goals
source install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}
}"
```

**How It Works:**
1. Global planner builds occupancy grid from sensor data
2. A* algorithm computes collision-free path to goal
3. Path published on `/plan` topic
4. DWA local planner follows path while reacting to dynamic obstacles
5. Global planner replans every 2 seconds to adapt to environment changes

---

## 📡 Topic Reference

### Subscribed Topics

```bash
# Robot odometry (position, orientation, velocities)
/odom    (nav_msgs/Odometry)

# Laser scan data (360° range measurements)
/scan    (sensor_msgs/LaserScan)

# Goal position (if using goal-based modes)
/goal_pose    (geometry_msgs/PoseStamped)    [Optional]
```

### Published Topics

```bash
# Velocity commands sent to robot
/cmd_vel    (geometry_msgs/Twist)

# Visualization: all sampled trajectories
/visual_paths    (visualization_msgs/Marker)

# Visualization: best selected trajectory
/dwa/best_trajectory    (visualization_msgs/Marker)

# Global path (if using global planner)
/plan    (nav_msgs/Path)    [Optional]
```

### Inspecting Topics

```bash
# List all active topics
ros2 topic list

# Monitor velocity commands (live stream)
ros2 topic echo /cmd_vel

# Check robot position
ros2 topic echo /odom --once

# Display laser scan statistics
ros2 topic echo /scan --once

# Verify goal reception
ros2 topic echo /goal_pose --once
```

---

## 🎯 Goal Specification Examples

### Publish Single Goal

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

### Goal Examples

**Forward 2 meters:**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"
```

**Diagonal:**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 3.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}
}"
```

**Behind the robot:**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: -2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"
```

**With specific orientation (45° rotation):**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.383, w: 0.924}
  }
}"
```

---

## 🔍 Monitoring & Debugging

### View Real-Time Status

```bash
# Watch velocity commands
watch -n 0.1 'ros2 topic echo /cmd_vel --once'

# Monitor distance to goal
ros2 run dwa_local_planner dwa_node 2>&1 | grep -i "distance\|goal"

# Log all topics to CSV (advanced)
ros2 bag record -o dwa_session /cmd_vel /odom /scan
```

### RViz Visualization

```bash
# Launch RViz with default DWA configuration
rviz2 -d install/dwa_local_planner/share/dwa_local_planner/config/dwa_rviz.rviz

# Or manually add displays:
# 1. Add MarkerArray display → Topic: /visual_paths (RGB 100,100,100)
# 2. Add MarkerArray display → Topic: /dwa/best_trajectory (RGB 0,255,0)
# 3. Add LaserScan display → Topic: /scan (RGB 255,0,0)
# 4. Add TF display to see coordinate frames
```

### Check Node Status

```bash
# List all active nodes
ros2 node list

# View node information
ros2 node info /dwa_local_planner

# Monitor node resource usage
ros2 top

# View published/subscribed topics for a node
ros2 node info /dwa_local_planner
```

---

## 🐛 Troubleshooting Guide

### ❌ Problem: "Robot Not Moving"

**Checklist:**

1. **Verify topic publishing:**
   ```bash
   ros2 topic echo /cmd_vel
   # Should see changing linear.x and angular.z values
   ```

2. **Check node is running:**
   ```bash
   ros2 node list | grep dwa
   # Should show /dwa_local_planner
   ```

3. **Verify sensors connected:**
   ```bash
   ros2 topic echo /odom --once
   ros2 topic echo /scan --once
   # Both should have recent timestamps
   ```

4. **Check goal was set:**
   ```bash
   ros2 topic echo /goal_pose --once
   # Should show your goal position
   ```

**Solutions:**
- Ensure Gazebo is running: `ps aux | grep gazebo`
- Rebuild if you made code changes: `colcon build --packages-select dwa_local_planner`
- Check for errors in terminal: Look for red text or exceptions

---

### ❌ Problem: "Robot Keeps Colliding"

**Root Causes & Fixes:**

| Cause | Solution |
|-------|----------|
| Safety margin too small | Increase `safety_margin` to 0.3-0.4 m |
| Obstacle weight too low | Increase `obstacle_weight` to 2.0+ |
| Robot radius mismatch | Set `robot_radius` to actual robot size |
| Sensor range limited | Check `/scan` range — may need closer obstacles |

**Quick test:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  safety_margin:=0.4 \
  obstacle_weight:=2.0
```

---

### ❌ Problem: "Robot Too Slow / Timid"

**Causes & Fixes:**

| Symptom | Solution |
|---------|----------|
| Too cautious around obstacles | **Decrease** `obstacle_weight` (try 0.5) |
| Maximum speed not reached | **Increase** `max_speed` (try 0.22 m/s) |
| Takes long curvy paths | **Increase** `heading_weight` (try 3.0) |
| Won't go through narrow passage | **Decrease** `safety_margin` (try 0.2 m) |

---

### ❌ Problem: "Jerky / Unstable Motion"

**Fixes:**

```bash
# Reduce acceleration limits
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  max_turn:=1.5 \
  num_samples:=100

# Or modify in dwa_node.py to test:
# self.declare_parameter('max_turn', 1.5)  # Lower than default 2.5
```

---

### ❌ Problem: "No Path Found / Goal Unreachable"

**Check:**
1. Goal is within world bounds (typically ±5m from origin)
2. Goal is not inside an obstacle
3. Global planner has time to compute (wait 2+ seconds)

**Verify:**
```bash
# Check what global planner sees
ros2 topic echo /plan --once

# Try a closer goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"
```

---

## ⚙️ Parameter Tuning Workflow

See [DWA_TUNING_GUIDE.md](../DWA_TUNING_GUIDE.md) for **comprehensive tuning strategies**.

### Quick Parameter Reference

```bash
# Safety & Collision Avoidance
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  safety_margin:=0.25 \
  robot_radius:=0.105 \
  obstacle_weight:=1.5

# Speed & Responsiveness
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  max_speed:=0.18 \
  max_turn:=2.5 \
  heading_weight:=2.0

# Sampling & Performance
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  num_samples:=200 \
  lookahead_steps:=100
```

---

## 📊 Expected Behavior

### Success Indicators

✅ Robot moves smoothly toward goal  
✅ Smoothly avoids obstacles in path  
✅ Reaches goal and stops  
✅ Responsive to new goal commands  
✅ `/cmd_vel` shows reasonable velocities (typically <0.2 m/s)

### Typical Performance Metrics

| Metric | Value |
|--------|-------|
| Control Rate | 10 Hz |
| Planning Cycle | ~10 ms |
| Max Speed | 0.15 m/s (default, configurable) |
| Lookahead Distance | ~1.5 m (100 steps × 0.1s × 0.15 m/s) |
| Safety Margin | 0.3 m (configurable) |

---

## 🔗 Related Documentation

- **[DWA_TUNING_GUIDE.md](../DWA_TUNING_GUIDE.md)** — Parameter tuning and recipes
- **[LAUNCH.md](./LAUNCH.md)** — Launch file details
- **[README.md](../README.md)** — Project overview
- **[.github/copilot-instructions.md](../.github/copilot-instructions.md)** — For AI agents

---

## 💡 Pro Tips

1. **Start with Mode B** (integrated launch) to see everything at once
2. **Use RViz** to visualize trajectories — invaluable for debugging
3. **Log topics** with `ros2 bag` for post-analysis: `ros2 bag record /cmd_vel /odom /scan`
4. **Adjust one parameter at a time** — easier to understand cause/effect
5. **Test in Gazebo first** before deploying to real robot

---

## 🎓 Learning Path

1. Run Mode A (simple) to understand basic operation
2. Switch to Mode B (integrated) to visualize
3. Read [DWA_TUNING_GUIDE.md](../DWA_TUNING_GUIDE.md) for parameter understanding
4. Experiment with parameter changes using launch file arguments
5. Use RViz and log files to analyze behavior
6. Consider Mode C (two-layer) for complex environments

---

**Have questions? Check the README.md or open an issue!**
