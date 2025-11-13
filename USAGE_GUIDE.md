# DWA Local Planner Usage Guide

## System Overview

This project implements a **two-layer navigation system** for TurtleBot3:

1. **Global Planner** (`simple_global_planner.py`): Uses A* algorithm to plan collision-free paths around obstacles
2. **Local Planner** (`dwa_node.py`): Uses Dynamic Window Approach (DWA) to follow the global path while avoiding dynamic obstacles

## Quick Start

### Step 1: Launch Simulation (Terminal 1)
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Step 2: Start Global Planner (Terminal 2)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run dwa_local_planner global_planner
```

### Step 3: Start DWA Local Planner (Terminal 3)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run dwa_local_planner dwa_node
```

### Step 4: Send a Goal (Terminal 4)
```bash
cd ~/ros2_ws
source install/setup.bash

# Example: Navigate to (2.0, 0.0)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
}"
```

### More Goal Examples:
```bash
# Navigate to (3.0, 2.0)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}
}"

# Navigate to (-2.0, -1.5)
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'odom'},
  pose: {position: {x: -2.0, y: -1.5, z: 0.0}, orientation: {w: 1.0}}
}"
```

## How It Works

### Global Planner
- **Input**: Goal pose from `/goal_pose` topic
- **Process**: 
  1. Builds an occupancy grid from laser scans
  2. Inflates obstacles for safety
  3. Plans path using A* algorithm
  4. Republans every 2 seconds to adapt to changes
- **Output**: Path published to `/plan` topic

### Local Planner (DWA)
- **Input**: 
  - Global path from `/plan` topic
  - Odometry from `/odom`
  - Laser scans from `/scan`
- **Process**:
  1. Extracts local waypoint from global path (lookahead distance)
  2. Generates candidate trajectories within dynamic window
  3. Evaluates trajectories based on:
     - Heading toward local goal
     - Clearance from obstacles
     - Forward velocity
     - Rotation (when path blocked)
  4. Selects best trajectory
- **Output**: Velocity commands to `/cmd_vel`

## Monitoring

### View Published Topics
```bash
ros2 topic list
```

Key topics:
- `/plan` - Global path
- `/cmd_vel` - Velocity commands
- `/goal_pose` - Goal input
- `/odom` - Robot position
- `/scan` - Laser data

### Monitor DWA Output
```bash
ros2 topic echo /cmd_vel
```

### Monitor Path Planning
```bash
ros2 topic echo /plan
```

### View Logs
The DWA node logs useful information:
- Current velocity commands
- Distance to goal
- Whether forward path is blocked
- Emergency stop events

## Troubleshooting

### Robot Not Moving
1. Check if both nodes are running: `ros2 node list`
2. Verify goal was sent: `ros2 topic echo /goal_pose --once`
3. Check if path exists: `ros2 topic echo /plan --once`
4. View DWA logs for warnings

### Robot Stuck Near Obstacle
- The global planner will replan every 2 seconds
- If truly stuck, send a new goal or restart the planner

### No Path Found
- Goal may be:
  - Out of bounds (grid is 10x10m centered on start)
  - Inside an obstacle
  - Unreachable due to obstacles
- Try a closer goal first

### Robot Behavior Issues
See `DWA_TUNING_GUIDE.md` for parameter tuning instructions.

## Architecture

```
┌──────────────┐
│   Gazebo     │ (Simulation)
│  TurtleBot3  │
└──────┬───────┘
       │ /scan, /odom
       │
┌──────▼──────────────┐         ┌─────────────────┐
│ Global Planner      │◄────────┤  /goal_pose     │
│ (A* on Grid)        │         │  (User Input)   │
└──────┬──────────────┘         └─────────────────┘
       │ /plan
       │
┌──────▼──────────────┐
│  DWA Local Planner  │
│  (Trajectory Opt)   │
└──────┬──────────────┘
       │ /cmd_vel
       │
┌──────▼───────┐
│  TurtleBot3  │
│  (Movement)  │
└──────────────┘
```

## Key Parameters (DWA)

Located in `dwa_node.py` → `DWAConfig` class:

**For obstacle avoidance behavior:**
- `clearance_weight`: 2.5 (higher = more cautious)
- `heading_weight`: 1.5 (higher = more direct paths)
- `rotation_weight`: 1.0 (rotation when blocked)

**For robot limits:**
- `max_speed`: 0.22 m/s
- `max_yaw_rate`: 2.0 rad/s
- `max_accel`: 2.0 m/s² (higher = more responsive)

**For safety:**
- `robot_radius`: 0.22 m
- `safety_margin`: 0.05 m
- `min_obstacle_dist`: 0.15 m (emergency stop)

See `DWA_TUNING_GUIDE.md` for detailed tuning instructions.

## Advanced Usage

### Visualize Path in RViz2
```bash
rviz2
```

Add displays:
- Path → Topic: `/plan`
- LaserScan → Topic: `/scan`
- RobotModel
- TF

### Run Without Global Planner
```bash
# Only run DWA node
ros2 run dwa_local_planner dwa_node
```
Robot will head directly to hardcoded goal (2.0, 0.0) in `dwa_node.py`.

### Change Hardcoded Goal
Edit `dwa_node.py` line 100:
```python
self.final_goal = np.array([2.0, 0.0])  # Change these values
```

## Performance Tips

1. **Smoother motion**: Decrease `max_accel` and `max_delta_yaw_rate`
2. **Faster planning**: Increase `v_res` and `yaw_res` (fewer samples)
3. **Better obstacle avoidance**: Increase `clearance_weight`
4. **More aggressive**: Increase `velocity_weight`, decrease `clearance_weight`

## Limitations

- Global planner grid is fixed at 10x10 meters
- No support for dynamic obstacles in global planning
- Laser scan range limits obstacle detection to ~5 meters
- Simple A* planner (no optimization, no dynamic cost updates)

## Future Improvements

- Use Nav2 global planners (Smac, Theta*)
- Add replanning triggers based on path deviation
- Implement recovery behaviors
- Add map-based planning (SLAM integration)
- Tune costs based on environment type
