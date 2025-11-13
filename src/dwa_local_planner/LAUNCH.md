# DWA Local Planner Launch Guide

## Quick Start

### Build the package
```bash
cd ~/ros2_ws
colcon build --packages-select dwa_local_planner
source install/setup.bash
```

### Run the complete simulation with DWA, Gazebo, and RViz
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py
```

This command will:
- Start Gazebo with TurtleBot3 in the world
- Launch the DWA local planner node
- Open RViz with the DWA visualization configuration

## Launch File Details

**File:** `launch/dwa_gazebo.launch.py`

Includes:
- **Gazebo simulation**: TurtleBot3 in the world environment
- **DWA planner node**: Subscribes to `/scan` and `/odom`, publishes to `/cmd_vel`
- **RViz visualization**: Shows:
  - Grid and coordinate frames
  - Laser scan data
  - All predicted trajectories (faint lines - red=collision, green=valid)
  - Chosen trajectory (bright green line)
  - Trajectory endpoint (orange sphere)

## Parameter Configuration

Default parameters are loaded from the launch file. To override:
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py goal_x:=3.0 goal_y:=2.0
```

Available parameters:
- `use_sim_time`: Use Gazebo simulation time (default: true)
- `max_vel_x`: Maximum forward speed (default: 0.18 m/s)
- `goal_x`, `goal_y`: Target position (default: 2.0, 1.0)
- See `dwa_node.py` for all parameter descriptions

## RViz Configuration

**File:** `config/dwa_rviz.rviz`

Pre-configured displays:
- **LaserScan**: Sensor data from `/scan`
- **Chosen Path**: Green line showing the selected trajectory from `/dwa/chosen_path`
- **Predicted Trajectories**: Faint lines showing all sampled trajectories from `/dwa/trajectories`
- **TF**: Robot frames (odom, base_link, etc.)
- **Grid**: Reference grid in XY plane

## Manual Testing

If you want to run components separately:

**Only Gazebo + Robot:**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Only DWA Planner:**
```bash
ros2 run dwa_local_planner dwa_node
```

**Only RViz:**
```bash
rviz2 -d install/dwa_local_planner/share/dwa_local_planner/config/dwa_rviz.rviz
```

## Troubleshooting

- **Gazebo not found**: Ensure `turtlebot3_simulations` package is in the workspace
- **RViz config not loading**: Check that the config file path is correct after building
- **No visualization**: Verify the `/dwa/trajectories` and `/dwa/chosen_path` topics are being published (use `ros2 topic list`)

## Notes

- Default goal is set to (2.0, 1.0) m in the world frame
- The robot starts at origin (0, 0) facing forward
- Laser scan obstacles and planning visualization update at 10 Hz (see `dt` and `control_rate` parameters)
