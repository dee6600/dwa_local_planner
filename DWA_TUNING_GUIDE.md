# DWA Parameter Tuning Guide

This guide explains how to tune the DWA (Dynamic Window Approach) planner parameters for optimal performance.

## Quick Start

The main parameters to tune are in the `DWAConfig` class in `src/dwa_local_planner/dwa_local_planner/dwa_node.py`.

## Parameter Categories

### 1. Robot Kinematic Limits

These should match your robot's physical capabilities.

| Parameter | Default | Description | Tuning Tips |
|-----------|---------|-------------|-------------|
| `max_speed` | 0.22 m/s | Maximum linear velocity | **Reduce** if robot overshoots goals or acts unstable. **Increase** for faster navigation (don't exceed hardware limits). |
| `min_speed` | -0.1 m/s | Minimum linear velocity (negative = reverse) | Keep small. Too negative makes robot back up frequently. |
| `max_yaw_rate` | 2.0 rad/s | Maximum angular velocity | **Higher** = sharper turns but may cause instability. **Lower** = smoother, wider turns. |
| `max_accel` | 0.5 m/s² | Maximum linear acceleration | **Lower** = smoother acceleration, gentler motion. **Higher** = more responsive but jerky. |
| `max_delta_yaw_rate` | 1.5 rad/s² | Maximum angular acceleration | **Lower** = smoother turning. **Higher** = more abrupt direction changes. |

### 2. Sampling Resolution

Controls how finely the algorithm searches for optimal velocities.

| Parameter | Default | Description | Tuning Tips |
|-----------|---------|-------------|-------------|
| `v_res` | 0.01 m/s | Linear velocity sampling step | **Smaller** = more samples, better optimization, **slower computation**. Try 0.02 for faster planning. |
| `yaw_res` | 0.1 rad/s | Angular velocity sampling step | **Smaller** = finer angular control. Try 0.05 for smoother paths or 0.2 for faster computation. |

### 3. Prediction Parameters

| Parameter | Default | Description | Tuning Tips |
|-----------|---------|-------------|-------------|
| `predict_time` | 2.0 s | How far ahead to simulate trajectories | **Longer** = more foresight, avoids distant obstacles, **slower**. **Shorter** = faster but may miss far obstacles. Try 1.5-3.0s. |
| `dt` | 0.1 s | Simulation time step | Should match control loop rate (10 Hz = 0.1s). Don't change unless you change the timer. |

### 4. Cost Function Weights ⭐ **MOST IMPORTANT FOR TUNING**

These determine the robot's behavior priorities. Higher weight = higher priority.

| Parameter | Default | Description | When to Adjust |
|-----------|---------|-------------|----------------|
| `heading_weight` | 1.0 | How much to prioritize moving toward goal | **Increase** (1.5-2.0) if robot wanders or takes inefficient paths. **Decrease** (0.5-0.8) if robot is too aggressive and hits obstacles. |
| `clearance_weight` | 1.5 | How much to prioritize staying away from obstacles | **Increase** (2.0-3.0) if robot collides or cuts corners too close. **Decrease** (0.8-1.2) if robot is too timid or gets stuck. |
| `velocity_weight` | 0.8 | How much to prioritize moving fast | **Increase** (1.0-1.5) if robot moves too slowly. **Decrease** (0.3-0.6) if robot is too fast and reckless. |

**Recommended Starting Configurations:**

- **Cautious (safe, slower)**
  ```python
  heading_weight = 0.8
  clearance_weight = 2.5
  velocity_weight = 0.5
  ```

- **Balanced (default)**
  ```python
  heading_weight = 1.0
  clearance_weight = 1.5
  velocity_weight = 0.8
  ```

- **Aggressive (fast, risky)**
  ```python
  heading_weight = 1.5
  clearance_weight = 1.0
  velocity_weight = 1.2
  ```

### 5. Safety Parameters

| Parameter | Default | Description | Tuning Tips |
|-----------|---------|-------------|-------------|
| `robot_radius` | 0.22 m | Robot's physical radius | Should match or **slightly exceed** actual robot size. TurtleBot3 Waffle Pi ≈ 0.22m. |
| `safety_margin` | 0.1 m | Extra clearance buffer | **Increase** (0.15-0.2) for more caution. **Decrease** (0.05-0.08) if robot can't fit through passages. |
| `min_obstacle_dist` | 0.25 m | Emergency stop distance | Robot stops if any obstacle is closer. Keep > `robot_radius`. |

## Common Issues and Solutions

### Issue: Robot Collides with Obstacles

**Solutions:**
1. ✅ **Increase** `clearance_weight` to 2.0 or higher
2. ✅ **Increase** `safety_margin` to 0.15m
3. ✅ **Decrease** `max_speed` to 0.15 m/s
4. ✅ **Decrease** `velocity_weight` to 0.5
5. ✅ Check that `robot_radius` matches your robot

### Issue: Robot is Too Cautious / Gets Stuck

**Solutions:**
1. ✅ **Decrease** `clearance_weight` to 1.0-1.2
2. ✅ **Decrease** `safety_margin` to 0.05m
3. ✅ **Increase** `predict_time` to 2.5-3.0s (look farther ahead)
4. ✅ **Increase** `min_speed` to allow more reverse motion (-0.15)

### Issue: Robot Takes Inefficient/Winding Paths

**Solutions:**
1. ✅ **Increase** `heading_weight` to 1.5-2.0
2. ✅ **Decrease** `clearance_weight` to 1.0-1.2
3. ✅ **Increase** `predict_time` to 2.5s

### Issue: Robot Moves Too Slowly

**Solutions:**
1. ✅ **Increase** `velocity_weight` to 1.2-1.5
2. ✅ **Increase** `max_speed` to 0.26 m/s (don't exceed hardware limits)
3. ✅ **Increase** `max_accel` to 0.8 m/s²

### Issue: Robot Motion is Jerky/Unstable

**Solutions:**
1. ✅ **Decrease** `max_accel` to 0.3 m/s²
2. ✅ **Decrease** `max_delta_yaw_rate` to 1.0 rad/s²
3. ✅ **Decrease** `max_yaw_rate` to 1.5 rad/s
4. ✅ **Increase** `v_res` to 0.02 m/s (fewer samples, smoother)

### Issue: Planner is Too Slow / High CPU Usage

**Solutions:**
1. ✅ **Increase** `v_res` to 0.02 m/s
2. ✅ **Increase** `yaw_res` to 0.15-0.2 rad/s
3. ✅ **Decrease** `predict_time` to 1.5s
4. ✅ Consider reducing `max_speed` and `max_yaw_rate` ranges

## Systematic Tuning Process

Follow these steps for best results:

### Step 1: Verify Robot Physical Parameters
```python
# Make sure these match your robot
robot_radius = 0.22  # TurtleBot3 Waffle Pi
max_speed = 0.22     # Check robot specs
max_yaw_rate = 2.0   # Check robot specs
```

### Step 2: Start with Conservative Weights
```python
heading_weight = 1.0
clearance_weight = 2.0  # Start high for safety
velocity_weight = 0.5   # Start low
```

### Step 3: Test in Simple Environment
- Run robot toward a goal with no obstacles
- Verify it reaches the goal smoothly

### Step 4: Test with Obstacles
- Add obstacles between robot and goal
- Observe behavior

### Step 5: Tune Iteratively
1. If collisions occur → **increase `clearance_weight`**
2. If too cautious → **decrease `clearance_weight`**, **increase `heading_weight`**
3. If too slow → **increase `velocity_weight`**, **increase `max_speed`**
4. If path is inefficient → **increase `heading_weight`**

### Step 6: Fine-tune for Smoothness
- Adjust acceleration limits for desired smoothness
- Adjust sampling resolution for performance vs quality trade-off

## Testing Commands

After changing parameters:

```bash
# Rebuild
colcon build --packages-select dwa_local_planner

# Source
source install/setup.bash

# Launch Gazebo (terminal 1)
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Run planner (terminal 2)
source install/setup.bash
ros2 run dwa_local_planner dwa_node
```

## Advanced: Environment-Specific Tuning

### Narrow Corridors
```python
clearance_weight = 1.0  # Lower
safety_margin = 0.05    # Smaller
max_speed = 0.15        # Slower
```

### Open Spaces
```python
velocity_weight = 1.2   # Higher
max_speed = 0.26        # Faster
predict_time = 2.5      # Look farther
```

### Dense Obstacles
```python
clearance_weight = 2.5  # Higher
predict_time = 2.5      # Look farther
max_speed = 0.18        # Moderate
```

### Dynamic Environments
```python
predict_time = 1.5      # Shorter for responsiveness
max_accel = 0.8         # More responsive
clearance_weight = 2.0  # Stay cautious
```

## Monitoring and Debugging

Add these to see what's happening:

```python
# In dwa_planning method, after finding best trajectory:
self.get_logger().info(
    f'Best cmd: v={best_v:.2f}, w={best_w:.2f}, score={best_score:.2f}'
)
```

Watch the logs to see if:
- Scores are reasonable
- Robot finds valid trajectories
- Velocities make sense

## Parameter File Template

Create a copy for your custom settings:

```python
# my_dwa_config.py
class MyDWAConfig(DWAConfig):
    def __init__(self):
        super().__init__()
        # Override defaults here
        self.heading_weight = 1.2
        self.clearance_weight = 1.8
        self.velocity_weight = 0.9
        # ... etc
```

Then in the node:
```python
self.config = MyDWAConfig()  # Instead of DWAConfig()
```
