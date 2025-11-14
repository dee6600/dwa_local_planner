# 🎛️ DWA Parameter Tuning Guide

> Master the Dynamic Window Approach planner by tuning parameters for your specific robot and environment.

---

## 📋 Quick Navigation

- **New to tuning?** → Start with [Quick Start](#-quick-start)
- **Know what you want?** → Jump to [Parameter Reference](#-parameter-reference)
- **Have a problem?** → See [Common Issues & Solutions](#-common-issues--solutions)
- **Ready to experiment?** → Follow [Systematic Tuning Process](#-systematic-tuning-process)

---

## ⚡ Quick Start

The main parameters are ROS2 parameters in `src/dwa_local_planner/dwa_local_planner/dwa_node.py` (lines 30-70).

**Fastest way to test:**

```bash
# Test with parameters from command line
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  max_speed:=0.20 \
  safety_margin:=0.25 \
  obstacle_weight:=2.0

# Monitor behavior
ros2 topic echo /cmd_vel
```

**Or modify in code:**
Edit `dwa_node.py` lines 30-70 and rebuild.

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

---

## 📊 Parameter Reference (Current Defaults)

All parameters in `dwa_node.py` (lines 30-70):

| Parameter | Default | Min | Max | Units | Impact |
|-----------|---------|-----|-----|-------|--------|
| `max_speed` | 0.15 | 0.05 | 0.30 | m/s | High — controls max velocity |
| `max_turn` | 2.5 | 0.5 | 4.0 | rad/s | High — controls turning ability |
| `step_time` | 0.1 | 0.05 | 0.2 | s | Medium — should match control rate |
| `num_samples` | 200 | 50 | 400 | count | High — more samples = better but slower |
| `safety_margin` | 0.3 | 0.1 | 0.5 | m | High — collision prevention |
| `robot_radius` | 0.105 | 0.05 | 0.3 | m | High — must match actual robot |
| `goal_weight` | 5.0 | 0.1 | 10.0 | scalar | Medium — reaching goal priority |
| `heading_weight` | 2.0 | 0.5 | 5.0 | scalar | Medium — moving toward goal priority |
| `obstacle_weight` | 1.0 | 0.1 | 5.0 | scalar | High — obstacle avoidance priority |
| `smoothness_weight` | 0.1 | 0.0 | 1.0 | scalar | Low — motion smoothness |
| `lookahead_steps` | 100 | 20 | 200 | count | Medium — prediction horizon |
| `goal_tolerance` | 0.1 | 0.05 | 0.3 | m | Low — goal reached threshold |
| `near_goal_distance` | 0.3 | 0.1 | 1.0 | m | Low — trigger near-goal mode |
| `near_goal_heading_boost` | 5.0 | 1.0 | 10.0 | scalar | Low — heading priority near goal |
| `near_goal_min_speed` | 0.01 | 0.001 | 0.05 | m/s | Low — min speed for in-place rotation |

---

## 🎯 Quick Tuning Recipes

### 🚀 "I want FAST navigation"

```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  max_speed:=0.22 \
  heading_weight:=1.0 \
  obstacle_weight:=0.8 \
  safety_margin:=0.2 \
  num_samples:=150
```

**Result:** Robot moves quickly, may cut corners. Good for open spaces.

---

### 🛡️ "I want SAFE navigation"

```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  max_speed:=0.10 \
  heading_weight:=0.5 \
  obstacle_weight:=2.5 \
  safety_margin:=0.4 \
  num_samples:=250
```

**Result:** Robot moves slowly, stays far from obstacles. Good for delicate environments.

---

### ⚖️ "BALANCED (Default starting point)"

```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  max_speed:=0.15 \
  heading_weight:=2.0 \
  obstacle_weight:=1.0 \
  safety_margin:=0.3 \
  num_samples:=200
```

**Result:** Good all-around performance. Start here and adjust.

---

### 🎪 "NARROW CORRIDORS"

```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  robot_radius:=0.10 \
  safety_margin:=0.15 \
  max_speed:=0.12 \
  heading_weight:=2.5
```

**Result:** Robot fits through tight spaces.

---

### 🏃 "PERFORMANCE PRIORITY (Low CPU)"

```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  num_samples:=100 \
  lookahead_steps:=50 \
  step_time:=0.2
```

**Result:** Faster planning, less CPU. May be less optimal.

---

## 🔍 Diagnosis Guide

### Symptom: Excessive Collisions

**Cause → Solution:**

| Cause | Solution |
|-------|----------|
| Safety margin too small | Increase to 0.3-0.4 |
| Obstacle weight too low | Increase to 2.0+ |
| Robot radius incorrect | Set to actual size |
| Lookahead too short | Increase `lookahead_steps` to 150 |

**Quick fix:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  safety_margin:=0.40 obstacle_weight:=2.0
```

---

### Symptom: Robot Gets Stuck / Too Cautious

**Cause → Solution:**

| Cause | Solution |
|-------|----------|
| Obstacle weight too high | Decrease to 0.5-1.0 |
| Safety margin too large | Decrease to 0.2 |
| Heading weight too low | Increase to 3.0+ |
| Not looking far ahead | Increase `lookahead_steps` |

**Quick fix:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  obstacle_weight:=0.8 heading_weight:=3.0 lookahead_steps:=150
```

---

### Symptom: Inefficient / Winding Paths

**Cause → Solution:**

| Cause | Solution |
|-------|----------|
| Heading weight too low | Increase to 3.0+ |
| Goal weight too low | Increase to 7.0+ |
| Sampling too coarse | Increase `num_samples` |

**Quick fix:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  heading_weight:=3.5 goal_weight:=7.0 num_samples:=250
```

---

### Symptom: Jerky / Unstable Motion

**Cause → Solution:**

| Cause | Solution |
|-------|----------|
| Max turn rate too high | Decrease `max_turn` to 1.5 |
| Acceleration limits too aggressive | Not directly tunable (hardcoded) |
| Sampling too sparse | Increase `num_samples` |

**Quick fix:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  max_turn:=1.5 num_samples:=250
```

---

### Symptom: Too Slow

**Cause → Solution:**

| Cause | Solution |
|-------|----------|
| Max speed too low | Increase to 0.20+ |
| Obstacle weight too high | Decrease to 0.8 |
| Velocity weight too low | Increase to 1.5+ |

**Quick fix:**
```bash
ros2 launch dwa_local_planner dwa_gazebo.launch.py \
  max_speed:=0.20 velocity_weight:=1.5 obstacle_weight:=0.8
```

---

## 🧪 Testing Methodology

### Step-by-Step Tuning Process

**Phase 1: Safety (Baseline)**
1. Launch with defaults
2. If collisions → increase `safety_margin` and `obstacle_weight`
3. If stuck → decrease both slightly
4. Goal: Robot moves without colliding

**Phase 2: Efficiency (Optimize Path)**
1. Increase `heading_weight` to 2.5-3.0
2. Test path quality
3. Goal: Smooth, direct paths

**Phase 3: Speed (Maximize Velocity)**
1. Increase `max_speed` up to hardware limit
2. Monitor for instability
3. Goal: Fastest safe speed

**Phase 4: Performance (CPU Usage)**
1. Reduce `num_samples` if CPU high
2. Reduce `lookahead_steps` if needed
3. Goal: <10% CPU on typical hardware

### Validation Checklist

- ✅ Robot reaches goals consistently
- ✅ No collisions with obstacles
- ✅ Path looks efficient (not winding)
- ✅ Motion is smooth (no jerky turns)
- ✅ CPU usage reasonable (<15%)
- ✅ Response time acceptable

---

## 🌍 Environment-Specific Recipes

### 🏭 Industrial Environment (Open, Fast)

```bash
max_speed=0.22          # Full speed
safety_margin=0.2       # Tighter margins
obstacle_weight=0.8     # Less cautious
heading_weight=2.5      # Direct paths
num_samples=150         # Faster
```

### 🏠 Home / Indoor (Clutter, Safe)

```bash
max_speed=0.12          # Moderate
safety_margin=0.35      # Larger margins
obstacle_weight=1.5     # More cautious
heading_weight=2.0      # Balanced
num_samples=200         # Standard
```

### 🌳 Outdoor (Natural Obstacles)

```bash
max_speed=0.18          # Reasonable
safety_margin=0.3       # Standard
obstacle_weight=1.2     # Normal caution
heading_weight=2.5      # Track path
num_samples=200         # Standard
lookahead_steps=150     # Look ahead
```

### 🎪 Narrow Passage

```bash
robot_radius=0.10       # Smaller effective size
safety_margin=0.15      # Minimal
max_speed=0.10          # Slow
heading_weight=3.0      # Stay on path
```

---

## 📈 Performance Monitoring

### View Planning Performance

```bash
# Check planning frequency
ros2 topic hz /cmd_vel
# Target: 10 Hz (10 commands/second)

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Check for stuck situations
ros2 node info /dwa_local_planner
```

### Log Analysis

```bash
# Record data for analysis
ros2 bag record /cmd_vel /odom /scan -o session.bag

# Replay and analyze
ros2 bag play session.bag

# Extract specific topic
ros2 bag info session.bag
```

---

## 💾 Saving Your Tuning

### Method 1: Launch File

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
                'max_speed': 0.18,
                'heading_weight': 2.5,
                'obstacle_weight': 1.2,
                'safety_margin': 0.28,
                'num_samples': 200,
            }]
        ),
    ])
```

Launch: `ros2 launch my_config.launch.py`

### Method 2: Parameter File

Create `my_params.yaml`:

```yaml
dwa_local_planner:
  ros__parameters:
    max_speed: 0.18
    heading_weight: 2.5
    obstacle_weight: 1.2
    safety_margin: 0.28
    num_samples: 200
```

Launch: `ros2 run dwa_local_planner dwa_node --ros-args --params-file my_params.yaml`

---

## 🔗 Related Resources

- **[README.md](README.md)** — Project overview
- **[LAUNCH.md](src/dwa_local_planner/LAUNCH.md)** — Launch configuration
- **[USAGE_GUIDE.md](USAGE_GUIDE.md)** — Usage modes
- **[DWA Algorithm](https://www.researchgate.net/publication/4040859_The_Dynamic_Window_Approach_to_Collision_Avoidance)** — Original paper

---

## 💡 Pro Tips

1. **Change one parameter at a time** — easier to see cause/effect
2. **Use the launch file** — parameters update without rebuilding
3. **Monitor `/cmd_vel`** — see what velocities are being sent
4. **Use RViz** — visualize trajectories to debug behavior
5. **Record with ros2 bag** — replay for detailed analysis
6. **Start conservative** — increase speed gradually
7. **Test thoroughly** — verify in multiple environments

---

**Happy tuning! 🎛️ If you achieve interesting configurations, consider sharing them!**
