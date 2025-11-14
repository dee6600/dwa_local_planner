#!/usr/bin/env python3
"""
Launch file for DWA Local Planner with Gazebo simulation and RViz visualization.

Runs:
  - Gazebo with TurtleBot3 in the world
  - DWA local planner node
  - RViz for visualization (starts after TurtleBot3 is loaded)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package shares
    dwa_pkg_share = FindPackageShare('dwa_local_planner')
    turtlebot3_gazebo_share = FindPackageShare('turtlebot3_gazebo')

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (for Gazebo)'
    )

    # Include TurtleBot3 Gazebo world launch
    gazebo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            turtlebot3_gazebo_share,
            'launch',
            'turtlebot3_world.launch.py'
            #'turtlebot3_dqn_stage1.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'model': 'burger',                          # Use TurtleBot3 model
        }.items()
    )

    # DWA Local Planner node (wrapped in TimerAction to start after Gazebo is ready)
    dwa_node = Node(
        package='dwa_local_planner',
        executable='dwa_node',
        name='dwa_local_planner',
        output='screen',
        # Tuning notes (inline guidance):
        # - `max_speed`: Increase for faster traversal; lower for safer, more stable control.
        #   Suggested range: 0.05 - 0.30 m/s for small differential-drive robots.
        # - `max_turn`: Max angular velocity. Larger values allow quick rotations but
        #   may produce oscillations. Suggested range: 1.0 - 3.0 rad/s.
        # - `step_time`: Control loop timestep and integration step. Smaller values
        #   increase simulation fidelity but cost CPU. Suggested 0.05 - 0.15 s.
        # - `lookahead_steps`: Number of integration steps per candidate trajectory.
        #   Combined with `step_time` gives the effective planning horizon
        #   (`predict_time = step_time * lookahead_steps`). Longer horizons improve
        #   goal-directed behavior but increase computation and may reduce reactivity.
        # - `num_samples`: Number of sampled velocity candidates per control cycle.
        #   Higher -> better coverage and smoother choices, but higher CPU. Try
        #   100-500; lower on slow hardware.
        # - `safety_margin`: Extra buffer when checking collisions (m). Increase for
        #   noisy sensors or to be conservative. Typical: 0.05 - 0.5 m.
        # - `goal_weight` vs `obstacle_weight`: Larger `goal_weight` makes the
        #   planner more aggressive toward the goal; larger `obstacle_weight`
        #   increases conservatism. Tune to trade off speed vs safety.
        # - `smoothness_weight`: Penalizes high angular rates to encourage smoother
        #   trajectories. Increase to reduce oscillations.
        # - `visual_frame`: Frame id for RViz markers. Usually `odom` or `map`.
        parameters=[{
            'use_sim_time': use_sim_time,                   # Use Gazebo simulation time instead of system time
            # DWAPlannerNode parameter values (tune these for performance/behavior)
            'max_speed': float(0.10),                       # max forward speed (m/s). Lower -> safer, higher -> faster
            'max_turn': float(2.5),                         # max angular speed (rad/s). Higher -> quicker rotations
            'step_time': float(0.1),                        # integration/control timestep (s). Lower -> more precise
            'num_samples': int(200),                        # number of sampled candidates per cycle; trade CPU vs quality
            'safety_margin': float(0.3),                    # safety buffer for collision checks (m). Increase for noisy sensors
            'lookahead_steps': int(300),                    # integration steps per candidate; horizon = step_time * lookahead_steps
            'goal_x': float(2.0),                           # goal x-coordinate in `visual_frame` (m)
            'goal_y': float(1.0),                           # goal y-coordinate in `visual_frame` (m)
            'goal_tolerance': float(0.05),                  # distance to goal considered reached (m)
            # scoring weights (adjust to change trade-offs):
            'goal_weight': float(1.0),                      # higher -> prioritize reaching the goal faster
            'heading_weight': float(0.1),                   # higher -> prefer trajectories pointing towards the goal
            'smoothness_weight': float(0.1),                # higher -> penalize sharp turns / oscillations
            'obstacle_weight': float(0.5),                  # higher -> more conservative obstacle avoidance
            # visualization frame for RViz markers (usually 'odom' or 'map')
            'visual_frame': 'odom',
        }],
        remappings=[('visual_paths', '/dwa/trajectories')]
    )

    # Delay DWA node start by a few seconds to ensure Gazebo and sensors are ready
    delayed_dwa_start = TimerAction(
        period=3.0,
        actions=[dwa_node]
    )

    # RViz node with config file (wrapped in TimerAction to wait for TurtleBot to load)
    rviz_config = PathJoinSubstitution([
        dwa_pkg_share,
        'config',
        'my_turtlebot_config.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    # Delay RViz start by several seconds to allow Gazebo and TurtleBot3 to fully load
    delayed_rviz_start = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )

    # Create launch description with all components
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gazebo_launch)
    ld.add_action(delayed_rviz_start)     # RViz starts after 5 seconds
    ld.add_action(delayed_dwa_start)      # DWA node starts after a short delay
   

    return ld
