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
        parameters=[{
            'use_sim_time': use_sim_time,                   # Use Gazebo simulation time instead of system time
            # DWAPlannerNode parameter values (aligned with node defaults)
            'max_speed': float(0.15),                       # Node default max_speed (m/s)
            'max_turn': float(2.5),                         # Node default max_turn (rad/s)
            'step_time': float(0.1),                        # Node default step_time (s)
            'num_samples': int(200),                        # Node default num_samples
            'safety_margin': float(0.3),                    # Node default safety_margin (m)
            'lookahead_steps': int(100),                    # Node default lookahead_steps
            'goal_x': float(2.0),                           # Node default goal_x (m)
            'goal_y': float(1.0),                           # Node default goal_y (m)
            'goal_tolerance': float(0.05),                  # Node default goal_tolerance (m)
            # scoring weights (node defaults)
            'goal_weight': float(5.0),
            'heading_weight': float(2.0),
            'smoothness_weight': float(0.1),
            'obstacle_weight': float(1.0),
            # visualization frame
            'visual_frame': 'odom',
            # Legacy params (kept for external compatibility) - aligned when sensible
            'max_vel_x': float(0.15),
            'min_vel_x': float(-0.15),
            'max_yaw_rate': float(2.5),
            'max_acc_x': float(0.5),
            'max_acc_yaw': float(2.0),
            'v_resolution': float(0.02),
            'omega_resolution': float(0.1),
            # Keep predict_time consistent with lookahead (predict_time = step_time * lookahead_steps)
            'predict_time': float(0.1 * 100),
            'dt': float(0.1),
            'robot_radius': float(0.105),
            'obstacle_inflation': float(0.05),
            'min_obstacle_dist': float(0.10),
            'to_goal_cost_gain': float(1.5),
            'speed_cost_gain': float(0.5),
            'obstacle_cost_gain': float(2.5),
            'control_rate': float(10.0),
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
