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
            #'turtlebot3_world.launch.py'
            'turtlebot3_dqn_stage1.launch.py'
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
            'max_vel_x': float(0.22),                       # Maximum forward speed (m/s) - TurtleBot3 Burger max: 0.22
            'min_vel_x': float(-0.22),                      # Minimum speed / reverse limit (m/s)
            'max_yaw_rate': float(2.84),                    # Maximum rotation rate (rad/s) - TurtleBot3 Burger max: 2.84
            'max_acc_x': float(0.5),                        # Maximum linear acceleration (m/s²)
            'max_acc_yaw': float(2.0),                      # Maximum angular acceleration (rad/s²)
            'v_resolution': float(0.02),                    # Linear velocity sampling step (m/s)
            'omega_resolution': float(0.1),                 # Angular velocity sampling step (rad/s)
            'predict_time': float(1.5),                     # Trajectory lookahead horizon (s)
            'dt': float(0.1),                               # Integration timestep (s)
            'robot_radius': float(0.105),                   # Robot body radius for collision detection (m) - TurtleBot3 Burger radius
            'obstacle_inflation': float(0.05),              # Safety margin around obstacles (m)
            'min_obstacle_dist': float(0.10),               # Minimum safe clearance distance (m)
            'to_goal_cost_gain': float(1.5),                # Goal attraction weight (higher = prioritize goal)
            'speed_cost_gain': float(0.5),                  # Speed preference weight (higher = prefer faster)
            'obstacle_cost_gain': float(2.5),               # Obstacle avoidance weight (higher = stronger avoidance)
            'control_rate': float(10.0),                    # Control loop frequency (Hz)
            'goal_x': float(1.0),                           # Goal x-coordinate (m, in odom frame)
            'goal_y': float(0.5),                           # Goal y-coordinate (m, in odom frame)
            'goal_tolerance': float(0.15),                  # Distance to goal considered reached (m)
        }]
    )

    # Delay DWA node start by 2 seconds to ensure Gazebo and sensors are ready
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

    # Delay RViz start by 5 seconds to allow Gazebo and TurtleBot3 to fully load
    delayed_rviz_start = TimerAction(
        period=1.0,
        actions=[rviz_node]
    )

    # Create launch description with all components
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(gazebo_launch)
    ld.add_action(delayed_rviz_start)     # RViz starts after 5 seconds
    ld.add_action(delayed_dwa_start)      # DWA node starts after 2 seconds
   

    return ld
