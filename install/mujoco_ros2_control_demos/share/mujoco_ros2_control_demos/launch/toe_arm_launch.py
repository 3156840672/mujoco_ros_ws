#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('mujoco_ros2_control_demos')

    # ===== URDF =====
    urdf_file = os.path.join(pkg_path, 'urdf', 'arm0.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ===== YAML =====
    controller_config = os.path.join(pkg_path, 'config', 'toe_arm.yaml')

    # ===== MJCF =====
    mujoco_model = os.path.join(pkg_path, 'mujoco_models', 'toe_arm.xml')

    return LaunchDescription([

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
        ),

        # MuJoCo + ros2_control
        Node(
            package='mujoco_ros2_control',
            executable='mujoco_ros2_control',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                controller_config,
                {'mujoco_model_path': mujoco_model},
                {'use_sim_time': True}
            ],
        ),

        # ===== 控制器加载（关键）=====
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller"],
            output="screen",
        ),

        # RViz（可选）
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}],
        # ),
    ])