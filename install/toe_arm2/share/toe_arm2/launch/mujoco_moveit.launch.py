#!/usr/bin/env python3
import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# MoveIt
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():

    # ================= 路径 =================
    mujoco_pkg = get_package_share_directory('mujoco_ros2_control_demos')
    moveit_pkg = get_package_share_directory('toe_arm2')

    # ================= xacro（统一模型）=================
    xacro_file = os.path.join(moveit_pkg, 'config', 'arm.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # ================= Mujoco =================
    controller_yaml = os.path.join(
        mujoco_pkg, 'config', 'toe_arm.yaml'
    )

    mujoco_model = os.path.join(
        mujoco_pkg, 'mujoco_models', 'toe_arm.xml'
    )

    # ================= MoveIt =================
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="arm",
            package_name="toe_arm2"
        )
        .robot_description(file_path="config/arm.urdf.xacro")
        .robot_description_semantic(file_path="config/arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    return LaunchDescription([

        # ===== use_sim_time =====
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        # ===== robot_state_publisher =====
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        ),

        # ===== MuJoCo（核心）=====
        Node(
            package='mujoco_ros2_control',
            executable='mujoco_ros2_control',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                controller_yaml,
                {'mujoco_model_path': mujoco_model},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),

        # ===== 加载 joint_state_broadcaster =====
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    output="screen",
                )
            ],
        ),

        # ===== 加载 arm_controller（关键）=====
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_controller"],
                    output="screen",
                )
            ],
        ),

        # ===== MoveIt =====
        TimerAction(
            period=8.0,
            actions=[
                *generate_move_group_launch(moveit_config).entities,
            ],
        ),

        # ===== RViz =====
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=[
                        '-d',
                        os.path.join(moveit_pkg, 'config', 'moveit.rviz')
                    ],
                    parameters=[
                        moveit_config.to_dict(),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                )
            ],
        ),
    ])