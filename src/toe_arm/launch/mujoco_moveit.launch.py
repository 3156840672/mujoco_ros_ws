#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
import xacro
# ⭐ MoveIt 正确加载方式
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():

    # ================= 路径 =================
    mujoco_demos_pkg = get_package_share_directory('mujoco_ros2_control_demos')
    moveit_config_pkg = get_package_share_directory('toe_arm')

    # ================= URDF =================
    xacro_file = os.path.join(moveit_config_pkg, 'config', 'arm.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toxml()

    # ================= Mujoco =================
    controller_config_file = os.path.join(
        mujoco_demos_pkg, 'config', 'toe_arm.yaml'
    )

    mujoco_model_path = os.path.join(
        mujoco_demos_pkg, 'mujoco_models', 'toe_arm.xml'
    )

    # ================= MoveIt（关键）=================
    moveit_config = (
    MoveItConfigsBuilder("arm", package_name="toe_arm")
    .robot_description(file_path="config/arm.urdf.xacro")
    .robot_description_semantic(file_path="config/arm.srdf")
    .moveit_cpp(file_path="config/moveit_controllers.yaml")  # ⭐关键
    .to_moveit_configs()
    )
    # ================= Launch =================
    return LaunchDescription([

        # ===== 参数 =====
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
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

        # ===== MuJoCo =====
        Node(
            package='mujoco_ros2_control',
            executable='mujoco_ros2_control',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                controller_config_file,
                {'mujoco_model_path': mujoco_model_path},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),

        # ===== 启动 controller =====
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'control', 'load_controller',
                        '--set-state', 'active',
                        'joint_state_broadcaster'
                    ],
                    output='screen',
                ),
            ]
        ),

        TimerAction(
            period=7.0,
            actions=[
                Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller"],
                output="screen",
            )
            ]
        ),

        # ===== MoveIt（关键修复点）=====
        TimerAction(
            period=10.0,
            actions=[
                # ⭐ 正确加载 MoveIt 全配置（包含 SRDF、规划器等）
                *generate_move_group_launch(moveit_config).entities,
            ]
        ),

        # ===== RViz =====
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    output='screen',
                    arguments=[
                        '-d',
                        os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
                    ],
                    parameters=[
                        moveit_config.to_dict(),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                ),
            ]
        ),
    ])