from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():

    pkg = get_package_share_directory("toe_arm")

    # MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("arm", package_name="toe_arm")
        .robot_description(file_path="config/arm.urdf.xacro")
        .robot_description_semantic(file_path="config/arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    ros2_controllers = os.path.join(pkg, "config", "ros2_controllers.yaml")

    return LaunchDescription([

        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[moveit_config.robot_description],
            output="screen",
        ),

        # ros2_control (FakeSystem)
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                ros2_controllers,
            ],
            output="screen",
        ),

        # controllers
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller"],
        ),
        

        # MoveIt
        TimerAction(
            period=3.0,
            actions=[
                *generate_move_group_launch(moveit_config).entities
            ],
        ),

        # RViz
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=["-d", os.path.join(pkg, "config/moveit.rviz")],
                    parameters=[moveit_config.to_dict()],
                )
            ],
        ),
    ])