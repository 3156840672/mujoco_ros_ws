from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():

    moveit_config = MoveItConfigsBuilder(
        robot_name="arm",
        package_name="toe_arm2"
    ).robot_description(
        file_path="config/arm.urdf.xacro"   # ⭐ 必须加
    ).robot_description_semantic(
        file_path="config/arm.srdf"
    ).trajectory_execution(
        file_path="config/moveit_controllers.yaml"
    ).to_moveit_configs()
    return generate_move_group_launch(moveit_config)