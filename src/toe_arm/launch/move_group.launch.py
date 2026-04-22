from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("arm", package_name="toe_arm")
        .robot_description(file_path="config/arm.urdf.xacro")
        .robot_description_semantic(file_path="config/arm.srdf")

        # ⭐关键：一定要加载控制器配置
        .trajectory_execution(file_path="config/moveit_controllers.yaml")

        .to_moveit_configs()
    )

    

    return generate_move_group_launch(moveit_config)