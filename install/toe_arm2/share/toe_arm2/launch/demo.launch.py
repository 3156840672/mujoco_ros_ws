from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # 使用 MoveItConfigsBuilder 构建配置
    # "arm" 应该与你的 URDF 文件中 <robot name="arm"> 的名称一致
    # package_name="toe_arm2" 应该是包含你的 MoveIt 配置包的 ROS2 包名
    moveit_config = MoveItConfigsBuilder("arm", package_name="toe_arm2").to_moveit_configs()
    
    # 生成并返回 demo launch 描述
    return generate_demo_launch(moveit_config)