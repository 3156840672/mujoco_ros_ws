import mujoco
import mujoco.viewer
import pinocchio as pin
import numpy as np

# ===== MuJoCo =====
model = mujoco.MjModel.from_xml_path('/home/yue/mujoco_ros_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/mujoco_models/toe_arm.xml')
data = mujoco.MjData(model)

# ===== Pinocchio =====
pin_model = pin.buildModelFromUrdf('/home/yue/mujoco_ros_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/urdf/arm0.urdf')
pin_data = pin_model.createData()

# ===== 启动可视化 =====
with mujoco.viewer.launch_passive(model, data) as viewer:
    print(model.joint_names)
    print(pin_model.names)
    while viewer.is_running():

        mujoco.mj_step(model, data)

        # 当前关节角
        q = data.qpos.copy()

        # 重力补偿
        tau_g = pin.computeGeneralizedGravity(pin_model, pin_data, q)

        # 控制输入
        data.ctrl[:] = tau_g

        viewer.sync()



