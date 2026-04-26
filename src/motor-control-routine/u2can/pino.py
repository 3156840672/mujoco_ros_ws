import mujoco
import mujoco.viewer
import pinocchio as pin
import numpy as np

# 加载模型
model = mujoco.MjModel.from_xml_path('/home/yue/mujoco_ros_ws/src/toe_arm.xml')
data = mujoco.MjData(model)

pin_model = pin.buildModelFromUrdf('/home/yue/mujoco_ros_ws/src/toe_arm.urdf')
pin_data = pin_model.createData()

# ====== 建立名称映射（确保关节顺序一致） ======
mj_joint_names = [model.joint(i).name for i in range(model.njnt)]
pin_joint_names = list(pin_model.names)   # 包含 universe 等基座

# 只取实际关节名称
arm_joint_names = [n for n in pin_joint_names if n in mj_joint_names]

# 按 Pinocchio 的顺序获取 MuJoCo qpos 索引
qpos_idxs = [model.jnt_qposadr[model.joint(name).id] for name in arm_joint_names]

# 按 Pinocchio 的顺序获取 MuJoCo ctrl 索引（因为 actuator 顺序与关节一致）
ctrl_idxs = [model.joint(name).id for name in arm_joint_names]

# ====== 校准系数 ======
def calibrate_gravity_scale():
    """ 计算 Pinocchio 重力力矩到 MuJoCo 真实重力力矩的缩放系数 """
    # 使用当前姿态（通常静止）
    q = data.qpos[qpos_idxs].copy()
    tau_pin = pin.computeGeneralizedGravity(pin_model, pin_data, q)
    
    # MuJoCo 真实重力矩（速度为零时即为纯重力矩）
    tau_mj = data.qfrc_bias[ctrl_idxs].copy()  # 只取机械臂部分
    
    # 避免除零，对每个关节计算比例
    scale = np.ones_like(tau_pin)
    nonzero = np.abs(tau_pin) > 1e-6
    scale[nonzero] = tau_mj[nonzero] / tau_pin[nonzero]
    
    print("Calibration scale:", scale)
    return scale

# 运行一步让仿真初始化
mujoco.mj_step(model, data)
scale = calibrate_gravity_scale()

# ====== 可视化循环 ======
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        
        # 当前关节角
        q = data.qpos[qpos_idxs].copy()
        
        # 修正后的重力补偿力矩
        tau_g_pin = pin.computeGeneralizedGravity(pin_model, pin_data, q)
        tau_compensated = tau_g_pin * scale
        
        # 施加到 MuJoCo（此处只有重力补偿）
        data.ctrl[ctrl_idxs] = tau_compensated
        
        viewer.sync()