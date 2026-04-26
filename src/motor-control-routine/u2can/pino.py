import math
import time
import numpy as np
import serial
import pinocchio as pin
import mujoco
import mujoco.viewer

from DM_CAN import *

# =========================================================
# 1️⃣ 电机定义
# =========================================================
Motor1 = Motor(DM_Motor_Type.DM4340, 0x01, 0x11)
Motor2 = Motor(DM_Motor_Type.DM4340, 0x02, 0x12)
Motor3 = Motor(DM_Motor_Type.DM4340, 0x03, 0x13)
Motor4 = Motor(DM_Motor_Type.DM4310, 0x04, 0x14)
Motor5 = Motor(DM_Motor_Type.DM4310, 0x05, 0x15)
Motor6 = Motor(DM_Motor_Type.DM4310, 0x06, 0x16)

motors = [Motor1, Motor2, Motor3, Motor4, Motor5, Motor6]

# =========================================================
# 2️⃣ 串口
# =========================================================
serial_device = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=921600,
    timeout=0.5
)

mc = MotorControl(serial_device)

for m in motors:
    mc.addMotor(m)

# =========================================================
# 3️⃣ MIT模式 + 使能
# =========================================================
for m in motors:
    mc.switchControlMode(m, Control_Type.MIT)

time.sleep(1)

for m in motors:
    mc.enable(m)

print("All motors enabled")
time.sleep(1)

# =========================================================
# 4️⃣ Pinocchio
# =========================================================
pin_model = pin.buildModelFromUrdf(
    '/home/yue/mujoco_ros_ws/src/toe_arm.urdf'
)
pin_data = pin_model.createData()

# =========================================================
# 5️⃣ MuJoCo
# =========================================================
mj_model = mujoco.MjModel.from_xml_path(
    "/home/yue/mujoco_ros_ws/src/toe_arm.xml"
)
mj_data = mujoco.MjData(mj_model)

viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

# =========================================================
# 6️⃣ 关节标定（必须保留）
# =========================================================
q_offset = np.array([
    -2.777,
    -2.296,
    0.3,
    -0.34,
    2.3,
    3.0
], dtype=np.float64)

# =========================================================
# 7️⃣ 电机方向修正
# =========================================================

S = np.array([-1, 1.1, -1.2, -1.1, -1.1, 1])

# =========================================================
# 8️⃣ 主循环参数
# =========================================================
DT = 0.002

print("Start control + MuJoCo visualization...")

# =========================================================
# 9️⃣ 主循环
# =========================================================
while viewer.is_running():

    # =========================
    # 1. 读取电机角度
    # =========================
    q_raw = np.array([
        -Motor1.getPosition(),
        Motor2.getPosition(),
        -Motor3.getPosition(),
        -Motor4.getPosition(),
        -Motor5.getPosition(),
        Motor6.getPosition()
    ], dtype=np.float64)

    q = q_raw + q_offset

    # =========================
    # 2. Pinocchio 正向运动学 + 动力学
    # =========================
    pin.forwardKinematics(pin_model, pin_data, q)

    tau_g = pin.rnea(
        pin_model,
        pin_data,
        q,
        np.zeros(6),
        np.zeros(6)
    )

    tau_g = np.clip(tau_g, -8.0, 8.0)

    # =========================
    # 3. MIT 控制（实机）
    # =========================
    for i, m in enumerate(motors):
        mc.controlMIT(
            m,
            kp=0.0,
            kd=1.5,
            q=0.0,
            dq=0.0,
            tau=S[i]* tau_g[i]
        )

    # =========================
    # 4. MuJoCo 同步（核心）
    # =========================
    mj_data.qpos[:6] = q
    mj_data.qvel[:] = 0.0

    mujoco.mj_forward(mj_model, mj_data)
    viewer.sync()

    # =========================
    # 5. debug 输出
    # =========================
    print("--------------------------------------------------")
    for i in range(6):
        print(f"M{i+1}: q={q[i]:.3f} tau={tau_g[i]:.3f}")

    time.sleep(DT)

# =========================================================
# 10️⃣ 关闭串口
# =========================================================
serial_device.close()