import math
from DM_CAN import *
import serial
import time
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import mujoco
import glfw


# =========================================================
# 🔥 1️⃣ 电机定义
# =========================================================
Motor1 = Motor(DM_Motor_Type.DM4340, 0x01, 0x11)
Motor2 = Motor(DM_Motor_Type.DM4340, 0x02, 0x12)
Motor3 = Motor(DM_Motor_Type.DM4340, 0x03, 0x13)
Motor4 = Motor(DM_Motor_Type.DM4310, 0x04, 0x14)
Motor5 = Motor(DM_Motor_Type.DM4310, 0x05, 0x15)
Motor6 = Motor(DM_Motor_Type.DM4310, 0x06, 0x16)


# =========================================================
# 🔥 2️⃣ 串口
# =========================================================
serial_device = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=921600,
    timeout=0.5
)

MotorControl1 = MotorControl(serial_device)

motors = [Motor1, Motor2, Motor3, Motor4, Motor5, Motor6]

for m in motors:
    MotorControl1.addMotor(m)
    MotorControl1.switchControlMode(m, Control_Type.MIT)
    MotorControl1.enable(m)


# =========================================================
# 🔥 MuJoCo 初始化
# =========================================================
model = mujoco.MjModel.from_xml_path(
    '/home/yue/mujoco_ros_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/mujoco_models/toe_arm.xml'
)
data = mujoco.MjData(model)

if not glfw.init():
    raise RuntimeError("GLFW fail")

window = glfw.create_window(1000, 800, "Arm Sim", None, None)
glfw.make_context_current(window)

cam = mujoco.MjvCamera()
opt = mujoco.MjvOption()
scene = mujoco.MjvScene(model, maxgeom=10000)
con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

mujoco.mjv_defaultCamera(cam)
mujoco.mjv_defaultOption(opt)

cam.distance = 1.5


# =========================================================
# 🔥 ROS2 Node
# =========================================================
class JointNode(Node):

    def __init__(self):
        super().__init__("joint_node")

        self.lock = threading.Lock()

        self.target = np.zeros(6)

        # ✔ 固定顺序（关键）
        self.joint_order = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6"
        ]

        self.create_subscription(
            JointState,
            "/joint_states",
            self.cb,
            10
        )

    def cb(self, msg):

        if len(msg.position) < 6:
            return

        if np.any(np.isnan(msg.position)):
            return

        # ✔ name → value 映射（修复乱序核心）
        joint_map = dict(zip(msg.name, msg.position))

        try:
            q = np.array([joint_map[j] for j in self.joint_order])
        except KeyError:
            return

        with self.lock:
            self.target = q


# =========================================================
# 🔥 ROS2 启动
# =========================================================
rclpy.init()
node = JointNode()
joint_order = node.joint_order

threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()


# =========================================================
# 🔥 主循环
# =========================================================
while True:

    if glfw.window_should_close(window):
        break

    with node.lock:
        q = node.target.copy()


    # =====================================================
    # 1️⃣ 电机控制
    # =====================================================
    cmd = [
        float(q[0]) - 2.777,
        -float(q[1]) + 2.296,
        float(q[2]) - 0.0,
        -float(q[3]) -0.34,
        2.0 - float(q[4]),
        float(q[5]) + 3.0
    ]

    for i in range(6):
        MotorControl1.controlMIT(motors[i],25, 0.1, cmd[i], 0, 0)

    # for i in range(6):
    #     MotorControl1.control_Pos_Vel(motors[i], cmd[i], 0.1)

    pos = [
        motors[0].getPosition()+ 2.777,
        motors[1].getPosition()- 2.296,
        motors[2].getPosition()+ 0.0,
        motors[3].getPosition()+ 0.34,
        motors[4].getPosition()- 2.0,
        motors[5].getPosition()- 3.0
    ]
    



    # =====================================================
    # 2️⃣ MuJoCo 同步
    # =====================================================
    for i in range(6):
        data.qpos[i] = q[i]

    mujoco.mj_forward(model, data)

    viewport = mujoco.MjrRect(0, 0, 1000, 800)
    mujoco.mjv_updateScene(
        model, data, opt,
        None, cam,
        mujoco.mjtCatBit.mjCAT_ALL.value,
        scene
    )

    mujoco.mjr_render(viewport, scene, con)

    glfw.swap_buffers(window)
    glfw.poll_events()


    # =====================================================
    # 3️⃣ 打印（修复 self + 顺序）
    # =====================================================
    print("\n==============================")
    print("Joint States (ordered):")
    for i in range(6):
        print(f"{joint_order[i]}: {q[i]:.3f} || {pos[i]:.3f} ")
    print("==============================")


    time.sleep(0.01)


serial_device.close()