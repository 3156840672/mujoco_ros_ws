import math
from DM_CAN import *
import serial
import time

# ====== 1️⃣ 定义6个电机 ======
Motor1 = Motor(DM_Motor_Type.DM4340, 0x01, 0x11)#顺时针-，0- -5.59
Motor2 = Motor(DM_Motor_Type.DM4340, 0x02, 0x12)#上-，2.295- -0.83
Motor3 = Motor(DM_Motor_Type.DM4340, 0x03, 0x13)#上+，-0.065-3.027
Motor4 = Motor(DM_Motor_Type.DM4310, 0x04, 0x14)#顺时针-，-1.580-4.027
Motor5 = Motor(DM_Motor_Type.DM4310, 0x05, 0x15)#上+，0-3.148
Motor6 = Motor(DM_Motor_Type.DM4310, 0x06, 0x16) #顺时针-，5.671-0.26

# ====== 2️⃣ Linux 串口（⚠️关键修改）======
serial_device = serial.Serial(
    port='/dev/ttyACM0',   # ⚠️ Linux串口
    baudrate=921600,
    timeout=0.5
)

# ====== 3️⃣ 控制器 ======
MotorControl1 = MotorControl(serial_device)

# ====== 4️⃣ 注册电机 ======
MotorControl1.addMotor(Motor1)
MotorControl1.addMotor(Motor2)
MotorControl1.addMotor(Motor3)
MotorControl1.addMotor(Motor4)
MotorControl1.addMotor(Motor5)
MotorControl1.addMotor(Motor6)

if MotorControl1.switchControlMode(Motor1,Control_Type.POS_VEL):
    print("switch POS_VEL success")
if MotorControl1.switchControlMode(Motor2,Control_Type.POS_VEL):
    print("switch POS_VEL success")
if MotorControl1.switchControlMode(Motor3,Control_Type.POS_VEL):
    print("switch POS_VEL success")
if MotorControl1.switchControlMode(Motor4,Control_Type.POS_VEL):
    print("switch POS_VEL success")
if MotorControl1.switchControlMode(Motor5,Control_Type.POS_VEL):
    print("switch POS_VEL success")
if MotorControl1.switchControlMode(Motor6,Control_Type.POS_VEL):
    print("switch POS_VEL success")

# MotorControl1.save_motor_param(Motor1)
# MotorControl1.save_motor_param(Motor2)
# MotorControl1.save_motor_param(Motor3)
# MotorControl1.save_motor_param(Motor4)
# MotorControl1.save_motor_param(Motor5)
# MotorControl1.save_motor_param(Motor6)

print("Start reading motor states...")
time.sleep(1)
MotorControl1.disable(Motor1)
MotorControl1.disable(Motor2)
MotorControl1.disable(Motor3)
MotorControl1.disable(Motor4)
MotorControl1.disable(Motor5)
MotorControl1.disable(Motor6)

# print("PMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.PMAX))
# print("MST_ID:",MotorControl1.read_motor_param(Motor1,DM_variable.MST_ID))
# print("VMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.VMAX))
# print("TMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.TMAX))
# print("Motor2:")
# print("PMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.PMAX))
# print("MST_ID:",MotorControl1.read_motor_param(Motor2,DM_variable.MST_ID))
# print("VMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.VMAX))
# print("TMAX:",MotorControl1.read_motor_param(Motor2,DM_variable.TMAX))
# ====== 5️⃣ 主循环 ======
while True:

    # 👉 逐个刷新（更稳定）
    MotorControl1.refresh_motor_status(Motor1)
    time.sleep(0.002)

    MotorControl1.refresh_motor_status(Motor2)
    time.sleep(0.002)

    MotorControl1.refresh_motor_status(Motor3)
    time.sleep(0.002)

    MotorControl1.refresh_motor_status(Motor4)
    time.sleep(0.002)

    MotorControl1.refresh_motor_status(Motor5)
    time.sleep(0.002)

    MotorControl1.refresh_motor_status(Motor6)
    time.sleep(0.002)

    #MotorControl1.control_Pos_Vel(Motor6,3,0.1)
    # MotorControl1.control_Pos_Vel(Motor1, -2.777, 0.1)
    # MotorControl1.control_Pos_Vel(Motor2, 2.296, 0.1)
    # MotorControl1.control_Pos_Vel(Motor3, -0.06, 0.1)
    # MotorControl1.control_Pos_Vel(Motor4, -0.34, 0.1)
    # MotorControl1.control_Pos_Vel(Motor5, 2.0, 0.1)
    # MotorControl1.control_Pos_Vel(Motor6, 3.0, 0.1)

    # 👉 打印
    print("--------------------------------------------------")

    print(f"M1: {Motor1.getPosition():.3f}, {Motor1.getVelocity():.3f}, {Motor1.getTorque():.3f}")
    print(f"M2: {Motor2.getPosition():.3f}, {Motor2.getVelocity():.3f}, {Motor2.getTorque():.3f}")
    print(f"M3: {Motor3.getPosition():.3f}, {Motor3.getVelocity():.3f}, {Motor3.getTorque():.3f}")
    print(f"M4: {Motor4.getPosition():.3f}, {Motor4.getVelocity():.3f}, {Motor4.getTorque():.3f}")
    print(f"M5: {Motor5.getPosition():.3f}, {Motor5.getVelocity():.3f}, {Motor5.getTorque():.3f}")
    print(f"M6: {Motor6.getPosition():.3f}, {Motor6.getVelocity():.3f}, {Motor6.getTorque():.3f}")

    time.sleep(0.5)

# ====== 结束 ======
serial_device.close()