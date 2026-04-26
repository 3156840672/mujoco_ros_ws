#!/usr/bin/env python3
import math
import time
import numpy as np
import serial
import pinocchio as pin
import mujoco
import mujoco.viewer
import threading
import os
import curses
import sys
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
# 8️⃣ 拖动示教相关参数
# =========================================================
recording = False
recorded_trajectory = []
save_path = "recorded_trajectory.npy"
playback = False
playback_index = 0
playback_trajectory = []
PLAYBACK_DT = 0.02
playback_last_time = 0
MODE_NORMAL = 0
MODE_RECORD = 1
MODE_PLAYBACK = 2
current_mode = MODE_NORMAL
record_completed = False
playback_completed = False

# 请求标志（用于线程间通信）
request_start_record = False
request_stop_record = False
request_start_playback = False
request_stop_playback = False

# =========================================================
# 9️⃣ 控制台界面函数（使用 curses）
# =========================================================
def draw_status(stdscr):
    global request_start_record, request_stop_record, request_start_playback, request_stop_playback
    global current_mode, recording, playback, playback_index, playback_trajectory
    global recorded_trajectory, record_completed, playback_completed
    global q, tau_g

    curses.curs_set(0)
    stdscr.nodelay(1)
    stdscr.clear()

    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)

    mode_names = {MODE_NORMAL: "重力补偿模式", MODE_RECORD: "🔴 记录模式", MODE_PLAYBACK: "▶️ 回放模式"}
    mode_colors = {MODE_NORMAL: 1, MODE_RECORD: 3, MODE_PLAYBACK: 4}

    last_update = 0

    while True:
        # 处理按键请求
        try:
            key = stdscr.getch()
            if key != -1:
                ch = chr(key).upper()
                if ch == 'R':
                    request_start_record = True
                elif ch == 'S':
                    request_stop_record = True
                elif ch == 'P':
                    request_start_playback = True
                elif ch == 'N':
                    request_stop_playback = True
                elif ch == 'Q':
                    break
        except:
            pass

        # 每 0.1 秒刷新界面
        now = time.time()
        if now - last_update >= 0.1:
            last_update = now
            stdscr.clear()
            height, width = stdscr.getmaxyx()

            title = "=== 机器人拖动示教系统 ==="
            stdscr.addstr(0, (width - len(title))//2, title, curses.color_pair(1) | curses.A_BOLD)

            mode_line = f"▶ 当前模式: {mode_names.get(current_mode, '未知')}"
            stdscr.addstr(2, 2, mode_line, curses.color_pair(mode_colors.get(current_mode, 1)) | curses.A_BOLD)

            hint = "按键: [R]开始记录  [S]停止记录并保存  [P]回放  [N]停止回放  [Q]退出"
            stdscr.addstr(4, 2, hint, curses.color_pair(2))

            # 记录状态
            if recording:
                rec_status = f"📝 记录中...  已记录帧数: {len(recorded_trajectory)}"
                stdscr.addstr(6, 2, rec_status, curses.color_pair(3))
            elif record_completed:
                rec_status = f"✅ 记录已完成！共保存 {len(recorded_trajectory)} 帧到 {save_path}"
                stdscr.addstr(6, 2, rec_status, curses.color_pair(1))
            else:
                rec_status = f"💾 上次记录: {'无' if len(recorded_trajectory)==0 else f'{len(recorded_trajectory)} 帧'}"
                stdscr.addstr(6, 2, rec_status, curses.color_pair(4))

            # 回放状态
            if playback:
                total = len(playback_trajectory)
                percent = (playback_index / total * 100) if total > 0 else 0
                play_status = f"🎬 回放中: {playback_index}/{total} 帧 ({percent:.1f}%)"
                stdscr.addstr(7, 2, play_status, curses.color_pair(3))
            elif playback_completed:
                play_status = f"✅ 回放已完成！共 {len(playback_trajectory)} 帧"
                stdscr.addstr(7, 2, play_status, curses.color_pair(1))
            else:
                play_status = f"🎞 上次回放: {'无' if len(playback_trajectory)==0 else f'{len(playback_trajectory)} 帧'}"
                stdscr.addstr(7, 2, play_status, curses.color_pair(4))

            # 关节角度与力矩
            stdscr.addstr(9, 2, "关节 | 角度 (rad) | 力矩命令 (Nm)", curses.A_UNDERLINE)
            for i in range(6):
                line = f"  J{i+1}  |   {q[i]:6.3f}   |   {tau_g[i]:6.3f}"
                stdscr.addstr(10+i, 2, line)

            footer = "MuJoCo 视图已启动 | 按 Q 退出界面并关闭程序"
            stdscr.addstr(height-2, 2, footer, curses.color_pair(4))

            stdscr.refresh()

        time.sleep(0.05)

    curses.endwin()
    serial_device.close()
    sys.exit(0)

# =========================================================
# 🔟 主循环参数
# =========================================================
DT = 0.002
print("正在启动 MuJoCo viewer 和实时控制台界面...")
print("如果控制台未正常显示，请调整终端大小至少 80x24")


# 主循环中的全局变量（供界面线程读取）
q = np.zeros(6)
tau_g = np.zeros(6)

# 启动 curses 界面线程
curses_thread = threading.Thread(target=lambda: curses.wrapper(draw_status), daemon=True)
curses_thread.start()

# =========================================================
# 1️⃣1️⃣ 主循环
# =========================================================
while viewer.is_running():
    # 处理来自 curses 界面的请求
    if request_start_record:
        request_start_record = False
        if not recording:
            recording = True
            recorded_trajectory = []
            current_mode = MODE_RECORD
            record_completed = False
            print("\n[示教] 开始记录轨迹...")

    if request_stop_record:
        request_stop_record = False
        if recording:
            recording = False
            current_mode = MODE_NORMAL
            np.save(save_path, np.array(recorded_trajectory))
            record_completed = True
            print(f"\n[示教] 记录停止，共 {len(recorded_trajectory)} 帧，已保存至 {save_path}")

    if request_start_playback:
        request_start_playback = False
        if not os.path.exists(save_path):
            print(f"\n[示教] 轨迹文件 {save_path} 不存在，请先记录")
        elif not playback:
            playback_trajectory = np.load(save_path)
            if len(playback_trajectory) > 0:
                playback = True
                playback_index = 0
                current_mode = MODE_PLAYBACK
                playback_last_time = time.time()
                playback_completed = False
                print(f"\n[示教] 开始回放，共 {len(playback_trajectory)} 帧")
            else:
                print("\n[示教] 轨迹为空")

    if request_stop_playback:
        request_stop_playback = False
        if playback:
            playback = False
            current_mode = MODE_NORMAL
            print("\n[示教] 回放已停止，返回重力补偿模式")

    # 1. 读取电机角度
    q_raw = np.array([
        -Motor1.getPosition(),
        Motor2.getPosition(),
        -Motor3.getPosition(),
        -Motor4.getPosition(),
        -Motor5.getPosition(),
        Motor6.getPosition()
    ], dtype=np.float64)

    q = q_raw + q_offset

    # 2. 根据模式执行控制
    if current_mode == MODE_PLAYBACK and playback:
        now = time.time()
        if now - playback_last_time >= PLAYBACK_DT:
            playback_last_time = now
            if playback_index < len(playback_trajectory):
                q_desired_0 = -playback_trajectory[playback_index][0]-2.77
                q_desired_1 = playback_trajectory[playback_index][1]+2.296
                q_desired_2 = -playback_trajectory[playback_index][2]+0.3
                q_desired_3 = -playback_trajectory[playback_index][3]-0.34
                q_desired_4 = -playback_trajectory[playback_index][4]+2.3
                q_desired_5 = playback_trajectory[playback_index][5]-3.0
                # 可选：重新组合成数组（如果后面需要统一处理）
                q_desired = np.array([q_desired_0, q_desired_1, q_desired_2,
                                    q_desired_3, q_desired_4, q_desired_5])
                for i, m in enumerate(motors):
                    mc.controlMIT(
                        m,
                        kp=25.0,
                        kd=0.5,
                        q=float(q_desired[i]),
                        dq=0.0,
                        tau=0.0
                    )
                playback_index += 1
                mj_data.qpos[:6] = q_desired
                mj_data.qvel[:] = 0.0
                mujoco.mj_forward(mj_model, mj_data)
                viewer.sync()
            else:
                playback = False
                current_mode = MODE_NORMAL
                playback_completed = True
        time.sleep(DT)
        continue

    # 正常模式（重力补偿）
    pin.forwardKinematics(pin_model, pin_data, q)
    tau_g = pin.rnea(pin_model, pin_data, q, np.zeros(6), np.zeros(6))
    tau_g = np.clip(tau_g, -8.0, 8.0)

    for i, m in enumerate(motors):
        mc.controlMIT(
            m,
            kp=0.0,
            kd=0.1,
            q=0.0,
            dq=0.0,
            tau=float(S[i] * tau_g[i])
        )

    if recording:
        recorded_trajectory.append(q.copy())

    mj_data.qpos[:6] = q
    mj_data.qvel[:] = 0.0
    mujoco.mj_forward(mj_model, mj_data)
    viewer.sync()

    time.sleep(DT)

serial_device.close()