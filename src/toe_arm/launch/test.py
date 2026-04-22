import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import mujoco
import numpy as np
import glfw
import threading

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__("joint_state_subscriber")
        
        # 添加互斥锁保护共享数据
        self.lock = threading.Lock()
        
        # 加载机械臂模型
        self.model = mujoco.MjModel.from_xml_path(
            '/home/yue/mujoco_ros_ws/src/mujoco_ros2_control/mujoco_ros2_control_demos/mujoco_models/toe_arm.xml'
        )
        self.data = mujoco.MjData(self.model)
        
        # 打印模型信息
        print("模型信息:")
        print(f"关节数量: {self.model.nq}")
        print(f"关节名称: {[mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(self.model.njnt)]}")
        
        # 初始化关节状态
        self.initial_q = self.data.qpos[:6].copy() if self.model.nq >= 6 else self.data.qpos.copy()
        self.target_positions = self.initial_q.copy()  # 目标位置
        self.current_positions = self.initial_q.copy()  # 当前插值位置
        
        # 记录关节映射
        self.joint_mapping = {}  # 关节名 -> 索引映射
        self.joint_names = [f"joint{i}" for i in range(1, 7)]
        
        # 平滑参数
        self.smoothing_factor = 0.2
        self.positions_received = False
        
        # 订阅关节状态
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            10
        )
        
        # 初始化GLFW
        if not glfw.init():
            raise RuntimeError("GLFW初始化失败")
            
        self.window = glfw.create_window(1200, 900, 'Toe Arm 机械臂控制', None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("窗口创建失败")
            
        glfw.make_context_current(self.window)
        glfw.set_scroll_callback(self.window, self.scroll_callback)
        
        # 初始化渲染器
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()
        mujoco.mjv_defaultCamera(self.cam)
        mujoco.mjv_defaultOption(self.opt)
        self.pert = mujoco.MjvPerturb()
        self.con = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        
        # 相机设置
        self.cam.distance = 1.5
        self.cam.azimuth = 135
        self.cam.elevation = -20
        
        # 创建定时器
        self.timer1 = self.create_timer(0.01, self.timer_callback1)
        
        print("关节状态订阅器已初始化，等待关节状态消息...")
    
    def scroll_callback(self, window, xoffset, yoffset):
        """鼠标滚轮回调"""
        self.cam.distance *= 1 - 0.1 * yoffset
    
    def timer_callback1(self):
        """定时器回调，更新MuJoCo渲染"""
        if glfw.window_should_close(self.window):
            return
        
        with self.lock:
            # 平滑插值更新关节位置
            for i in range(min(len(self.current_positions), len(self.target_positions))):
                self.current_positions[i] = (1 - self.smoothing_factor) * self.current_positions[i] + \
                                          self.smoothing_factor * self.target_positions[i]
        
        # 应用当前关节位置
        for i in range(min(len(self.current_positions), self.model.nq)):
            self.data.qpos[i] = self.current_positions[i]
        
        # 前向动力学
        mujoco.mj_forward(self.model, self.data)
        
        # 渲染
        viewport = mujoco.MjrRect(0, 0, 1200, 900)
        mujoco.mjv_updateScene(self.model, self.data, self.opt, self.pert, 
                             self.cam, mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)
        mujoco.mjr_render(viewport, self.scene, self.con)
        
        # 交换缓冲区
        glfw.swap_buffers(self.window)
        glfw.poll_events()
    
    def callback(self, msg: JointState):
        """关节状态回调函数"""
        if not self.positions_received:
            self.get_logger().info(f"首次接收到关节状态，共{len(msg.name)}个关节")
            self.positions_received = True
        
        # 构建关节名称到索引的映射
        joint_indices = []
        positions = []
        
        # 方法1: 按joint1-joint6顺序查找
        for joint_name in self.joint_names:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                joint_indices.append(idx)
        
        # 如果找到6个关节
        if len(joint_indices) == 6:
            positions = [msg.position[idx] for idx in joint_indices]
        else:
            # 方法2: 查找所有包含"joint"的关节
            joint_indices = []
            for i, name in enumerate(msg.name):
                if "joint" in name.lower() and i < len(msg.position):
                    joint_indices.append(i)
            
            if len(joint_indices) >= 6:
                positions = [msg.position[idx] for idx in joint_indices[:6]]
            else:
                # 方法3: 使用前6个位置
                if len(msg.position) >= 6:
                    positions = msg.position[:6]
                else:
                    self.get_logger().warn(f"关节数不足: 期望6，收到{len(msg.position)}")
                    return
        
        # 检查位置是否有效
        if any(np.isnan(pos) or np.isinf(pos) for pos in positions):
            self.get_logger().warn("接收到无效关节位置")
            return
        
        # 更新目标位置
        with self.lock:
            for i, pos in enumerate(positions[:6]):
                if i < len(self.target_positions):
                    self.target_positions[i] = float(pos)
        
        # ============== 新增：打印关节角度 ==============
        # 以友好的格式打印所有6个关节的角度
        self.print_joint_angles(positions[:6])
        
        # 原有的调试打印（可选，可以保留或删除）
        if self.get_logger().get_effective_level() <= 20:  # DEBUG级别
            joint_str = ", ".join([f"{pos:.3f}" for pos in positions[:3]])
            self.get_logger().debug(f"关节角度: [{joint_str}, ...]")
    
    def print_joint_angles(self, positions):
        """打印关节角度的专用函数"""
        if len(positions) < 6:
            return
        
        # 将弧度转换为角度
        angles_deg = [np.degrees(pos) for pos in positions]
        
        # 以表格形式打印关节角度
        print("\n" + "="*60)
        print("当前关节角度 (弧度 | 角度):")
        print("-"*60)
        for i, (rad, deg) in enumerate(zip(positions, angles_deg), 1):
            # 根据角度值设置颜色（可选，如果终端支持）
            if abs(deg) > 90:
                color_code = "\033[91m"  # 红色
            elif abs(deg) > 45:
                color_code = "\033[93m"  # 黄色
            else:
                color_code = "\033[92m"  # 绿色
            
            reset_code = "\033[0m"
            print(f"joint{i}: {color_code}{rad:.4f} rad | {deg:.2f}°{reset_code}")
        

        print("-"*60)
        print("="*60 + "\n")
    
    def destroy_node(self):
        """清理资源"""
        glfw.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = JointStateSubscriber()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        rclpy.shutdown()
        print("程序已退出")

if __name__ == "__main__":
    main()