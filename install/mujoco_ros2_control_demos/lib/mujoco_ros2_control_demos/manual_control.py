#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class SimplePositionControl(Node):
    def __init__(self):
        super().__init__('simple_position_control')
        
        # 创建位置命令发布器
        self.position_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/commands',
            10
        )
        
        self.get_logger().info("简单位置控制器已启动")
        
    def send_position(self, positions):
        """发送位置命令到所有关节"""
        msg = Float64MultiArray()
        msg.data = positions
        self.position_pub.publish(msg)
        self.get_logger().info(f"发送位置: {positions}")

def main(args=None):
    rclpy.init(args=args)
    controller = SimplePositionControl()
    
    try:
        # 等待系统初始化
        time.sleep(2.0)
        controller.get_logger().info("开始位置控制测试...")
        
        # 测试序列
        test_sequences = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],    # 位置1: 零位
            [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],    # 位置2: 关节1移动到0.5
            [0.5, 0.3, 0.0, 0.0, 0.0, 0.0],    # 位置3: 关节2也移动
            [0.5, 0.3, -0.2, 0.0, 0.0, 0.0],   # 位置4: 关节3移动
            [0.5, 0.3, -0.2, 0.4, 0.0, 0.0],   # 位置5: 关节4移动
            [0.5, 0.3, -0.2, 0.4, -0.3, 0.0],  # 位置6: 关节5移动
            [0.5, 0.3, -0.2, 0.4, -0.3, 0.1],  # 位置7: 所有关节移动
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],    # 位置8: 回到零位
        ]
        
        for i, positions in enumerate(test_sequences):
            controller.get_logger().info(f"发送位置 {i+1}/{len(test_sequences)}")
            controller.send_position(positions)
            time.sleep(3.0)  # 等待3秒
        
        controller.get_logger().info("测试完成!")
        
    except KeyboardInterrupt:
        controller.get_logger().info("测试被用户中断")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()