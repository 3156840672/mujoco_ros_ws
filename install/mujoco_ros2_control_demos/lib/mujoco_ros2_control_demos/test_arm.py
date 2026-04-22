#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class ArmTester(Node):
    def __init__(self):
        super().__init__('arm_tester')
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.get_logger().info("机械臂测试器已启动")
    
    def move_arm(self, positions, duration=3.0):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 
            'joint4', 'joint5', 'joint6'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        goal_msg.trajectory.points.append(point)
        
        self._action_client.wait_for_server()
        self.get_logger().info(f'发送目标位置: {positions}')
        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    tester = ArmTester()
    
    # 测试序列
    test_positions = [
        [0.5, 0.3, -0.2, 0.4, -0.3, 0.1],    # 位置1
        [-0.5, -0.3, 0.2, -0.4, 0.3, -0.1],  # 位置2
        [0.3, 0.5, -0.4, 0.2, 0.1, -0.2],    # 位置3
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],      # 回到零位
    ]
    
    try:
        for i, pos in enumerate(test_positions):
            tester.get_logger().info(f'测试位置 {i+1}/{len(test_positions)}')
            future = tester.move_arm(pos, 4.0)
            rclpy.spin_until_future_complete(tester, future)
            time.sleep(5.0)  # 等待运动完成
        
        tester.get_logger().info('所有测试完成！')
        
    except KeyboardInterrupt:
        tester.get_logger().info('测试被用户中断')
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()