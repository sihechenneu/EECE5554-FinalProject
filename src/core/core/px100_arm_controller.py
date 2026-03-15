#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from core_interfaces.action import PickUp

import time

class PX100ArmController(Node):
    def __init__(self):
        super().__init__('px100_arm_controller')
        self.cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            PickUp,
            'pickup_trash',
            self.execute_callback,
            callback_group=self.cb_group
        )
        
        self.get_logger().info('PX100 Arm Controller Action Server started.')
        
    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing pickup for target at: '
                               f'({goal_handle.request.target_position.x:.2f}, '
                               f'{goal_handle.request.target_position.y:.2f}, '
                               f'{goal_handle.request.target_position.z:.2f})')
                               
        feedback_msg = PickUp.Feedback()
        
        feedback_msg.status = "Moving to pre-grasp pose..."
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1.0)
        
        feedback_msg.status = "Grasping object..."
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1.0)
        
        feedback_msg.status = "Lifting..."
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1.0)
        
        goal_handle.succeed()
        
        result = PickUp.Result()
        result.success = True
        self.get_logger().info('Pickup execution completed successfully.')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = PX100ArmController()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
