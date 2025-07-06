#!/usr/bin/env python3

'''
This is the action server of the arm movement action.
It listens for goals to move the arm to a specified angle and provides feedback during the execution.
'''

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from kratos_omkar.action import MoveArm
from rclpy.action import GoalResponse, CancelResponse
import time

class MoveArmActionServer(Node):
    def __init__(self):
        super().__init__('move_arm_action_server')
        # Create an Action Server for the MoveArmAction
        # The execute_callback will be called when a goal is received.
        # The goal_callback will be called to accept or reject the goal.
        # The cancel_callback will be called when a cancel request is received.
        self._server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.get_logger().info('MoveArm Action Server Started')

    # Callback to handle incoming goal requests.
    # It logs the target angle and accepts the goal.
    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: {goal_request.target_angle}')
        return GoalResponse.ACCEPT

    # Callback to handle cancel requests.
    # It logs the cancel request and accepts it.
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    # This is the main execution callback for the action.
    # It processes the goal, simulates moving the arm, and provides feedback.
    # It runs until the target angle is reached, publishing feedback at each step.
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        target = goal_handle.request.target_angle
        current = 0
        feedback_msg = MoveArm.Feedback()

        while current < target:
            current += 1
            feedback_msg.current_angle = current
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Current angle: {current}')
            time.sleep(1)

        goal_handle.succeed()
        result = MoveArm.Result()
        result.success = True
        self.get_logger().info('Goal execution completed successfully.')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveArmActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
