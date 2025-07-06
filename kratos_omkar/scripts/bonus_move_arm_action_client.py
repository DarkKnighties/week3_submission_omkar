#!/usr/bin/env python3

'''
This is the action client for the arm movement action.
It sends a goal to move the arm to a specified angle and receives feedback during the execution.
'''

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from kratos_omkar.action import MoveArm

class MoveArmActionClient(Node):
    def __init__(self):
        super().__init__('move_arm_action_client')

        # Initialize the ActionClient with the action type (MoveArm) and action name ('move_arm')
        self._client = ActionClient(self, MoveArm, 'move_arm')

    def send_goal(self, target_angle):
        # Wait until the action server becomes available
        self._client.wait_for_server()

        # Create a goal message of type MoveArm.Goal and assign the target angle
        goal_msg = MoveArm.Goal()
        goal_msg.target_angle = target_angle

        # Send the goal asynchronously and assign the callbacks
        send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Called when the server accepts or rejects the goal
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Called when the result is received
        result = future.result().result
        self.get_logger().info(f'Goal result received: Success={result.success}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        # Extract the feedback from the message and log the current angle
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current angle: {feedback.current_angle}')

def main(args=None):
    # Initialize ROS 2 communication
    rclpy.init(args=args)

    # Instantiate the client node
    client = MoveArmActionClient()

    # Ask user for target angle input
    try:
        angle = int(input("Enter target angle (0–180): "))
        if 0 <= angle <= 180:
            client.send_goal(angle)
        else:
            print("Angle out of range (0–180).")
            return
    except ValueError:
        print("Invalid input. Please enter an integer.")
        return

    # Keep the node alive to continue receiving feedback
    rclpy.spin(client)

if __name__ == '__main__':
    main()
