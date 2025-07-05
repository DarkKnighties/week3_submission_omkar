#!/usr/bin/env python3
'''
# This is the publisher script 1 for the red light green light question.
# It publishes a message to the `/s1` topic, which alternates between 'green' and 'red'.
# The state changes to 'red' after 10 seconds of being 'green'.
'''


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class S1Publisher(Node):
    def __init__(self):
        super().__init__('s1_publisher')
        '''
        Create a publisher for the '/s1' topic.
        '''
        self.publisher_ = self.create_publisher(String, '/s1', 10)
        self.msg = String()
        
        #Initialize the state and the last switch time
        self.state = 'green'
        self.last_switch_time = time.time()

        # Create a timer that calls the timer_callback every half a second
        self.timer = self.create_timer(0.5, self.timer_callback)

    '''
    The timer callback function that publishes the state
    It checks if 10 seconds have passed since the last switch
    If so, it switches the state and updates the last switch time
    Then it publishes the current state to the '/s1' topic
    '''
    def timer_callback(self):
        current_time = time.time()
        if current_time - self.last_switch_time >= 10.0:
            self.state = 'red' if self.state == 'green' else 'green'
            self.last_switch_time = current_time

        self.msg.data = self.state
        self.publisher_.publish(self.msg)
        self.get_logger().info(f'S1 publishing: {self.msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = S1Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
