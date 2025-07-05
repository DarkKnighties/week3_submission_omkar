#!/usr/bin/env python3
'''
# This is the publisher script for the "Hello World" question.
# It publishes a message to the `/new` topic at a rate of 15 Hz.
# The message type is `std_msgs/String`, and the content of the message is "Hello World !".
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        '''
        Create a publisher for the '/new' topic.
        The message type is String.
        '''
        self.publisher_ = self.create_publisher(String, '/new', 10)
        
        # Create a timer that calls the `timer_callback` function at the specified period.
        timer_period = 1.0 / 15  # 15 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = String()
        self.msg.data = 'Hello World !'

    # The callback function that is called at the specified timer period.
    # It publishes the message to the '/new' topic.
    def timer_callback(self):
        self.publisher_.publish(self.msg)
        self.get_logger().info('Published message.')

# Main function to initialize the ROS2 node and start spinning.
def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()