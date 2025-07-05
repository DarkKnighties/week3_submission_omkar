#!/usr/bin/env python3
'''
# This is the publisher script 2 for the red light green light question.
# It subscribes to the `/s1` topic and publishes to the `/s2` topic.
# The message alternates between 'green' and 'red' based on the received message.
# If it receives 'green', it publishes 'red', and vice versa.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class S2Publisher(Node):
    def __init__(self):
        super().__init__('s2_publisher')
        '''
        Create a publisher for the '/s2' topic.
        '''
        self.publisher_ = self.create_publisher(String, '/s2', 10)
        '''Create a subscription to the '/s1' topic.
        The callback function `listener_callback` will be called whenever a new message is received.
        '''
        self.subscription = self.create_subscription(
            String,
            '/s1',
            self.listener_callback,
            10
        )
        self.msg = String()

    def listener_callback(self, msg):
        # The callback function that processes incoming messages.
        # It alternates the message data between 'green' and 'red', solely based on the received message.
        self.msg.data = 'red' if msg.data == 'green' else 'green'
        self.publisher_.publish(self.msg)
        self.get_logger().info(f'S2 publishing: {self.msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = S2Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
