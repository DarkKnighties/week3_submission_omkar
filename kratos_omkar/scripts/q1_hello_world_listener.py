#!/usr/bin/env python3
'''
This is the listener script for the "Hello World" question.
It subscribes to the `/new` topic and logs the messages received.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldSubscriber(Node):
    def __init__(self):
        super().__init__('hello_world_subscriber')
        '''
        Create a subscription to the '/new' topic.
        The callback function `listener_callback` will be called whenever a new message is received.
        The queue size is set to 10, meaning it can hold up to 10 messages (buffer size, common).
        '''
        self.subscription = self.create_subscription(
            String,
            '/new',
            self.listener_callback,
            10
        )

    
    # The callback function that processes incoming messages.
    # It logs the content of the message to the console.
    def listener_callback(self, msg):
        self.get_logger().info(msg.data)


# Main function to initialize the ROS2 node and start spinning.
# This keeps the node active and listening for messages.
def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()