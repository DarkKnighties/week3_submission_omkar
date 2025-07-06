#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ClockSubscriber(Node):
    def __init__(self):
        super().__init__('clock_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/clock',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = ClockSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
