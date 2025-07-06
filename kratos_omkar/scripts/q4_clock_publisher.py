#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class ClockPublisher(Node):
    def __init__(self):
        super().__init__('clock_publisher')
        self.second_pub = self.create_publisher(Int32, '/second', 10)
        self.minute_pub = self.create_publisher(Int32, '/minute', 10)
        self.hour_pub = self.create_publisher(Int32, '/hour', 10)
        self.clock_pub = self.create_publisher(String, '/clock', 10)

        self.seconds = 0
        self.minutes = 0
        self.hours = 0

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.second_pub.publish(Int32(data=self.seconds))
        self.minute_pub.publish(Int32(data=self.minutes))
        self.hour_pub.publish(Int32(data=self.hours))

        time_str = f'{self.hours:02d}:{self.minutes:02d}:{self.seconds:02d}'
        self.clock_pub.publish(String(data=time_str))
        self.get_logger().info('Clock time published')

        self.seconds += 1
        if self.seconds >= 60:
            self.seconds = 0
            self.minutes += 1
            if self.minutes >= 60:
                self.minutes = 0
                self.hours += 1

def main(args=None):
    rclpy.init(args=args)
    node = ClockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()