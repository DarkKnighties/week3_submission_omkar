#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from builtin_interfaces.msg import Duration
from kratos_omkar.msg import RoverStatus
import random
import time

class RoverStatusPublisher(Node):
    def __init__(self):
        super().__init__('rover_status_publisher')
        self.publisher_ = self.create_publisher(RoverStatus, '/rover_status', 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = RoverStatus()

        # Simulate velocity
        msg.velocity.linear.x = random.uniform(0.0, 1.0)
        msg.velocity.angular.z = random.uniform(-0.5, 0.5)

        # Simulate distance
        msg.distance_traveled = random.uniform(0.0, 100.0)

        # Simulate position
        msg.position.position.x = random.uniform(-10.0, 10.0)
        msg.position.position.y = random.uniform(-10.0, 10.0)
        msg.position.position.z = 0.0

        # Simulate battery level
        msg.battery_level = random.uniform(20.0, 100.0)

        # Simulate time of travel
        elapsed = self.get_clock().now() - self.start_time
        msg.time_of_travel = Duration(sec=elapsed.seconds_nanoseconds()[0], nanosec=elapsed.seconds_nanoseconds()[1])

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published RoverStatus')

def main(args=None):
    rclpy.init(args=args)
    node = RoverStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()