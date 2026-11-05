#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        self.declare_parameter('v', 0.0)
        self.declare_parameter('d', 0.0)

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            'drive',
            10
        )

        self.timer = self.create_timer(0.001, self.publish_drive)

    def publish_drive(self):
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()

        message.drive.speed = float(
            self.get_parameter('v').value
        )
        message.drive.steering_angle = float(
            self.get_parameter('d').value
        )

        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

