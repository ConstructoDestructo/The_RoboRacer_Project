#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class Relay(Node):
    def __init__(self):
        super().__init__('relay')

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            'drive_relay',
            10
        )

        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.relay_message,
            10
        )

    def relay_message(self, incoming):
        outgoing = AckermannDriveStamped()
        outgoing.header.stamp = self.get_clock().now().to_msg()
        outgoing.header.frame_id = incoming.header.frame_id

        outgoing.drive.speed = incoming.drive.speed * 3.0
        outgoing.drive.steering_angle = (
            incoming.drive.steering_angle * 3.0
        )

        self.publisher.publish(outgoing)


def main(args=None):
    rclpy.init(args=args)
    node = Relay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
