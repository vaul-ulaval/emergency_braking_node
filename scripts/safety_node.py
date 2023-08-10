#!/usr/bin/env python3

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

QUEUE_SIZE = 1
TTC_THRESHOLD = 0.15


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """

    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.pubDrive = self.create_publisher(
            AckermannDriveStamped, '/drive', QUEUE_SIZE)

        self.subScan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QUEUE_SIZE)

        self.subOdom = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            QUEUE_SIZE)

        self.speedX = 0.0

        print("Safety node started")

    def odom_callback(self, odomMsg):
        if odomMsg is None:
            self.speedX = 0.0
            return

        self.speedX = -odomMsg.twist.twist.linear.x

    def scan_callback(self, scanMsg):
        if scanMsg is None:
            return

        minTTC = math.inf
        for i, distance in enumerate(scanMsg.ranges):
            if math.isnan(distance) or math.isinf(distance):
                continue

            angle = scanMsg.angle_min + i * scanMsg.angle_increment
            projectedSpeedX = math.cos(angle) * self.speedX
            if projectedSpeedX == 0:
                continue

            curTTC = distance / max(0., -projectedSpeedX)

            if curTTC < minTTC:
                minTTC = curTTC

        if minTTC < TTC_THRESHOLD:
            self.brake()

    def brake(self):
        print("Braking")
        while True:
            ackermannMsg = AckermannDriveStamped()
            ackermannMsg.drive.speed = 0.0
            self.pubDrive.publish(ackermannMsg)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
