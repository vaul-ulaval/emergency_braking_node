#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        NOTE that the x component of the linear velocity in odom is the speed
        """

        # TODO: create ROS subscribers and publishers.
        self.drivecommand = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.laserscan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odomcar = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.speed = 0.
        print("Emergency braking node started!")

        

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        donnees = int((scan_msg.angle_max - scan_msg.angle_min)/scan_msg.angle_increment)
        for i in range(0, donnees):
            ran = scan_msg.ranges[i]
            ran_speed = self.speed*math.cos((i*scan_msg.angle_increment)+scan_msg.angle_min)
            if ran_speed == 0:
                continue
            else:
                TTC = ran/(ran_speed)
                if 0 < TTC < 1:  #Changer le TTC selon calibration necessaire et faire des tests
                    print("Braking", TTC)
                    ackermannMsg = AckermannDriveStamped()
                    ackermannMsg.drive.speed = 0.0
                    self.drivecommand.publish(ackermannMsg)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
