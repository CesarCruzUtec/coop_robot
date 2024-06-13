#!/usr/bin/env python3

import rospy
import numpy as np

from nav_msgs.msg import Odometry


class tb_odom:
    def __init__(self):
        self.X0 = 0
        self.Y0 = 0
        self.X1 = 0
        self.Y1 = 0

        rospy.init_node("tb_odom")

        rate = rospy.Rate(10)

        rospy.Subscriber("/tb3_0/odom", Odometry, self.odom_tb3_0)
        rate.sleep()
        rospy.Subscriber("/tb3_1/odom", Odometry, self.odom_tb3_1)
        rate.sleep()

        while not rospy.is_shutdown():
            print(
                f"Distance: {np.sqrt((self.X0 - self.X1)**2 + (self.Y0 - self.Y1)**2)}"
            )

            tb3_0 = rospy.get_param("/tb3_0_status")
            tb3_1 = rospy.get_param("/tb3_1_status")

            if tb3_0 == "end" and tb3_1 == "end":
                break

            rate.sleep()

    def odom_tb3_0(self, data):
        self.X0 = data.pose.pose.position.x
        self.Y0 = data.pose.pose.position.y

    def odom_tb3_1(self, data):
        self.X1 = data.pose.pose.position.x
        self.Y1 = data.pose.pose.position.y


if __name__ == "__main__":
    tbo = tb_odom()
