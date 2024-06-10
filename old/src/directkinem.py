#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
import argparse


class DirectKinematics:
    def __init__(self, vel_lin=0.0, vel_ang=0.0, robot=""):
        self.vel_lin = max(min(vel_lin, 0.26), -0.26)
        self.vel_ang = max(min(vel_ang, 1.82), -1.82)
        self.robot = robot
        self.velocity = Twist(
            Vector3(self.vel_lin, 0.0, 0.0), Vector3(0.0, 0.0, self.vel_ang)
        )
        print(
            "Velocity: linear = {0:>6.4f} m/s, angular = {1:>6.4f} rad/s".format(
                self.velocity.linear.x, self.velocity.angular.z
            )
        )

    def run(self):
        if self.robot == "":
            topic_name = "/cmd_vel"
        else:
            topic_name = "/" + self.robot + "/cmd_vel"

        print("Publishing to topic: " + topic_name)
        print("Press Ctrl+C to stop")

        pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                pub.publish(self.velocity)
                rate.sleep()
        except KeyboardInterrupt:
            print("Stopping")
            self.velocity = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(self.velocity)
            print("Stopped")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Direct Kinematics")
    parser.add_argument("-l", type=float, default=0.26, help="linear velocity")
    parser.add_argument("-a", type=float, default=0.3, help="angular velocity")
    parser.add_argument("-r", type=str, default="", help="robot to manipulate")

    args = parser.parse_args()

    rospy.init_node("directkinem", disable_signals=True)
    dk = DirectKinematics(vel_lin=args.l, vel_ang=args.a, robot=args.r)
    dk.run()
