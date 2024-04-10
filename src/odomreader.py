#!/usr/bin/env python3

import os

import rospy
import tf.transformations
from nav_msgs.msg import Odometry
import datetime


class OdomReader:
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.theta = 0.0
        self.vel_lin = 0.0
        self.vel_ang = 0.0
        script_dir = os.path.dirname(__file__)
        parent_dir = os.path.dirname(script_dir)
        current_time = datetime.datetime.now().strftime("%H-%M-%S")
        self.file_name = os.path.join(parent_dir, "data", current_time + ".txt")
        os.makedirs(os.path.dirname(self.file_name), exist_ok=True)
        self.data_ready = False

    def callback(self, data):
        self.posx = data.pose.pose.position.x
        self.posy = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        self.theta = euler[2]
        self.vel_lin = data.twist.twist.linear.x
        self.vel_ang = data.twist.twist.angular.z
        self.data_ready = True

    def run(self):
        rospy.Subscriber("/odom", Odometry, self.callback)
        rate = rospy.Rate(10)
        try:
            with open(self.file_name, "a") as f:
                while not rospy.is_shutdown():
                    if not self.data_ready:
                        continue
                    os.system("clear")
                    print("X = %6.4f m, Y = %6.4f m" % (self.posx, self.posy))
                    print("Theta = %6.4f rad" % (self.theta))
                    print("Vel_lin = %6.4f m/s" % (self.vel_lin))
                    print("Vel_ang = %6.4f rad/s" % (self.vel_ang))
                    f.write(
                        "%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n"
                        % (
                            rospy.get_time(),
                            self.posx,
                            self.posy,
                            self.theta,
                            self.vel_lin,
                            self.vel_ang,
                        )
                    )
                    rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            os.system("clear")
            print("Shutting down")
        finally:
            self.posx = 0.0
            self.posy = 0.0
            self.theta = 0.0
            self.vel_lin = 0.0
            self.vel_ang = 0.0


if __name__ == "__main__":
    rospy.init_node("odomreader", anonymous=True)
    odom_reader = OdomReader()
    odom_reader.run()
