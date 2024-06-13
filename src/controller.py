#!/usr/bin/env python3

import rospy
import numpy as np

from tf.transformations import euler_from_quaternion as efq

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

MAX_ANG_VEL = 1.82
MAX_LIN_VEL = 0.26

ANG_TOL = 0.01
LIN_TOL = 0.01


class Controller:
    def __init__(self, ns):
        self.ns = ns
        self.other = "tb3_0" if ns == "tb3_1" else "tb3_1"
        self.get_param()

        rospy.Subscriber(f"/{ns}/odom", Odometry, self.callback)
        self.pub = rospy.Publisher(f"/{ns}/cmd_vel", Twist, queue_size=10)

        self.rate = rospy.Rate(10)
        self.rate.sleep()

        self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        self.no_vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    def get_param(self):
        ang_param = rospy.get_param("/ang_param")
        lin_param = rospy.get_param("/lin_param")
        self.distance = rospy.get_param("/distance")

        self.akp = ang_param["kp"]
        self.lkp = lin_param["kp"]

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        euler = efq([quat.x, quat.y, quat.z, quat.w])
        theta = euler[2]

        self.current = [x, y, theta]
        self.ownStatus = rospy.get_param(f"/{self.ns}_status")
        self.otherStatus = rospy.get_param(f"/{self.other}_status")

    def move_to(self, x, y, theta=None, first=False):
        err = [x - self.current[0], y - self.current[1]]
        ang_ref = np.arctan2(err[1], err[0])
        self.rotate_to(ang_ref)

        finish = False
        while not finish:
            rospy.set_param(f"/{self.ns}_status", "moving")
            if self.otherStatus == "rotating" and not first:
                self.pub.publish(self.no_vel)
                self.rate.sleep()
                continue

            err = [x - self.current[0], y - self.current[1]]
            ang_ref = np.arctan2(err[1], err[0])

            err_ang = ang_ref - self.current[2]
            err_lin = np.linalg.norm(err)

            lp = self.lkp * err_lin
            ap = self.akp * err_ang

            if np.abs(err_lin) < LIN_TOL:
                lp = 0.0
                ap = 0.0
                finish = True
                # rospy.set_param(f"/{self.ns}_status", "idle")

            # if self.otherStatus == "rotating" and not first:
            #     lp = 0.0
            #     ap = 0.0

            self.vel.linear.x = np.clip(lp, -MAX_LIN_VEL, MAX_LIN_VEL)
            self.vel.angular.z = np.clip(ap, -MAX_ANG_VEL, MAX_ANG_VEL)
            self.pub.publish(self.vel)

            self.rate.sleep()

        if theta is not None:
            self.rotate_to(theta)

    def rotate_to(self, theta):
        finish = False
        while not finish:
            rospy.set_param(f"/{self.ns}_status", "rotating")
            err_ang = theta - self.current[2]

            if err_ang > np.pi:
                err_ang -= 2 * np.pi
            elif err_ang < -np.pi:
                err_ang += 2 * np.pi

            ap = self.akp * err_ang
            if np.abs(err_ang) < ANG_TOL:
                ap = 0.0
                finish = True
                # rospy.set_param(f"/{self.ns}_status", "idle")

            self.vel.angular.z = np.clip(ap, -MAX_ANG_VEL, MAX_ANG_VEL)
            self.pub.publish(self.vel)

            self.rate.sleep()
