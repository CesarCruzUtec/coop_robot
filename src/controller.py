#!/usr/bin/env python3

import rospy
import numpy as np

from tf.transformations import euler_from_quaternion as efq

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

MAX_ANG_VEL = 1.82
MAX_LIN_VEL = 0.26

ANG_TOL = 0.02  # 1.14°
LIN_TOL = 0.02  # 5%
DIS_TOL = 0.02  # 2%


class Controller:
    def __init__(self, ns):
        # Set initial status
        rospy.set_param(f"/{ns}", {"status": "idle", "1step": False})
        # Establish the current and other robot namespace
        self.ns = ns
        self.other = "tb3_0" if ns == "tb3_1" else "tb3_1"
        # Get the parameters
        self.get_param()
        # Set the rate to 10 Hz
        self.rate = rospy.Rate(10)
        self.rate.sleep()
        # Subscribe to the odometry topic
        rospy.Subscriber(f"/{self.ns}/odom", Odometry, self.callback, self.ns)
        rospy.Subscriber(f"/{self.other}/odom", Odometry, self.callback, self.other)
        self.rate.sleep()
        # Subscribe to the cmd_vel topic
        rospy.Subscriber(f"/{self.other}/cmd_vel", Twist, self.vel_callback)
        self.rate.sleep()
        # Set the publisher to the cmd_vel topic
        self.pub = rospy.Publisher(f"/{ns}/cmd_vel", Twist, queue_size=10)
        # Set the initial velocity to 0 and a no velocity message
        self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        self.no_vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        self.current = [0, 0, 0]
        self.otherPos = [0, 0, 0]
        self.offset = [0, 0, 0]

    def get_param(self):
        ang_param = rospy.get_param("/ang_param")
        lin_param = rospy.get_param("/lin_param")
        self.distanceD = rospy.get_param("/distance")

        self.akp = ang_param["kp"]
        self.lkp = lin_param["kp"]

        self.offset = rospy.get_param("/offset")

    def callback(self, msg, ns):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        euler = efq([quat.x, quat.y, quat.z, quat.w])
        theta = euler[2]

        if ns == "tb3_1":
            x += self.offset[0]
            y += self.offset[1]
            theta += self.offset[2]

        if ns == self.ns:
            self.current = [x, y, theta]
        else:
            self.otherPos = [x, y, theta]
            # self.otherVel = msg.twist.twist.linear.x

    def vel_callback(self, msg):
        self.otherVel = msg.linear.x
        # Calculate distance between robots
        if self.ns == "tb3_0":
            leader, follower = self.current, self.otherPos
        else:
            leader, follower = self.otherPos, self.current

        vFL = np.array(leader[:2]) - np.array(follower[:2])
        dFL = np.linalg.norm(vFL)
        # aFL = np.arctan2(vFL[1], vFL[0])

        self.distance = dFL
        # self.angle = aFL
        # self.constant = np.cos(aFL - leader[2]) / np.cos(aFL - follower[2])
        # rospy.loginfo(f"Constant: {self.constant:.2f}")

    def fix_angle(self, ang: float) -> float:
        if ang > np.pi:
            ang -= 2 * np.pi
        elif ang < -np.pi:
            ang += 2 * np.pi
        return ang

    def getLinearVel(
        self,
        first: bool,
        oStatus: str,
        x: float,
        y: float,
        ox: float,
        oy: float,
    ) -> float:
        err1 = [x - self.current[0], y - self.current[1]]
        d1 = np.linalg.norm(err1)
        if first or oStatus == "end":
            return self.lkp * d1

        err2 = [ox - self.otherPos[0], oy - self.otherPos[1]]
        d2 = np.linalg.norm(err2)
        vel = self.lkp * min(d1, d2)
        if x == ox and y == oy:
            # derr = 1 - self.distance / self.distanceD
            derr = self.distance - self.distanceD
            if derr > DIS_TOL and self.ns == "tb3_0":
                vel *= 0.25
            if derr < -DIS_TOL and self.ns == "tb3_1":
                vel *= 0.25
            return vel

        if self.ns == "tb3_1":
            d1, d2 = d2, d1
        err3 = [x - ox, y - oy]
        d3 = np.linalg.norm(err3) - d1
        ang = abs(np.pi - abs(self.current[2] - self.otherPos[2]))

        k = (d3 - d2 * np.cos(ang)) / (d2 - d3 * np.cos(ang))
        rospy.loginfo(f"K = {k:.2f}")
        
        if k < 1 and self.ns == "tb3_1":
            return vel * k
        if k > 1 and self.ns == "tb3_0":
            return vel / k
        return vel

    def move_to(self, x, y, theta=None, first=False):
        rospy.set_param(f"/{self.ns}/objective", [float(x), float(y)])

        err = [x - self.current[0], y - self.current[1]]
        ang_ref = np.arctan2(err[1], err[0])
        self.rotate_to(ang_ref)

        finish = False
        rospy.set_param(f"/{self.ns}/status", "moving")
        prev_lin_err = np.inf
        while not finish:
            otherStatus = rospy.get_param(f"/{self.other}/status")

            if otherStatus == "rotating" and not first:
                self.pub.publish(self.no_vel)
                self.rate.sleep()
                continue

            ox, oy = rospy.get_param(f"/{self.other}/objective")
            err = [x - self.current[0], y - self.current[1]]

            # if first or otherStatus == "end":
            #     oerr = [np.inf, np.inf]
            # else:
            #     oerr = [ox - self.otherPos[0], oy - self.otherPos[1]]

            err_lin = np.linalg.norm(err)
            # oerr_lin = np.linalg.norm(oerr)

            # derr = self.distanceD - self.distance
            # if first or otherStatus == "end":
            #     lp = self.lkp * err_lin
            # else:
            #     lp = self.lkp * min(err_lin, oerr_lin)
            #     if x == ox and y == oy:
            #         if derr > DIS_TOL:
            #             lp = 0 if self.ns == "tb3_1" else self.lkp * derr
            #         elif derr < -DIS_TOL:
            #             lp = 0 if self.ns == "tb3_0" else self.lkp * -derr
            #     else:
            #         if self.ns == "tb3_0" and self.constant > 1:
            #             lp = self.otherVel / self.constant
            #         elif self.ns == "tb3_1" and self.constant < 1:
            #             lp = self.otherVel * self.constant

            lp = self.getLinearVel(first, otherStatus, x, y, ox, oy)

            err_ang = self.fix_angle(ang_ref - self.current[2])
            ap = self.akp * err_ang

            if err_lin < LIN_TOL or err_lin - prev_lin_err > 1e-3:
                lp = 0.0
                ap = 0.0
                finish = True

            prev_lin_err = err_lin
            self.vel.linear.x = np.clip(lp, -MAX_LIN_VEL, MAX_LIN_VEL)
            self.vel.angular.z = np.clip(ap, -MAX_ANG_VEL, MAX_ANG_VEL)
            self.pub.publish(self.vel)

            self.rate.sleep()

        if theta is not None:
            self.rotate_to(theta)

    def rotate_to(self, theta):
        finish = False
        rospy.set_param(f"/{self.ns}/status", "rotating")
        rospy.set_param(f"/{self.ns}/objective_angle", float(theta))
        while not finish:
            err_ang = theta - self.current[2]

            if err_ang > np.pi:
                err_ang -= 2 * np.pi
            elif err_ang < -np.pi:
                err_ang += 2 * np.pi

            ap = self.akp * err_ang

            if np.abs(err_ang) < ANG_TOL:
                ap = 0.0
                finish = True

            self.vel.linear.x = 0
            self.vel.angular.z = np.clip(ap, -MAX_ANG_VEL, MAX_ANG_VEL)
            self.pub.publish(self.vel)

            self.rate.sleep()
