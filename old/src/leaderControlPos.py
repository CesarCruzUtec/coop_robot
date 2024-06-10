#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations
import os
import argparse

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

MAX_ANGLE_VEL = 1.82
MAX_LIN_VEL = 0.26


class ControlPosition:
    def __init__(self):  # reference = [x, y]
        parser = argparse.ArgumentParser(description="Control Position")

        parser.add_argument("-x", type=float, default=-2.0, help="x reference")
        parser.add_argument("-y", type=float, default=2.0, help="y reference")
        parser.add_argument("-t", type=float, default=45.0, help="theta reference")
        parser.add_argument("-d", type=float, default=2.0, help="distance reference")

        parser.add_argument(
            "-kang", type=float, nargs=3, default=[2, 0.0, 0.0], help="angular control"
        )
        parser.add_argument(
            "-klin", type=float, nargs=3, default=[1, 0.0, 0.0], help="linear control"
        )

        parser.add_argument(
            "-v", "--verbose", action="store_true", help="increase output verbosity"
        )

        args = rospy.myargv()
        self.args = parser.parse_args(args[1:])
        self.args.t = np.deg2rad(self.args.t)

        self.dim = "ang"
        self.reference = [self.args.x, self.args.y, self.args.t]
        self.control = [self.args.kang, self.args.klin]

        self.refAng = None
        self.reachControl = None

        self.err_ang = 0.0
        self.err_lin = 0.0
        self.msg = ""

    def callback(self, data):
        t = rospy.get_time()
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        theta = euler[2]
        self.pos = np.array([t, x, y, theta])

    def angControl(self, last=False):
        kp, ki, kd = self.control[0]

        err = [self.reference[0] - self.pos[1], self.reference[1] - self.pos[2]]
        if not last:
            self.refAng = np.arctan2(err[1], err[0])
        else:
            self.refAng = self.reference[2]
        self.err_ang = self.refAng - self.pos[3]
        self.err_lin = np.linalg.norm(err)
        self.err = err

        if self.err_ang > np.pi:
            self.err_ang -= 2 * np.pi
        elif self.err_ang < -np.pi:
            self.err_ang += 2 * np.pi

        p = kp * self.err_ang
        if np.abs(self.err_ang) < 0.01:  # 0.01 rad = 0.57 deg
            p = 0.0
            self.dim = "lin" if not last else "end"

        # self.vel.linear.x = 0.0
        self.vel.angular.z = np.clip(p, -MAX_ANGLE_VEL, MAX_ANGLE_VEL)

    def linControl(self):
        akp, aki, akd = self.control[0]
        lkp, lki, lkd = self.control[1]

        err = [self.reference[0] - self.pos[1], self.reference[1] - self.pos[2]]
        self.err_ang = np.arctan2(err[1], err[0]) - self.pos[3]
        self.err_lin = np.linalg.norm(err)
        self.err = err

        lp = lkp * self.err_lin
        ap = akp * self.err_ang

        if np.abs(self.err_lin) < 0.01:
            lp = 0.0
            ap = 0.0
            self.dim = "lastang"

        self.vel.angular.z = np.clip(ap, -MAX_ANGLE_VEL, MAX_ANGLE_VEL)
        self.vel.linear.x = np.clip(lp, -MAX_LIN_VEL, MAX_LIN_VEL)

    def printStatus(self):
        data = [
            self.pos[0],  # time
            self.reference[0],  # reference x
            self.reference[1],  # reference y
            self.refAng,  # reference angle
            self.pos[1],  # x
            self.pos[2],  # y
            self.pos[3],  # angle
            self.err[0],  # x error
            self.err[1],  # y error
            self.err_ang,  # angular error
            self.vel.linear.x,  # linear velocity
            self.vel.angular.z,  # angular velocity
            self.dim,  # current process
        ]
        self.msg = ",".join([str(x) for x in data])

        if self.args.verbose:
            os.system("clear")
            print("Time: {0:>6.4f} s".format(data[0]))
            print(
                "Reference: {0:>6.4f} m, {1:>6.4f} m | {2:>6.4f} rad".format(*data[1:4])
            )
            print(
                "Position: {0:>6.4f} m, {1:>6.4f} m | {2:>6.4f} rad".format(*data[4:7])
            )
            print("Error: {0:>6.4f} m, {1:>6.4f} m | {2:>6.4f} rad".format(*data[7:10]))
            print("Control: {0:>6.4f} m/s, {1:>6.4f} rad/s".format(data[10], data[11]))

    def run(self):
        rospy.Subscriber("/tb3_0/odom", Odometry, self.callback)

        pub = rospy.Publisher("/tb3_0/cmd_vel", Twist, queue_size=10)
        dat = rospy.Publisher("/tb3_0/data", String, queue_size=10)

        rate = rospy.Rate(10)
        rate.sleep()

        self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

        try:
            while not rospy.is_shutdown() and self.dim != "end":
                pub.publish(self.vel)
                dat.publish(self.msg)

                if self.dim == "ang":
                    self.angControl()
                elif self.dim == "lin":
                    self.linControl()
                elif self.dim == "lastang":
                    self.angControl(last=True)

                self.printStatus()

                rate.sleep()

            self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(self.vel)
        except rospy.exceptions.ROSInterruptException:
            self.vel = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
            pub.publish(self.vel)
            print("Stopping")


if __name__ == "__main__":
    os.system("clear")
    rospy.init_node("leaderControlPos")

    cp = ControlPosition()
    cp.run()
