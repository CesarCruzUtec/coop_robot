#!/usr/bin/env python3

import rospy
import sys
import numpy as np

from tf.transformations import euler_from_quaternion as efq

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

# Usage example: rosrun coop_robot simple_controller.py tb3_0 0.5 0.5 0.0

MAX_ANG_VEL = 1.82
MAX_LIN_VEL = 0.26

ANG_TOL = 0.02  # 1.14°
LIN_TOL = 0.02  # 5%

KPL = 1.0
KPA = 1.5


class SimpleController():
    def __init__(self) -> None:
        args = sys.argv[1:]

        self.ns = args[0]
        if len(args) == 2:
            t = np.deg2rad(float(args[1]))
            self.dest = [None, None, t]
        elif len(args) == 3:
            self.dest = [float(args[1]), float(args[2]), None]
        else:
            t = np.deg2rad(float(args[3]))
            self.dest = [float(args[1]), float(args[2]), t]
        
        self.demo()
        if self.ns == "tb3_d":
            sys.exit(0)

        self.pos = [0, 0, 0]
        self.rate = rospy.Rate(10)

        rospy.Subscriber(f"/{self.ns}/odom", Odometry, self.callback)
        self.pub = rospy.Publisher(f"/{self.ns}/cmd_vel", Twist, queue_size=10)
        self.vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        self.move_to(*self.dest)
        self.rotate_to(self.dest[2])
    
    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        euler = efq([quat.x, quat.y, quat.z, quat.w])
        theta = euler[2]

        self.pos = [x, y, theta]
    
    def demo(self):
        print("Demo mode")
        print(f"Destination: {self.dest}")
    
    def move_to(self, x, y, t):
        if x is None and y is None:
            self.rotate_to(t)
            return
        
        err = [x - self.pos[0], y - self.pos[1]]
        ang_ref = np.arctan2(err[1], err[0])
        self.rotate_to(ang_ref)

        finished = False
        while not finished:
            err = [x - self.pos[0], y - self.pos[1]]
            ang_ref = np.arctan2(err[1], err[0])

            lin_err = np.linalg.norm(err)
            ang_err = ang_ref - self.pos[2]

            lin_vel = KPL * lin_err
            ang_vel = KPA * ang_err

            if np.linalg.norm(err) < LIN_TOL:
                lin_vel = 0
                ang_vel = 0
                finished = True
            
            self.vel.linear.x = np.clip(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
            self.vel.angular.z = np.clip(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)
            self.pub.publish(self.vel)

            self.rate.sleep()
            
        if t is not None:
            self.rotate_to(t)

    def rotate_to(self, ang):
        if ang is None:
            return
        finished = False
        while not finished:
            ang_err = ang - self.pos[2]

            if ang_err > np.pi:
                ang_err -= 2 * np.pi
            elif ang_err < -np.pi:
                ang_err += 2 * np.pi

            ang_vel = KPA * ang_err

            if np.abs(ang_err) < ANG_TOL:
                ang_vel = 0
                finished = True
            
            self.vel.linear.x = 0
            self.vel.angular.z = np.clip(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)
            self.pub.publish(self.vel)

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("simple_controller")

    sc = SimpleController()
