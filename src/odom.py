#!/usr/bin/env python3

import rospy
import numpy as np
import os

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from coop_robot.msg import coop_data
from tf.transformations import euler_from_quaternion as efq


class tb_odom:
    def __init__(self):
        self.pos = {
            "tb3_0": [0, 0, 0],
            "tb3_1": [0, 0, 0],
        }
        self.vel = {
            "tb3_0": 0,
            "tb3_1": 0,
        }
        self.distance = rospy.get_param("/distance")
        self.cd_0 = rospy.Publisher("/tb3_0/coop_data", coop_data, queue_size=10)
        self.cd_1 = rospy.Publisher("/tb3_1/coop_data", coop_data, queue_size=10)

        rospy.init_node("tb_odom")

        rate = rospy.Rate(10)

        rospy.Subscriber("/tb3_0/odom", Odometry, self.callback, "tb3_0")
        rospy.Subscriber("/tb3_0/cmd_vel", Twist, self.velcallback, "tb3_0")
        rate.sleep()
        rospy.Subscriber("/tb3_1/odom", Odometry, self.callback, "tb3_1")
        rospy.Subscriber("/tb3_1/cmd_vel", Twist, self.velcallback, "tb3_1")
        rate.sleep()

        pmax_error = 0

        while not rospy.is_shutdown():
            tb3_0_1step = (
                rospy.get_param("/tb3_0/1step")
                if rospy.has_param("/tb3_0/1step")
                else False
            )
            tb3_1_1step = (
                rospy.get_param("/tb3_1/1step")
                if rospy.has_param("/tb3_1/1step")
                else False
            )

            d = np.array(self.pos["tb3_0"][:2]) - np.array(self.pos["tb3_1"][:2])
            distance = np.linalg.norm(d)
            angle = np.arctan2(d[1], d[0])
            angle = np.rad2deg(angle)

            ed = abs(distance - self.distance) * 100
            ped = ed / self.distance
            if ped > pmax_error and tb3_0_1step and tb3_1_1step:
                pmax_error = ped
                max_error = ed
                

            os.system("clear")
            print(f"Distance: {distance:.4f} m, {angle:.2f}°")
            print(f"Error: {ed:.2f} cm, %: {ped:.2f}\n")
            print(f"Leader: {self.pos['tb3_0'][2]:.2f}°, {self.vel['tb3_0']:.2f} m/s")
            print(f"Follow: {self.pos['tb3_1'][2]:.2f}°, {self.vel['tb3_1']:.2f} m/s\n")

            tb3_0 = rospy.get_param("/tb3_0/status")
            tb3_1 = rospy.get_param("/tb3_1/status")

            if tb3_0 == "end" and tb3_1 == "end":
                break

            rate.sleep()

        print(f"\nMax error: {max_error:.2f} cm, %: {pmax_error:.2f}")

    def callback(self, data, ns):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        euler = efq([quat.x, quat.y, quat.z, quat.w])
        tr = euler[2]
        t = np.rad2deg(tr)

        v = data.twist.twist.linear.x
        w = data.twist.twist.angular.z

        self.pos[ns] = [x, y, t]
        
        msg = coop_data()
        msg.X = x
        msg.Y = y
        msg.T = tr
        msg.V = v
        msg.W = w
        msg.Xd = 0
        msg.Yd = 0
        msg.Td = 0

        if rospy.has_param(f"/{ns}/objective"):
            obj = rospy.get_param(f"/{ns}/objective")
            msg.Xd = obj[0]
            msg.Yd = obj[1]
            if rospy.has_param(f"/{ns}/objective_angle"):
                msg.Td = rospy.get_param(f"/{ns}/objective_angle")
            
        if ns == "tb3_0":
            self.cd_0.publish(msg)
        else:
            self.cd_1.publish(msg)

    def velcallback(self, data, ns):
        x = data.linear.x

        self.vel[ns] = x


if __name__ == "__main__":
    tbo = tb_odom()
