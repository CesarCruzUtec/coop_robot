#!/usr/bin/env python3

import rospy
import numpy as np
import os
import re

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from coop_robot.msg import coop_data
from tf.transformations import euler_from_quaternion as efq


class Robot:
    def __init__(self, ns: str = "tb3"):
        self.ns: str = ns
        self.pos: np.ndarray = np.array([0.0, 0.0])
        self.ang: float = 0.0
        self.vel: list[float] = [0.0, 0.0]
        self.true_vel: list[float] = [0.0, 0.0]
        self.pub: rospy.Publisher = None
        self.dist: float = 0.0
        self.err: float = 0.0

    def get_1step(self) -> None:
        return rospy.get_param(f"/{self.ns}/1step", False)


class tb_odom:
    def __init__(self):
        self.ns: str = rospy.get_param("~ns", "tb3")
        self.leader: str = self.ns + "_" + str(rospy.get_param("~leader", 0))
        self.send_data: bool = rospy.get_param("~send_data", False)
        self.distance = rospy.get_param("/distance", [0.0])
        if isinstance(self.distance, float) or isinstance(self.distance, int):
            self.distance = [0, self.distance]
        else:
            self.distance.insert(0, 0)
        self.robot: dict[str, Robot] = {}
        self.rate = rospy.Rate(5)

    def printStatus(self, robot: Robot):
        print(f"NS: {robot.ns}, Time: {rospy.get_time():.2f} s")
        print(f"X: {robot.pos[0]:.2f} m, Y: {robot.pos[1]:.2f} m, T: {robot.ang:.2f}°")
        print(f"V: {robot.true_vel[0]:.2f} m/s, W: {robot.true_vel[1]:.2f} rad/s")

    def run(self):
        if not self.obtainTopics():
            return

        self.rate.sleep()

        while not rospy.is_shutdown():
            os.system("clear")
            finished = 0
            for robot in self.robot.values():
                self.printStatus(robot)

                if not robot.get_1step():
                    continue
                
                status = rospy.get_param(f"/{robot.ns}/status", "idle")
                print(f"Status: {status}")
                if status == "end":
                    finished += 1

                if robot.ns == self.leader:
                    continue

                dLF = self.robot[self.leader].pos - robot.pos
                d = np.linalg.norm(dLF)
                print(f"Distance to leader: {d:.2f} m")

                ed = abs(d - robot.dist) * 100
                if ed > robot.err:
                    robot.err = ed

            if finished == len(self.robot):
                break

            self.rate.sleep()

        for robot in self.robot.values():
            if robot.ns == self.leader:
                continue
            p_err = robot.err / robot.dist
            print(f"Final error for {robot.ns}: {robot.err:.2f} cm / {p_err:.2f}%")

    def obtainTopics(self) -> bool:
        topics = rospy.get_published_topics()
        for topic in topics:
            if not re.match(rf"/{self.ns}_\d+/odom", topic[0]):
                continue

            topic_name: str = topic[0]
            robot_name: str = topic_name.split("/")[1]
            self.robot[robot_name] = Robot(robot_name)
            self.robot[robot_name].dist = self.distance.pop(0)

            rospy.Subscriber(topic_name, Odometry, self.callback, robot_name)
            rospy.Subscriber(
                f"/{robot_name}/cmd_vel", Twist, self.velcallback, robot_name
            )
            self.robot[robot_name].pub = rospy.Publisher(
                f"/{robot_name}/coop_data", coop_data, queue_size=10
            )
            print(f"Subscribed to {topic_name}")

        if not self.robot:
            print("No topics found")
            return False

        return True

    def callback(self, data, ns):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        euler = efq([quat.x, quat.y, quat.z, quat.w])
        tr = euler[2]
        t = np.rad2deg(tr)

        v = data.twist.twist.linear.x
        w = data.twist.twist.angular.z

        self.robot[ns].pos = np.array([x, y])
        self.robot[ns].ang = t
        self.robot[ns].true_vel = [v, w]

        if not self.send_data:
            return

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

        self.robot[ns].pub.publish(msg)

    def velcallback(self, data, ns):
        x = data.linear.x
        w = data.angular.z

        self.robot[ns].vel = [x, w]


if __name__ == "__main__":
    rospy.init_node("tb_odom")

    tbo = tb_odom()
    tbo.run()
