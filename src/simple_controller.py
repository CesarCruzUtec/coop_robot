#!/usr/bin/env python3

import rospy
import numpy as np
import re

from tf.transformations import euler_from_quaternion as efq

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

from coop_robot.srv import sendGoal


MAX_ANG_VEL = 1.82
MAX_LIN_VEL = 0.26

ANG_TOL = 0.02  # 1.14°
LIN_TOL = 0.02  # 5%

KPL = 1.0
KPA = 1.5


class Robot:
    def __init__(self, ns: str = "tb3") -> None:
        self.ns: str = ns
        self.pos: list[float] = [0.0, 0.0, 0.0]
        self.running: bool = False
        self.pub: rospy.Publisher = None
        self.vel: Twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.rate: rospy.Rate = rospy.Rate(10)

    def move_to(self, x: float, y: float, t: float):
        if x is None and y is None:
            self.rotate_to(t)
            return

        err = [x - self.pos[0], y - self.pos[1]]
        ang_ref = np.arctan2(err[1], err[0])
        self.rotate_to(ang_ref)

        finished = False
        while not finished:
            err = [x - self.pos[0], y - self.pos[1]]
            # ang_ref = np.arctan2(err[1], err[0])

            lin_err = np.linalg.norm(err)
            ang_err = ang_ref - self.pos[2]

            if ang_err > np.pi:
                ang_err -= 2 * np.pi
            elif ang_err < -np.pi:
                ang_err += 2 * np.pi

            # lin_vel = KPL * (np.cos(self.pos[2]) * err[0] + np.sin(self.pos[2]) * err[1])
            lin_vel = KPL * lin_err
            ang_vel = KPA * ang_err

            if lin_err < LIN_TOL:
                lin_vel = 0
                ang_vel = 0
                finished = True

            self.vel.linear.x = np.clip(lin_vel, -MAX_LIN_VEL, MAX_LIN_VEL)
            self.vel.angular.z = np.clip(ang_vel, -MAX_ANG_VEL, MAX_ANG_VEL)
            self.pub.publish(self.vel)

            self.rate.sleep()

        self.rotate_to(t)

    def rotate_to(self, ang: float):
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
        
    def robot_stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel)


class SimpleController:
    def __init__(self) -> None:
        self.ns = rospy.get_param("~ns", "tb3")
        self.robot: dict[str, Robot] = {}

        self.srv = rospy.Service("/sendGoal", sendGoal, self.srv_call)

        self.obtainTopics()

    def obtainTopics(self) -> bool:
        topics = rospy.get_published_topics()
        for topic in topics:
            if not re.match(rf"/{self.ns}_\d+/odom", topic[0]):
                continue

            topic_name: str = topic[0]
            robot_name: str = topic_name.split("/")[1]
            self.robot[robot_name] = Robot(robot_name)

            rospy.Subscriber(topic_name, Odometry, self.callback, robot_name)
            self.robot[robot_name].pub = rospy.Publisher(
                f"/{robot_name}/cmd_vel", Twist, queue_size=10
            )
            print(f"Subscribed to {topic_name} and published to /{robot_name}/cmd_vel")

        if not self.robot:
            print("No topics found")
            return False

        return True

    def srv_call(self, req) -> bool:
        if req.ns_goal == "topic":
            return self.obtainTopics()

        req_str = req.ns_goal.split(",")
        ns = req_str[0]
        arg1 = float(req_str[1]) if len(req_str) > 1 else None
        arg2 = float(req_str[2]) if len(req_str) > 2 else None
        arg3 = float(req_str[3]) if len(req_str) > 3 else None

        if ns not in self.robot:
            print(f"Robot {ns} not found")
            return False

        if arg3 is not None:
            dest = [arg1, arg2, np.deg2rad(arg3)]
        elif arg2 is not None:
            dest = [arg1, arg2, None]
        elif arg1 is not None:
            dest = [None, None, np.deg2rad(arg1)]
        else:
            print(f"Robot {ns} not moving")
            self.robot[ns].running = False
            return False

        if self.robot[ns].running:
            print(f"Robot {ns} is already moving")
            return False

        print(f"Moving {ns} to {dest[0]}, {dest[1]}, {dest[2]}")
        self.robot[ns].running = True
        self.robot[ns].move_to(*dest)
        self.robot[ns].running = False

        return True

    def callback(self, msg, robot_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        euler = efq([quat.x, quat.y, quat.z, quat.w])
        theta = euler[2]

        self.robot[robot_name].pos = [x, y, theta]

    def run(self):
        print("Running simple controller")
        rospy.spin()
        for robot in self.robot.values():
            robot.robot_stop()
        print("Simple controller stopped")


if __name__ == "__main__":
    rospy.init_node("simple_controller")

    sc = SimpleController()
    sc.run()
