#!/usr/bin/env python3

import rospy
import numpy as np

from controller import Controller


class follower(Controller):
    def __init__(self):
        super(follower, self).__init__("tb3_1")

    def follow(self, reference, distance):
        leaderRef = reference
        newReference = [
            leaderRef[0] - distance * np.cos(leaderRef[2]),
            leaderRef[1] - distance * np.sin(leaderRef[2]),
            leaderRef[2],
        ]

        return newReference

    def plan_trajectory(self):
        initial_pos = rospy.get_param("/initial_position")
        xi, yi, ti = list(initial_pos.values())
        final_pos = rospy.get_param("/goal_position")
        xf, yf, tf = list(final_pos.values())
        distance = rospy.get_param("/distance")

        self.trajectory = [
            self.follow([xi, yi, ti], distance),
            [xi, yi],
            [1, 2],
            [2.5, 1],
            [2, -1],
            self.follow([xf, yf, tf], distance),
        ]

    def run(self):
        self.plan_trajectory()
        self.rate.sleep()
        
        current_dest = self.trajectory.pop(0)
        self.move_to(*current_dest, first=True)
        rospy.set_param(f"/{self.ns}_1step", True)

        while not rospy.get_param("/tb3_0_1step"):
            self.rate.sleep()
            pass

        while not rospy.is_shutdown() and self.trajectory:
            current_dest = self.trajectory.pop(0)
            self.move_to(*current_dest)
        
        rospy.set_param(f"/{self.ns}_status", "end")


if __name__ == "__main__":
    rospy.init_node("follower")

    follower = follower()
    follower.run()
