#!/usr/bin/env python3

import rospy
import numpy as np

from controller import Controller


class follower(Controller):
    def __init__(self):
        super(follower, self).__init__("tb3_1")

    def follow(self, reference, distance):
        if len(reference) == 2:
            trajectory = rospy.get_param("/intermediate_points")
            prev = trajectory[-1]
            angle = np.arctan2(reference[1] - prev[1], reference[0] - prev[0])
            reference.append(angle)

        newReference = [
            reference[0] - distance * np.cos(reference[2]),
            reference[1] - distance * np.sin(reference[2]),
            reference[2],
        ]

        return newReference

    def plan_trajectory(self):
        initial_pos = rospy.get_param("/initial_position")
        final_pos = rospy.get_param("/goal_position")
        distance = rospy.get_param("/distance")

        trajectory = rospy.get_param("/intermediate_points")
        trajectory.insert(0, initial_pos)
        trajectory.insert(0, self.follow(initial_pos, distance))
        trajectory.append(self.follow(final_pos, distance))

        self.trajectory = trajectory

    def run(self):
        self.plan_trajectory()
        self.rate.sleep()
        
        # Move to the first point and positionate
        current_dest = self.trajectory.pop(0)
        self.move_to(*current_dest, first=True)
        rospy.set_param(f"/{self.ns}/1step", True)

        while not rospy.get_param(f"/{self.other}/1step"):
            self.rate.sleep()

        while not rospy.is_shutdown() and self.trajectory:
            current_dest = self.trajectory.pop(0)
            self.move_to(*current_dest)
        
        rospy.set_param(f"/{self.ns}/status", "end")


if __name__ == "__main__":
    rospy.init_node("follower")

    follower = follower()
    follower.run()
