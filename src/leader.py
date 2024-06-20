#!/usr/bin/env python3

import rospy

from controller import Controller


class leader(Controller):
    def __init__(self):
        super(leader, self).__init__("tb3_0")

    def plan_trajectory(self):
        initial_pos = rospy.get_param("/initial_position")
        final_pos = rospy.get_param("/goal_position")

        # Will be replaced by a more complex algorithm
        trajectory = rospy.get_param("/intermediate_points")
        trajectory.insert(0, initial_pos)
        trajectory.append(final_pos)

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
    rospy.init_node("leader")

    leader = leader()
    leader.run()
