#!/usr/bin/env python3

import rospy

from controller import Controller


class leader(Controller):
    def __init__(self):
        super(leader, self).__init__("tb3_0")

    def plan_trajectory(self):
        initial_pos = rospy.get_param("/initial_position")
        xi, yi, ti = list(initial_pos.values())
        final_pos = rospy.get_param("/goal_position")
        xf, yf, tf = list(final_pos.values())

        self.trajectory = [
            [xi, yi, ti],
            [1, 2],
            [2.5, 1],
            [2, -1],
            [xf, yf, tf],
        ]

    def run(self):
        self.plan_trajectory()
        self.rate.sleep()

        current_dest = self.trajectory.pop(0)
        self.move_to(*current_dest, first=True)
        rospy.set_param(f"/{self.ns}_1step", True)

        while not rospy.get_param("/tb3_1_1step"):
            self.rate.sleep()
            pass

        while not rospy.is_shutdown() and self.trajectory:
            current_dest = self.trajectory.pop(0)
            self.move_to(*current_dest)
        
        rospy.set_param(f"/{self.ns}_status", "end")

if __name__ == "__main__":
    rospy.init_node("leader")

    leader = leader()
    leader.run()
