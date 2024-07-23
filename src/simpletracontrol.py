#!/usr/bin/env python3

import rospy

from coop_robot.srv import sendGoal

TRA = [[2.0, 0.0],[3.0, -1.0], [3.0, -2.0]]

class SimpleTra:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.robot_name = rospy.get_param("~robot_name", "tb3_0")

        print("Waiting for sendGoal service")
        rospy.wait_for_service("sendGoal")
        self.sendServiceGoal = rospy.ServiceProxy("sendGoal", sendGoal)
        rospy.set_param("/send_data", False)

    def run(self):
        input("Press Enter to start")
        rospy.set_param("/send_data", True)
        self.rate.sleep()
        while not rospy.is_shutdown():
            self.rate.sleep()

            if len(TRA) == 0:
                rospy.loginfo("Finished")
                break

            dest = TRA.pop(0)
            rospy.loginfo(f"Going to {dest[0]}, {dest[1]}")
            srvMessage: str = f"{self.robot_name},{dest[0]},{dest[1]}"
            resp = self.sendServiceGoal(srvMessage)

            if not resp.success:
                rospy.logerr("Failed to send goal")
                break

if __name__ == "__main__":
    rospy.init_node("simple_tra")
    st = SimpleTra()
    st.run()
    rospy.spin()