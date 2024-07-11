#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid

from coop_robot.srv import startFE

class FrontExpl:
    def __init__(self):
        self.FEmap: OccupancyGrid = OccupancyGrid()
        self.nodeStart: bool = False

        self.pub = rospy.Publisher("edges_map", OccupancyGrid, queue_size=10)
        self.srv = rospy.Service("start_fe", startFE, self.srvcallback)
        rospy.Subscriber("map", OccupancyGrid, self.callback)
    
    def srvcallback(self, req):
        print("Start the Frontier Exploration")
        self.nodeStart = True
        return True

    def callback(self, data):
        self.FEmap = data
        self.FEmap.header.frame_id = "map"
        print("Received map")

    def run(self):
        print("Running")
        
        rospy.wait_for_service




if __name__ == "__main__":
    rospy.init_node("frontier_exploration")
    front_expl = FrontExpl()
    front_expl.run()
    rospy.spin()
