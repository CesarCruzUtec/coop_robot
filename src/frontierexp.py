#!/usr/bin/env python3

import rospy
import os
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, TransformStamped
from coop_robot.srv import startFE
from coop_robot.srv import sendGoal


class FrontExpl:
    def __init__(self):
        self.robot_name: str = rospy.get_param("~robot_name", "")
        self.map_topic: str = f"{self.robot_name}/map"
        self.frame_topic: str = f"{self.robot_name}/base_footprint"
        print(f"Map topic: {self.map_topic}")
        print(f"Frame topic: {self.frame_topic}")

        self.FEmap: OccupancyGrid = OccupancyGrid()
        self.nodeStart: bool = False
        self.getMap: bool = True

        self.pub = rospy.Publisher("edges_map", OccupancyGrid, queue_size=10)
        self.srv = rospy.Service("start_fe", startFE, self.srvcallback)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.callback)
        self.rate = rospy.Rate(1)

    def srvcallback(self, req) -> bool:
        print("Start the Frontier Exploration")
        self.nodeStart = True
        return True

    def callback(self, msg: OccupancyGrid):
        if self.getMap:
            self.FEmap = msg
            self.FEmap.data = list(self.FEmap.data)
            self.FEmap.header.frame_id = "map"

    def run(self):
        print("Running")

        print("Waiting for start")
        while not self.nodeStart:
            self.rate.sleep()

        print("Waiting for sendGoal service")
        rospy.wait_for_service("sendGoal")
        sendServiceGoal = rospy.ServiceProxy("sendGoal", sendGoal)

        tfBuffer = tf2_ros.Buffer(debug=False)
        listener = tf2_ros.TransformListener(tfBuffer, queue_size=1)

        prev_centX: list[float] = []
        prev_centY: list[float] = []

        input("Press Enter to continue...")

        while not rospy.is_shutdown():
            self.rate.sleep()
            os.system("clear")
            print(f"\nTime: {rospy.get_time()}")

            # Check if the map is received
            if not self.FEmap.data:
                print("No map received")
                continue

            # Stop getting the map until the end of the exploration
            self.getMap = False

            # Find all edges
            print("Finding edges")
            edges: list[int] = []
            H = self.FEmap.info.height
            W = self.FEmap.info.width
            R = self.FEmap.info.resolution
            X = self.FEmap.info.origin.position.x
            Y = self.FEmap.info.origin.position.y

            for w in range(1, W - 1):
                for h in range(1, H - 1):
                    cell = h * W + w

                    if self.FEmap.data[cell] != -1:
                        continue

                    neighbors = [
                        cell - W - 1,  # Top left
                        cell - W,  # Top
                        cell - W + 1,  # Top right
                        cell - 1,  # Left
                        cell + 1,  # Right
                        cell + W - 1,  # Bottom left
                        cell + W,  # Bottom
                        cell + W + 1,  # Bottom right
                    ]
                    neighbors = sorted(set(neighbors))
                    for neighbor in neighbors:
                        if self.FEmap.data[neighbor] != 0:
                            continue

                        self.FEmap.data[neighbor] = 10
                        edges.append(neighbor)

            # Check if there are edges
            if not edges:
                print("No edges found")
                self.getMap = True
                continue

            # Sort the edges and publish the map
            edges = sorted(set(edges))
            self.pub.publish(self.FEmap)

            # Find the frontier regions and centroids
            edgeGroup: list[int] = []
            centroids: list[int] = []
            edgeCount: int = 0
            prevEdgeCount: int = 0
            print("Finding frontier regions")
            for edge, nedge in zip(edges, edges[1:]):
                neighbors = [
                    edge - W - 1,  # Top left
                    edge - W,  # Top
                    edge - W + 1,  # Top right
                    edge - 1,  # Left
                    edge + 1,  # Right
                    edge + W - 1,  # Bottom left
                    edge + W,  # Bottom
                    edge + W + 1,  # Bottom right
                ]
                neighbors = sorted(set(neighbors))

                for neighbor in neighbors:
                    if neighbor != nedge:
                        continue

                    edgeGroup.append(edge)
                    edgeGroup = sorted(set(edgeGroup))
                    edgeCount += 1

                if edgeCount != prevEdgeCount:
                    prevEdgeCount = edgeCount
                    continue

                if edgeCount < 5:
                    edgeGroup = []
                    edgeCount = 0
                    continue

                centroid: int = len(edgeGroup) // 2
                centroids.append(edgeGroup[centroid])
                edgeGroup = []
                edgeCount = 0

            # Find the transform to get the robot position
            print("Getting robot transform")
            transform: TransformStamped = tfBuffer.lookup_transform(
                self.map_topic, self.frame_topic, rospy.Time(0), rospy.Duration(3.0)
            )

            if not isinstance(transform, TransformStamped):
                continue

            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = self.frame_topic
            transform.child_frame_id = self.map_topic

            robot_pose = Pose()
            robot_pose.position.x = transform.transform.translation.x
            robot_pose.position.y = transform.transform.translation.y
            robot_pose.position.z = 0.0
            robot_pose.orientation = transform.transform.rotation

            print(f"Robot position: {robot_pose.position.x}, {robot_pose.position.y}")

            # Convert the centroids index to x, y coordinates
            print("Converting centroids to coordinates")
            centX: list[float] = []
            centY: list[float] = []
            distances: list[float] = []
            centroidVisited: bool = False
            for centroid in centroids:
                x = (centroid % W) * R + X
                y = (centroid // W) * R + Y
                rx = robot_pose.position.x
                ry = robot_pose.position.y

                if abs(x - rx) < 0.01 and abs(y - ry) < 0.01:
                    print(f"Centroid too close to map origin X: {x:.2f}, Y: {y:.2f}")
                    continue

                for prevX, prevY in zip(prev_centX, prev_centY):
                    if abs(x - prevX) > 0.01 or abs(y - prevY) > 0.01:
                        continue

                    centroidVisited = True
                    print(f"Centroid already visited X: {prevX}, Y: {prevY}")

                if not centroidVisited:
                    centX.append(x)
                    centY.append(y)

                    dist: float = ((x - rx) ** 2 + (y - ry) ** 2) ** 0.5
                    distances.append(dist)

            # If there are no centroids, move to closest frontier edge
            if not centX or not centY:
                print("No centroids found, moving to closest frontier edge")
                centX = []
                centY = []
                distances = []

                for edge in edges:
                    x = (edge % W) * R + X
                    y = (edge // W) * R + Y
                    rx = robot_pose.position.x
                    ry = robot_pose.position.y

                    centX.append(x)
                    centY.append(y)

                    dist: float = ((x - rx) ** 2 + (y - ry) ** 2) ** 0.5
                    distances.append(dist)

            # Find the closest centroid
            print("Finding the closest centroid")
            smallest: float = 999_999.0
            mx: float = 0.0
            my: float = 0.0
            for dist, x, y in zip(distances, centX, centY):
                if dist < 0.1:
                    continue

                if dist < smallest:
                    smallest, mx, my = dist, x, y

            # Move the robot to the closest centroid
            print(
                f"Moving to closest centroid X: {mx:.2f}, Y: {my:.2f}, Distance: {smallest:.2f}"
            )

            prev_centX.append(mx)
            prev_centY.append(my)

            srvMessage: str = f"{self.robot_name},{mx},{my}"
            resp = sendServiceGoal(srvMessage)

            if resp.success:
                print("Goal not reached")
            else:
                print("Goal reached")

            # Start getting the map again
            self.getMap = True

            input("Press Enter to continue...")


if __name__ == "__main__":
    rospy.init_node("frontier_exploration")
    front_expl = FrontExpl()
    front_expl.run()
    rospy.spin()
