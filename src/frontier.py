#!/usr/bin/env python

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from multi_robot_exploration.srv import tb3_0_start
import math
import actionlib


class FrontierExplorer:
    def __init__(self):
        rospy.init_node("tb3_0_frontier_exploration")
        self.map_pub = rospy.Publisher("edges_map_0", OccupancyGrid, queue_size=1)
        self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        self.start_service = rospy.Service("tb3_0_start", tb3_0_start, self.start_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.move_base_client = SimpleActionClient("tb3_0/move_base", MoveBaseAction)
        self.map_frame = "tb3_0/map"
        self.base_frame = "tb3_0/base_footprint"
        self.occupancy_grid = OccupancyGrid()
        self.robot_pose = PoseStamped()
        self.frontier_edges, self.neighbor_indices, self.neighbor_values = [], [], []
        self.centroids, self.temp_region = [], []
        self.centroid_x_coords, self.centroid_y_coords, self.centroid_distances, self.visited_centroids_x, self.visited_centroids_y = [], [], [], [], []
        self.region_size, self.prev_region_size = 0, 0
        self.centroid_index, self.target_centroid_index = 0, 0
        self.map_width, self.map_height = 0, 0
        self.closest_centroid_distance = float('inf')
        self.distance_to_centroid = 0.0
        self.unique_edge_flag = True
        self.start_exploration = False

    def start_callback(self, req):
        rospy.loginfo("Received start exploration request for tb3_0")
        self.start_exploration = True
        return True

    def map_callback(self, msg):
        self.occupancy_grid = msg
        self.occupancy_grid.header.frame_id = "edges_map_0"
        rospy.loginfo("Received map data")

    def get_neighbors(self, cell):
        self.neighbor_indices.clear()
        self.neighbor_values.clear()

        neighbors = [
            cell - self.map_width - 1,
            cell - self.map_width,
            cell - self.map_width + 1,
            cell - 1,
            cell + 1,
            cell + self.map_width - 1,
            cell + self.map_width,
            cell + self.map_width + 1,
        ]
        self.neighbor_indices = list(set(neighbors))
        self.neighbor_indices.sort()

    def find_frontiers(self):
        rospy.loginfo("Finding frontiers for tb3_0")

        for cell in range(self.map_width + 1, (self.map_width * self.map_height) - self.map_width - 1):
            if self.occupancy_grid.data[cell] == -1:
                self.get_neighbors(cell)
                for neighbor_index in self.neighbor_indices:
                    if self.occupancy_grid.data[neighbor_index] == 0:
                        self.occupancy_grid.data[neighbor_index] = 10
                        self.frontier_edges.append(neighbor_index)

    def check_unique_edge(self, current_cell, next_cell):
        return current_cell != next_cell

    def find_frontier_regions(self):
        rospy.loginfo("Finding frontier regions for tb3_0")
        for i in range(len(self.frontier_edges) - 1):
            self.unique_edge_flag = self.check_unique_edge(self.frontier_edges[i], self.frontier_edges[i+1])

            if self.unique_edge_flag:
                self.get_neighbors(self.frontier_edges[i])
                for neighbor_index in self.neighbor_indices:
                    if neighbor_index == self.frontier_edges[i+1]:
                        self.temp_region.append(self.frontier_edges[i])
                        self.temp_region = list(set(self.temp_region))
                        self.temp_region.sort()
                        self.region_size += 1

                    if self.region_size == self.prev_region_size:
                        if self.region_size >= 5:
                            self.centroid_index = len(self.temp_region) // 2
                            self.centroids.append(self.temp_region[self.centroid_index])
                        self.region_size = 0
                        self.temp_region.clear()
                    else:
                        self.prev_region_size = self.region_size

    def get_robot_transform(self):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(3.0)
            )
            self.robot_pose.pose.position.x = transform_stamped.transform.translation.x
            self.robot_pose.pose.position.y = transform_stamped.transform.translation.y
            self.robot_pose.pose.position.z = 0.0
            self.robot_pose.pose.orientation = transform_stamped.transform.rotation
            rospy.loginfo(
                "Robot pose is  %f , %f", self.robot_pose.pose.position.x, self.robot_pose.pose.position.y
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e) 
            
    def convert_centroid_index_to_point(self):
        for centroid_index in self.centroids:
            point_x = (centroid_index % self.occupancy_grid.info.width) * self.occupancy_grid.info.resolution + self.occupancy_grid.info.origin.position.x
            point_y = math.floor(centroid_index / self.occupancy_grid.info.width) * self.occupancy_grid.info.resolution + self.occupancy_grid.info.origin.position.y

            if any(math.isclose(prev_x, point_x) and math.isclose(prev_y, point_y) for prev_x, prev_y in zip(self.visited_centroids_x, self.visited_centroids_y)):
                rospy.loginfo("Already visited centroid at (%f, %f)", point_x, point_y)
                continue  

            if math.isclose(point_x, self.occupancy_grid.info.origin.position.x, abs_tol=0.05) and math.isclose(point_y, self.occupancy_grid.info.origin.position.y, abs_tol=0.05):
                continue 

            self.centroid_x_coords.append(point_x)
            self.centroid_y_coords.append(point_y)

            distance = math.dist((point_x, point_y), (self.robot_pose.pose.position.x, self.robot_pose.pose.position.y))
            self.centroid_distances.append(distance)

    def find_closest_centroid(self):
        self.closest_centroid_distance = float('inf')
        for i, distance in enumerate(self.centroid_distances):
            if 0.1 < distance < self.closest_centroid_distance:
                self.closest_centroid_distance = distance
                self.target_centroid_index = i

    def convert_edge_index_to_point(self):
        for edge_index in self.frontier_edges:
            point_x = (edge_index % self.occupancy_grid.info.width) * self.occupancy_grid.info.resolution + self.occupancy_grid.info.origin.position.x
            point_y = math.floor(edge_index / self.occupancy_grid.info.width) * self.occupancy_grid.info.resolution + self.occupancy_grid.info.origin.position.y

            self.centroid_x_coords.append(point_x)
            self.centroid_y_coords.append(point_y)

            distance = math.dist((point_x, point_y), (self.robot_pose.pose.position.x, self.robot_pose.pose.position.y))
            self.centroid_distances.append(distance)

    def main_loop(self):
        rospy.loginfo("Entered the main loop for frontier exploration")

        self.move_base_client.wait_for_server(rospy.Duration(5.0))
        rospy.loginfo("Connected to move base server")

        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            rospy.loginfo("Frontier exploration loop running")
            rospy.sleep(0.025)

            if self.occupancy_grid.data and self.start_exploration:
                self.map_width = self.occupancy_grid.info.width
                self.map_height = self.occupancy_grid.info.height

                self.find_frontiers()

                if not self.frontier_edges:
                    rospy.loginfo("No frontiers found. Skipping iteration.")
                    continue  

                self.frontier_edges = list(set(self.frontier_edges))
                self.frontier_edges.sort()

                self.map_pub.publish(self.occupancy_grid)
                self.find_frontier_regions()
                self.get_robot_transform()
                self.convert_centroid_index_to_point() 

                if not self.centroid_x_coords or not self.centroid_y_coords:
                    self.centroid_x_coords.clear()
                    self.centroid_y_coords.clear()
                    self.centroid_distances.clear()
                    rospy.loginfo("No valid centroids found. Moving to closest frontier edge.")
                    self.convert_edge_index_to_point()

                self.find_closest_centroid()

                rospy.loginfo("Moving to centroid at (%f, %f)", self.centroid_x_coords[self.target_centroid_index], self.centroid_y_coords[self.target_centroid_index])
                rospy.loginfo("Distance to target centroid: %f", self.closest_centroid_distance)

                self.visited_centroids_x.append(self.centroid_x_coords[self.target_centroid_index])
                self.visited_centroids_y.append(self.centroid_y_coords[self.target_centroid_index]) 

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = self.map_frame
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = self.centroid_x_coords[self.target_centroid_index]
                goal.target_pose.pose.position.y = self.centroid_y_coords[self.target_centroid_index]
                goal.target_pose.pose.orientation.w = 1.0

                rospy.loginfo("Sending goal to move_base")
                self.move_base_client.send_goal(goal)
                self.move_base_client.wait_for_result()

                if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Successfully reached the goal!")
                else:
                    rospy.loginfo("Failed to reach the goal.")

            else:
                rospy.loginfo("Waiting for map data and start command.")

            # Reset variables for the next iteration
            self.centroid_index, self.target_centroid_index = 0, 0
            self.centroid_distances.clear()
            self.frontier_edges.clear()
            self.neighbor_indices.clear()
            self.neighbor_values.clear()
            self.centroids.clear()
            self.centroid_x_coords.clear()
            self.centroid_y_coords.clear()

            rate.sleep()

if __name__ == "__main__":
    frontier_explorer = FrontierExplorer()
    frontier_explorer.main_loop()
    rospy.spin()