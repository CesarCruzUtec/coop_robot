#!/usr/bin/env python3

import rospy
import argparse
import re
import os

from nav_msgs.msg import OccupancyGrid


class MapExpansion:
    def __init__(self) -> None:
        self.rate = rospy.Rate(5)

        parser = argparse.ArgumentParser()
        parser.add_argument(
            "robot_namespace",
            type=str,
            help="Namespace of the robot",
            default="tb3",
        )
        parser.add_argument(
            "expanded_topic",
            type=str,
            help="Name of the expanded map topic",
            default="new_map",
        )

        self.maps: dict[str, OccupancyGrid] = {}
        self.pubs: dict[str, rospy.Publisher] = {}
        self.exp_maps: dict[str, OccupancyGrid] = {}

        args = parser.parse_args()

        robot_namespace = args.robot_namespace
        expanded_topic = args.expanded_topic

        # Get all topics of of the form /tb3_*/map
        topics = rospy.get_published_topics()
        for topic in topics:
            if not re.match(rf"/{robot_namespace}_\d+/map", topic[0]):
                continue

            topic_name: str = topic[0]
            robot_name: str = topic_name.split("/")[1]
            rospy.Subscriber(topic_name, OccupancyGrid, self.callback, robot_name)
            self.pubs[robot_name] = rospy.Publisher(
                f"/{robot_name}/{expanded_topic}", OccupancyGrid, queue_size=10
            )
            self.maps[robot_name] = OccupancyGrid()
            self.exp_maps[robot_name] = OccupancyGrid()
            self.rate.sleep()

        self.create_expanded_map()

    def create_expanded_map(self):
        self.info = {
            "resolution": 0.05,
            "width": 0,
            "height": 0,
            "origin": [0, 0, 0],
            "end": [0, 0, 0],
        }

        for robot_name in self.maps.keys():
            x = self.maps[robot_name].info.origin.position.x
            y = self.maps[robot_name].info.origin.position.y
            w = self.maps[robot_name].info.width
            h = self.maps[robot_name].info.height
            r = self.maps[robot_name].info.resolution

            self.info["origin"] = [
                min(self.info["origin"][0], x),
                min(self.info["origin"][1], y),
                0,
            ]
            self.info["end"] = [
                max(self.info["end"][0], x + w*r),
                max(self.info["end"][1], y + h*r),
                0,
            ]

        self.info["width"] = int(
            (self.info["end"][0] - self.info["origin"][0]) / self.info["resolution"]
        )
        self.info["height"] = int(
            (self.info["end"][1] - self.info["origin"][1]) / self.info["resolution"]
        )

        for robot_name in self.maps.keys():
            self.exp_maps[robot_name].header = self.maps[robot_name].header
            self.exp_maps[robot_name].info = self.maps[robot_name].info
            self.exp_maps[robot_name].info.width = self.info["width"]
            self.exp_maps[robot_name].info.height = self.info["height"]
            self.exp_maps[robot_name].info.origin.position.x = self.info["origin"][0]
            self.exp_maps[robot_name].info.origin.position.y = self.info["origin"][1]
            self.exp_maps[robot_name].data = (
                [-1] * self.info["width"] * self.info["height"]
            )
            self.exp_maps[robot_name].info.resolution = self.info["resolution"]
            self.pubs[robot_name].publish(self.exp_maps[robot_name])

    def run(self):
        X = self.info["origin"][0]
        Y = self.info["origin"][1]
        W = self.info["width"]
        H = self.info["height"]

        while not rospy.is_shutdown():
            print(f"\nExpanding maps, Time: {rospy.Time.now().to_sec():.4f}s")
            print(f"X: {X:.2f}")
            print(f"Y: {Y:.2f}")
            print(f"Width: {W}")
            print(f"Height: {H}")

            if any(map(lambda x: len(x.data) == 0, self.maps.values())):
                self.rate.sleep()
                continue

            for robot_name in self.maps.keys():
                x = self.maps[robot_name].info.origin.position.x
                y = self.maps[robot_name].info.origin.position.y
                w = self.maps[robot_name].info.width
                h = self.maps[robot_name].info.height
                r = self.maps[robot_name].info.resolution

                for i in range(h):
                    px = int(W * (y - Y) / r + (x - X) / r + i * W)
                    self.exp_maps[robot_name].data[px : px + w] = self.maps[
                        robot_name
                    ].data[i * w : (i + 1) * w]

                self.pubs[robot_name].publish(self.exp_maps[robot_name])

            self.rate.sleep()

    def callback(self, msg, topic_name):
        self.maps[topic_name].header = msg.header
        self.maps[topic_name].info = msg.info
        self.maps[topic_name].data = msg.data


if __name__ == "__main__":
    rospy.init_node("map_expansion")

    me = MapExpansion()
    me.run()
