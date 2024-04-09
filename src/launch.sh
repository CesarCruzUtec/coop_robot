#!/bin/bash

# Source your ROS workspace
source /opt/ros/noetic/setup.bash
source ~/pfc_ws/devel/setup.bash

# Launch the first launch file
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch &

# Wait for a while
sleep 5

# Launch the second launch file
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch &

# Keep script running until it is manually terminated
wait