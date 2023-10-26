#!/bin/bash

echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source /opt/ros/noetic/setup.bash
catkin build

roslaunch --wait gazebo_ros empty_world.launch world_name:="/home/catkin_ws/src/gazebo/world/world.world"

sleep 1d