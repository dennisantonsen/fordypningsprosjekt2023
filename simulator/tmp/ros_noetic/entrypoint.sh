#!/bin/bash

echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source /opt/ros/noetic/setup.bash
catkin build

roscore