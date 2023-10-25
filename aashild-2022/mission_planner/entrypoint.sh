#!/bin/bash

echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source /opt/ros/noetic/setup.bash

echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc
source /home/catkin_ws/devel/setup.bash

catkin build

# Workspace packages have changed, please re-source setup files to use them.
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source /opt/ros/noetic/setup.bash

echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc
source /home/catkin_ws/devel/setup.bash

roslaunch --wait mission_planner mission_planner.launch

sleep 1d