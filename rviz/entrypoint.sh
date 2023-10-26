#!/bin/bash

echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
source /opt/ros/noetic/setup.bash

echo 'source /home/catkin_ws/devel/setup.bash' >> ~/.bashrc
source /home/catkin_ws/devel/setup.bash

catkin build

rosrun rviz rviz -d /home/catkin_ws/src/rviz/src/config.rviz

sleep 1d