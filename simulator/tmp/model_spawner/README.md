# Model spawner

This module is responsible for spawning all the models we want to show in our world.

Model spawning is done by publishing models to a ROS topic. Gazebo subscribes to the topic, and inserts the models into the Gazebo world.