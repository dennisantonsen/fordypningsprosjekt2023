# RViz

RViz is a tool for visualizing data. In this project, we use RViz for displaying the following graphically:

- The interpretation of walkways
- The position of points of interest
- The position and score of possible inspection points
- The chosen inspection point for a given POI.

RViz is connected to ROS. For visualizing data with RViz, we have to publish markers to a ROS topic. See e.g. `mission_planner/src/utils.py` for how we do this.

## /src/config.rviz

When RViz boots, it can read a config file. The config file `config.rviz` is placed in `/src`. The easiest way to edit the config file is to run RViz, make changes to the GUI, and then export the config file from RViz (remember: after exporting, the config file will end up inside the docker container. Copy it to your locally stored project by running `docker cp container-name:container-path:local-path`, while the container is still running.)