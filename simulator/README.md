# How to simulate

Notes on how to get output from godot into ISAR which then talks to the simulator.

TODO: Rewrite these notes into a proper guide when we got it working.


Main components:
- ISAR api    
  - Interface for controlling the robot by handing it a robot mission
  - Talks to the simulator via ROS-Bridge
  - [Repo](https://github.com/equinor/isar)
- ISAR-Turtlebot  
  - The simulator
  - [repo](https://github.com/equinor/isar-turtlebot)  
- Godot  
  - The game engine where we produce a robot mission



Preparations:
1. Simulator:
   1. Create map for Gazebo
   1. Create map for robot (SLAM)
1. ISAR api:
   1. Verify that we can run it
   1. Verify that it can accept output json from Godot


Normal simulation workflow:
1. Run ISAR api (https://github.com/equinor/isar) - This will be the main point of interaction with the robot.
1. Run ISAR-Turtlebot - This will be the simulator, controlled by the IAR Api
1. Connect ISAR Api to simulator via ROS bridge

Verify json output using ISAR Api:
1. Run ISAR api and ISAR-Robot (the api need something to talk to)
1. Open ISAR in a browser, inspect api and upload the json
1. Inspect output from ISAR




## How to use

### Configure Equinor's `isar-turtlebot` simulator for use with our models

1. Clone the simulator repo into the `simulator/` directory  
   ```sh
   cd simulator
   git clone https://github.com/equinor/isar-turtlebot
   ```

1. Copy `sdf` models to the `isar-turtlebot`:  
   `cp -r world/huldra/sdf/huldra* isar-turtlebot/models/`
1. Copy simulator config to the `isar-turtlebot`:  
   `cp -r world/huldra/huldra.cfg isar-turtlebot/config/`

### Run

```sh
WORLD_NAME=huldra docker compose up --build

# Error - I'm still missing the planner map
# [ERROR] [1714128993.649111124]: Map_server could not open /home/catkin_ws/src/isar_turtlebot/maps/huldra.yaml. 

# https://automaticaddison.com/how-to-create-a-map-for-ros-from-a-floor-plan-or-blueprint/
# Add docker dev env for tooling to create slam map from 2d image based on top-down view of 3d object/model
```