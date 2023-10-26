# Gazebo

We use Gazebo as our simulation tool. In Gazebo we spawn all our 3D models (in this project: a robot, the Huldra rig, and maybe some valve-like objects).

This module simply installs and runs Gazebo.

Note: To be able to spawn models in Gazebo correctly, the models might have to be stored inside the Gazebo container. Links inside model files will be relative to the container path.

## /world/world.world

This is the world we launch in Gazebo. It's an empty world, similar to the default Gazebo world empty.world. The only difference is the light settings (we use 0 shadows in our world.world).

## How Huldra models spawn in Gazebo

Huldra models (located in /resources/huldra-models at the project root) are mounted into the Gazebo container. See docker-compose.yml at project root for details.