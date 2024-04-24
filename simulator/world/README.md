# World

When we talk about "model" in this context, we are talking about models that are used as maps or world objects.  
Example: Huldra is one big 3d object. We use this, or a smalle slice, as a world object where the robot can perfom missions.  

The robot is `turtlebot` which comes with its own model from the vendor for use in the simulator.  
To navigate it requires a map, and we create this map from the Huldra model.

The game engine (godot), simulator (gazebo) and planner (turtlebot) each require their own format based of the same "world" model:
- Gazebo: `*.sdf`
- Godot: `*.gltf`
- Planner:`*.pgm`

