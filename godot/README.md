# GODOT

The optimization algorithm is run in the game engine Godot. This is where the algorithm uses the 3D model of a facility to find optimal inspection poses of requested valves within the facility. The algorithm will output a JSON file containing the most optimal inspection poses found during its inspection. The output JSON file is then input to ISAR to either be simulated on ISAR-Turtlebot or to be used on a physical robot.

## Setup guide for the algorithm Godot environment:

### Importing and setting up the facility:

1. Make a new project in Godot.
2. Import the gLTF files of the facility to the `res://` folder in the Godot project.
3. Import the gLTF file as a scene in Godot.
    - **Additional description:** This can be done by dragging the gLTF file of the facility found in the `FileSystem` window in the bottom left corner into the 3D viewer window in the middle of the screen. If the 3D viewer is not visible in the middle of the screen, go to the `3D` tab and it should be immediately visible. Also note that for the case of larger facilities, there may be more than one gLTF files, which will result in multiple scenes. Just repeat each action for each individual scene.
4. In the `Scene` menu on the left side in Godot, right-click the facility scene and check the box for `Editable Children`.
    - **Additional description:** This will make all components of the facility scene visible in the `Scene` menu.
5. In the `Scene` menu, select all the objects that belong to the facility scene, go to the `Mesh` menu that can be seen in the 3D viewer window in the middle of the screen and select `Create Trimesh Static Body`.
    - **Additional description:** This will give all the nodes in the facility scene a collision shape, which is important so that the ray-casts can identify these objects.

### Setting up and defining the valves:

1. In the `Scene` menu, right-click the `Node3D` scene, select `Add Child Node`, and add a `StaticBody3D` node.
2. Change the name of the `StaticBody3D` node to `Valve 1`.
    - **Additional description:** This is done by double-clicking the name of the node in the `Scene` menu.
3. In the `Scene` menu, right-click `Valve 1` and select `Add Child Node`, then add a `MeshInstance3D` node.
4. In the `Scene` menu, select the new `MeshInstance3D` node, go to the `Inspector` menu located on the right side of Godot. In the `Inspector` menu, go to the `Mesh` property and select a shape from the drop-down menu.
    - **Additional description:** The rest of this guide will assume the selection of a sphere shape, which is the shape that was used to define the valves in this project. The size of the sphere can be changed by changing the radius in the `Inspect` menu. However, it is recommended to change the `Scale` property of the `StaticBody3D` node (now renamed `Valve 1`) under the `Transform` menu in the `Inspector` window instead of changing the radius of the `MeshInstance3D` sphere directly. It is also easier to determine how to size and dimension the sphere when it is placed on the valve within the facility.
5. In the `Scene` menu, select the `MeshInstance3D` within the `Valve 1` node, go to the `Mesh` menu in the 3D viewer, and select `Create TriMesh Collision Sibling`.
    - **Additional description:** Similarly to the nodes in the facility scene, this will give `Valve 1` a collision shape which is important so that the ray-casts can identify the valve.
6. In the `Scene` menu, select `Valve 1`. At the top of the `Inspector` window, there will be a selection of tabs. Go to the `Node` tab, and select the `Groups` tab within the `Node` tab. Add the `Valve 1` tab to the `valves` group.
    - **Additional description:** To add `Valve 1` to the `valves` group, one has to type the name of the group manually in the empty text field in the `Groups` tab. In this empty text field, type "valves" and hit enter.
7. Within the 3D viewer, move the sphere by selecting `Valve 1` in the `Scenes` menu and dragging the sphere to the facility within the 3D viewer.
    - **Additional description:** Here it is important that the `StaticBody3D` node (the node now renamed `Valve 1`) is selected and not the `MeshInstance3D` node or the `CollisionShape3D` node. A useful tip when dragging the sphere to the facility is to zoom very far out in the 3D viewer, drag the sphere within close proximity to the facility, and double click `Valve 1` within the `Scene` menu to zoom in on the sphere. Continue this process until the sphere is in the facility.
8. Within the 3D viewer, place the sphere on any valve that is to be inspected using the algorithm.
    - **Additional description:** Which valve the sphere is placed on within the facility is up to the user. The sphere can also be reshaped by changing the scale of the `Valve 1` node to define the point of interest on the valve. An additional tip to make only the valves within the facility visible is to import the valves separately in a separate file and turn off the visibility of the facility by clicking the eye in the `Scene` menu. Another approach is to go through all of the nodes in the facility scene and hide every node that is not a valve. It is also entirely possible to locate the valves by moving through the facility in the 3D viewer.
9. When the first sphere is placed on the first valve, right-click the `Valve 1` node, copy it, and paste it in the `Node3D` scene, all in the `Scene` menu. Place the copied sphere on the next valve that the algorithm should inspect in the 3D viewer. Repeat this procedure until all valves of interest have a sphere marking the point of interest on the valve.
    - **Additional description:** The first copied sphere should have a `StaticBody3D` node automatically renamed to `Valve 2`. Also, keep in mind that it is very important that it is this `StaticBody3D` node that is moved around when placing the spheres on the valves and not any of its children (the `MeshInstance3D` node and the `CollisionShape3D` node). The visibility of the spheres can be turned off so that they do not cover the valves in the resulting inspection images output by the algorithm. This is done by clicking the small eye icon next to each valve node in the `Scene` menu.

### Defining the walkway:

1. In Godot, locate the `MeshInstance3D` nodes that make up the walkway, and add them to the `walkway` group.
    - **Additional description:** This can be done by either importing the walkway files separately, navigating through the facility scene in the `Scene` menu and locating the nodes there, or by clicking on the objects that make up the walkway in the 3D viewer one by one. The latter is probably the easiest, but one has to make sure that all objects that make up the walkway are accounted for. Adding the nodes to the `walkway` group is done in the exact same way as the valves are added to the `valves` group. When adding the nodes to the `walkway` group, make sure that it is the `MeshInstance3D` node that gets added to the group.

### Adding the algorithm scripts:

1. Navigate to the project files for the current project.
    - **Additional description:** This can be done by right-clicking the `res://` in the `FileSystem` menu in the bottom left corner in Godot.
2. Import the `robot.gd` script (in the GitHub repository: /Main/Godot/Godot Test Simulation/robot.gd), the `robot.tscn` scene (in the GitHub repository: /Main/Godot/Godot Test Simulation/robot.tscn), and the `node_3d.gd` script (in the GitHub repository: /Main/Godot/Godot Test Simulation/node_3d.gd) to this project folder.
3. Restart Godot.
    - **Additional description:** Once Godot has restarted, the files will be visible under the `res://` folder in the `FileSystem` menu.
4. In the `Scenes` menu, right-click the `Node3D` scene, select `Attach Script` with path `res://node_3d.gd`, and click `Load`.
    - **Additional description:** This will add the `node_3d.gd` script to the `Node3D` scene to allow for communication between the `node_3d.tscn` scene and the `robot.tscn` scene.

### Setting up the robot:

1. In the `Scenes` menu, right-click the `Node3D` scene, select `Add Child Node`, and add a `GridContainer` node.
2. In the `Scenes` menu, right-click the `GridContainer` node and add a `SubViewportContainer` node.
3. In the `Scenes` menu, right-click the `SubViewportContainer` node and add a `SubViewport` node.
4. Place the `robot.tscn` scene in the `SubViewport` node.
    - **Additional description:** This can be done by dragging the `robot.tscn` scene from the `FileSystem` menu to the `SubViewport` node in the `Scenes` menu.
5. Optional: Change the scale of the robot for more realistic proportions.
    - **Additional description:** This can be done by selecting the `Robot` scene in the `Scene` menu and going to the `Inspector` menu. In the `Inspector` menu, go to `Transform` and change the `Scale` property. A scale of 0.25 was used in this project.

### Adding the JSON mission definition template file:

1. Similarly to when the algorithm scripts were added, locate the project files and add a JSON mission definition file.
    - **Additional description:** The mission definition file can either be empty (example in the GitHub repository: /Main/Godot/Godot Test Simulation/default_turtlebot_empty.json) or have tasks already within the file. The algorithm will use this file as a template when creating the final output file and will add tasks to the existing tasks list in the template file. This template file will not be overwritten, as the algorithm will create a mission definition file for the output.

2. Define the path to the JSON template file in the algorithm settings.
    - **Additional description:** To change the algorithm settings, go to the `Script` tab, select the `robot.gd` script, and locate the settings menu at the top of the script. If the option to select the `robot.gd` script does not appear in the `Script` tab, go to the `Scenes` menu and go to the `Robot` scene under the `SubViewport` node and click the small script symbol on the `Robot` scene. Change the `json_file_template` setting to accurately point to the JSON template file.

### Optional - Add lighting:

1. In the `Scenes` menu, right-click `Node3D`, select `Add Child Node`, and add a `DirectionalLight3D` node.
    - **Additional description:** The `DirectionalLight3D` node is not necessary for the setup to function, but it is recommended to increase visibility within the facility. The images saved by the robot will greatly increase in quality if there is one or several light sources lighting up the facility.

2. In the 3D viewer, move the light source to where it lights up the facility after preference.
    - **Additional description:** A tip is to do an automatic translation to the position of one of the valves. This is done by finding the position of a valve by going to the `Scenes` menu and selecting for instance `Valve 1` and going to the `Inspector` menu. Under `Transform` in the `Inspector` menu, take note of the coordinates in the `Position` property. Copy these coordinates to the corresponding `Position` property for the `DirectionalLight3D` node. Also, edit the `Rotation` property right below the `Position` property to rotate the node -90 degrees around the X-axis. After this is done, move the `DirectionalLight3D` node a good distance above the facility in the 3D viewer. This should light up large portions of the facility. Another option is to add an `OmniLight3D` which emits light in all directions.

3. Adjust the light source so that it gives the desired effect. It is also possible to add several light sources to cover more of the facility.

### Optional - Add a third-person camera

1. In the `Scenes` menu, right-click the `GridContainer` node, select `Add Child Node`, and add a `SubViewportContainer` node and a `SubViewport` child to this node.
2. In the `SubViewport` child node, add a `Camera3D` node by right-clicking the `SubViewport`.
    - **Additional description:** This `Camera3D` node is not necessary for the setup to function, but it is useful for viewing the facility in third-person as the robot moves around inspecting valves.
3. Move the `Camera3D` node to the facility.
    - **Additional description:** A tip is to use the same technique with automatic translation that was presented when explaining how to add lighting to the facility. A key difference, however, is that the camera needs to be positioned differently than the light source.
4. Position and rotate the camera into a preferred pose.
    - **Additional description:** When the camera seems to be positioned correctly, and it seems like it covers most of the facility, click the `Preview` button in the 3D viewer while the camera is selected in the `Scenes` menu.

### Preparing to start the simulation:

1. Change the settings in the `robot.gd` script to fit the needs of the simulation.
2. Run the algorithm by clicking the start symbol at the top of the Godot window.
    - **Additional description:** The first time the algorithm is run, the user is prompted to select a main scene. If this happens, select the `Select` option and select the `node_3d.tscn` scene. The results after the algorithm has been run are stored in the `res://` folder.
