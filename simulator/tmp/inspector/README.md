# Inspector

This project is mainly about finding good positions for inspection points of interest. We don't really care about carrying out inspections, and we don't really need a robot for calculating positions.
However, for debugging purposes, we need the Inspector module with the inspector robot.

The inspector module is responsible for spawning an inspector robot, and making the robot carry out inspections. Put simply, the module takes a list of POIs and a list of corresponding inspection poses. Then, the inspector robot takes a photo of each POI, from the corresponding inspection pose.

## Docs

- [Procedural Generation for Gazebo - Dynamically creating objects](https://boschresearch.github.io/pcg_gazebo_pkgs/tutorials/simulation/objects/#sphere)


## Docker

Terminal 1
```sh
# Run everything
./run.sh
```

Terminal 2
```sh
# Open a shell in the "inspector" container
docker exec -it inspector bash

# From here we can manually start/kill the inspector ros node
# ctrl+c to kill it
roslaunch inspector inspector.launch
```

Terminal 3 (optional)
```sh
# Sometimes it is just nice to be able to query the ROS network will nodes are running in other processes
# Hence can open another session in the inspector container

# Open a shell in the "inspector" container
docker exec -it inspector bash

# Inspect env vars
printenv

# List node
rosnode list

# Kill nodes the gentle way
rosnode kill /inspector
```

### Workaround when things are not in sync

rosnode `inspector` has a dependency on output from rosnode `mission_planner`.  
Sometimes (often?) `mission_planner` will complete and exit before `inspector` is ready to recieve info.  

The workaround is to simply restart the `mission_planner` when you see the `inspector` is just hanging around with nothing to do.

Terminal:
```sh
# Enter the "mission_planner" container
docker exec -it mission_planner bash

# When inside, restart the rosnode
roslaunch mission_planner mission_planner.launch

# And now you should see the "inspector" rosnode start doing some work when the planner is done.
```