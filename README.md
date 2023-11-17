# Fordypningsprosjekt2023

TODO: English translation?

Industrianlegg har gjerne tusenvis av ventiler, sensorer og andre elementer som bør inspiseres visuelt med jevne mellomrom, og autonome roboter kan brukes til å automatisere denne prosessen. Det krever at roboten er i stand til å finne en tilgjengelig posisjon som gir god sikt til hvert aktuelle element. Dersom anlegget har en god digital tvilling kan planleggingen i mange tilfeller gjøres på forhånd ut fra 3D-modellen av anlegget. I en masteroppgave våren 2023, samt tidligere arbeider, har det blitt implementert en optimaliseringsalgoritme for inspeksjon med 3D-modellen av anlegget Huldra som testområde. En svakhet ved de eksisterende algoritmene er at de har for lang kjøretid ved bruk i store anlegg, og et forsøk på å lage en mer effektiv søkealgoritme førte til at mindre optimale lokale minimum ble funnet i mange tilfeller. Algoritmene er også noe begrenset med hensyn på hvilke lovlige posisjoner som finnes, og robotens fleksibilitet til å flytte kameraet horisontalt og vertikalt i forhold til robotens base.

I denne oppgaven settes fokus på å øke fleksibiliteten for kameraplassering i forhold til roboten, og på å utarbeide en bedre søkealgoritme for robotposisjonen.

Prosjektet kan oppsummeres i de følgende punktene:
- Skaffe oversikt over verktøykjeden og algoritmene etablert i tidligere arbeider
- Parametrisere kameraposisjon i forhold til robotens base som funksjon av lengde på arm og mulige vinkler.
- Etablere søkealgoritme som finner optimal plassering av robot og kamera.
- Vurdere og teste tiltak for å unngå at algoritmen finner suboptimale lokale minium.
- Hvis mulig, forberede test av algoritmen ved tilgjengelig fysisk anlegg med fysisk robot.
- Gi anbefalinger for det videre arbeidet

Tidligere arbeid:
- [erlend master 2021](https://github.com/erlendb/ntnu-masteroppgave)
- [aashild ntnu oppgave 2022](https://github.com/aashilbr/aashild-ntnu-oppgave-2022/tree/produce_json)  
  Branch `produce_json` er den som inneholder siste status.


## Table of Contents

- [Technologies](#technologies)
- [Components](#components)
- [How to run](#how-to-run)
- [How to develop](#how-to-develop)
  - [Development workflow](./development-workflow.md)
- [Knows issues](#known-issues)


## Technologies

_Robotics_  
- [ROS - Robot Operating System](https://www.ros.org/)  
  > The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications.  
  From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it's all open source.
- [Gazebo](https://gazebosim.org/home)  
  > Gazebo is a collection of open source software libraries designed to simplify development of high-performance applications. The primary audience for Gazebo are robot developers, designers, and educators.  

  This project make use of the simulator to verify algorithms.
- [RViz](https://wiki.ros.org/rviz)  
  > 3D visualization tool for ROS
- [ISAR](https://github.com/equinor/isar)  
  > ISAR - Integration and Supervisory control of Autonomous Robots - is a tool for integrating robot applications into operator systems.  
  Through the ISAR API you can send commands to a robot to do missions and collect results from the missions.
- [ISAR-turtlebot](https://github.com/equinor/isar-turtlebot)  
  > ISAR implementation for the Turtlebot3 Waffle Pi

_Data_  
- [Huldra](https://data.equinor.com/dataset/Huldra)  
  An open sourced 3D model of the Huldra asset from Equinor

_Development_  
- [Docker](https://www.docker.com/)  
  > Docker is a set of platform as a service products that use OS-level virtualization to deliver software in packages called containers. The service has both free and premium tiers. The software that hosts the containers is called Docker Engine.
- [Python](https://www.python.org/)
- Shell scripting (bash)


## Components

- [Inspector](./inspector/)  
  TODO: what is it and what does it do?
- [Mission Planner](./mission_planner/)  
  TODO: what is it and what does it do?
- [Model Spawner](./model_spawner/)   
  TODO: what is it and what does it do?


## Security

The entrypoint script [run.sh](./run.sh) will handle most, if not all, setting and removal of permissions, see [How to run](#how-to-run).

### X11  

Docker require access to X11 on the host in order to display the `gazebo` and `rviz` desktop applications.  
This require the following permissions:
- Docker read and write to host directory `/tmp/.X11-unix`  
  - SET: Declare volume mount(s) with read-write access in [docker-compose.yml](./docker-compose.yml), then call `docker compose up`
  - REMOVE: `docker compose down`
- Docker write to host `xhost`  
  - SET: `xhost +localhost`
  - REMOVE: `xhost -localhost` 

### Containers have read-write access to host directories

The docker containers are used as development containers, meaning we mount in the repo directories in order to have live code running inside them.  
These mounts live as long as the containers are running.  

- SET: Declare volume mount(s) with read-write access in [docker-compose.yml](./docker-compose.yml), then call `docker compose up`
- REMOVE: `docker compose down`


## How to run

Requirements:
- Linux
- X11 - The docker containers will open `gazebo` and `rviz` as X11 desktop apps
- Docker

The main entrypoint to get everyhing up and running is the script [run.sh](./run.sh)  
It is a simple wrapper for calling `docker compose` and set permissions, then clean up when exiting.  


```sh
# HELP
# To view optional arguments
. ./run.sh --help

# START
# Call the script and let it run in your current shell
# If this is the first time then expect some build time
. ./run.sh

# EXIT
# Press CTRL+C and the script will clean up any lingering docker resources and bindings
```

If WSL cannot find Docker, make sure that Docker Desktop is running.

### Expected output

TODO: clean up, add missing outputs

- You should see desktop windows for `gazebo` and `rviz`
- `gazebo` should show some 3d models from Huldra
- Mission planner will start running calculations



## How to develop

When you start up the simulation as described in paragraph [How to run](#how-to-run) then everything is build and run automatically, which can take some time to complete.  
To get at faster feedback loop when developing you should enter the docker container running the code you are working on and restart the script directly via commands.  

As long as the files your are working on are _mounted_ into the container then your are good to go.


### Start

You will want to use 2-3 terminal sessions for this as it is easier to see what is going on where, and your favorite editor for working on the code.

#### _Terminal session 1: Build and start the ROS containers_

Follow the instructions in paragraph [How to run](#how-to-run)


#### _Terminal session 2: Run commands inside the ROS containers_

```sh
# Get an overview of all containers that are running
docker ps 

# Open a bash session into fex mission_planner
docker exec -it mission_planner bash

# From inside the container you can then run any unix or ROS command
# TODO: Add examples of viewing, stopping and starting ROS processes

# Now you can start changing code then restart the ROS code from inside the container to test it

```

### Stop

Follow the instructions in paragraph [How to run](#how-to-run)



## Known issues

- `gazebo_msgs.srv` and `gazebo_msgs.msg` Python packages are not recognized by VSCode in the devcontainer
- If the startup phase of the software takes too much time (e.g. the first time you run it or the first time after a system cleanup), the program might not run as expected. This happens because certain containers are dependent on each other, and when some containers use more time booting than usual, they might not start in the expected order.

### No support for MacOS

This project will unfortunately not run on MacOS primary because of a OpenGL version mismatch.  
The docker/ros/rviz/x11 combo depend on opengl to draw the desktop app and the recommended x11 server on mac, [XQuarts](https://www.xquartz.org/), use an older version of opengl.  
Hence we can see the windows start to get drawn (even get a logo) and then they crash.  
For more details see `rviz` git issue [Impossible to run Rviz2 from a Docker container on Apple Silicon](https://github.com/ros2/rviz/issues/929).
