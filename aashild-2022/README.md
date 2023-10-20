# Inspection planner

This project is a part of my master thesis in Cybernetics and Robotics at NTNU.

The thesis problem was formulated together with NTNU and Equinor.

## Master thesis

### Task

We want to find inspection positions blablabla

### Master thesis report

A link might appear here in June.

## Technical documentation

For more details see documentation (README.md) inside each module/directory

### Dependencies

### config.env

### How to run it

`sudo ./run.sh`

### Known bugs

- `gazebo_msgs.srv` and `gazebo_msgs.msg` Python packages are not recognized by VSCode in the devcontainer
- If the startup phase of the software takes too much time (e.g. the first time you run it or the first time after a system cleanup), the program might not run as expected. This happens because certain containers are dependent on each other, and when some containers use more time booting than usual, they might not start in the expected order.