FROM ros:noetic-ros-core
WORKDIR /home/

RUN \
  apt-get update && \
  apt-get install -y \
  build-essential \
  python3-osrf-pycommon \
  python3-catkin-tools \
  python3-rosdep

RUN \
  apt-get update && \
  apt-get install -y \
  curl
RUN curl -sSL http://get.gazebosim.org | sh 

RUN \
  apt-get update && \
  apt-get install -y \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control

WORKDIR /home/
RUN mkdir -p catkin_ws/src
COPY ./ catkin_ws/src/gazebo

WORKDIR /home/catkin_ws/
RUN \
  rosdep init && \
  rosdep update && \
  rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

CMD /home/catkin_ws/src/gazebo/entrypoint.sh