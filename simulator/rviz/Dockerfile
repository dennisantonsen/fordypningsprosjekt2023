FROM ros:noetic-ros-core
ENV PYTHONUNBUFFERED=1
WORKDIR /home/

RUN \
  apt-get update && \
  apt-get install -y \
  build-essential \
  python3-osrf-pycommon \
  python3-catkin-tools \
  python3-rosdep \
  ros-noetic-rviz

WORKDIR /home/
RUN mkdir -p catkin_ws/src
COPY ./ catkin_ws/src/rviz

WORKDIR /home/catkin_ws/
RUN \
  rosdep init && \
  rosdep update && \
  rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

CMD /home/catkin_ws/src/rviz/entrypoint.sh