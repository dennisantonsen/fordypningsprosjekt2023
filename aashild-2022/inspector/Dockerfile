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
  ros-noetic-gazebo-ros-pkgs \
  python3-pip
  
# Force all calls to python to be python3
RUN ln -s /usr/bin/python3 /usr/bin/python

WORKDIR /home/
RUN mkdir -p catkin_ws/src
COPY ./ catkin_ws/src/inspector

WORKDIR /home/
RUN pip install markupsafe==2.0.1
RUN pip install pcg-gazebo

WORKDIR /home/catkin_ws/
RUN \
  rosdep init && \
  rosdep update && \
  rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

CMD /home/catkin_ws/src/inspector/entrypoint.sh