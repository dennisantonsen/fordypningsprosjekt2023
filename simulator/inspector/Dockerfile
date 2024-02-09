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
RUN pip install --upgrade pip

# Install extra tooling
WORKDIR /home/
RUN pip install markupsafe==2.0.1
# Install pcg-gazebo from source, https://github.com/boschresearch/pcg_gazebo#from-source
# The pip package will not work due to bad/missing depencencies
RUN apt-get install -y git libspatialindex-dev pybind11-dev libgeos-dev libfcl-dev liboctomap-dev
RUN git clone https://github.com/boschresearch/pcg_gazebo.git \
  && cd pcg_gazebo \
  && pip install .


# And finally the ROS components
RUN mkdir -p catkin_ws/src
COPY ./ catkin_ws/src/inspector
WORKDIR /home/catkin_ws/
RUN \
  rosdep init && \
  rosdep update && \
  rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

CMD /home/catkin_ws/src/inspector/entrypoint.sh