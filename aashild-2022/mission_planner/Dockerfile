FROM ros:noetic-ros-core
ENV PYTHONUNBUFFERED=1

WORKDIR /home/

# Update and install base tools
RUN \
  apt-get update && apt-get install -y \
  build-essential \
  python3-osrf-pycommon \
  python3-catkin-tools \
  python3-rosdep \
  ros-noetic-gazebo-ros-pkgs \
  git \
  libeigen3-dev \
  libgmp-dev \
  libgmpxx4ldbl \
  libmpfr-dev \
  libboost-dev \
  libboost-thread-dev \
  libtbb-dev \
  python3-dev \
  python3-pip

# Force all calls to python to be python3
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install PyMesh  
# The requirement.txt has a dependency conflict: 
#   ERROR: scipy 1.9.2 has requirement numpy<1.26.0,>=1.18.5, but you'll have numpy 1.17.4 which is incompatible.
# Possible workaround: run `pip3 install "numpy>=1.18.5"`  before "pip3 install..."


RUN mkdir -p catkin_ws
#   && cd catkin_ws
# RUN mkdir -p catkin_ws \
#   && cd catkin_ws \
#   && git clone https://github.com/PyMesh/PyMesh.git \
#   && cd PyMesh \
#   && git submodule update --init \
#   && pip3 install "numpy>=1.18.5" \
#   && pip3 install -r /home/catkin_ws/PyMesh/python/requirements.txt \
#   && python3 setup.py build \
#   && python3 setup.py install

WORKDIR /home/
COPY ./requirements.txt ./requirements.txt
RUN pip3 install -r requirements.txt

WORKDIR /home/
RUN pip install introcs
RUN pip install pandas
RUN pip install markupsafe==2.0.1
RUN pip install pcg-gazebo


WORKDIR /home/
RUN mkdir -p catkin_ws/src
COPY ./ catkin_ws/src/mission_planner

WORKDIR /home/catkin_ws/
RUN \
  rosdep init && \
  rosdep update && \
  rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y

CMD /home/catkin_ws/src/mission_planner/entrypoint.sh