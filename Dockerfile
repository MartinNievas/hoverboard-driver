FROM ros:melodic

WORKDIR /home/catkin_ws


RUN apt update && \
    apt install ros-melodic-tf2 -y &&\
    apt install ros-melodic-hardware-interface -y &&\
    apt install ros-melodic-controller-manager -y &&\
    apt install ros-melodic-realtime-tools -y

## Setup environment variables
RUN echo "source /opt/ros/melodic/setup.bash" >> /.bashrc

# Copy files inside docker image
# COPY . ./src/hoverboard-driver

# Init workspace
# RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace /home/catkin_ws/src'

# Make package
# RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /home/catkin_ws; catkin_make'

