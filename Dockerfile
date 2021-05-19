FROM ros:melodic

WORKDIR /home/catkin_ws


RUN apt update && \
    apt install -y ros-melodic-tf2 \
    ros-melodic-hardware-interface \
    ros-melodic-joint-state-controller \
    ros-melodic-diff-drive-controller \
    ros-melodic-controller-manager \
    ros-melodic-realtime-tools

## Setup environment variables
RUN echo "source /opt/ros/melodic/setup.bash" >> /.bashrc

# Copy files inside docker image
COPY . ./src/hoverboard-driver

# Init workspace
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace /home/catkin_ws/src'

# Make package at start
ENTRYPOINT /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /home/catkin_ws; catkin_make; . /home/catkin_ws/devel/setup.bash; roslaunch hoverboard_driver hoverboard.launch'

