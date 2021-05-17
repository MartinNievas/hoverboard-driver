FROM ros:melodic


RUN apt update && \
    apt install ros-melodic-tf2 -y

## Setup environment variables
RUN echo "source /opt/ros/melodic/setup.bash" >> /.bashrc



