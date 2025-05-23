FROM osrf/ros:humble-desktop-full


RUN apt-get update && apt-get install -y \
    python3-pip \
    nano \
    ros-humble-rclpy \
    && rm -rf /var/lib/apt/lists/* \
    python3-tf-transformations 
    

RUN apt-get update && apt-get install -y \
    libgz-sim6-dev \
    ros-humble-ros-gz* 

RUN apt-get update && apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-msgs \
    python3-opencv 

    
RUN mkdir -p /AGV_ws/src
WORKDIR /AGV_ws

# RUN pip install ultralytics
#COPY ./src ./src

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m ${USERNAME} \
    && mkdir /home/${USERNAME}/.config && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config 

RUN . /opt/ros/humble/setup.sh && colcon build


# Fuente del setup de ROS
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Comando por defecto
CMD ["bash"]