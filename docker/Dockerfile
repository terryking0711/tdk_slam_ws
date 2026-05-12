FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV TERM=xterm-256color

ARG USERNAME=tdk
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo gosu git curl cmake python3-pip usbutils build-essential \
    python3-colcon-common-extensions \
    # ros basic tools
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-rviz2 \
    # for web connection (e.g. foxglove)
    ros-humble-rosbridge-server \
    ros-humble-foxglove-bridge \
    # for nav2
    ros-humble-nav2-msgs \
    ros-humble-nav2-util \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-mppi-controller \
    # for localization
    ros-humble-slam-toolbox \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-laser-filters \
    ros-humble-pcl-ros \
    # driver pkg
    ros-humble-phidgets-api \
    ros-humble-libphidget22 \
    # for simulation
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-teleop-tools \
    ros-humble-teleop-twist-keyboard \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && usermod -aG dialout $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME
WORKDIR /home/$USERNAME/tdk_slam_ws
ARG WS_PATH=/home/$USERNAME/tdk_slam_ws

COPY entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash", "-ic"]