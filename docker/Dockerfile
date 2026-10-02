ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV TERM=xterm-256color

SHELL ["/bin/bash", "-c"]

# System / dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo gosu git curl wget gnupg ca-certificates cmake build-essential \
    python3-pip python3-colcon-common-extensions python3-rosdep \
    usbutils iputils-ping net-tools tmux vim nano less bash-completion \
    mesa-utils libgl1-mesa-dri \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# ROS packages used by src/ (package.xml + launch files)
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ros basic tools
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    # for web connection (e.g. foxglove)
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-foxglove-bridge \
    # for nav2 (+ build deps of the vendored nav2_smac_planner)
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-mppi-controller \
    ros-${ROS_DISTRO}-ompl \
    ros-${ROS_DISTRO}-angles \
    nlohmann-json3-dev \
    libeigen3-dev \
    # for localization
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-robot-localization \
    # for laser pipeline (ira_laser_tools / laser filters)
    ros-${ROS_DISTRO}-laser-filters \
    ros-${ROS_DISTRO}-laser-geometry \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    libpcl-dev \
    # driver pkg
    ros-${ROS_DISTRO}-phidgets-api \
    ros-${ROS_DISTRO}-libphidget22 \
    # for simulation
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-teleop-tools \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Non-root user. UID/GID should match the host user that owns the repo
# (docker_run.sh passes them in), otherwise build/ install/ log/ written from
# inside the container end up owned by the wrong user on the host.
ARG USERNAME=tdk
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN if getent group ${USER_GID} >/dev/null; then groupmod -n ${USERNAME} $(getent group ${USER_GID} | cut -d: -f1); \
    else groupadd --gid ${USER_GID} ${USERNAME}; fi \
    && if getent passwd ${USER_UID} >/dev/null; then userdel -r $(getent passwd ${USER_UID} | cut -d: -f1); fi \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG dialout,plugdev,video ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

ENV ROS_WS=/home/${USERNAME}/tdk_slam_ws

COPY bashrc /tmp/tdk_bashrc
RUN cat /tmp/tdk_bashrc >> /home/${USERNAME}/.bashrc \
    && rm /tmp/tdk_bashrc \
    && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
RUN rosdep update --rosdistro ${ROS_DISTRO}
WORKDIR ${ROS_WS}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
