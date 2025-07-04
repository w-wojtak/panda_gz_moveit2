ARG ROS_DISTRO=galactic
FROM ros:${ROS_DISTRO}-ros-base

### Use bash by default
SHELL ["/bin/bash", "-c"]

### Define working directory
ARG WS_DIR=/root/ws
ENV WS_DIR=${WS_DIR}
ENV WS_SRC_DIR=${WS_DIR}/src
ENV WS_INSTALL_DIR=${WS_DIR}/install
ENV WS_LOG_DIR=${WS_DIR}/log
WORKDIR ${WS_DIR}

### Set up headless environment
ENV DISPLAY=:99
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV ROS_LOCALHOST_ONLY=0

### Install essential dependencies first
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    xvfb \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

### Install Ignition Edifice (not Fortress!)
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    ignition-edifice \
    && rm -rf /var/lib/apt/lists/*

### Install core ROS2 packages that we know work
RUN apt-get update && \
    apt-get install -yq --no-install-recommends \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-image-transport \
    && rm -rf /var/lib/apt/lists/*

### Initialize rosdep
RUN rosdep init || true && rosdep update

### Import and install dependencies
COPY ./panda_ign_moveit2.repos ${WS_SRC_DIR}/panda_ign_moveit2/panda_ign_moveit2.repos
RUN vcs import --shallow ${WS_SRC_DIR} < ${WS_SRC_DIR}/panda_ign_moveit2/panda_ign_moveit2.repos && \
    apt-get update && \
    rosdep install -y -r -i --rosdistro "${ROS_DISTRO}" --from-paths ${WS_SRC_DIR} --skip-keys "ignition-gazebo3 ignition-gazebo6" && \
    rm -rf /var/lib/apt/lists/*

### Build the workspace (skip problematic packages)
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release" \
    --packages-ignore ign_ros2_control gripper_controllers ros_ign_gazebo ros_ign_bridge ros_ign_image \
    --continue-on-error || true && \
    rm -rf ${WS_LOG_DIR}

### Copy your main package and build it
COPY ./ ${WS_SRC_DIR}/panda_ign_moveit2/
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    source "${WS_INSTALL_DIR}/setup.bash" && \
    colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release" \
    --packages-ignore ign_ros2_control gripper_controllers ros_ign_gazebo ros_ign_bridge ros_ign_image \
    --continue-on-error || true && \
    rm -rf ${WS_LOG_DIR}

### Set up environment
RUN sed -i '$i source "${WS_INSTALL_DIR}/local_setup.bash" --' /ros_entrypoint.sh && \
    sed -i '$a source "/opt/ros/${ROS_DISTRO}/setup.bash"' ~/.bashrc && \
    sed -i '$a source "${WS_INSTALL_DIR}/setup.bash"' ~/.bashrc

CMD ["/bin/bash"]