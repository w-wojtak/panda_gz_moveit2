ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

### Use bash by default
SHELL ["/bin/bash", "-c"]

### Working directory
WORKDIR /root/ws

### Set up headless environment for WSL2
ENV DISPLAY=:99
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV ROS_LOCALHOST_ONLY=0

### Install system dependencies
RUN apt-get update && \
    apt-get install -y \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    wget \
    curl \
    xvfb \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

### Install ROS2 desktop packages
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

### Install MoveIt2 packages
RUN apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-planners \
    ros-${ROS_DISTRO}-moveit-simple-controller-manager \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-geometric-shapes \
    ros-${ROS_DISTRO}-pilz-industrial-motion-planner \
    && rm -rf /var/lib/apt/lists/*

### Create headless runner script
RUN echo '#!/bin/bash\n\
    if [ -z "$DISPLAY" ]; then\n\
    export DISPLAY=:99\n\
    Xvfb :99 -screen 0 1024x768x24 > /dev/null 2>&1 &\n\
    sleep 2\n\
    fi\n\
    exec "$@"' > /usr/local/bin/headless-run.sh && \
    chmod +x /usr/local/bin/headless-run.sh

### Initialize rosdep
RUN rosdep init || true && rosdep update

### Create workspace and clone Panda simulation packages
RUN mkdir -p src && \
    cd src && \
    git clone https://github.com/moveit/moveit2_tutorials.git -b ${ROS_DISTRO} && \
    git clone https://github.com/ros-planning/moveit_resources.git -b ${ROS_DISTRO}

### Install dependencies for cloned packages
RUN cd /root/ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} || true && \
    rm -rf /var/lib/apt/lists/*

### Build the workspace
RUN cd /root/ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --continue-on-error || true

### Set up environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /root/ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc && \
    echo "echo 'ROS2 Humble + Gazebo + MoveIt2 environment ready'" >> ~/.bashrc

CMD ["/bin/bash"]