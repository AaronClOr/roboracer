# 1. Start with the base image (Docker will auto-select amd64 or arm64)
FROM dustynv/ros:humble-desktop-l4t-r36.2.0

# 2. Set the workspace environment
ENV ROS_DISTRO=humble
ENV WORKSPACE=/opt/roboracer_ws
WORKDIR $WORKSPACE

# 3. Install system dependencies
#RUN apt-get update && apt-get install -y \
#    python3-pip \
#    ros-humble-teleop-twist-joy \
#    && rm -rf /var/lib/apt/lists/*

RUN apt-get update || true && \
    apt-get install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-teleop-twist-joy \
    ros-humble-ackermann-msgs \
    ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*
# 4. Copy the source code
# Ensure your local folder is named 'ros2_ws/src'
COPY ros2_ws/src ./src

# 4.5 Install ROS dependencies automatically
# This looks at your package.xml and installs missing libraries
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*

# 5. Build the workspace
# Using --symlink-install is great for development!
RUN . /opt/ros/humble/install/setup.sh && \
    colcon build --symlink-install

# 6. Automate environment sourcing
# This ensures 'ros2 run' works as soon as you open a shell
RUN echo "source /opt/ros/humble/install/setup.bash" >> ~/.bashrc && \
    echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc

# 7. Start your main racing node
CMD ["bash", "-c", "source /opt/ros/humble/install/setup.bash && source /opt/roboracer_ws/install/setup.bash && ros2 run my_racer racer_node"]