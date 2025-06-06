FROM osrf/ros:kilted-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WORKSPACE=/workspace

# Install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    curl \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Control and MoveIt dependencies
RUN apt-get update && apt-get install -y \
    ros-kilted-ros2-control \
    ros-kilted-ros2-controllers \
    ros-kilted-hardware-interface \
    ros-kilted-controller-manager \
    ros-kilted-controller-interface \
    ros-kilted-moveit \
    ros-kilted-moveit-ros-planning-interface \
    ros-kilted-moveit-planners-ompl \
    ros-kilted-moveit-configs-utils \
    ros-kilted-moveit-setup-assistant \
    ros-kilted-joint-state-publisher \
    ros-kilted-joint-state-publisher-gui \
    ros-kilted-robot-state-publisher \
    ros-kilted-xacro \
    ros-kilted-rviz2 \
    ros-kilted-tf2-tools \
    ros-kilted-rqt \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo (GZ) packages compatible with ROS 2 Kilted
RUN apt-get update && apt-get install -y \
    ros-kilted-gz-sim-vendor \
    ros-kilted-gz-ros2-control \
    ros-kilted-gz-msgs-vendor \
    ros-kilted-gz-transport-vendor \
    ros-kilted-gz-math-vendor \
    ros-kilted-gz-common-vendor \
    ros-kilted-simulation-interfaces \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies and libserial for controllers
RUN apt-get update && apt-get install -y \
    python3-pygame \
    python3-serial \
    python3-numpy \
    libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 serial driver and dependencies
RUN apt-get update && apt-get install -y \
    ros-kilted-serial-driver \
    ros-kilted-asio-cmake-module \
    && rm -rf /var/lib/apt/lists/*

# Install missing packages for simulation
RUN apt-get update && apt-get install -y \
    ros-kilted-urdf-launch \
    ros-kilted-ros-gz-sim \
    ros-kilted-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR $ROS_WORKSPACE

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/kilted/setup.bash" >> ~/.bashrc
RUN echo "if [ -f $ROS_WORKSPACE/install/setup.bash ]; then source $ROS_WORKSPACE/install/setup.bash; fi" >> ~/.bashrc

# Set default command
CMD ["/bin/bash"]