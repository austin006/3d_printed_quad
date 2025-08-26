FROM ubuntu:24.04

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    lsb-release \
    gnupg2 \
    software-properties-common \
    python3-pip \
    cmake \
    build-essential \
    sudo \
    # Minimal GUI support packages
    x11-apps \
    libgl1-mesa-dev \
    libgl1-mesa-dri \
    libxext6 \
    libx11-6 \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Jazzy
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y ros-jazzy-desktop \
    && rm -rf /var/lib/apt/lists/*

# Clone and install PX4 Autopilot
WORKDIR /root
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

# Build Micro DDS Agent
WORKDIR /root
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && make install \
    && ldconfig /usr/local/lib/

# Create workspace and clone repositories
WORKDIR /root/ros2_ws
RUN mkdir -p src \
    && cd src \
    && git clone https://github.com/PX4/px4_msgs.git -b release/1.15

# Copy PX4 ROS2 packages
RUN /root/PX4-Autopilot/Tools/copy_to_ros_ws.sh /root/ros2_ws

# Install colcon and dependencies
RUN apt-get update && \
    apt-get install -y python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*
    
# Source ROS2 and build workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Set up environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws
CMD ["/bin/bash"]