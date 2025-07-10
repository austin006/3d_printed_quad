# Multi-stage Dockerfile for PX4/ROS2/Gazebo/Ollama development environment
FROM ros:jazzy-ros-base AS base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC
ENV ROS_DISTRO=jazzy
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install Python 3.11+ and basic tools
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update && apt-get install -y \
    python3.11 \
    python3.11-dev \
    python3.11-venv \
    python3-pip \
    python3-yaml \
    curl \
    wget \
    git \
    sudo \
    lsb-release \
    gnupg \
    ca-certificates \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Make python3.11 the default python3
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1 \
    && update-alternatives --set python3 /usr/bin/python3.11

# Install PX4 build dependencies
RUN apt-get update && apt-get install -y \
    astyle \
    ccache \
    clang \
    clang-tidy \
    cppcheck \
    doxygen \
    file \
    g++ \
    gcc \
    gdb \
    git \
    lcov \
    libfuse2 \
    libxml2-dev \
    libxml2-utils \
    make \
    ninja-build \
    python3-pip \
    python3-pygments \
    python3-wheel \
    rsync \
    shellcheck \
    unzip \
    xsltproc \
    zip \
    && rm -rf /var/lib/apt/lists/*

# Install PX4 Python dependencies
RUN pip3 install --no-cache-dir \
    argcomplete \
    coverage \
    empy==3.3.4 \
    jinja2>=2.8 \
    kconfiglib \
    matplotlib \
    numpy \
    nunavut>=1.1.0 \
    packaging \
    pkgconfig \
    pyros-genmsg \
    pyulog>=0.5.0 \
    pyyaml \
    requests \
    serial \
    setuptools \
    six>=1.12.0 \
    toml \
    wheel>=0.31.1 \
    lxml

# Install Gazebo Harmonic
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Install QGroundControl dependencies and X11 apps
RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    x11-apps \
    xauth \
    && rm -rf /var/lib/apt/lists/*

# Install Micro-XRCE-DDS-Agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && cd /tmp/Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig /usr/local/lib/ \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

# Install Ollama
RUN curl -fsSL https://ollama.com/install.sh | sh

# Install additional ROS2 packages and tools
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-interfaces \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up workspace directory
RUN mkdir -p /workspace && chown -R $USERNAME:$USERNAME /workspace

# Switch to non-root user
USER $USERNAME
WORKDIR /workspace

# Set up ROS2 environment in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc \
    && echo "export GAZEBO_MODEL_PATH=/workspace/src/px4_offboard/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc \
    && echo "# Source workspace if it exists" >> ~/.bashrc \
    && echo "if [ -f /workspace/install/setup.bash ]; then" >> ~/.bashrc \
    && echo "    source /workspace/install/setup.bash" >> ~/.bashrc \
    && echo "fi" >> ~/.bashrc

# Create entrypoint script
USER root
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Start Ollama service in background\n\
ollama serve &\n\
OLLAMA_PID=$!\n\
\n\
# Wait for Ollama to be ready\n\
echo "Waiting for Ollama to start..."\n\
for i in {1..30}; do\n\
    if curl -s http://localhost:11434/api/tags >/dev/null 2>&1; then\n\
        echo "Ollama is ready"\n\
        break\n\
    fi\n\
    sleep 1\n\
done\n\
\n\
# Pull the model if not already present\n\
echo "Checking for llama3.2:3b model..."\n\
if ! ollama list | grep -q "llama3.2:3b"; then\n\
    echo "Pulling llama3.2:3b model..."\n\
    ollama pull llama3.2:3b\n\
fi\n\
\n\
# Source ROS2\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
\n\
# Build workspace if source files exist and build hasn'\''t been done\n\
if [ -d "/workspace/src" ] && [ ! -d "/workspace/install" ]; then\n\
    echo "Building ROS2 workspace..."\n\
    cd /workspace\n\
    # Install Python requirements if they exist\n\
    if [ -f "/workspace/requirements.txt" ]; then\n\
        echo "Installing Python requirements..."\n\
        pip3 install --user -r /workspace/requirements.txt\n\
    fi\n\
    # Initialize rosdep\n\
    sudo rosdep init || true\n\
    rosdep update\n\
    # Install dependencies\n\
    rosdep install --from-paths src --ignore-src -r -y\n\
    # Build the workspace\n\
    colcon build --symlink-install\n\
    source install/setup.bash\n\
fi\n\
\n\
# Execute the command passed to docker run\n\
exec "$@"\n\
' > /entrypoint.sh && chmod +x /entrypoint.sh

USER $USERNAME

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

# Expose ports
# 11434 - Ollama API
# 8888 - MicroDDS
# 14550 - MAVLink (QGroundControl)
# 14556 - MAVLink (Additional instances)
EXPOSE 11434 8888 14550 14556