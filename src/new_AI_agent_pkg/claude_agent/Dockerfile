# Multi-stage build for efficient drone control agent
FROM ros:jazzy-ros-base AS base

# Install Python dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /drone_agent

# Copy requirements first for better caching
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Copy agent files
COPY agent.py drone_tools.py system_prompt.txt config.yaml ./
COPY launch_agent.sh test_agent.py ./
RUN chmod +x launch_agent.sh

# Source ROS2 setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Default command
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && python3 agent.py"]

# For development with Ollama on host
# docker run -it --rm \
#   --network host \
#   --env ROS_DOMAIN_ID=0 \
#   drone-agent