#!/bin/bash

# Launch script for the Drone Control Agent

echo "🚁 Drone Control Agent Launcher"
echo "==============================="

# Check if Ollama is running
if ! pgrep -x "ollama" > /dev/null; then
    echo "⚠️  Ollama is not running. Starting Ollama..."
    ollama serve &
    sleep 3
else
    echo "✅ Ollama is already running"
fi

# Check if the model is available
if ! ollama list | grep -q "llama3.2:3b"; then
    echo "⚠️  Model llama3.2:3b not found. Pulling model..."
    ollama pull llama3.2:3b
else
    echo "✅ Model llama3.2:3b is available"
fi

# Source ROS2 environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✅ ROS2 Jazzy environment sourced"
else
    echo "❌ ROS2 Jazzy not found. Please install ROS2 Jazzy."
    exit 1
fi

# Source local workspace if it exists
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Local workspace sourced"
fi

# Check if PX4 topics are available
echo "🔍 Checking for PX4 topics..."
if ros2 topic list | grep -q "/fmu/in/vehicle_command"; then
    echo "✅ PX4 topics detected"
else
    echo "⚠️  PX4 topics not found. Make sure PX4 SITL is running."
    echo "   Run: make px4_sitl gz_x500"
fi

# Launch the agent
echo ""
echo "🚀 Starting Drone Control Agent..."
echo "===================================="
python3 agent.py