# Efficient LangGraph Drone Control Agent

An optimized AI agent for controlling quadcopters using natural language commands, built with LangGraph and ROS2 Jazzy. Designed to run on resource-constrained hardware like Raspberry Pi 4.

## Compatibility

- **ROS2 Distribution**: Jazzy
- **Target Hardware**: Raspberry Pi 4 (onboard computer)
- **Simulation**: PX4 SITL with Gazebo
- **DDS Bridge**: MicroXRCEAgent

## Installation

1. **Prerequisites**
   ```bash 
   # Complete set-up for px4_offboard package  
   
   # Install Ollama
   curl -fsSL https://ollama.com/install.sh | sh
   
   # Pull the model
   ollama pull llama3.2:3b
   ```

2. **Python Dependencies** - use a python virtual environment
   ```bash
   # Navigate to your AI_agent package
   cd ~/3d_printed_quad/src/AI_agent

   # Create a virtual environment
   python3 -m venv venv

   # Activate it
   source venv/bin/activate

   # Install dependencies
   pip install langgraph langgraph-prebuilt langchain-core langchain_ollama 
   ```

3. **Test the Agent**
   ```
   cd ~/ros2_workspaces/3d_printed_quad/src/ai_agent
   source venv/bin/activate
   source ~/ros2_workspaces/3d_printed_quad/install/setup.bash
   export PYTHONPATH=$PYTHONPATH:~/ros2_workspaces/3d_printed_quad/src/ai_agent
   python3 ai_agent/agent.py
   ```

## Usage with Simulation

### Option 1: Run AI Agent with Gazebo Simulation

1. **Start Ollama** (terminal 1)
   ```bash
   ollama serve
   ```

2. **Use Modified Launch File** (terminal 2)
   ```bash
   # Use the launch file
   ros2 launch ai_agent offboard_agent_control.launch.py
   ```

### Option 2: Run AI Agent Separately

1. **Start Ollama** (terminal 2)
   ```bash
   ollama serve
   ```

2. **Run the AI agent** (terminal 3)
   ```bash
   ros2 launch ai_agent test_agent.launch.py
   ```

### Option 3: Manual Setup

1. **Start MicroXRCEAgent** (terminal 1)
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

2. **Start PX4 SITL** (terminal 2)
   ```bash
   cd ~/PX4-Autopilot && make px4_sitl gz_x500
   ```

3. **Start Ollama**
   ```bash
   ollama serve
   ```

4. **Start QGroundControl** (terminal 3)
   ```
   cd ~/QGroundControl && ./QGroundControl.AppImage
   ```

4. **Launch the Agent** (terminal 4)
   ```bash
   python3 agent.py
   ```

## Example Commands

```
# Basic Operations
"Arm the drone"
"Take off to 5 meters"
"Land"

# Navigation
"Fly to position x=10, y=5, z=3"
"Go forward 5 meters"  
"Hold position"

# Status Queries
"What's the battery level?"
"Show me the drone status"
"What is the current position?"

# Complex Commands
"Arm the drone and take off to 10 meters"
"Fly in a square pattern at 5 meters altitude"
```

## Troubleshooting

1. **Ollama Connection Issues**
   - Ensure Ollama is running: `ollama serve`
   - Check URL in agent.py matches your setup
   - For Docker: use `http://host.docker.internal:11434`

2. **ROS2 Communication**
   - Check topics: `ros2 topic list`
   - Monitor commands: `ros2 topic echo /fmu/in/vehicle_command`
   - Verify QoS settings match PX4

3. **Slow Response Times**
   - Reduce `num_predict` in LLM config
   - Use smaller model if available
   - Check CPU/GPU utilization

## Future Improvements

   - Waypoint missions
   - Return-to-home functionality
   - Geofencing support
   - Multi-drone coordination