# Efficient LangGraph Drone Control Agent

An optimized AI agent for controlling quadcopters using natural language commands, built with LangGraph and ROS2 Jazzy. Designed to run on resource-constrained hardware like Raspberry Pi 4.

## Compatibility

- **ROS2 Distribution**: Jazzy
- **Target Hardware**: Raspberry Pi 4 (onboard computer)
- **Simulation**: PX4 SITL with Gazebo
- **DDS Bridge**: MicroXRCEAgent

## Key Design Decisions

### 1. **ROS2 Publishers vs Terminal Commands**
- **Choice**: ROS2 Publishers
- **Reason**: More efficient, better control, and proper integration with ROS2 ecosystem
- Terminal commands have overhead from process spawning and string parsing

### 2. **Simplified Agent Architecture**
- Removed unnecessary state updates between tool calls
- Minimal state tracking (only messages)
- Direct tool execution without intermediate processing
- Status monitoring through ROS2 subscriptions (asynchronous)

### 3. **Optimized LLM Configuration**
- Limited response length (`num_predict=100`)
- Temperature set to 0 for deterministic responses
- Concise system prompt focused on essential information
- Reduced tool count (only drone-specific tools)

### 4. **Efficient Tool Design**
- Tools return immediately after publishing commands
- No blocking waits for command completion
- Batch publishing for reliability (trajectory setpoints)
- Input validation to prevent errors

## Installation

1. **Prerequisites**
   ```bash
   # ROS2 Jazzy and PX4 dependencies
   sudo apt update
   sudo apt install ros-jazzy-desktop python3-colcon-common-extensions
   
   # Install Ollama
   curl -fsSL https://ollama.com/install.sh | sh
   
   # Pull the model
   ollama pull llama3.2:3b
   ```

2. **Python Dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Build PX4 and px4_msgs**
   ```bash
   # Clone and build PX4-Autopilot if not already done
   cd ~
   git clone https://github.com/PX4/PX4-Autopilot.git --recursive
   cd PX4-Autopilot
   make px4_sitl gz_x500
   
   # Build px4_msgs
   cd ~/your_ws/src
   git clone https://github.com/PX4/px4_msgs.git
   cd ..
   colcon build --packages-select px4_msgs
   source install/setup.bash
   ```

## Usage with Simulation

### Option 1: Run AI Agent with Existing Launch File

1. **Start Ollama** (terminal 1)
   ```bash
   ollama serve
   ```

2. **Use Modified Launch File** (terminal 2)
   ```bash
   # Copy the launch file to your workspace
   cp offboard_agent_control.launch.py ~/your_ws/src/px4_offboard/launch/
   
   # Run the integrated launch
   ros2 launch px4_offboard offboard_agent_control.launch.py
   ```

### Option 2: Run AI Agent Separately

1. **Start your existing simulation** (terminal 1)
   ```bash
   ros2 launch px4_offboard offboard_velocity_control.launch.py
   ```

2. **Start Ollama** (terminal 2)
   ```bash
   ollama serve
   ```

3. **Run the AI agent** (terminal 3)
   ```bash
   cd ~/drone_agent
   source /opt/ros/jazzy/setup.bash
   source ~/your_ws/install/setup.bash  # For px4_msgs
   python3 agent.py
   ```

### Option 3: Manual Setup

1. **Start MicroXRCEAgent** (terminal 1)
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

2. **Start PX4 SITL** (terminal 2)
   ```bash
   cd ~/PX4-Autopilot
   make px4_sitl gz_x500
   ```

3. **Start Ollama** (terminal 3)
   ```bash
   ollama serve
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

## Performance Optimizations

1. **Reduced Latency**
   - Direct ROS2 publishing (no subprocess overhead)
   - Minimal state management
   - Short LLM responses
   - No unnecessary tool calls

2. **Resource Efficiency**
   - Lightweight state tracking
   - Asynchronous status monitoring
   - No polling or busy-waiting
   - Efficient message serialization

3. **Reliability**
   - Multiple publishes for critical commands
   - Input validation in tools
   - Clear error messages
   - Status feedback integration

## Architecture Overview

The agent is designed to work alongside your existing PX4 simulation infrastructure:

```
User Input → LangGraph Agent → LLM → Tool Selection → ROS2 Publishers → MicroXRCEAgent → PX4
     ↑                                                                           ↓
     ←──────────── Status Feedback ←── ROS2 Subscribers ←──────────────────────
```

### Compatibility with Existing Simulation
- Works with your `offboard_velocity_control.launch.py` setup
- Compatible with MicroXRCEAgent for DDS bridging
- Can run alongside or replace keyboard control
- Uses same QoS profiles as your velocity_control.py

## Safety Features

- Altitude limits (0.5-50m)
- Position limits (±100m from home)
- Input validation on all commands
- Emergency stop function
- Clear status reporting

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

1. **Advanced Features**
   - Waypoint missions
   - Return-to-home functionality
   - Geofencing support
   - Multi-drone coordination

2. **Performance**
   - GPU acceleration for LLM
   - Response caching for common commands
   - Predictive command completion

3. **Safety**
   - Obstacle avoidance integration
   - Weather condition checks
   - Battery-based flight planning