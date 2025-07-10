# Simulation Integration Guide

This guide explains how the AI Drone Control Agent integrates with your existing PX4/Gazebo simulation setup.

## Architecture Overview

Your existing simulation setup:
```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────┐
│  MicroXRCEAgent │────▶│  PX4 SITL/Gazebo │────▶│    RViz2    │
└─────────────────┘     └──────────────────┘     └─────────────┘
         ▲                       ▲
         │                       │
         └───────┬───────────────┘
                 │
         ┌───────┴────────┐
         │ velocity_control│
         │   + control.py  │
         └────────────────┘
```

With AI Agent integration:
```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────┐
│  MicroXRCEAgent │────▶│  PX4 SITL/Gazebo │────▶│    RViz2    │
└─────────────────┘     └──────────────────┘     └─────────────┘
         ▲                       ▲
         │                       │
         ├───────┬───────────────┤
         │       │               │
    ┌────┴───┐   │      ┌────────┴────────┐
    │AI Agent│   │      │ velocity_control│
    │   +    │   │      │   + control.py  │
    │ Ollama │   │      └─────────────────┘
    └────────┘   │
                 │
          (Direct PX4 Commands)
```

## Key Integration Points

### 1. **Communication with PX4**
Both systems communicate with PX4 through MicroXRCEAgent:
- **Your velocity_control.py**: Uses `/offboard_velocity_cmd` → TrajectorySetpoint
- **AI Agent**: Direct publishing to PX4 topics:
  - `/fmu/in/vehicle_command` (arming, mode changes)
  - `/fmu/in/trajectory_setpoint` (position control)
  - `/fmu/in/offboard_control_mode` (when needed)

### 2. **QoS Profile Compatibility**
Both systems use identical QoS profiles:
```python
QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

### 3. **Non-Conflicting Operation**
The systems can coexist because:
- They publish to the same PX4 topics but at different times
- Only one control source should be active at a time
- PX4 handles command arbitration based on the most recent commands

## Usage Scenarios

### Scenario 1: AI Agent Only
Best for: Autonomous missions, natural language control

1. Launch simulation without keyboard control:
```bash
# Modify your launch file to exclude control.py and velocity_control.py
ros2 launch px4_offboard offboard_agent_control.launch.py
```

2. Control via natural language:
```
> arm the drone
> take off to 5 meters
> fly to position x=10, y=5, z=5
> land
```

### Scenario 2: Keyboard Control with AI Backup
Best for: Manual flying with AI assistance

1. Launch full simulation:
```bash
ros2 launch px4_offboard offboard_velocity_control.launch.py
```

2. In separate terminal, run AI agent:
```bash
python3 agent.py
```

3. Use keyboard for primary control, AI for complex commands

### Scenario 3: Development/Testing
Best for: Testing AI agent behavior

1. Run simulation components manually
2. Test specific AI agent functions
3. Monitor all topics:
```bash
ros2 topic echo /fmu/in/vehicle_command
ros2 topic echo /fmu/in/trajectory_setpoint
ros2 topic echo /fmu/out/vehicle_status
```

## Important Differences

### State Management
- **velocity_control.py**: Implements full state machine (IDLE → ARMING → TAKEOFF → LOITER → OFFBOARD)
- **AI Agent**: Simpler approach, relies on PX4's internal state management

### Control Methods
- **velocity_control.py**: Velocity-based control with world frame transformation
- **AI Agent**: Position-based control with direct trajectory setpoints

### Offboard Mode
- **velocity_control.py**: Manages offboard mode transitions automatically
- **AI Agent**: Switches to offboard mode only when needed (during takeoff/position commands)

## Troubleshooting

### Both Systems Running
If both control systems are active:
1. PX4 will accept commands from both
2. Last command wins
3. This can cause erratic behavior

**Solution**: Use only one control method at a time

### AI Agent Not Responding
Check:
1. MicroXRCEAgent is running
2. Ollama server is active
3. ROS_DOMAIN_ID matches (default: 0)
4. Topics are being published:
   ```bash
   ros2 topic list | grep fmu
   ```

### Mode Conflicts
If drone won't enter offboard mode:
1. Ensure drone is armed first
2. Check that trajectory setpoints are being published continuously
3. Verify no other node is commanding different modes

## Performance on Raspberry Pi 4

Optimizations for RPi4:
1. Reduce LLM token generation (`num_predict: 50`)
2. Disable status monitoring if not needed
3. Use minimal visualization (disable RViz if necessary)
4. Consider using smaller Ollama models

```yaml
# config.yaml for RPi4
llm:
  num_predict: 50
performance:
  enable_status_monitoring: false
  status_in_prompt: false
```

## Advanced Integration

### Custom Launch Configuration
Create a unified launch file that selects control method:
```python
# In launch file
DeclareLaunchArgument(
    'control_mode',
    default_value='ai',
    description='Control mode: ai, keyboard, or both'
)
```

### Topic Namespacing
To run multiple drones:
```bash
ROS_DOMAIN_ID=1 python3 agent.py --namespace drone1
ROS_DOMAIN_ID=2 python3 agent.py --namespace drone2
```

### Hybrid Control
Implement a supervisor node that switches between control methods based on conditions:
- Use AI for waypoint navigation
- Switch to manual for precise maneuvering
- Emergency override always available