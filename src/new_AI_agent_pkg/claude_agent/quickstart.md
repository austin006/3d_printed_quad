# Quick Start Guide

## 🚀 5-Minute Setup

### Prerequisites
- Ubuntu 24.04 with ROS2 Jazzy
- Python 3.11+
- Docker (optional)
- Existing PX4 simulation setup

### Option 1: Integration with Existing Simulation

```bash
# 1. Install Ollama
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.2:3b

# 2. Clone agent to home directory
cd ~
git clone <your-repo> drone_agent
cd drone_agent
pip3 install -r requirements.txt

# 3. Start Ollama server
ollama serve

# 4. In another terminal, run your simulation
ros2 launch px4_offboard offboard_velocity_control.launch.py

# 5. In a third terminal, run the AI agent
cd ~/drone_agent
source /opt/ros/jazzy/setup.bash
python3 agent.py
```

### Option 2: Standalone with Manual Setup

```bash
# Terminal 1: MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888

# Terminal 2: PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 3: Ollama
ollama serve

# Terminal 4: AI Agent
cd ~/drone_agent
source /opt/ros/jazzy/setup.bash
python3 agent.py
```

## 🎮 Basic Commands

Once running, try these commands:

```
> arm the drone
> take off to 5 meters
> fly to x=10, y=5, z=5
> what's the battery level?
> land
```

## ⚙️ Configuration

Edit `config.yaml` to tune performance:

```yaml
llm:
  num_predict: 50  # Reduce for faster responses
  
safety:
  max_altitude: 20.0  # Lower limits for indoor testing
```

## 🧪 Testing

Run automated tests:
```bash
make test
```

Run unit tests:
```bash
python3 test_tools.py
```

## 📊 Performance Monitoring

Check response times:
```bash
make benchmark
```

Monitor ROS2 topics:
```bash
ros2 topic echo /fmu/out/vehicle_status
```

## 🐛 Troubleshooting

1. **"Ollama not found"**
   ```bash
   ollama serve  # Start Ollama server
   ```

2. **"No PX4 topics"**
   - Ensure PX4 SITL is running
   - Check `ROS_DOMAIN_ID` matches

3. **Slow responses**
   - Reduce `num_predict` in config.yaml
   - Disable status monitoring

## 📚 Next Steps

- Read [OPTIMIZATIONS.md](OPTIMIZATIONS.md) for performance tuning
- Check [README.md](README.md) for detailed documentation
- Explore advanced features in the config file

## 💡 Tips

- Keep commands simple and direct
- Always arm before takeoff
- Use "emergency stop" if needed (drone will fall!)
- Monitor battery levels during flight

Happy flying! 🚁