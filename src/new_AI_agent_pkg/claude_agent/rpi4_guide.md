# Raspberry Pi 4 Deployment Guide

This guide covers deploying the AI Drone Control Agent on a Raspberry Pi 4 as the onboard computer for your quadcopter.

## Hardware Requirements

- Raspberry Pi 4 (4GB or 8GB recommended)
- MicroSD card (32GB minimum, Class 10)
- Adequate cooling (heatsinks/fan)
- Reliable power supply (5V 3A minimum)

## OS Setup

### 1. Install Ubuntu Server 24.04 for RPi

```bash
# Download Ubuntu Server 24.04 LTS for ARM64
# Flash to SD card using Raspberry Pi Imager or dd
```

### 2. Initial Configuration

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential packages
sudo apt install -y python3-pip git curl build-essential

# Set up swap (important for 4GB models)
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## ROS2 Jazzy Installation

```bash
# Add ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy (base, not desktop)
sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools

# Source ROS2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Ollama Installation (ARM64)

```bash
# Install Ollama for ARM64
curl -fsSL https://ollama.com/install.sh | sh

# Pull smaller model optimized for RPi4
ollama pull tinyllama  # 1.1B parameters, faster than llama3.2:3b

# Or if you have 8GB RAM, you can try:
# ollama pull llama3.2:3b
```

## Agent Installation

```bash
# Clone the agent
cd ~
git clone <your-repo> drone_agent
cd drone_agent

# Install Python dependencies
pip3 install --no-cache-dir -r requirements.txt

# Create optimized config for RPi4
cat > config_rpi4.yaml << EOF
llm:
  model: "tinyllama"  # Smaller model for RPi4
  base_url: "http://localhost:11434"
  temperature: 0.0
  num_predict: 30  # Very short responses
  
safety:
  max_altitude: 20.0  # Lower limits for safety
  min_altitude: 0.5
  max_distance: 50.0
  
publishing:
  trajectory_repeat: 5  # Fewer repeats to save CPU
  position_repeat: 3
  publish_delay: 0.1
  
performance:
  enable_status_monitoring: false  # Disable to save resources
  status_in_prompt: false
  max_response_wait: 15.0  # Longer timeout for slower inference
EOF
```

## Performance Optimizations

### 1. CPU Governor
```bash
# Set to performance mode
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Make persistent
sudo apt install cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
```

### 2. Reduce Background Services
```bash
# Disable unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable avahi-daemon
sudo systemctl disable snapd
```

### 3. Ollama Optimization
```bash
# Limit Ollama memory usage
export OLLAMA_MAX_LOADED_MODELS=1
export OLLAMA_NUM_PARALLEL=1
echo "export OLLAMA_MAX_LOADED_MODELS=1" >> ~/.bashrc
echo "export OLLAMA_NUM_PARALLEL=1" >> ~/.bashrc
```

## Running on Hardware

### 1. Serial Connection to Flight Controller
```bash
# Install serial communication package
sudo apt install -y python3-serial

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in for group change to take effect

# Configure MicroXRCEAgent for serial
# Assuming /dev/ttyACM0 for USB or /dev/ttyAMA0 for GPIO serial
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600
```

### 2. Systemd Service Setup
```bash
# Create service for Ollama
sudo tee /etc/systemd/system/ollama.service << EOF
[Unit]
Description=Ollama Service
After=network.target

[Service]
Type=simple
User=$USER
ExecStart=/usr/local/bin/ollama serve
Restart=always

[Install]
WantedBy=multi-user.target
EOF

# Create service for drone agent
sudo tee /etc/systemd/system/drone-agent.service << EOF
[Unit]
Description=Drone AI Agent
After=ollama.service
Requires=ollama.service

[Service]
Type=simple
User=$USER
WorkingDirectory=/home/$USER/drone_agent
ExecStart=/usr/bin/python3 /home/$USER/drone_agent/agent.py
Restart=always
Environment="ROS_DOMAIN_ID=0"

[Install]
WantedBy=multi-user.target
EOF

# Enable services
sudo systemctl enable ollama.service
sudo systemctl enable drone-agent.service
sudo systemctl start ollama.service
sudo systemctl start drone-agent.service
```

## Monitoring and Debugging

### Check System Resources
```bash
# Monitor CPU and memory
htop

# Check service status
sudo systemctl status drone-agent
sudo journalctl -u drone-agent -f

# Monitor ROS2 topics
ros2 topic list
ros2 topic hz /fmu/out/vehicle_status
```

### Common Issues

1. **Out of Memory**
   - Use tinyllama instead of llama3.2
   - Increase swap size
   - Disable status monitoring

2. **Slow Response Times**
   - Reduce `num_predict` to 20 or less
   - Disable unnecessary ROS2 nodes
   - Use performance CPU governor

3. **Serial Communication Issues**
   - Check device permissions
   - Verify baud rate matches FC settings
   - Test with `screen /dev/ttyACM0 921600`

## Safety Considerations

1. **Failsafe Configuration**
   - Set up RC override capability
   - Configure geofence in PX4
   - Test emergency stop thoroughly

2. **Temperature Monitoring**
   ```bash
   # Add to agent startup script
   vcgencmd measure_temp
   ```

3. **Watchdog Timer**
   ```bash
   # Enable hardware watchdog
   sudo apt install watchdog
   sudo systemctl enable watchdog
   ```

## Benchmarks on RPi4

Typical performance with optimizations:
- Command processing: ~2.5s (tinyllama)
- Tool execution: ~0.3s
- Memory usage: ~1.2GB
- CPU usage: 60-80% during inference

## Next Steps

1. Test thoroughly in simulation first
2. Implement companion computer link
3. Add telemetry logging
4. Consider edge TPU for acceleration