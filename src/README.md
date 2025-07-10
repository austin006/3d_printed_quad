# Multi-Quadcopter Control with PX4 and ROS2 🚁

This directory is designed to be the **src** directory for a ROS2 workspace. Each subdirectory is a seperate package with different purposes. 

An overview of each package is provided as well as instructions for [**setting up your environment**](#set-up).

* [px4_offboard](#px4_offboard)
* [AI_agent](#ai_agent)
* [**set-up**](#set-up)

## px4_offboard

Multi-agent quadrotor simulation using ROS2, PX4, Gazebo, QGroundControl, MicroXRCEAgent, and RViz.

Much of the intial code is based off work from: 
- Jaeyoung Lim's Offboard example [https://github.com/Jaeyoung-Lim/px4-offboard](https://github.com/Jaeyoung-Lim/px4-offboard)
- ARK Electronics' example [https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example](https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example)

I've used their examples to expand functionalities such as multi-agent control. It may be helpful to start with their examples first.

### Explanation of launch files

| Launch File | Description |
| :--- | :--- |
| `circle.launch` | Launch a quadrotor to fly in a circle |
| `square.launch` | Launch a quadrotor to fly in a square |
| `waypoint.launch` | Launch a quadrotor to fly to a set of user-supplied waypoints |
| `HITL_offboard_velocity_control.launch` | Run on a real quadrotor for keyboard control |
| `multi_waypoint_swarm.launch` | Launch a swarm of quadrotors to follow supplied waypoints |
| `offboard_velocity_control.launch` | Launch a quadrotor controlled by keyboard inputs |
| `swarm_control.launch` | Launch a swarm of quadrotors to be controlled by keyboard inputs |

### Run a launch file

Make sure you have built the ROS2 workspace and sourced the environment

```
colcon build
source install/setup.bash
```

Then you can start the launch file of your choice (assuming have already completed the set-up for PX4, Gazebo, MicroXRCEAgent, and the ROS2 workspace)

```
ros2 launch px4_offboard <package_name>
```

Several launch files support multiple vehicles with a launch configuration parameter called 'num_vehicles'

```
ros2 launch px4_offboard multi_waypoint_swarm.launch.py num_vehicles:=3
```

### Notes on `circle.launch.py` and `square.launch.py` launch files

These launch files do not call on the processes.py node. Therefore you need to start the different processes manually in different terminals.

**Terminal 1**
```
ros2 launch px4_offboard circle.launch.py
```
**Terminal 2**
```
MicroXRCEAgent udp4 -p 8888
```
**Terminal 3**
```
cd ~/PX4-Autopilot && make px4_sitl gz_x500
```
**Terminal 4**
```
cd ~/QGroundControl && ./QGroundControl.AppImage
```

After everything has started, arm the drone in QGroundControl and switch it into offboard mode.

## AI_agent

px4_offboard simulation expanded to interface with an AI Agent allowing for natural language interaction. Implented with LangChain, LangGraph, Ollama, 

For now, see the seperate README.md file in the package directory for more information.

## Set-up

### Install PX4 Autopilot
To [Install PX4](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets) run this code 
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.15
```

Run this script in a bash shell to install everything

```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

You will now need to restart your computer before continuing

### Install ROS2 Jazzy
To install ROS2 Jazzy follow the steps [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

### Install Dependencies

Install Python dependencies as mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2) with this code

```
pip3 install --user -U empy pyros-genmsg setuptools
```

### Build Micro DDS
As mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#setup-micro-xrce-dds-agent-client) run this code in order to build MicroDDS on your machine

```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Clone in Repo and Packages
Run this code to clone the repo

```
git clone https://github.com/austin006/3d_printed_quad.git
```

We need the px4_msgs package. Our ROS2 nodes rely on the message definitions in this package in order to communicate with PX4. Read [here](https://docs.px4.io/main/en/ros/ros2_comm.html#overview:~:text=ROS%202%20applications,different%20PX4%20releases) for more information.

Be sure you're in the src directory of the workspace and run this code to clone in the px4_msgs repo

```
cd src
git clone https://github.com/PX4/px4_msgs.git -b release/1.15
```

For some reason I couldn't get this to work in jazzy without also including [px4_msgs_old](https://docs.px4.io/main/en/middleware/uorb.html#message-versioning) and the [translation_node](https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node.html) packages. To copy these packages, make sure you are in your workspace directory and have installed PX4-Autopilot in your root directory. Run the following code

```
~/PX4-Autopilot/Tools/copy_to_ros_ws.sh .
```

Clone in all required packages before building the workspace

### Build the Workspace

Source the ROS2 installation, build the workspace within the workspace directory (not in src), and source the workspace

```
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Your workspace directory should now look similar to the following

```
.
├── .github
├── build
├── docs
├── install
├── log
├── src
│   ├── other_ros_packages
│   ├── px4_offboard
│   ├── px4_msgs
│   ├── px4_msgs_old
│   ├── translation_node
├── .gitignore
├── mkdocs.yml
├── README.md
```

### Running the code

Follow the instructions for each package to run the code