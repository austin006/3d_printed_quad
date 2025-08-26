# Docker Container
The `Dockerfile` and `devcontainer.json` will create the needed environment (as described [here](https://austin006.github.io/3d_printed_quad/software/ros2/#set-up)) in a Docker container using the VSCode extension called Dev Containers. 

## Set-up with Docker
Clone this github repo into your desired directory

`git clone https://github.com/austin006/3d_printed_quad.git`

Install the "Dev Containers" extension in VSCode and use the command palette (Ctrl+Shift+P) to select the option "Dev Containers: Rebuild and Reopen in Container". The container will take a long time to build the first time and it will have ROS2 Jazzy, PX4, QGroundControl, and MicroXRCE on a base image of Ubuntu 22.04.

Make sure you source, build, and source before running any launch files.
```
cd /root/ros2_ws
source opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Path Modifications

The original code assumes that QGroundControl and PX4-Autopilot are located in the home directory (~). The Docker container places these in a directory at `/root`. You will essentially need to replace `~/` with `/root/` for the file path locations of these applications.

### Gnome-Terminal Support

In addition, `gnome-terminal` is not configured in this Docker container. Instead I use a simpler alternative called `xterm`. By default the code uses `xterm` and so you will need to comment/uncomment some lines to switch to `gnome-terminal` if desired. The two node files to change are `processes.py` and `swarm_spawner.py` located at src/px4_offboard/px4_offboard and the launch files to change are `HITL_offboardvelocity_control.launch.py`, `offboard_velocity_control.launch.py`, `swarm_control.launch.py`, and `waypoint.launch.py` all located at src/px4_offboard/launch.

## ORC Set-up with Apptainer

ROS2 & PX4 Quadrotor Gazebo Simulation Environment. This tutorial explains how to use Apptainer in the BYU ORC server in order to set up a ROS2 & PX4 simulation environment for a quadrotor.

```
# Create a directory to host your workspace
mkdir ros2_workspace

# Copy the px4_offboard package to the workspace
cd ros2_workspace
mkdir src
# From local computer to supercomputer:
scp Documents/px4_offboard username@ssh.rc.byu.edu:/home/netid/ros2_workspace/src

# Pull Docker container using Apptainer
module load apptainer
apptainer pull ros2_px4_sim.sif docker://frostin/ros2-px4:latest

# On host - allow X11 access
xhost +local:

# Create a directory overlay
mkdir -p ./overlay_dir
    
# Run Apptainer with full GUI support, px4_offboard package mount, and directory overlay
apptainer shell \
    --cleanenv \
    --overlay ./overlay_dir \
    --bind /tmp/.X11-unix:/tmp/.X11-unix \
    --bind /dev/dri:/dev/dri \
    --bind ./src/px4_offboard:/root/ros2_ws/src/px4_offboard \
    --env DISPLAY=$DISPLAY \
    --env XDG_RUNTIME_DIR=/tmp \
    ros2_px4_sim.sif

# Source, build, source the workspace
cd /root/ros2_ws
source opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# Try one of the ROS2 launch files
ros2 launch px4_offboard offboard_velocity_control.launch.py
ros2 launch px4_offboard multi_waypoint_swarm.launch.py num_vehicles:=3
```

If that didn't work try testing everything separately, follow these steps:

```
# Launch 4 terminals
xterm & xterm & xterm & xterm & 

# Launch QGC
sudo -u user qgroundcontrol
# or this 
/root/QGroundControl/AppRun

# Launch PX4 with Gazebo
cd /root/PX4-Autopilot && make px4_sitl gz_x500

# Start Micro DDS Agent
MicroXRCEAgent udp4 -p 8888

# Test simple ROS2 Launch file
ros2 launch px4_offboard circle.launch.py

# Final step: In QGC switch the quadrotor to offboard mode and arm the quadrotor.
```

Refer to the `CL_Control.sh` file located at `src/px4_offboard/` for help with debugging by sending manual commands.

## Notes
- The `no_gui` folder is previous design of a simpler Docker container that doesn't support GUIs. 
- The ollama and LangGraph environment for the `ai_agent` package is not included in the container as of now.