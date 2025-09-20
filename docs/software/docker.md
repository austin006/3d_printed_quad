# Docker Container
The `Dockerfile` and `devcontainer.json` will create the needed environment (as described [here](https://austin006.github.io/3d_printed_quad/software/ros2/#set-up)) in a Docker container using the VSCode extension called Dev Containers. 

## Set-up with Docker
I don't think this will work with Docker Desktop becuase of the virtualization that it creates which prevents X11 forwarding for the GUIs. I couldn't get it to function. Instead I used [Docker engine](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) without Docker Desktop. 

Clone this github repo into your desired directory

```bash
git clone https://github.com/austin006/3d_printed_quad.git
```

Navigate to the `devcontainer.json` file. Make sure to build from the image, not a dockerfile. This is shown below:

```json
"name": "PX4 ROS2 Environment",
"image": "frostin/ros2-px4",
// "build": {
//     "dockerfile": "Dockerfile"
// },
```

Allow X11 access for any local user (run this on your host machine)

```bash
xhost +local:
```

Be sure to undo xhost permission to re-enable X11 security after finishing with the container

```bash
`xhost -`
```

Install the "Dev Containers" extension in VSCode and use the command palette (Ctrl+Shift+P) to select the option "Dev Containers: Reopen in Container". This may take a long time to open the container the first time. The container has ROS2 Jazzy, PX4, QGroundControl, and MicroXRCE on a base image of Ubuntu 22.04.

You can also pull the Docker image from Docker Hub beforehand and then opening the container. 

```bash
docker pull frostin/ros2-px4:latest
```

Once you are in the container, make sure you source, build, and source before running any launch files.

```bash
cd /root/ros2_ws
source opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Test out running a simulation with a launch file

```bash
ros2 launch px4_offboard offboard_velocity_control.launch.py
```

### QGC Path Modifications

The original code assumes that QGroundControl is located in the home directory (~). The Docker container places it in a directory at `/root`. You will essentially need to replace `~/` with `/root/` for the file path locations of these applications.

Alternatively you can simply find the terminal window where QGroundControl attempted to start from and run either of the following commands

```bash
# Launch QGC
sudo -u user qgroundcontrol
# or this 
/root/QGroundControl/AppRun
```

### Gnome-Terminal Support

In addition, `gnome-terminal` is not configured in this Docker container. Instead I use a simpler alternative called `xterm`. By default the code uses `xterm` and so you will need to comment/uncomment some lines to switch to `gnome-terminal` if desired. 

The two node files to change are located at `src/px4_offboard/px4_offboard`

- `processes.py` 
- `swarm_spawner.py`  

The launch files to change are all located at `src/px4_offboard/launch` 

- `HITL_offboardvelocity_control.launch.py`
- `offboard_velocity_control.launch.py`
- `swarm_control.launch.py`
- `waypoint.launch.py` 

## ORC Set-up with Apptainer

ROS2 & PX4 Quadrotor Gazebo Simulation Environment. This tutorial explains how to use Apptainer in the BYU ORC server in order to set up a ROS2 & PX4 simulation environment for a quadrotor.

### Complete Setup from scratch

```bash
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
mkdir -p ./overlay_dirAnother option is to

# Request an interactive compute node
salloc --time=1:00:00 -c 12 --mem=16G --gpus=1 --x11

# Source in order to load Apptainer
source /etc/profile
module load Apptainer

# Run Apptainer with full GUI support, px4_offboard package mount, and directory overlay
cd ~/ros2_workspace
apptainer shell \
  --cleanenv \
  --overlay ./overlay_dir \
  --bind "$XAUTHORITY:$XAUTHORITY" \
  --env DISPLAY="$DISPLAY" \
  --env XAUTHORITY="$XAUTHORITY" \
  ros2_px4_sim.sif

# Source, build, source the workspace
cd /root/ros2_ws
source opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# Try one of the ROS2 launch files
# 1 quadrotor, keyboard commands
ros2 launch px4_offboard offboard_velocity_control.launch.py
# 3 quadrotors, automatic flying to waypoints
ros2 launch px4_offboard multi_waypoint_swarm.launch.py num_vehicles:=3
# 5 quadrotors, automatic flying to waypoints
ros2 launch px4_offboard multi_waypoint_swarm.launch.py num_vehicles:=5

# Start QGroundControl. An xterm window will open up and try to start QGC but it is not saved in the expected location. Find the xterm window where QGC failed to launch and run the following command
/root/QGroundControl/Apprun
```

If the launch files didn't work try testing everything separately, follow these steps:

```bash
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

### Sequential Container Usage

If the Apptainer environment is already setup and ready to go, use the following instructions for a quicker launch.

```bash
# Request an interactive compute node
salloc --time=1:00:00 -c 12 --mem=16G --gpus=1 --x11

# Source in order to load Apptainer
source /etc/profile
module load Apptainer

# Run Apptainer with full GUI support, px4_offboard package mount, and directory overlay
cd ~/ros2_workspace
apptainer shell \
  --cleanenv \
  --overlay ./overlay_dir \
  --bind "$XAUTHORITY:$XAUTHORITY" \
  --env DISPLAY="$DISPLAY" \
  --env XAUTHORITY="$XAUTHORITY" \
  ros2_px4_sim.sif

# Source the workspace
cd /root/ros2_ws
source opt/ros/jazzy/setup.bash
source install/setup.bash

# Perform simulations. Use any node or launch file to run the ROS2-PX4 Gazebo simulation.
ros2 launch px4_offboard offboard_velocity_control.launch.py
ros2 launch px4_offboard multi_waypoint_swarm.launch.py num_vehicles:=3

# Don't forget to start QGroundControl separately 
/root/QGroundControl/Apprun
```

### VirtualGL 

Improve framerate by wrapping the simulation with the vglrun command to ensure it utilizes the GPU for rendering. The issue is this currently only works on the login nodes.

**Make sure you use the docker image with VirtualGL installed. This image is called `frostin/ros2-px4-vgl`**

```bash
# Launch the apptainer in a login node
apptainer shell \
    --cleanenv \
    --overlay ./overlay_dir \
    --bind /tmp/.X11-unix:/tmp/.X11-unix \
    --bind /dev/dri:/dev/dri \
    --bind ./src/px4_offboard:/root/ros2_ws/src/px4_offboard \
    --env DISPLAY=$DISPLAY \
    --env XDG_RUNTIME_DIR=/tmp \
    --nv \ 
    ros2_px4_vgl_sim.sif

# Source, build, source the workspace
cd /root/ros2_ws
source opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

# Wrap the ROS2 launch file with vglrun command
vglrun ros2 launch px4_offboard offboard_velocity_control.launch.py
```

The `--nv` flag automatically makes the host server's proprietary NVIDIA drivers and libraries available inside your container.

## Notes
- The ollama and LangGraph environment for the `ai_agent` package is not included in the container as of now.