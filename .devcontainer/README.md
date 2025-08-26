# Docker Container
The `Dockerfile` and `devcontainer.json` will create the needed environment (as described [here](https://austin006.github.io/3d_printed_quad/software/ros2/#set-up)) in a Docker container using the VSCode extension called Dev Containers. 

## Set-up
Clone this github repo into your desired directory

`git clone https://github.com/austin006/3d_printed_quad.git`

Install the "Dev Containers" extension in VSCode and use the command palette (Ctrl+Shift+P) to select the option "Dev Containers: Rebuild and Reopen in Container". The container will take a long time to build the first time and it will have ROS2 Jazzy, PX4, and MicroXRCE on a base image of Ubuntu 22.04. It does not install QGroundControl. You can launch that separately outside the container and with the port/x11 forwarding already enabled the PX4 instances should automatically connect.

In addition, `gnome-terminal` is not configured in this Docker container. Instead I use a simpler alternative called `xterm`. By default the code uses `gnome-terminal` and so you will need to comment/uncomment some lines to switch to `xterm`. The two node files to change are `processes.py` and `swarm_spawner.py` located at src/px4_offboard/px4_offboard and the launch files to change are `HITL_offboardvelocity_control.launch.py`, `offboard_velocity_control.launch.py`, `swarm_control.launch.py`, and `waypoint.launch.py` all located at src/px4_offboard/launch.

## Notes
- The `no_gui` folder is previous design of a simpler Docker container that doesn't support GUIs. 
- The ollama and LangGraph environment for the `ai_agent` package is not included in the container as of now.