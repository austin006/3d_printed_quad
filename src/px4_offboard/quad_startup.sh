#!/bin/bash

# Start a GNOME Terminal window with multiple tabs.

# Define commands for each tab
declare -a commands=(
    "cd ~; ./Downloads/QGroundControl-x86_64.AppImage & disown"
    "ssh magicc@192.168.1.83"
    "source /opt/ros/jazzy/setup.bash; ros2 launch vrpn_mocap client.launch.yaml server:=192.168.1.202 port:=3883"
    "cd ~/px4_msgs_ws; source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 run mocap_px4_bridge mocap_px4_bridge"
    "cd ~/px4_msgs_ws; source /opt/ros/jazzy/setup.bash; source install/setup.bash; ros2 topic echo /fmu/in/vehicle_visual_odometry"
)

declare -a titles=(
    "QGroundControl"
    "SSH to 192.168.1.83"
    "VRPN Mocap Client"
    "mocap_px4_bridge"
    "VIO Topic Echo"
)

# Start the first tab (creates the window)
gnome-terminal --tab --title="${titles[0]}" -- bash -c "${commands[0]}; exec bash"

# Give it a moment to initialize the window
sleep 1

# Open remaining tabs in the same window
for i in {1..4}; do
    gnome-terminal --tab --title="${titles[$i]}" -- bash -c "${commands[$i]}; exec bash"
done
