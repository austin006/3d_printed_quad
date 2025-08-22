#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation
    "cd ~/PX4-Autopilot && make px4_sitl gz_x500",

    # Run QGroundControl
    "cd ~/QGroundControl && ./QGroundControl.AppImage"
]

# Loop through each command in the list
for command in commands:
    # If NOT using in Docker container, use gnome-terminal command 
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # If using in Docker container, use xterm command below and comment out the gnome-terminal command (gnome-terminal GUI isn't set up in Dockerfile)
    # subprocess.Popen(["xterm", "-e", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)
