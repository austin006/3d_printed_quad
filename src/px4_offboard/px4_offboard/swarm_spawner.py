#!/usr/bin/env python3

import subprocess
import time
import argparse
import os

def main():
    """
    Spawns a specified number of PX4 SITL instances in Gazebo,
    along with the Micro XRCE-DDS agent and QGroundControl.
    """
    # --- Argument Parsing ---
    # Set up an argument parser to accept the number of vehicles
    parser = argparse.ArgumentParser(description="Spawn multiple PX4 SITL instances.")
    parser.add_argument('num_vehicles', type=int, help='The number of vehicles to spawn.')
    # Use parse_known_args() to ignore extra arguments passed by ros2 launch,
    # such as '--ros-args'. This prevents the script from crashing.
    args, unknown = parser.parse_known_args()
        
    num_vehicles = args.num_vehicles
    if num_vehicles <= 0:
        print("Error: Number of vehicles must be a positive integer.")
        return

    print(f"--- Spawning {num_vehicles} vehicles ---")

    # Use os.path.expanduser to handle the '~' correctly
    home_dir = os.path.expanduser('~')

    # --- Base Commands ---
    # These commands are run once, regardless of the number of vehicles.
    # The agent and QGC are started first.
    commands = [
        # Run the Micro XRCE-DDS Agent
        "MicroXRCEAgent udp4 -p 8888",
        # Run QGroundControl
        f"cd {home_dir}/QGroundControl && ./QGroundControl.AppImage"
    ]
    
    # --- Dynamic Vehicle Command Generation ---
    px4_base_path = f"{home_dir}/PX4-Autopilot"
    px4_executable_path = f"{px4_base_path}/build/px4_sitl_default/bin/px4"

    # Loop to generate the command for each vehicle
    for i in range(1, num_vehicles + 1):
        # MAV_SYS_ID for the instance. Instance 'i' gets MAV_SYS_ID 'i'.
        mav_sys_id = i
        
        # The first vehicle has a slightly different command
        if i == 1:
            vehicle_cmd = (
                f"cd {px4_base_path} && "
                f"PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 "
                f"./build/px4_sitl_default/bin/px4 -i {mav_sys_id}"
            )
        else:
            # Subsequent vehicles are spawned as standalone with a new position
            # Position is set along the y-axis to avoid collision
            y_pos = (i - 1) * 2.0 # Spread them out by 2 meters
            vehicle_cmd = (
                f"cd {px4_base_path} && "
                f"PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 "
                f"PX4_GZ_MODEL_POSE=\"0,{y_pos}\" PX4_SIM_MODEL=gz_x500 "
                f"./build/px4_sitl_default/bin/px4 -i {mav_sys_id}"
            )
        
        # Append each vehicle command to the end of the list.
        # This ensures the agent is started before the vehicles.
        commands.append(vehicle_cmd)

    # --- Execute All Commands ---
    # Loop through each generated command
    for command in commands:
        # If NOT using in Docker container, use gnome-terminal command 
        # Each command is run in a new tab of the gnome-terminal
        # subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
        
        # If using in Docker container, use xterm command below and comment out the gnome-terminal command(gnome-terminal GUI isn't set up in Dockerfile)
        subprocess.Popen(["xterm", "-e", "bash", "-c", command + "; exec bash"])

        # Pause to ensure processes start correctly
        time.sleep(1)

if __name__ == '__main__':
    main()