#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Generates the LaunchDescription for the waypoint offboard control.
    Launches the visualizer, the waypoint offboard control node, and RViz2.
    """
    package_dir = get_package_share_directory('px4_offboard') 
    # Define your waypoints as a JSON string.
    # The coordinates are [x, y, z] in ENU (East, North, Up) frame relative to the takeoff point.
    # Example: A square at 2m altitude with 5m sides.
    # Start -> Up -> Point1 -> Point2 -> Point3 -> Point4
    example_waypoints_json = str([
        [0.0, 0.0, 0.0],  # Takeoff/Start Point
        [0.0, 0.0, 2.0],  # Ascend to 2m altitude
        [5.0, 0.0, 2.0],  # Move East 5m
        [5.0, 5.0, 2.0],  # Move North 5m
        [0.0, 5.0, 2.0],  # Move West 5m
        [0.0, 0.0, 2.0]   # Move South 5m (back to starting side)
    ])

    return LaunchDescription([
        # Node for the visualizer
        Node(
            package='px4_offboard', 
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer'
        ),
        # Node for the waypoint offboard control
        Node(
            package='px4_offboard', 
            namespace='px4_offboard',
            executable='waypoint_offboard_control',
            name='waypoint_offboard_control',
            parameters= [
                # Waypoint acceptance threshold (distance in meters)
                {'acceptance_threshold': 0.5},
                # Waypoints as a JSON string
                {'waypoints_json': example_waypoints_json},
                # Set to True for continuous looping, False for single pass
                {'loop_waypoints': True} 
            ]
        ),
        # Node to run the Micro XRCE-DDS Agent, the PX4 SITL simulation, QGroundControl
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            prefix='xterm -e' #prefix='gnome-terminal --'
        ),
        # Node for RViz2 visualization
        Node(
            package='rviz2',
            namespace='', 
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'rviz', 'visualize.rviz')]]
        )
    ])
