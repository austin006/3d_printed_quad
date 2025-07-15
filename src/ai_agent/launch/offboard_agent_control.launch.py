#!/usr/bin/env python3

"""
Launch file to integrate the AI agent with existing PX4 simulation
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch the AI drone control agent alongside PX4 simulation
    """
    
    # Get package directory (adjust as needed for your setup)
    # package_dir = get_package_share_directory('px4_offboard')
    
    return LaunchDescription([
        # Start the existing simulation infrastructure
        # (MicroDDS, Gazebo, etc - handled by processes.py)
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        
        # Run the AI agent
        Node(
            package='ai_agent',
            namespace='ai_agent',
            executable='agent',
            name='agent',
            prefix='gnome-terminal --'
        )
    ])
