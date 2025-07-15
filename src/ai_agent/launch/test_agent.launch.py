#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch the AI drone control agent
    """
    
    return LaunchDescription([
        # Run the AI agent
        Node(
            package='ai_agent',
            namespace='ai_agent',
            executable='agent',
            # name='agent',
            prefix='gnome-terminal --'
        )
    ])
