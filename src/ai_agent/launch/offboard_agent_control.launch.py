#!/usr/bin/env python3

"""
Launch file to integrate the AI agent with existing PX4 simulation
This can be run alongside or instead of the velocity control example
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
        
        # Visualizer for RViz
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer'
        ),
        
        # Option 1: Run ONLY the AI agent (without keyboard control)
        # Comment out this section if you want to use keyboard control
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 
                'source /opt/ros/jazzy/setup.bash && '
                'source ~/ros2_workspaces/3d_printed_quad/install/setup.bash && '
                'ros2 run ai_agent agent; exec bash'],
            output='screen',
            name='ai_agent'
        ),
        
        # Option 2: Run BOTH keyboard control and AI agent
        # Uncomment these to have both control options available
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='control',
        #     name='control',
        #     prefix='gnome-terminal --',
        # ),
        # Node(
        #     package='px4_offboard',
        #     namespace='px4_offboard',
        #     executable='velocity_control',
        #     name='velocity'
        # ),
        
        # RViz2 for visualization
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        )
    ])
