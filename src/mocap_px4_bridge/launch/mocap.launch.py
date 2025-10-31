#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description(): 
    return LaunchDescription([
        # Launch VRPN MoCap client using ExecuteProcess
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'vrpn_mocap', 'client.launch.yaml',
                'server:=192.168.1.191',
                'port:=3883'
            ],
            output='screen'
        ),

        # Your existing nodes
        Node(
            package='mocap_px4_bridge',
            namespace='mocap_px4_bridge',
            executable='mocap_px4_bridge',
            name='mocap_px4_bridge'
        )
    ])