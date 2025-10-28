from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    
    # # Command to run the Micro XRCE-DDS Agent in a new terminal
    # micro_xrce_agent = ExecuteProcess(
    #     cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
    #     prefix='xterm -e', #prefix='gnome-terminal --'
    #     output='screen'
    # )

    return LaunchDescription([
        # # Launch the Micro XRCE-DDS Agent
        # micro_xrce_agent,

        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        Node( 
            package='px4_offboard',
            namespace='px4_offboard',
            executable='control',
            name='control',
            prefix='xterm -e' #prefix='gnome-terminal --'
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='velocity_control',
            name='velocity'
        )
    ])