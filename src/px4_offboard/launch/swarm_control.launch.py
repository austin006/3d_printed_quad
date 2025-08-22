import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    """
    This function is executed at launch time.
    It dynamically creates the actions needed to spawn and control the vehicles.
    """
    # Get the value of the 'num_vehicles' launch argument
    try:
        num_vehicles = int(context.perform_substitution(LaunchConfiguration('num_vehicles')))
        if num_vehicles <= 0:
            raise ValueError()
    except ValueError:
        return [LogInfo(msg="Error: 'num_vehicles' must be a positive integer.")]
    
    # This list will hold all the actions to be executed
    launch_actions = []

    # 1. Node to run the swarm_spawner script.
    #    The `num_vehicles` argument from the command line is passed to the script.
    swarm_spawner_node = Node(
        package='px4_offboard',
        executable='swarm_spawner',  # This name must match the entry point in setup.py
        name='swarm_spawner',
        output='screen',
        arguments=[LaunchConfiguration('num_vehicles')]
    )
    launch_actions.append(swarm_spawner_node)

    # 2. Loop to dynamically create a velocity_control and control node for each vehicle
    launch_actions.append(LogInfo(msg=f"--- LAUNCHING DYNAMIC NODES FOR {num_vehicles} VEHICLE(S) ---"))

    for i in range(1, num_vehicles + 1):
        vehicle_id = str(i)
        
        # Action to launch the velocity_control node for the current vehicle
        velocity_control_node = Node(
            package='px4_offboard',
            executable='velocity_control',
            name=f'velocity_control_{vehicle_id}',
            parameters=[{'vehicle_id': vehicle_id}]
        )
        
        # Action to launch the control node (teleop) for the current vehicle
        # This will open in its own terminal window.
        control_node = Node(
            package='px4_offboard',
            executable='control',
            name=f'control_{vehicle_id}',
            prefix='xterm -e', #prefix='gnome-terminal --'
            parameters=[{'vehicle_id': vehicle_id}]
        )
        
        launch_actions.append(velocity_control_node)
        launch_actions.append(control_node)

    return launch_actions

def generate_launch_description():
    """The main entrypoint for the launch file."""
    return LaunchDescription([
        # Declare an argument for the number of vehicles, with a default of 1
        DeclareLaunchArgument(
            'num_vehicles',
            default_value='1',
            description='The number of vehicles to spawn and control.'
        ),
        
        # Use OpaqueFunction to defer the node generation logic until launch time,
        # allowing it to access the value of the 'num_vehicles' argument.
        OpaqueFunction(function=launch_setup)
    ])
