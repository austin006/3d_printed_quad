import os
import json
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def load_waypoints_from_file(package_share_directory):
    """
    Loads waypoints from a YAML file.
    """
    waypoints_file_path = os.path.join(package_share_directory, 'config', 'magicc.yaml')
    
    if not os.path.exists(waypoints_file_path):
        raise FileNotFoundError(f"Waypoints file not found at: {waypoints_file_path}")

    with open(waypoints_file_path, 'r') as file:
        waypoints_data = yaml.safe_load(file)
    
    # Extract the list of waypoints for drones
    waypoints_raw = waypoints_data.get('drone_waypoints', [])
    
    # Convert each drone's waypoints list to a JSON string
    waypoints_for_drones_json = [json.dumps(wp_list) for wp_list in waypoints_raw]
    
    return waypoints_for_drones_json

def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('px4_offboard')

    try:
        # Load waypoints directly when launch_setup is called
        # This allows the launch file to find the config file relative to the package.
        waypoints_for_drones = load_waypoints_from_file(package_dir)
    except FileNotFoundError as e:
        return [LogInfo(msg=f"Error loading waypoints: {e}")]
    except Exception as e:
        return [LogInfo(msg=f"An unexpected error occurred while loading waypoints: {e}")]

    try:
        num_vehicles = int(context.perform_substitution(LaunchConfiguration('num_vehicles')))
        if num_vehicles <= 0:
            raise ValueError()
    except ValueError:
        return [LogInfo(msg="Error: 'num_vehicles' must be a positive integer.")]
    
    if num_vehicles > len(waypoints_for_drones):
        return [LogInfo(msg=f"Error: Not enough waypoint sets defined. Requested {num_vehicles} vehicles, but only {len(waypoints_for_drones)} waypoint sets are available.")]

    launch_actions = []

    swarm_spawner_node = Node(
        package='px4_offboard',
        executable='swarm_spawner',  
        name='swarm_spawner',
        output='screen',
        arguments=[LaunchConfiguration('num_vehicles')]
    )
    launch_actions.append(swarm_spawner_node)

    launch_actions.append(LogInfo(msg=f"--- LAUNCHING WAYPOINT CONTROL NODES FOR {num_vehicles} VEHICLE(S) ---"))

    for i in range(1, num_vehicles + 1):
        vehicle_id = str(i)
        current_waypoints_json = waypoints_for_drones[i-1] 

        waypoint_control_node = Node(
            package='px4_offboard',
            executable='waypoint_offboard_control',
            name=f'waypoint_offboard_control_{vehicle_id}',
            namespace=f'px4_{vehicle_id}',
            parameters=[
                {'vehicle_id': vehicle_id},
                {'waypoints_json': current_waypoints_json},
                {'acceptance_threshold': 0.5},
                {'loop_waypoints': True},
                {'takeoff_altitude': 5.0}
            ],
            output='screen'
        )
        
        launch_actions.append(waypoint_control_node)
        
    # # RViz Node - Pass num_vehicles parameter to the visualizer
    package_dir = get_package_share_directory('px4_offboard')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen', # Added to see RViz errors
        arguments=['-d', [os.path.join(package_dir, 'rviz', 'visualize_multi.rviz')]]
    )
    launch_actions.append(rviz_node)

    # Add the Visualizer node, passing the num_vehicles argument
    visualizer_node = Node(
        package='px4_offboard',
        executable='multi_visualizer',
        name='multi_visualizer',
        parameters=[{'num_vehicles': LaunchConfiguration('num_vehicles')}], # Pass num_vehicles
        output='screen'
    )
    launch_actions.append(visualizer_node)

    return launch_actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_vehicles',
            default_value='1', 
            description='The number of vehicles to spawn and control with waypoint missions.'
        ),
        OpaqueFunction(function=launch_setup)
    ])
