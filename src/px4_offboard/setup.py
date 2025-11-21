import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # This line is essential for ROS 2 package discovery
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # This is installing your package.xml
        ('share/' + package_name, ['package.xml']),
        
        # Install all launch files from the 'launch' directory into 'share/<package_name>/launch/'
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        
        # Install all YAML files from the 'config' directory into 'share/<package_name>/config/'
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        
        # Install all RViz configuration files from the 'rviz' directory into 'share/<package_name>/rviz/'
        # This assumes you have moved visualize.rviz and visualize_multi.rviz into a 'rviz' folder
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Austin',
    maintainer_email='austinmcglashan1@gmail.com',
    description='PX4 offboard control and multi-drone visualization',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = px4_offboard.offboard_control:main',
                'visualizer = px4_offboard.visualizer:main',
                'velocity_control = px4_offboard.velocity_control:main',
                'control = px4_offboard.control:main',
                'processes = px4_offboard.processes:main',
                'swarm_spawner = px4_offboard.swarm_spawner:main',
                'circle_offboard_control = px4_offboard.circle_offboard_control:main',
                'square_offboard_control = px4_offboard.square_offboard_control:main',
                'waypoint_offboard_control = px4_offboard.waypoint_offboard_control:main',
                'multi_visualizer = px4_offboard.multi_visualizer:main',
                'orbit_offboard_control = px4_offboard.orbit_offboard_control:main',
                'trajectory_control = px4_offboard.trajectory_control:main',
                'visualizer_lidar = px4_offboard.visualizer_lidar:main',
                'visualizer_invariant_set = px4_offboard.visualizer_invariant_set:main',
        ],
    },
)