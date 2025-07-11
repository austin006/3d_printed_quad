import os
from glob import glob
from setuptools import setup

package_name = 'ai_agent'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amcglash',
    maintainer_email='austinmcglashan1@gmail.com',
    description='AI agent and ROS2 PX4 quadrotor simulation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent = ai_agent.agent_wrapper:main',
        ],
    },
)