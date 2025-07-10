from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ai_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config and prompt files
        (os.path.join('share', package_name, 'config'), 
            ['ai_agent/config.yaml', 'ai_agent/system_prompt.txt']),
        # Install launch files if you have any
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
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
            'agent = ai_agent.agent:main',
        ],
    },
)