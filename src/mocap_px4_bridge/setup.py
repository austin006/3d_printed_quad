from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mocap_px4_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files from the 'launch' directory into 'share/<package_name>/launch/'
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='austin',
    maintainer_email='austin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_px4_bridge = mocap_px4_bridge.mocap_px4_bridge:main'
        ],
    },
)
