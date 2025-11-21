# Instructions to run a lidar demo with PX4 SITL, Gazebo, and ROS2 in Docker container.

# Start RViz2 with the lidar configuration
rviz2 -d src/myproject/px4_offboard/rviz/lidar.rviz

# Start the invariant set visualizer
ros2 run px4_offboard visualizer_invariant_set 

# Bridge Lidar data from Gazebo to ROS2
ros2 run ros_gz_bridge parameter_bridge /world/forest/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan --ros-args -r /world/forest/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan:=/lidar/scan

# Start Gazebo with the forest world and the x500 drone equipped with a 2D lidar
make px4_sitl gz_x500_lidar_2d PX4_GZ_WORLD=forest

# Start the Micro XRCE-DDS Agent for communication with PX4
MicroXRCEAgent udp4 -p 8888

# Start QGroundControl for drone monitoring and control
sudo -u user qgroundcontrol