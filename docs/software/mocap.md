# Quadrotor Flight with ROS2 and PX4 - Guide

## Set-up

Set correct IP address for the client if not automatically configured
```bash
uxrce_dds_client stop
param set UXRCE_DDS_AG_IP -1062731775
param save
uxrce_dds_client start
```

An alternative to setting the parameter is to run client with override flag
```bash
uxrce_dds_client start -h 192.168.0.1 -p 8888
```

Check connection from Jetson to Pixhawk
```bash
ping 192.168.0.3
```

Pull and run the simplest ROS Jazzy container for ARM64
```bash
docker run -it \
    --name ros2-jazzy-px4 \
    --network host \
    --privileged \
    -v /dev:/dev \
    -v $HOME/workspace:/workspace \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    --restart unless-stopped \
    ros:jazzy bash
```

## Starting a Session

### Login to Jetson

- ssh command: `ssh magicc@192.168.1.73` or `ssh magicc@ubuntu.local`
- username: `magicc`
- password: `magicc`

### Environment Set-up

Restart the existing container
```bash
docker start ros2-jazzy-px4
docker exec -it ros2-jazzy-px4 bash
```

Start the agent

```bash
MicroXRCEAgent udp4 -p 8888
```    
    
## MOCAP Set-up

Install vrpn-mocap package
```bash
sudo apt install ros-<rosdistro>-vrpn-mocap
```

Run the mocap node
```bash
ros2 run vrpn_client_ros vrpn_client_node 
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.1.3 port:=3883
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.1.191 port:=3883
```

Check mocap data being published
```bash
ros2 topic echo /vrpn_mocap/<asset_name>/pose
```

Check frequency of messages
```bash
ros2 topic hz /vrpn_mocap/<asset_name>/pose
```

### Resources
- [MAGICC Lab Tutorial](https://magicc.byu.edu/wiki/ros2_tutorials/mocap/mocap_tutorial/#data-collecting-with-ros)
- [Leon's Tutorial](https://www.notion.so/MOCAP-Room-24cdc719863380bbb0e2c94d5b7d8ec7)
- [VRPN Documentation](https://index.ros.org/r/vrpn_mocap/#jazzy)

## Helpful Commands to remember

Launch QGC
```bash
./Downloads/QGroundControl-x86_64.AppImage 
```

Clone `px4_msgs` package
```bash
git clone https://github.com/PX4/px4_msgs.git
```

Check the published mocap data
```bash
ros2 topic echo /vrpn_mocap/x650_quad/pose
ros2 topic hz /vrpn_mocap/x650_quad/pose
```

Check the position data on PX4 topic
```bash
ros2 topic echo /fmu/in/vehicle_visual_odometry
ros2 topic hz /fmu/in/vehicle_visual_odometry
```

Check the PX4 data received (in the MAVLink Console)
```bash
# See the recent data received on a PX4 topic
listener vehicle_visual_odometry
# Manage ekf2 
ekf2 stop
ekf2 start
ekf2 status
# Check PX4's internal time
uorb top -1 vehicle_visual_odometry
```

Rebuild the `mocap_px4_bridge` package
```bash
colcon build --packages-select mocap_px4_bridge
source install/setup.bash 
ros2 run mocap_px4_bridge mocap_px4_bridge
```