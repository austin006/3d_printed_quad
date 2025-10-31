# Jetson Nano and Quadrotor Integration

The following documentation gives instructions on how to set up a new Jetson Nano to control a quadrotor with PX4 autopilot. 

## Set-up

Follow the detailed instructions provided by PX4: [**Holybro Pixhawk Jetson Baseboard**](https://docs.px4.io/main/en/companion_computer/holybro_pixhawk_jetson_baseboard)

### Pixhawk<->Jetson Connection

The Jetson and Pixhawk are connected using an internal ethernet cable.

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

- ssh command: `ssh magicc@192.168.1.83` or `ssh magicc@ubuntu.local`
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
    