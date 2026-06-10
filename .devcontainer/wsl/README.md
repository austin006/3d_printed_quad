# WSL Setup

The devcontainer.json and Dockerfile don't work. I couldn't figure out the network connections and everything required for WSL compatability.

However, I got it working through a native ROS2 install inside of WSL.

## Instructions

1. Install ROS2 Jazzy in WSL [(instructions)](https://austin006.github.io/3d_printed_quad/software/ros2/#install-ros2-jazzy)

2. Install the repo, MicroDDS, and PX4 in the home directory as shown below. Refer to the [setup instructions](https://austin006.github.io/3d_printed_quad/software/ros2/#set-up) for help.

    ``` bash
    ~$ ls
    3d_printed_quad  Micro-XRCE-DDS-Agent  PX4-Autopilot
    ```

3. Add the following to the end of your `~/.bashrc` file:

    ``` bash
    # ROS2 Jazzy bash configuration
    source /opt/ros/jazzy/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export ROS_DOMAIN_ID=0
    export CYCLONEDDS_NETWORK_INTERFACE=lo

    # Auto-append --no-daemon for absolute WSL stability and bypass broken sockets
    ros2() {
        if [[ "$1" == "node" && "$2" == "list" ]]; then
            command ros2 node list --no-daemon "${@:3}" 2> >(grep -v "Failed to parse type hash" >&2)
        elif [[ "$1" == "topic" && "$2" == "list" ]]; then
            command ros2 topic list --no-daemon "${@:3}" 2> >(grep -v "Failed to parse type hash" >&2)
        elif [[ "$1" == "topic" && "$2" == "hz" ]]; then
            command ros2 topic hz --no-daemon "${@:3}" 2> >(grep -v "Failed to parse type hash" >&2)
        elif [[ "$1" == "topic" && "$2" == "echo" ]]; then
            command ros2 topic echo --no-daemon "${@:3}" 2> >(grep -v "Failed to parse type hash" >&2)
        else
            command ros2 "$@" 2> >(grep -v "Failed to parse type hash" >&2)
        fi
    }
    ```

4. Enable mirrored network mode. In the Windows user home directory add a `.wslconfig` file with the following:

    ``` Ini, TOML
    [wsl2]
    networkingMode=mirrored
    ```

## Set Up QGroundControl (Windows)

1. Go to the official website: [https://qgroundcontrol.com](https://qgroundcontrol.com)
2. Download the latest version for **Windows** and launch the application

### Configure the UDP Communication Link

3. Click the **Q icon** in the top-left corner
4. Go to **Application Settings → Comm Links**
5. Click **Add New Link**
6. Set the **Type** to `UDP`
7. Set the **Port** to `18570`

### Get Your WSL IP Address

8. In your Ubuntu terminal, run:

```bash
ifconfig
```

Look for the `inet` address under your network adapter — for example:

```
inet 172.20.106.139
```

9. Copy that IP address, paste it into the **Server Address** field in QGC, and click **Add Server**

## Helpful commands

Cleanly Fix the ROS2 Daemon

``` bash
# Force kill the root-owned zombie processes
sudo pkill -f ros2
sudo pkill -f daemon

# Clean up any leftover root-owned socket files
sudo rm -rf ~/.ros/ros2_daemon_*
```

Shutdown wsl to restart it

``` DOS
wsl --shutdown
```

## Notes from GenAI

### Why Keeping `--no-daemon` is Best for WSL

Even with the permissions fixed, keeping the `--no-daemon` bypass in your `.bashrc` is highly recommended for WSL 2.

The ROS 2 daemon was designed for physical Linux machines where network interfaces are static. In WSL 2—even with Mirrored Networking—virtual network adapters frequently wake up, sleep, or shift states when Windows handles VPNs, Wi-Fi switches, or sleep mode.

The daemon easily gets "confused" by these Windows host network shifts and caches stale DDS discovery data. Forcing `--no-daemon` tells ROS 2 to query the network live every single time you type a command, making your development environment significantly more stable and robust.

### WSL Mirror Network Mode (The WSL Networking Fix)

By default, WSL 2 uses a NAT (Network Address Translation) architecture. This means WSL has a completely different IP address than your Windows host, and internal loopbacks (127.0.0.1) sometimes fail to cross the bridge between Windows processes, WSL UI processes, and the DDS multicast ports.

Windows 11 allows you to set WSL to Mirrored Mode, which forces WSL to share the exact same network interfaces and IP addresses as your Windows host.