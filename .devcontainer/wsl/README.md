# WSL 2 Native ROS 2 & PX4 Setup

If you are encountering network routing or DDS discovery issues utilizing `devcontainer.json` or Docker environments inside Windows Subsystem for Linux (WSL), utilizing a native ROS 2 setup configured with Mirrored Networking yields a highly stable alternative.

## 1. Environment & Dependency Installation

1. **Install ROS 2 Jazzy:** Follow the native installation steps inside your WSL Ubuntu instance via the [ROS 2 Jazzy Installation Guide](https://austin006.github.io/3d_printed_quad/software/ros2/#install-ros2-jazzy).
2. **Install Drone Workspace Repositories:** Clone and compile your workspace repository, the Micro-XRCE-DDS Agent, and the PX4-Autopilot stack directly into your user's home directory. Refer to the [Project Setup Instructions](https://austin006.github.io/3d_printed_quad/software/ros2/#set-up) for build specifics. Your home directory layout should reflect the following structural alignment:
```bash
~$ ls
3d_printed_quad  Micro-XRCE-DDS-Agent  PX4-Autopilot
```

3. **Install CycloneDDS:** Ensure the preferred RMW implementation package is installed in your environment:
```bash
sudo apt update && sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

---

## 2. Shell Configuration (`~/.bashrc`)

Append the following blocks to the absolute end of your `~/.bashrc` file. This forces the system to use CycloneDDS bound to local loopback and declares a robust wrapper function to cleanly strip benign CycloneDDS type-hash errors while forcing ROS 2 to bypass erratic background daemons.

```bash
# ROS2 Jazzy Core Network Configuration
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_DOMAIN_ID=0
export CYCLONEDDS_NETWORK_INTERFACE=lo

# Custom Wrapper: Forces live execution (--no-daemon) and filters metadata log spam
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

*Run `source ~/.bashrc` in active terminals to apply configuration changes.*

---

## 3. Enable Windows WSL Mirrored Networking

By default, WSL 2 operates on an isolated Network Address Translation (NAT) schema, creating a virtual firewall between your Windows apps and internal Linux ROS 2 utilities. Enabling Mirrored Mode strips this boundary away.

1. In Windows, navigate to your User Profile directory (Press `Win + R`, type `%USERPROFILE%`, and hit Enter).
2. Create or modify a text file named exactly `.wslconfig`.
3. Add the following global runtime parameter block:
```ini
[wsl2]
networkingMode=mirrored
```

4. Apply the structural change by terminating all running instances of WSL via an administrative Windows PowerShell/Command Prompt window:
```cmd
wsl --shutdown
```

---

## 4. QGroundControl (Windows Host) Integration

Because Mirrored Mode is enabled, **WSL and your Windows host share the exact same network space**. There is no need to hunt down fluctuating dynamic IP assignments using `ifconfig`.

1. Download and install the native Windows bundle via [QGroundControl.com](https://qgroundcontrol.com).
2. Launch **QGroundControl** on your Windows desktop.
3. Access configuration configurations: Click the **Q Icon** in the upper-left viewport -> Select **Application Settings** -> Navigate to **Comm Links**.
4. Select **Add New Link** and input the following configuration properties:
    * **Type:** `UDP`
    * **Port:** `18570`
    * **Server Address:** `127.0.0.1` (or `localhost`)

5. Click **Add Server**, select your newly established comm link, and click **Connect**.

---

## Technical Appendix & Diagnostic Notes

### Why Explicitly Bypassing the Daemon (`--no-daemon`) Matters

The integrated ROS 2 background daemon process assumes standard, static infrastructure typical to standalone hardware environments. Within a virtualized host container runtime context (even with Mirrored Networking), host-side Windows system behaviors—such as cycling wireless network adapters, initializing corporate VPN tunnels, or system power-state transitions—frequently drop active virtual interfaces.

When this happens, the background daemon stalls out or caches corrupt, stale discovery paths. Forcing `--no-daemon` strips away this background caching layer completely, ensuring ROS 2 directly samples the state of the active system interface during execution.

### Diagnostic Command Routines

#### Purging Ghost Root-Owned Daemons

If you ever execute a ROS 2 command using `sudo` or elevation privileges, a persistent, system root-owned daemon process can initialize. This blocks regular non-root CLI processes from binding to internal communication pipes, manifesting as random terminal timeouts. Clear them using the following routine:

```bash
# Force-terminate system-wide dangling ROS/daemon threads
sudo pkill -f ros2
sudo pkill -f daemon

# Clear out lingering root-owned IPC socket configurations
sudo rm -rf ~/.ros/ros2_daemon_*
```

#### Complete Virtual System Hard Reset

To force clean reload cycles on network interface alterations, drop the subsystem completely from a host shell terminal:

```cmd
wsl --shutdown
```