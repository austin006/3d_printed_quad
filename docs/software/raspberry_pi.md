# Raspberry Pi and Quadrotor Integration 

The following documentation gives a detailed walk through on how to set up a new Raspberry Pi to control a quadrotor with PX4 autopilot. 

## Important Information 

- username: magicc 
- password: magicc 
- hostname: raspberrypi.local 
- ssh command with custom username:
```
ssh magicc@<IP address>
```
- wifi: MAGICC
- Dynamic IP address: 192.168.1.12 
```
ssh magicc@192.168.1.12 
```

## Set Up the Raspberry Pi  

**Starting requirements:** A new Raspberry Pi Compute Module, an IO Board, a small jumper, a micro-USB cable, and a 12V power cable.  

**End objective:** Raspberry Pi running ROS2 on Ubuntu. 

If you get stuck on steps 1 or 2 refer to this tutorial for help: [:simple-youtube: How to flash Raspberry Pi OS onto the Compute Module 4 eMMC with usbboot | Jeff Geerling](https://www.jeffgeerling.com/blog/2020/how-flash-raspberry-pi-os-compute-module-4-emmc-usbboot) 

### 1. Prepare to flash OS onto Raspberry Pi

The Raspberry Pi Compute Module 4 used in these instructions has a built in eMMC instead of an SD memory card slot. If your Raspberry Pi requires an SD card, you will download the OS directly onto that and skip to **step 3**. Otherwise follow these steps to flash an OS onto the eMMC: 

- Put the small jumper on the IO board. Looking straight down onto the board, it goes in a space at the top left. The location of the two pins is labelled with **Fit jumper to disable eMMC Boot**. This will disable the eMMC boot and instead allow you to flash an OS onto it. 
- Plug a USB cable from your computer into the IO board at the space labeled **USB Slave** and connect the board to power. You should see the **D1** red light turn on. 
- Download [usbboot](https://github.com/raspberrypi/usbboot) 
- With the board still powered on and connected to your computer, run the **rpiboot** executable found in the directory of usbboot. 

!!! Success
    If everything worked correctly, the eMMC storage now behaves just like a microSD card or USB drive that you plugged into your computer. 

### 2. Flash OS onto Raspberry Pi

We will use an application called Raspberry Pi imager to flash an OS to the eMMC. 

- Download and open [Raspberry Pi Imager](https://github.com/raspberrypi/rpi-imager)
- Click **Choose OS** > scroll down > select: 
    - Other general-purpose OS > Ubuntu > Ubuntu Server 22.04.4 LTS (64-bit) for Raspberry Pi 
- Click **Choose Storage** and select the newly mounted **eMMC**. 
- Before writing, click the **gear icon (Advanced options)** for first-boot config: 
    - Enable SSH: Choose password or SSH key 
    - Set hostname (optional) 
    - Set Wi-Fi (skip if using Ethernet) 
- Click **Save**, then **Write** to flash Ubuntu onto the eMMC. 
- Disconnect the RPi from your computer and power source and remove the jumper from the pins on the Compute Module IO Board. 
- Connect the RPi to power and it will boot up using the OS on the eMMC. 

### 3. Connect to RPi via SSH

We will use a computer to remotely ssh into the RPi over Wi-Fi. 

- Make sure the RPi is powered on and connected to the same network as your computer.
- From your own machine, run: 
```
ping raspberrypi.local 
```
- Once you’ve found the IP address of the Pi, then run: 
```
ssh pi@<IP_ADDRESS> 
```
- The default password is **raspberry**  
- If you set a custom user and password using Raspberry Pi Imager, you will instead run the following and enter your password when prompted. 
```
ssh <user>@<IP_ADDRESS> 
```

### 4. Install ROS2-jazzy on the RPi

Follow the documentation to install ROS2 on Ubuntu: [Ubuntu (deb packages) — ROS 2 Documentation: Jazzy documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 

!!! Note
    You probably don’t need all the GUI tools on the RPi that come with the Desktop install and you can just perform the ROS-Base install. 

- Test if ROS is working by running the following in the RPi ssh terminal. 
```
ros2 -h
```
- Don’t forget to add the following lines to your bash file to source your environment. Doing this will automatically source the environment every time you open a new terminal: 
```
source /opt/ros/jazzy/setup.bash 
source install/setup.bash 
```

## ROS Communication Between RPi and Computer  

**Starting requirements:** Both a Raspberry Pi and a computer with ROS2. 

**End objective:** ROS communication between the RPi and computer. 

There are multiple ways to run ROS on your computer. The most straightforward way is to run ROS natively in a Linux environment. This will automatically allow network communication between ROS nodes on different devices. Just be sure to set the following variables to the same values on the computer and RPi: 
```
export ROS_DOMAIN_ID=0 
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET 
```
Here are some other options if you don’t have a Linux environment:  

- Dual boot 
- A VM 
- Distrobox with Docker or Podman 
- Docker container in WSL 

On my Windows laptop, I used Distrobox with Podman in my WSL with mirrored network mode enambled. This worked for me. To do this, first [install Linux on Windows with WSL](https://learn.microsoft.com/en-us/windows/wsl/install). Then navigate to the **Installation** section in the [Distrobox documentation](https://distrobox.it/) and follow the given steps. 

## Send ROS Commands from RPi to PX4 

Use the following guide to set up the RPi as a companion computer to Pixhawk: 

[Raspberry Pi Companion with Pixhawk](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html)

This can easily be adapted for other flight controllers running PX4. 

### Tips 

It will be very helpful to install a terminal multiplexer on the RPi. Use the following commands to install tmux: 
```
sudo apt update  
sudo apt install tmux 
```
For quick reference, use this [tmux cheatsheet](https://snapcraft.io/install/tmux/raspbian). If you're completely new with tmux, check out [A Quick and Easy Guide to tmux](https://hamvocke.com/blog/a-quick-and-easy-guide-to-tmux/). 

Don’t forget to source the ROS 2 environment: 
```
source /opt/ros/jazzy/setup.bash 
source install/setup.bash 
```

Check status of client: 
```
uxrce_dds_client status 
```

If the client has not started for PX4, you can start it manually: 
```
uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600 
```

Start the agent on the RPi: 
```
sudo MicroXRCEAgent serial --dev /dev/serial0 -b 921600 
```

Or try: 
```
sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 
```

To source mavlink virtual environment: 
```
source ~/mavproxy_venv/bin/activate 
```

## Cross Compiling 

It can be beneficial to set up cross compiling on a computer so that you don’t have to wait hours for code to compile on the Raspberry Pi. For reference, it took 30 minutes to build the ROS package px4_msgs on the RPi but on a computer it takes 1-3 minutes. 

The [cross_compile](https://docs.ros.org/en/jazzy/How-To-Guides/Cross-compilation.html#cross-compilation) tool is no longer supported in ROS.

Check out this [possible alternative](https://discourse.ros.org/t/call-for-help-maintainership-of-the-ros-cross-compile-tool/26511/5) 

## Extra Guides 

Guide for setting up RPi as a companion computer.
[Link](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html)

Raspberry Pi 4 pinout diagram (pg 17).
[Link](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/5552/CM4%20Nano.pdf)

Pixhawk 6c mini pinout guide.
[Link](https://docs.holybro.com/autopilot/pixhawk-6c-mini/pixhawk-6c-mini-ports)

Guide for uXRCE-DDS (PX4-ROS 2/DDS Bridge).
[Link](https://docs.px4.io/main/en/middleware/uxrce_dds.html#starting-agent-and-client)

ROS2 with PX4.
[Link](https://docs.px4.io/main/en/ros2/user_guide.html)

Translation node information.
[Link](https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node.html#python)

Jaeyoung Lim’s px4-offboard example.
[Link](https://github.com/Jaeyoung-Lim/px4-offboard?tab=BSD-3-Clause-1-ov-file#readme)

Adapted SITL and HITL code.
[Link](https://github.com/austin006/ROS2_PX4_Offboard_Example/tree/master)

Pixracer Pro information.
[Link](https://mrobotics.io/docs/pixracer-pro/)

MoCap stuff.
[Link](https://index.ros.org/r/vrpn_mocap/#jazzy)