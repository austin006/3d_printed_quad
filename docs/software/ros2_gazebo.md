# ROS2 and Gazebo

This page links resources to learn ROS 2 basics and explains how to get started with the quadrotor

## Learn ROS2

There are lots of available resources for learning ROS2

- [ROS 2 from Scratch](https://learning.oreilly.com/library/view/ros-2-from/9781835881408/) by Edouard Renard
- [ROS2 Official tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [MAGICC Lab tutorials](https://byu-magicc.github.io/wiki/ros2_tutorials/intro/)
- [ROS2 Docker tutorial](https://docs.ros.org/en/jazzy/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html)
- [ROS2 For Beginners](https://learning.oreilly.com/videos/ros2-for-beginners/10000DIVC2022146/)

## Learn Gazebo

- [MAGICC Lab tutorials](https://byu-magicc.github.io/wiki/gazebo_tutorials/overview/)
- [Multi-Vehicle Simulation with Gazebo](https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation.html)
- [Gazebo Simulation](https://docs.px4.io/main/en/sim_gazebo_gz/#adding-new-worlds-and-models)
- [ros_gz_project_template](https://github.com/gazebosim/ros_gz_project_template) (A template project integrating ROS 2 and Gazebo simulator)

## Quadrotor ROS2 Set-up 

The following set-up information is repeated from the README in the src directory. It explains how to set up the proper environment to use the code in this repository. See the README for explanations of each ROS2 package.

### Install PX4 Autopilot
To [Install PX4](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets) run this code 
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.15
```

Run this script in a bash shell to install everything

```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

You will now need to restart your computer before continuing

### Install ROS2 Jazzy
To install ROS2 Jazzy follow the steps [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)

### Install Dependencies

Install Python dependencies as mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2) with this code

```
pip3 install --user -U empy pyros-genmsg setuptools
```

### Build Micro DDS
As mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#setup-micro-xrce-dds-agent-client) run this code in order to build MicroDDS on your machine

```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Clone in Repo and Packages
Run this code to clone the repo

```
git clone https://github.com/austin006/3d_printed_quad.git
```

We need the px4_msgs package. Our ROS2 nodes rely on the message definitions in this package in order to communicate with PX4. Read [here](https://docs.px4.io/main/en/ros/ros2_comm.html#overview:~:text=ROS%202%20applications,different%20PX4%20releases) for more information.

Be sure you're in the src directory of the workspace and run this code to clone in the px4_msgs repo

```
cd src
git clone https://github.com/PX4/px4_msgs.git -b release/1.15
```

I couldn't get this to work in jazzy without also including [px4_msgs_old](https://docs.px4.io/main/en/middleware/uorb.html#message-versioning) and the [translation_node](https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node.html) packages. To copy these packages, make sure you are in your workspace directory and have installed PX4-Autopilot in your root directory. Run the following code

```
~/PX4-Autopilot/Tools/copy_to_ros_ws.sh .
```

Clone in all required packages before building the workspace

### Build the Workspace

Source the ROS2 installation, build the workspace within the workspace directory (not in src), and source the workspace

```
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Your directory should now look similar to the following

```
.
├── .github
├── docs
├── build
├── install
├── log
├── src
│   ├── px4_offboard
│   ├── other_ros_packages
│   ├── px4_msgs
│   ├── px4_msgs_old
│   ├── translation_node
├── .gitignore
├── mkdocs.yml
├── README.md
```

### Running the code

Follow the instructions for each package to run the code