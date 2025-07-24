# Software Overview

## Full Set-up

![software_stack](../assets/full_stack.png)
/// caption
Layout of the whole software system
///

## Fitting in the Software

Follow along with the diagrams step by step to see how each piece builds upon the previous. This will allow for a natural, intuitive understanding if you are not familiar with many of the tools presented.

### Transmitter
![transmitter_stack](../assets/transmitter_stack.png)
/// caption
Connection between transmitter, PX4, and quadrotor
///

### ROS 2
![ros2](../assets/ros2.png)
/// caption
ROS 2 is an open-source framework designed to simplify the development of robotics software
///

### MicroXRCE-DDS
![microdds](../assets/microdds.png)
/// caption
MicroXRCE-DDS creates a communication bridge between ROS2 and PX4
///

### Gazebo
![gazebo](../assets/gazebo.png)
/// caption
Gazebo and RViz are used to simulate quadrotor flight
///

### Advanced Control
![waypoints](../assets/waypoints.png)
/// caption
ROS 2 provides framework to perform complex tasks and customize quadrotor control
///

### AI Agent
![software_stack](../assets/full_stack.png)
/// caption
An AI Agent can invoke LLMs and call tools to process natural language flight commands
///