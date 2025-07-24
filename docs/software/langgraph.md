# LangGraph Agents

This page explains what LangGraph is and how to use the drone control agent located in the ROS2 package called `ai_agent`.

## Drone Control Agent

![agent](../assets/ai_agent.png){width="50%"}
///caption
Process of the AI agent
///

## ai_agent

In this package, the `px4_offboard` simulation is expanded to interface with an AI Agent allowing for natural language interaction with the quadrotor. This is implented with LangChain, LangGraph, Ollama, 

### Set-up

Make sure you have downloaded ollama and pulled the model you wish to use

### Instructions to run the agent

To run the agent paste the following commands in a terminal:  

```
cd /ros2_workspaces/3d_printed_quad/src/ai_agent 
source venv/bin/activate 
source ~/ros2_workspaces/3d_printed_quad/install/setup.bash 
export PYTHONPATH=$PYTHONPATH:/ros2_workspaces/3d_printed_quad/src/ai_agent python3 
ai_agent/agent.py 
```