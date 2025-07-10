#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
from threading import Thread
import yaml
import sys
import os
from ament_index_python.packages import get_package_share_directory

# Add virtual environment to path
agent_dir = os.path.dirname(os.path.abspath(__file__))
venv_path = os.path.join(os.path.dirname(agent_dir), 'venv', 'lib', 'python3.12', 'site-packages')
if os.path.exists(venv_path):
    sys.path.insert(0, venv_path)
    print(f"✓ Added venv to path: {venv_path}")
else:
    print(f"⚠ Warning: venv path not found: {venv_path}")

# LangChain imports
from langchain_core.messages import SystemMessage, HumanMessage
from langchain_ollama import ChatOllama

# LangGraph imports  
from langgraph.graph import START, StateGraph, MessagesState
from langgraph.prebuilt import tools_condition, ToolNode

# Type hints
from typing import TypedDict, List, Dict, Any, Optional

# ROS2 message imports
from px4_msgs.msg import (
    VehicleCommand, 
    TrajectorySetpoint,
    VehicleStatus,
    BatteryStatus,
    VehicleLocalPosition,
    OffboardControlMode
)

# Import tools
from ai_agent.drone_tools import create_tools

def load_config():
    """Load configuration from config.yaml"""
    # Try to get from package share first
    try:
        package_share_dir = get_package_share_directory('ai_agent')
        config_path = os.path.join(package_share_dir, 'config', 'config.yaml')
    except:
        # Fallback for development
        config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
    
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            
        # Auto-detect if running in Docker
        if os.path.exists('/.dockerenv'):
            config['llm']['base_url'] = 'http://host.docker.internal:11434'
        
        return config
    else:
        # Default config if file doesn't exist
        return {
            'llm': {
                'model': 'llama3.2:3b',
                'base_url': 'http://host.docker.internal:11434',
                'temperature': 0.0,
                'num_predict': 100
            },
            'safety': {
                'max_altitude': 50.0,
                'min_altitude': 0.5,
                'max_distance': 100.0
            },
            'performance': {
                'enable_status_monitoring': True,
                'status_in_prompt': True
            }
        }

class DroneState(TypedDict):
    """Minimal state for efficient operation"""
    messages: List[Dict[str, Any]]
    
class DroneControlNode(Node):
    """ROS2 node that integrates with the LangGraph agent"""
    def __init__(self):
        super().__init__('drone_control_agent')
        
        # Get package share directory
        package_share_dir = get_package_share_directory('ai_agent')
        
        # Load configuration
        config_path = os.path.join(package_share_dir, 'config', 'config.yaml')
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            # Fallback to default config
            self.config = self.get_default_config()
        
        # QoS profile for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # Subscribers for status monitoring (if enabled)
        if self.config['performance']['enable_status_monitoring']:
            self.vehicle_status_sub = self.create_subscription(
                VehicleStatus, '/fmu/out/vehicle_status', 
                self.vehicle_status_callback, qos_profile)
            self.battery_sub = self.create_subscription(
                BatteryStatus, '/fmu/out/battery_status',
                self.battery_callback, qos_profile)
            self.position_sub = self.create_subscription(
                VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                self.position_callback, qos_profile)
        
        # Current state
        self.current_status = {
            'armed': False,
            'mode': 'UNKNOWN',
            'battery_percentage': 100,
            'position': [0.0, 0.0, 0.0],
            'is_landed': True
        }
        
        # Create tools with ROS2 node reference and config
        self.tools = create_tools(self, self.config)
        
        # Initialize LLM with tools
        self.llm = ChatOllama(
            model=self.config['llm']['model'], 
            base_url=self.config['llm']['base_url'],
            temperature=self.config['llm']['temperature'],
            num_predict=self.config['llm']['num_predict']
        )
        self.llm_with_tools = self.llm.bind_tools(self.tools)
        
        # Load system prompt
        system_prompt_path = os.path.join(package_share_dir, 'config', 'system_prompt.txt')
        with open(system_prompt_path, "r") as f:
            self.system_prompt = SystemMessage(content=f.read())
        
        # Build the graph
        self.graph = self._build_graph()
        
    def _build_graph(self):
        """Build the LangGraph agent"""
        # Create state graph
        builder = StateGraph(DroneState)
        
        # Add nodes
        builder.add_node("assistant", self._assistant_node)
        builder.add_node("tools", ToolNode(self.tools))
        
        # Add edges
        builder.add_edge(START, "assistant")
        builder.add_conditional_edges(
            "assistant",
            tools_condition,
            {
                "tools": "tools",
                "__end__": "__end__"
            }
        )
        builder.add_edge("tools", "assistant")
        
        return builder.compile()
    
    def _assistant_node(self, state: DroneState):
        """LLM assistant node"""
        messages = [self.system_prompt] + state["messages"]
        
        # Add current drone status to the latest message if enabled
        if self.config['performance']['status_in_prompt'] and messages[-1].get("role") == "user":
            status_msg = f"\nCurrent drone status: Armed={self.current_status['armed']}, Mode={self.current_status['mode']}, Battery={self.current_status['battery_percentage']}%, Position={self.current_status['position']}"
            messages[-1]["content"] += status_msg
            
        response = self.llm_with_tools.invoke(messages)
        return {"messages": [response]}
    
    def vehicle_status_callback(self, msg):
        """Update vehicle status"""
        self.current_status['armed'] = msg.arming_state == 2
        self.current_status['mode'] = self._get_mode_name(msg.nav_state)
        
    def battery_callback(self, msg):
        """Update battery status"""
        self.current_status['battery_percentage'] = int(msg.remaining * 100)
        
    def position_callback(self, msg):
        """Update position"""
        self.current_status['position'] = [msg.x, msg.y, -msg.z]  # NED to NWU
        self.current_status['is_landed'] = abs(msg.z) < 0.1
        
    def _get_mode_name(self, nav_state):
        """Convert nav_state to readable mode name"""
        modes = {
            0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 
            3: "AUTO_MISSION", 4: "AUTO_LOITER", 5: "AUTO_RTL",
            6: "OFFBOARD"
        }
        return modes.get(nav_state, "UNKNOWN")
    
    def process_command(self, command: str) -> str:
        """Process a natural language command"""
        try:
            # Run the graph
            initial_state = {
                "messages": [{"role": "user", "content": command}]
            }
            
            result = self.graph.invoke(initial_state)
            
            # Extract the final response
            for msg in reversed(result["messages"]):
                if hasattr(msg, 'content') and msg.content and not hasattr(msg, 'tool_calls'):
                    return msg.content
                    
            return "Command processed successfully."
            
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
            return f"Error: {str(e)}"

def main():
    rclpy.init()
    node = DroneControlNode()
    
    # Spin in a separate thread
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    print("Drone Control Agent initialized. Type 'exit' to quit.")
    print("Example commands:")
    print("  - 'Arm the drone'")
    print("  - 'Take off to 5 meters'")
    print("  - 'Fly to position x=2, y=3, z=4'")
    print("  - 'Land the drone'")
    print("  - 'What is the battery level?'")
    
    try:
        while True:
            command = input("\nEnter command: ")
            if command.lower() == 'exit':
                break
                
            response = node.process_command(command)
            print(f"Agent: {response}")
            
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()