#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from threading import Thread
from ament_index_python.packages import get_package_share_directory

# LangChain imports
from langchain_core.messages import SystemMessage, HumanMessage
from langchain_ollama import ChatOllama

# LangGraph imports  
from langgraph.graph import START, StateGraph, MessagesState
from langgraph.prebuilt import tools_condition, ToolNode

# Type hints
from typing import TypedDict, List, Dict, Any

# Import the drone tools
from ai_agent.drone_tools import create_tools, get_controller, shutdown, emergency_stop

class DroneState(MessagesState):
    """State for the drone control agent - using MessagesState base"""
    pass

class DroneControlNode(Node):
    """ROS2 node that integrates with the LangGraph agent"""
    
    def __init__(self):
        node_name = 'drone_control_agent'
        super().__init__(node_name)
        
        # Load configuration
        self.config = self._load_config()
        
        # Create tools with proper initialization
        self.tools = create_tools(self, self.config)
        
        # Create LLM
        self.llm = ChatOllama(
            model=self.config['llm']['model'], 
            base_url=self.config['llm']['base_url'],
            temperature=self.config['llm']['temperature'],
            num_predict=self.config['llm']['num_predict']
        )
        self.llm_with_tools = self.llm.bind_tools(self.tools)
        
        # Create system message once
        self.system_message = SystemMessage(content=self._get_system_prompt())
        
        # Build the graph
        self.graph = self._build_graph()
        
        # Initialize current status
        self.current_status = {
            'armed': False,
            'mode': 'UNKNOWN',
            'battery_percentage': 0,
            'position': [0.0, 0.0, 0.0],
            'is_landed': True
        }
        
        # Create timer to periodically update status if monitoring enabled
        if self.config['performance']['enable_status_monitoring']:
            self.create_timer(1.0, self._update_status)
    
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration with proper fallback"""
        # Try package share directory first
        config_path = None
        try:
            package_share_dir = get_package_share_directory('ai_agent')
            config_path = os.path.join(package_share_dir, 'config', 'config.yaml')
        except:
            # Fallback to local directory
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
        
        # Load config if file exists
        if config_path and os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
        else:
            self.get_logger().info("Config file not found, using default configuration.")
            config = self._get_default_config()
        
        # Auto-detect Docker and adjust base URL
        if os.path.exists('/.dockerenv'):
            config['llm']['base_url'] = 'http://host.docker.internal:11434'
        
        return config
    
    def _get_default_config(self) -> Dict[str, Any]:
        """Get default configuration"""
        return {
            'llm': {
                'model': 'llama3.2:3b',
                'base_url': 'http://localhost:11434',
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
    
    def _get_system_prompt(self) -> str:
        """Get the system prompt"""
        return """You are a drone control assistant. Help users control their quadcopter using the available tools.

        Available tools:
        - arm_drone(): Arms motors
        - disarm_drone(): Disarms motors  
        - land(): Lands at current position
        - fly_to_position(x, y, z): Flies to coordinates
        - get_status(): Reports status

        Guidelines:
        1. Use appropriate tool
        2. Answer questions helpfully.
        3. Be concise. Keep responses under 50 words."""
    
    def _update_status(self):
        """Update current status from drone controller"""
        try:
            node = get_controller()
            pos = node.get_position_enu()
            
            self.current_status = {
                'armed': node.is_armed(),
                'mode': 'OFFBOARD' if node.is_in_offboard_mode() else 'OTHER',
                'battery_percentage': 100,  # Not available from basic status
                'position': pos,
                'is_landed': abs(pos[2]) < 0.1
            }
        except:
            # If controller not initialized yet, use defaults
            self.current_status = {
                'armed': False,
                'mode': 'UNKNOWN',  
                'battery_percentage': 0,
                'position': [0.0, 0.0, 0.0],
                'is_landed': True
            }
    
    def _build_graph(self):
        """Build the LangGraph agent"""
        builder = StateGraph(DroneState)
        
        # Add nodes
        builder.add_node("assistant", self._assistant_node)
        builder.add_node("tools", ToolNode(self.tools))
        
        # Add edges
        builder.add_edge(START, "assistant")
        builder.add_conditional_edges(
            "assistant",
            tools_condition,
            {"tools": "tools", "__end__": "__end__"}
        )
        builder.add_edge("tools", "assistant")
        
        return builder.compile()
    
    def _assistant_node(self, state: DroneState):
        """LLM assistant node"""
        messages = [self.system_message]
        
        # Update status before processing
        if self.config['performance']['enable_status_monitoring']:
            self._update_status()
        
        # Add conversation messages
        for msg in state["messages"]:
            if isinstance(msg, dict):
                if msg.get("role") == "user":
                    content = msg["content"]
                    # Add status if enabled and this is the latest user message
                    if (self.config['performance']['status_in_prompt'] and 
                        msg == state["messages"][-1] and
                        hasattr(self, 'current_status')):
                        content += f"\n[Status: Armed={self.current_status['armed']}, " \
                                  f"Mode={self.current_status['mode']}, " \
                                  f"Battery={self.current_status.get('battery_percentage', 'N/A')}%]"
                    messages.append(HumanMessage(content=content))
                else:
                    messages.append(msg)
            else:
                messages.append(msg)
        
        response = self.llm_with_tools.invoke(messages)
        return {"messages": [response]}
    
    def process_command(self, command: str) -> str:
        """Process a natural language command"""
        try:
            # Create initial state
            initial_state = {"messages": [HumanMessage(content=command)]}
            
            # Run the graph
            result = self.graph.invoke(initial_state)
            
            # Get the last message content
            if result["messages"]:
                last_msg = result["messages"][-1]
                
                # Extract content from different message types
                if hasattr(last_msg, 'content'):
                    return last_msg.content
                elif isinstance(last_msg, dict) and 'content' in last_msg:
                    return last_msg['content']
                else:
                    return "Command processed."
            
            return "No response generated."
            
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
            return f"Error: {str(e)}"


def main():
    """Main entry point"""
    rclpy.init()
    node = DroneControlNode()
    
    # Spin in a separate thread
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print(f"Ollama model: {node.config['llm']['model']}")
    print("\n🚁 Drone Control Agent Ready")
    print("=" * 40)
    print("Type commands naturally or 'exit' to quit")
    print("In case of emergency, type 'kill' to disarm motors immediately")
    print("Example commands:")
    print("  ▸ Take off to a height of 5 meters")
    print("  ▸ Where is the drone?")
    print("  ▸ Go to position x=10, y=5, z=8")
    print("  ▸ What's the battery level?")
    print("  ▸ Land here")
    print("  ▸ Arm the motors")
    print("=" * 40)
    
    try:
        while True:
            try:
                command = input("\n> ")
                if command.lower() in ['exit', 'quit']:
                    break

                if command.lower() in ['kill']:
                    print("⚠️  EMERGENCY STOP ACTIVATED ⚠️")
                    try:
                        result = emergency_stop()
                        print(f"Agent: {result}")
                    except Exception as e:
                        print(f"EMERGENCY STOP FAILED: {str(e)}")
                    continue   

                if command.strip():  # Only process non-empty commands
                    response = node.process_command(command)
                    print(f"Agent: {response}")
                    
            except EOFError:
                break
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    # Cleanup
    try:
        shutdown()  # Shutdown drone control system
    except:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()