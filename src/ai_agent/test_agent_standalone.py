#!/usr/bin/env python3
import sys
import os

# Add virtual environment to path
agent_dir = os.path.dirname(os.path.abspath(__file__))
venv_path = os.path.join(agent_dir, 'venv', 'lib', 'python3.11', 'site-packages')
sys.path.insert(0, venv_path)

from langchain_ollama import ChatOllama
from langchain_core.messages import HumanMessage

# Test basic LLM functionality
print("Testing Ollama connection...")
try:
    llm = ChatOllama(
        model="llama3.2:3b",
        base_url="http://localhost:11434",
        temperature=0.0
    )
    
    response = llm.invoke([HumanMessage(content="Say 'Hello drone world!' and nothing else.")])
    print(f"✓ Ollama response: {response.content}")
except Exception as e:
    print(f"✗ Ollama error: {e}")

# Test drone tools
print("\nTesting drone tools creation...")
try:
    from ai_agent.drone_tools import create_tools
    
    # Create a mock ROS node
    class MockNode:
        def __init__(self):
            self.current_status = {
                'armed': False,
                'mode': 'MANUAL',
                'battery_percentage': 85,
                'position': [0.0, 0.0, 0.0],
                'is_landed': True
            }
            self.vehicle_command_pub = None
            self.trajectory_pub = None
            
        def get_logger(self):
            class MockLogger:
                def info(self, msg): print(f"[INFO] {msg}")
                def error(self, msg): print(f"[ERROR] {msg}")
                def warn(self, msg): print(f"[WARN] {msg}")
            return MockLogger()
    
    mock_node = MockNode()
    tools = create_tools(mock_node)
    print(f"✓ Created {len(tools)} tools")
    
    # Test a tool
    status_tool = next((t for t in tools if t.__name__ == 'get_status'), None)
    if status_tool:
        result = status_tool()
        print(f"✓ Status tool output:\n{result}")
        
except Exception as e:
    print(f"✗ Drone tools error: {e}")
    import traceback
    traceback.print_exc()