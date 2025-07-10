#!/usr/bin/env python3
import sys
import os

# Add virtual environment to path
agent_dir = os.path.dirname(os.path.abspath(__file__))
venv_path = os.path.join(agent_dir, 'venv', 'lib', 'python3.12', 'site-packages')
if os.path.exists(venv_path):
    sys.path.insert(0, venv_path)

try:
    # Test imports from agent.py
    from ai_agent.drone_tools import create_tools
    print("✓ drone_tools import successful")
    
    # Test if we can create a dummy node
    import rclpy
    from ai_agent.agent import DroneControlNode
    print("✓ DroneControlNode import successful")
    
except Exception as e:
    print(f"✗ Import error: {e}")