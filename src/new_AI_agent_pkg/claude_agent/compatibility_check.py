#!/usr/bin/env python3

"""
Compatibility check script for the Drone Control Agent
Verifies that all necessary components are running
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import subprocess
import sys
import time
import requests

class CompatibilityChecker(Node):
    def __init__(self):
        super().__init__('compatibility_checker')
        self.checks_passed = True
        
    def check_ros2_topics(self):
        """Check if PX4 topics are available"""
        print("🔍 Checking ROS2 topics...")
        
        # Get list of topics
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True)
        
        if result.returncode != 0:
            print("❌ Failed to get ROS2 topics")
            self.checks_passed = False
            return
        
        topics = result.stdout.strip().split('\n')
        required_topics = [
            '/fmu/in/vehicle_command',
            '/fmu/in/trajectory_setpoint',
            '/fmu/out/vehicle_status'
        ]
        
        missing_topics = []
        for topic in required_topics:
            if topic not in topics:
                missing_topics.append(topic)
        
        if missing_topics:
            print(f"❌ Missing required topics: {missing_topics}")
            print("   Make sure PX4 SITL and MicroXRCEAgent are running")
            self.checks_passed = False
        else:
            print("✅ All required PX4 topics found")
    
    def check_ollama(self):
        """Check if Ollama is running"""
        print("\n🔍 Checking Ollama server...")
        
        try:
            response = requests.get('http://localhost:11434/api/health', timeout=2)
            if response.status_code == 200:
                print("✅ Ollama server is running")
                
                # Check if model is available
                response = requests.get('http://localhost:11434/api/tags', timeout=2)
                if response.status_code == 200:
                    models = response.json().get('models', [])
                    model_names = [m['name'] for m in models]
                    
                    if 'llama3.2:3b' in model_names:
                        print("✅ llama3.2:3b model is available")
                    elif 'tinyllama:latest' in model_names:
                        print("✅ tinyllama model is available (RPi4 optimized)")
                    else:
                        print("⚠️  No compatible model found. Run: ollama pull llama3.2:3b")
                        self.checks_passed = False
            else:
                print("❌ Ollama server returned error")
                self.checks_passed = False
                
        except requests.exceptions.ConnectionError:
            print("❌ Cannot connect to Ollama server")
            print("   Run: ollama serve")
            self.checks_passed = False
        except Exception as e:
            print(f"❌ Error checking Ollama: {e}")
            self.checks_passed = False
    
    def check_micro_xrce(self):
        """Check if MicroXRCEAgent is running"""
        print("\n🔍 Checking MicroXRCEAgent...")
        
        # Check if process is running
        result = subprocess.run(['pgrep', '-f', 'MicroXRCEAgent'], 
                              capture_output=True)
        
        if result.returncode == 0:
            print("✅ MicroXRCEAgent is running")
        else:
            print("⚠️  MicroXRCEAgent not detected")
            print("   If using serial connection, this is normal")
            print("   For UDP: MicroXRCEAgent udp4 -p 8888")
    
    def check_px4_sitl(self):
        """Check if PX4 SITL is running"""
        print("\n🔍 Checking PX4 SITL...")
        
        # Check for Gazebo
        result = subprocess.run(['pgrep', '-f', 'gz sim'], 
                              capture_output=True)
        
        if result.returncode == 0:
            print("✅ Gazebo simulation is running")
        else:
            print("⚠️  Gazebo not detected")
            print("   This is normal if using hardware")
    
    def check_conflicting_nodes(self):
        """Check for potentially conflicting nodes"""
        print("\n🔍 Checking for conflicting nodes...")
        
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, text=True)
        
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            
            if '/px4_offboard/velocity' in nodes:
                print("⚠️  velocity_control node is running")
                print("   This may conflict with AI agent commands")
                print("   Consider stopping it or using one control method at a time")
            else:
                print("✅ No conflicting velocity control nodes detected")
    
    def check_python_deps(self):
        """Check Python dependencies"""
        print("\n🔍 Checking Python dependencies...")
        
        required_packages = [
            'langchain',
            'langchain-ollama',
            'langgraph',
            'rclpy',
            'px4_msgs'
        ]
        
        missing_packages = []
        for package in required_packages:
            try:
                __import__(package.replace('-', '_'))
            except ImportError:
                missing_packages.append(package)
        
        if missing_packages:
            print(f"❌ Missing Python packages: {missing_packages}")
            print("   Run: pip3 install -r requirements.txt")
            self.checks_passed = False
        else:
            print("✅ All Python dependencies installed")

def main():
    print("🚁 Drone Control Agent Compatibility Checker")
    print("=" * 50)
    
    rclpy.init()
    checker = CompatibilityChecker()
    
    # Run all checks
    checker.check_python_deps()
    checker.check_ollama()
    checker.check_ros2_topics()
    checker.check_micro_xrce()
    checker.check_px4_sitl()
    checker.check_conflicting_nodes()
    
    # Summary
    print("\n" + "=" * 50)
    if checker.checks_passed:
        print("✅ All critical checks passed!")
        print("   You can now run: python3 agent.py")
    else:
        print("❌ Some checks failed. Please fix the issues above.")
        sys.exit(1)
    
    checker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()