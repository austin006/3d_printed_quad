#!/usr/bin/env python3

"""
Test script for the Drone Control Agent
Tests basic functionality without manual input
Designed to work with PX4 SITL simulation
"""

import rclpy
from agent import DroneControlNode
import time
from threading import Thread

def wait_for_simulation():
    """Wait for PX4 simulation to be ready"""
    print("⏳ Waiting for PX4 simulation to initialize...")
    time.sleep(5)  # Give simulation time to start
    
def run_test_sequence(node):
    """Run a sequence of test commands"""
    
    # Wait for simulation
    wait_for_simulation()
    
    test_commands = [
        ("Check initial status", "What is the drone status?"),
        ("Wait for pre-flight checks", None, 3),  # Wait for PX4 to be ready
        ("Arm the drone", "Arm the drone"),
        ("Wait for arming", None, 2),  # 2 second pause
        ("Take off", "Take off to 3 meters"),
        ("Wait for takeoff", None, 8),  # 8 seconds for takeoff
        ("Check status after takeoff", "Show me the current status"),
        ("Move to position", "Fly to x=2, y=2, z=3"),
        ("Wait for movement", None, 5),  # 5 second pause
        ("Hold position", "Hold your current position"),
        ("Final status check", "What's the battery level and position?"),
        ("Land", "Land the drone"),
        ("Wait for landing", None, 8),  # 8 seconds for landing
        ("Disarm", "Disarm the drone"),
    ]
    
    print("\n🧪 Starting automated test sequence...")
    print("=" * 50)
    print("⚠️  Ensure PX4 SITL and MicroXRCEAgent are running!")
    print("=" * 50)
    
    for item in test_commands:
        step_name = item[0]
        command = item[1] if len(item) > 1 else None
        wait_time = item[2] if len(item) > 2 else 1
        
        print(f"\n📍 {step_name}")
        
        if command is None:
            # This is a wait command
            time.sleep(wait_time)
        else:
            print(f"   Command: '{command}'")
            response = node.process_command(command)
            print(f"   Response: {response}")
            time.sleep(wait_time)  # Wait between commands
    
    print("\n✅ Test sequence completed!")
    print("=" * 50)

def main():
    """Main test function"""
    rclpy.init()
    
    try:
        # Create node
        node = DroneControlNode()
        
        # Spin in separate thread
        spin_thread = Thread(target=rclpy.spin, args=(node,))
        spin_thread.start()
        
        # Give the node time to initialize
        time.sleep(2)
        
        # Run test sequence
        run_test_sequence(node)
        
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        rclpy.shutdown()
        return 1
    
    return 0

if __name__ == '__main__':
    exit(main())