#!/usr/bin/env python3

"""
Drone Tools Module for AI Agent

This module provides LangChain-compatible tools for controlling a PX4 quadrotor.
"""

import rclpy
import numpy as np
import time
from typing import List, Dict, Any, Optional
from threading import Thread
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleStatus,
    VehicleOdometry,
    VehicleCommand,
    BatteryStatus
)

from langchain_core.tools import tool


# Global controller instance
_controller_node = None
_spin_thread = None
_executor = None


def init_drone_control(vehicle_id: str = ""):
    """Initialize the drone control system."""
    global _controller_node, _spin_thread, _executor
    
    if not rclpy.ok():
        rclpy.init()
    
    if _controller_node is None:
        _controller_node = DroneControlNode(vehicle_id)
        _executor = rclpy.executors.SingleThreadedExecutor()
        _executor.add_node(_controller_node)
        
        # Start spinning in a separate thread
        _spin_thread = Thread(target=_executor.spin, daemon=True)
        _spin_thread.start()
        
        # Wait for initialization
        time.sleep(1.0)


def get_node():
    """Get the drone control node."""
    if _controller_node is None:
        init_drone_control()
    return _controller_node


class DroneControlNode(Node):
    """ROS2 node for drone control."""
    
    def __init__(self, vehicle_id: str = ""):
        node_name = f'drone_control_node_{vehicle_id}' if vehicle_id else f'drone_control_node'
        super().__init__(node_name)
        
        # Vehicle setup
        if vehicle_id:
            self.target_system_id = int(vehicle_id) + 1
            self.namespace = f"/px4_{vehicle_id}"
        else:
            self.target_system_id = 1
            self.namespace = ""
        
        # QoS Profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, 
            f'{self.namespace}/fmu/in/offboard_control_mode', 
            qos_profile
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, 
            f'{self.namespace}/fmu/in/trajectory_setpoint', 
            qos_profile
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, 
            f"{self.namespace}/fmu/in/vehicle_command", 
            10
        )
        
        # Subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'{self.namespace}/fmu/out/vehicle_status_v1',
            self.status_callback,
            qos_profile
        )
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            f'{self.namespace}/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )        
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            f'{self.namespace}/fmu/out/battery_status',
            self.battery_callback,
            qos_profile
        )
        
        # State variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.flight_check_passed = False
        self.failsafe_active = False
        
        # Position (NED)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # Control
        self.target_position_ned = [0.0, 0.0, 0.0]
        self.offboard_timer = None
        
        # Start offboard control timer
        self.start_offboard_control()
    
    def status_callback(self, msg):
        """Handle status updates."""
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe_active = msg.failsafe
        self.flight_check_passed = msg.pre_flight_checks_pass
    
    def odometry_callback(self, msg):
        """Handle odometry updates."""
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = msg.position[2]

    def battery_callback(self, msg):
        """Handle battery status updates."""
        self.battery_percentage = int(msg.remaining * 100)  # 0 to 100
    
    def publish_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = self.target_system_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)
    
    def start_offboard_control(self):
        """Start offboard control timer."""
        if self.offboard_timer is None:
            self.offboard_timer = self.create_timer(0.02, self.offboard_control_callback)
    
    def offboard_control_callback(self):
        """Send offboard control messages."""
        # Publish offboard mode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.offboard_mode_pub.publish(offboard_msg)
        
        # Publish trajectory if armed and in offboard mode
        if self.is_armed() and self.is_in_offboard_mode():
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_msg.position[0] = float(self.target_position_ned[0])
            trajectory_msg.position[1] = float(self.target_position_ned[1])
            trajectory_msg.position[2] = float(self.target_position_ned[2])
            self.trajectory_pub.publish(trajectory_msg)
    
    def arm(self):
        """Arm the drone."""
        self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    
    def disarm(self):
        """Disarm the drone."""
        self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    
    def land_cmd(self):
        """Send land command."""
        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    
    def set_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
    
    def set_target_position_enu(self, position: List[float]):
        """Set target position in ENU coordinates."""
        # Convert ENU to NED
        self.target_position_ned = [
            position[1],    # Y_ENU -> X_NED (North)
            position[0],    # X_ENU -> Y_NED (East)
            -position[2]    # Z_ENU -> Z_NED (Down)
        ]
    
    def get_position_enu(self) -> List[float]:
        """Get current position in ENU coordinates."""
        # Convert NED to ENU
        return [
            self.current_y,   # Y_NED -> X_ENU (East)
            self.current_x,   # X_NED -> Y_ENU (North)
            -self.current_z   # -Z_NED -> Z_ENU (Up)
        ]
    
    def is_armed(self) -> bool:
        """Check if armed."""
        return self.arm_state == VehicleStatus.ARMING_STATE_ARMED
    
    def is_in_offboard_mode(self) -> bool:
        """Check if in offboard mode."""
        return self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def get_battery_status(self) -> Dict[str, Any]:
        """Get battery status information."""
        return {
            'percentage': self.battery_percentage,
        }


# === LangChain Tool Functions ===

@tool
def arm_drone() -> str:
    """Arms the drone motors."""
    try:
        node = get_node()
        node.arm()
        
        # Wait for arming
        for _ in range(50):  # 5 seconds timeout
            time.sleep(0.1)
            if node.is_armed():
                return "Drone armed successfully"
        
        return "Failed to arm drone - check pre-flight conditions"
    except Exception as e:
        return f"Error arming drone: {str(e)}"


@tool
def disarm_drone() -> str:
    """Disarms the drone motors."""
    try:
        node = get_node()
        node.disarm()
        
        # Wait for disarming
        for _ in range(30):  # 3 seconds timeout
            time.sleep(0.1)
            if not node.is_armed():
                return "Drone disarmed successfully"
        
        return "Failed to disarm drone"
    except Exception as e:
        return f"Error disarming drone: {str(e)}"


@tool
def land() -> str:
    """Lands the drone at current position."""
    try:
        node = get_node()
        node.land_cmd()
        return "Landing initiated"
    except Exception as e:
        return f"Error during landing: {str(e)}"


@tool
def fly_to_position(x: float, y: float, z: float) -> str:
    """Flies to specified position in meters.
    
    Args:
        x: Target X position in meters (East)
        y: Target Y position in meters (North)
        z: Target Z position in meters (Up)
    """
    try:
        node = get_node()
        
        # Ensure in offboard mode
        if not node.is_in_offboard_mode():
            node.set_offboard_mode()
            time.sleep(0.5)
        
        # Ensure armed
        if not node.is_armed():
            arm_result = arm_drone.invoke({})
            if "successfully" not in arm_result:
                return arm_result
        
        # Set target position
        node.set_target_position_enu([x, y, z])
        
        # Wait for position to be reached
        timeout = 30.0
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            current_pos = node.get_position_enu()
            distance = np.sqrt(
                (current_pos[0] - x)**2 + 
                (current_pos[1] - y)**2 + 
                (current_pos[2] - z)**2
            )
            
            if distance < 0.5:  # Within 0.5m
                return f"Reached position ({x}, {y}, {z})"
            
            time.sleep(0.1)
        
        return f"Moving to position ({x}, {y}, {z}) - timeout reached"
    except Exception as e:
        return f"Error flying to position: {str(e)}"
    

@tool
def get_status() -> str:
    """Gets current drone status including position, armed state, and battery."""
    try:
        node = get_node()
        
        pos = node.get_position_enu()
        armed = "Armed" if node.is_armed() else "Disarmed"
        mode = "Offboard" if node.is_in_offboard_mode() else "Other"
        battery = node.battery_percentage
        
        return (f"Status: {armed}, Mode: {mode}, "
                f"Position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}), "
                f"Battery: {battery}%")
    except Exception as e:
        return f"Error getting status: {str(e)}"


def create_tools(node=None, config=None):
    """
    Create and return the list of tools for the agent.
    Compatible with the existing agent structure.
    """
    # Initialize the drone control system
    init_drone_control()
    
    # Return all tool functions
    return [
        arm_drone,
        disarm_drone,
        land,
        fly_to_position,
        get_status
    ]


def get_controller():
    """Get controller for compatibility with agent."""
    return get_node()


def emergency_stop():
    """Emergency stop - immediately disarms motors. WARNING: Drone will fall!"""
    try:
        node = get_node()
        node.disarm()
        node.land_cmd()
        return "EMERGENCY STOP - Motors disarmed!"
    except Exception as e:
        return f"Error during emergency stop: {str(e)}"


def shutdown():
    """Shutdown the drone control system."""
    global _controller_node, _executor, _spin_thread
    
    if _executor:
        _executor.shutdown()
    
    if _controller_node:
        _controller_node.destroy_node()
    
    if _spin_thread:
        _spin_thread.join(timeout=1.0)
    
    _controller_node = None
    _executor = None
    _spin_thread = None