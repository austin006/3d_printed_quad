#!/usr/bin/env python3

import time
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode
from langchain_core.tools import tool 

# PX4 Command constants (in case they're not imported)
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
VEHICLE_CMD_DO_SET_MODE = 176  
VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_NAV_TAKEOFF = 22

def create_tools(ros_node, config=None):
    """Create tools bound to a ROS2 node for efficient drone control"""
    
    # Use config or defaults
    if config is None:
        config = {
            'safety': {'max_altitude': 50.0, 'min_altitude': 0.5, 'max_distance': 100.0},
            'publishing': {'trajectory_repeat': 10, 'position_repeat': 5, 'publish_delay': 0.1},
            'ros2': {'system_id': 1, 'component_id': 1}
        }
    
    # Extract config values
    max_alt = config['safety']['max_altitude']
    min_alt = config['safety']['min_altitude']
    max_dist = config['safety']['max_distance']
    traj_repeat = config.get('publishing', {}).get('trajectory_repeat', 10)
    pos_repeat = config.get('publishing', {}).get('position_repeat', 5)
    pub_delay = config.get('publishing', {}).get('publish_delay', 0.1)
    sys_id = config.get('ros2', {}).get('system_id', 1)
    comp_id = config.get('ros2', {}).get('component_id', 1)
    
    def get_timestamp():
        """Get current timestamp in microseconds"""
        return int(time.time() * 1e6)
    
    @tool
    def arm_drone() -> str:
        """Arms the drone motors."""
        msg = VehicleCommand()
        msg.timestamp = get_timestamp()
        msg.param1 = 1.0  # 1.0 to arm
        msg.command = VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = sys_id
        msg.target_component = comp_id
        msg.source_system = sys_id
        msg.source_component = comp_id
        msg.from_external = True
        
        ros_node.vehicle_command_pub.publish(msg)
        return "Arm command sent. Drone should arm within 2 seconds."
    
    @tool
    def disarm_drone() -> str:
        """Disarms the drone motors."""
        msg = VehicleCommand()
        msg.timestamp = get_timestamp()
        msg.param1 = 0.0  # 0.0 to disarm
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        ros_node.vehicle_command_pub.publish(msg)
        return "Disarm command sent. Drone should disarm immediately."
    
    @tool
    def switch_to_offboard_mode() -> str:
        """Switches the drone to offboard control mode."""
        msg = VehicleCommand()
        msg.timestamp = get_timestamp()
        msg.param1 = 1.0
        msg.param2 = 6.0  # OFFBOARD mode
        msg.command = VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        ros_node.vehicle_command_pub.publish(msg)
        return "Switching to offboard mode."
    
    @tool
    def takeoff(altitude: float = 2.0) -> str:
        """Commands the drone to take off to a specified altitude. Must be armed first.
        
        Args:
            altitude: Target altitude in meters (default: 2.0)
        """
        if altitude < min_alt or altitude > max_alt:
            return f"Invalid altitude {altitude}m. Must be between {min_alt} and {max_alt} meters."
        
        # Create offboard control mode message
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = get_timestamp()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        
        # Create trajectory setpoint
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [0.0, 0.0, -altitude]  # NED frame (negative Z is up)
        trajectory_msg.yaw = 0.0
        
        # First publish setpoints for 2+ seconds to establish offboard signal
        ros_node.get_logger().info("Publishing setpoints to establish offboard signal...")
        
        # Publish offboard mode and setpoints for 2.5 seconds before mode switch
        start_time = time.time()
        while time.time() - start_time < 2.5:
            offboard_msg.timestamp = get_timestamp()
            trajectory_msg.timestamp = get_timestamp()
            
            # Publish both offboard mode and trajectory
            if hasattr(ros_node, 'offboard_mode_pub'):
                ros_node.offboard_mode_pub.publish(offboard_msg)
            trajectory_msg.timestamp = get_timestamp()
            ros_node.trajectory_pub.publish(trajectory_msg)
            time.sleep(0.05)  # 20 Hz
        
        # Now switch to offboard mode
        ros_node.get_logger().info("Switching to offboard mode...")
        switch_to_offboard_mode()
        
        # Continue publishing for a bit more to ensure mode switch
        for _ in range(20):  # 1 more second
            offboard_msg.timestamp = get_timestamp()
            trajectory_msg.timestamp = get_timestamp()
            
            if hasattr(ros_node, 'offboard_mode_pub'):
                ros_node.offboard_mode_pub.publish(offboard_msg)
            ros_node.trajectory_pub.publish(trajectory_msg)
            time.sleep(0.05)
            
        return f"Takeoff command sent for altitude {altitude} meters. Drone should be climbing."
    
    @tool
    def land() -> str:
        """Commands the drone to land at current position."""
        msg = VehicleCommand()
        msg.timestamp = get_timestamp()
        msg.command = VEHICLE_CMD_NAV_LAND
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        ros_node.vehicle_command_pub.publish(msg)
        return "Land command sent. Drone will land at current position."
    
    @tool
    def goto_position(x: float, y: float, z: float) -> str:
        """Sends the drone to a specific position. Drone must be in offboard mode.
        
        Args:
            x: X coordinate in meters (North)
            y: Y coordinate in meters (East) 
            z: Z coordinate in meters (altitude above takeoff)
        """
        # Validate coordinates
        if abs(x) > max_dist or abs(y) > max_dist:
            return f"Position too far from home. Limit is {max_dist}m in any direction."
        if z < min_alt or z > max_alt:
            return f"Altitude must be between {min_alt} and {max_alt} meters."
        
        # Check if we're in offboard mode, if not, establish it first
        if ros_node.current_status['mode'] != 'OFFBOARD':
            # Create offboard control mode message
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = get_timestamp()
            offboard_msg.position = True
            offboard_msg.velocity = False
            offboard_msg.acceleration = False
            
            # Publish setpoints to establish offboard signal
            msg = TrajectorySetpoint()
            msg.position = [x, y, -z]  # Convert to NED frame
            msg.yaw = 0.0
            
            ros_node.get_logger().info("Establishing offboard signal before position command...")
            
            # Publish for 2.5 seconds to establish signal
            start_time = time.time()
            while time.time() - start_time < 2.5:
                offboard_msg.timestamp = get_timestamp()
                msg.timestamp = get_timestamp()
                
                if hasattr(ros_node, 'offboard_mode_pub'):
                    ros_node.offboard_mode_pub.publish(offboard_msg)
                ros_node.trajectory_pub.publish(msg)
                time.sleep(0.05)
            
            # Switch to offboard mode
            switch_to_offboard_mode()
            time.sleep(0.5)
        
        # Now send position commands
        msg = TrajectorySetpoint()
        msg.timestamp = get_timestamp()
        msg.position = [x, y, -z]  # Convert to NED frame
        msg.yaw = 0.0
        
        # Publish multiple times for reliability
        for _ in range(pos_repeat):
            msg.timestamp = get_timestamp()
            ros_node.trajectory_pub.publish(msg)
            time.sleep(pub_delay)
            
        return f"Moving to position: x={x}m, y={y}m, z={z}m"
    
    @tool
    def hold_position() -> str:
        """Commands the drone to hold its current position."""
        # Get current position from node
        current_pos = ros_node.current_status['position']
        
        msg = TrajectorySetpoint()
        msg.timestamp = get_timestamp()
        msg.position = [current_pos[0], current_pos[1], -current_pos[2]]
        msg.yaw = 0.0
        
        ros_node.trajectory_pub.publish(msg)
        return "Holding current position."
    
    @tool
    def get_status() -> str:
        """Returns the current status of the drone."""
        status = ros_node.current_status
        return (f"Drone Status:\n"
                f"- Armed: {status['armed']}\n"
                f"- Mode: {status['mode']}\n"
                f"- Battery: {status['battery_percentage']}%\n"
                f"- Position: x={status['position'][0]:.1f}m, "
                f"y={status['position'][1]:.1f}m, z={status['position'][2]:.1f}m\n"
                f"- Landed: {status['is_landed']}")
    
    @tool
    def emergency_stop() -> str:
        """Emergency stop - immediately disarms the drone. WARNING: This will cause the drone to fall!"""
        disarm_drone()
        return "EMERGENCY STOP executed! Drone disarmed."
    
    # Return the tools
    return [
        arm_drone,
        disarm_drone,
        takeoff,
        land,
        goto_position,
        hold_position,
        get_status,
        emergency_stop
    ]