#!/usr/bin/env python3

import time
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, OffboardControlMode

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
    
    def arm_drone() -> str:
        """
        Arms the drone motors.
        
        Returns:
            str: Status message
        """
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
    
    def disarm_drone() -> str:
        """
        Disarms the drone motors.
        
        Returns:
            str: Status message
        """
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
    
    def switch_to_offboard_mode() -> str:
        """
        Switches the drone to offboard control mode.
        
        Returns:
            str: Status message
        """
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
    
    def takeoff(altitude: float = 2.0) -> str:
        """
        Commands the drone to take off to a specified altitude.
        Must be armed first.
        
        Args:
            altitude: Target altitude in meters (default: 2.0)
            
        Returns:
            str: Status message
        """
        if altitude < min_alt or altitude > max_alt:
            return f"Invalid altitude {altitude}m. Must be between {min_alt} and {max_alt} meters."
        
        # First switch to offboard mode
        switch_to_offboard_mode()
        
        # Then send trajectory setpoint
        msg = TrajectorySetpoint()
        msg.timestamp = get_timestamp()
        msg.position = [0.0, 0.0, -altitude]  # NED frame (negative Z is up)
        msg.yaw = 0.0
        
        # Publish multiple times to ensure offboard mode engages
        for _ in range(traj_repeat):
            msg.timestamp = get_timestamp()
            ros_node.trajectory_pub.publish(msg)
            time.sleep(pub_delay)
            
        return f"Takeoff command sent for altitude {altitude} meters."
    
    def land() -> str:
        """
        Commands the drone to land at current position.
        
        Returns:
            str: Status message
        """
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
    
    def goto_position(x: float, y: float, z: float) -> str:
        """
        Sends the drone to a specific position.
        Drone must be in offboard mode.
        
        Args:
            x: X coordinate in meters (North)
            y: Y coordinate in meters (East) 
            z: Z coordinate in meters (altitude above takeoff)
            
        Returns:
            str: Status message
        """
        # Validate coordinates
        if abs(x) > max_dist or abs(y) > max_dist:
            return f"Position too far from home. Limit is {max_dist}m in any direction."
        if z < min_alt or z > max_alt:
            return f"Altitude must be between {min_alt} and {max_alt} meters."
        
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
    
    def hold_position() -> str:
        """
        Commands the drone to hold its current position.
        
        Returns:
            str: Status message
        """
        # Get current position from node
        current_pos = ros_node.current_status['position']
        
        msg = TrajectorySetpoint()
        msg.timestamp = get_timestamp()
        msg.position = [current_pos[0], current_pos[1], -current_pos[2]]
        msg.yaw = 0.0
        
        ros_node.trajectory_pub.publish(msg)
        return "Holding current position."
    
    def get_status() -> str:
        """
        Returns the current status of the drone.
        
        Returns:
            str: Formatted status information
        """
        status = ros_node.current_status
        return (f"Drone Status:\n"
                f"- Armed: {status['armed']}\n"
                f"- Mode: {status['mode']}\n"
                f"- Battery: {status['battery_percentage']}%\n"
                f"- Position: x={status['position'][0]:.1f}m, "
                f"y={status['position'][1]:.1f}m, z={status['position'][2]:.1f}m\n"
                f"- Landed: {status['is_landed']}")
    
    def emergency_stop() -> str:
        """
        Emergency stop - immediately disarms the drone.
        WARNING: This will cause the drone to fall!
        
        Returns:
            str: Status message
        """
        disarm_drone()
        return "EMERGENCY STOP executed! Drone disarmed."
    
    # Return only the essential tools
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