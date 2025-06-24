#!/usr/bin/env python

import rclpy
import numpy as np
import json 
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry 
from px4_msgs.msg import VehicleCommand


class WaypointOffboardControl(Node):
    """
    ROS2 node to control a single quadcopter to visit a sequence of 3D waypoints.
    Automatically arms the quadcopter and switches to offboard mode.
    Waypoints are passed as a JSON string parameter.
    Supports continuous looping through waypoints or single pass.
    Compatible with multi-vehicle setups via 'vehicle_id' parameter.
    """

    def __init__(self):
        super().__init__('waypoint_offboard_control') # Node name can be constant, instance name will be set by launch file

        # --- Parameter for Vehicle ID ---
        # Declare a parameter 'vehicle_id' which can be set on launch. Default is empty string.
        # This ID is used for topic namespaces and setting the target_system_id.
        self.declare_parameter('vehicle_id', '')
        vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().string_value

        # --- Dynamic Topic Naming and Set target_system_id based on vehicle_id---
        if vehicle_id:
            # If vehicle_id is provided (e.g., '1', '2'), use it for namespacing topics
            # and setting the target MAVLink system ID.
            # The swarm_spawner uses PX4 -i {MAV_SYS_ID}, where MAV_SYS_ID is 1, 2, 3...
            # The vehicle_id passed from launch will be '1', '2', '3'..., matching the MAV_SYS_ID.
            self.target_system_id = int(vehicle_id) + 1
            self.get_logger().info(f"--- Running for Vehicle ID: {vehicle_id}, Targeting MAV_SYS_ID: {self.target_system_id} ---")
            self.namespace = f"/px4_{vehicle_id}"
        else:
            # If no vehicle_id, default to single-vehicle mode, targeting MAV_SYS_ID=1.
            self.target_system_id = 1
            self.get_logger().info(f"--- Running in default single-vehicle mode, Targeting MAV_SYS_ID: {self.target_system_id} ---")
            self.namespace = ""
            
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Publishers ---
        # Publishers are created with dynamic topic names based on the namespace
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, f'{self.namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, f'{self.namespace}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, f"{self.namespace}/fmu/in/vehicle_command", 10)
        
        # --- Subscriptions ---
        # Subscriptions are created with dynamic topic names based on the namespace
        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'{self.namespace}/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)
        
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            f'{self.namespace}/fmu/out/vehicle_odometry', 
            self.odometry_callback,
            qos_profile)
        
        # --- Timers ---
        timer_period = 0.02  # seconds (50 Hz) for main control loop
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        arm_timer_period = .1 # seconds (10 Hz) for arming/mode switching FSM
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        self.dt = timer_period # Time step for general control loop

        # --- Parameters ---
        self.declare_parameter('acceptance_threshold', 0.5) 
        self.acceptance_threshold = self.get_parameter('acceptance_threshold').value

        # Waypoints as a JSON string. Coordinates are [x, y, z] in ENU (East, North, Up).
        self.declare_parameter('waypoints_json', '[[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 3.0, 1.0], [3.0, 3.0, 3.0], [6.0, 1.0, 10.0]]')
        waypoints_json_str = self.get_parameter('waypoints_json').value

        self.declare_parameter('loop_waypoints', False)
        self.loop_waypoints = self.get_parameter('loop_waypoints').value
        
        self.declare_parameter('takeoff_altitude', 5.0)
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value

        # Parse the JSON string into a Python list of lists for waypoints
        try:
            self.waypoints = json.loads(waypoints_json_str)
            # Basic validation
            if not isinstance(self.waypoints, list) or \
               not all(isinstance(wp, list) and len(wp) == 3 and all(isinstance(coord, (int, float)) for coord in wp) for wp in self.waypoints):
                raise ValueError("Waypoints JSON must be a list of [x, y, z] lists.")
            if not self.waypoints: 
                raise ValueError("Waypoints list cannot be empty.")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse waypoints_json: {e}. Ensure it's a valid JSON string. Defaulting to a single point at [0,0,0].")
            self.waypoints = [[0.0, 0.0, 0.0]] 
        except ValueError as e:
            self.get_logger().error(f"Invalid waypoints_json format: {e}. Defaulting to a single point at [0,0,0].")
            self.waypoints = [[0.0, 0.0, 0.0]]
        
        self.num_waypoints = len(self.waypoints)
        self.current_waypoint_index = 0 

        # --- Vehicle State Variables for Arming/Offboard FSM ---
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.flight_check_passed = False 
        self.failsafe_active = False 

        # Current position from odometry (NED coordinates)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0 

        # FSM state for arming and offboard mode management
        self.fsm_current_state = "IDLE"
        self.fsm_last_state = self.fsm_current_state
        self.fsm_counter = 0 
        
        self.get_logger().info(f"Waypoint Offboard Control initialized for MAV_SYS_ID {self.target_system_id}.")
        self.get_logger().info(f"Total waypoints: {self.num_waypoints}, Acceptance threshold: {self.acceptance_threshold}m.")
        self.get_logger().info(f"Loop waypoints: {self.loop_waypoints}")
        self.get_logger().info(f"Takeoff Altitude: {self.takeoff_altitude}m")
        self.get_logger().info(f"Parsed Waypoints (ENU): {self.waypoints}")


    def vehicle_status_callback(self, msg):
        """
        Callback for VehicleStatus messages.
        Updates the quadcopter's navigation and arming states, and pre-flight check status.
        """
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"MAV {self.target_system_id}: NAV_STATUS changed to: {msg.nav_state}")
        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"MAV {self.target_system_id}: ARM_STATUS changed to: {msg.arming_state}")
        if (msg.failsafe != self.failsafe_active):
            self.get_logger().info(f"MAV {self.target_system_id}: FAILSAFE status changed to: {msg.failsafe}")
        if (msg.pre_flight_checks_pass != self.flight_check_passed):
            self.get_logger().info(f"MAV {self.target_system_id}: Pre-Flight Checks Passed: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe_active = msg.failsafe
        self.flight_check_passed = msg.pre_flight_checks_pass

    def odometry_callback(self, msg):
        """
        Callback for VehicleOdometry messages.
        Updates the quadcopter's current position (x, y, z) in NED coordinates.
        """
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = msg.position[2] 

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """
        Publishes a VehicleCommand message to /fmu/in/vehicle_command.
        :param command: Command ID from px4_msgs.msg.VehicleCommand
        :param param1, param2, param7: Parameters for the command as defined in PX4 docs.
        """
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
        self.vehicle_command_publisher_.publish(msg)
        # self.get_logger().info(f"MAV {self.target_system_id}: Sent VehicleCommand: {command}") # Suppress for less spam

    def arm_vehicle(self):
        """Arms the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def takeoff_vehicle(self):
        """Takes off the vehicle to a user specified altitude (meters)."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=self.takeoff_altitude) 

    def set_offboard_mode(self):
        """Switches the vehicle to offboard control mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)

    def arm_timer_callback(self):
        """
        Finite State Machine (FSM) for handling arming, takeoff, and
        switching to offboard mode. This callback runs periodically.
        """
        if (self.fsm_last_state != self.fsm_current_state):
            self.get_logger().info(f"MAV {self.target_system_id} FSM State Transition: {self.fsm_last_state} -> {self.fsm_current_state}")
            self.fsm_last_state = self.fsm_current_state

        match self.fsm_current_state:
            case "IDLE":
                if self.flight_check_passed:
                    self.fsm_current_state = "ARMING"
                    self.fsm_counter = 0 

            case "ARMING":
                if not self.flight_check_passed or self.failsafe_active:
                    self.fsm_current_state = "IDLE"
                    self.get_logger().warn(f"MAV {self.target_system_id}: Arming aborted: Pre-flight checks failed or failsafe active.")
                elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.fsm_counter > 20: 
                    self.fsm_current_state = "TAKEOFF"
                    self.get_logger().info(f"MAV {self.target_system_id}: Vehicle armed. Proceeding to Takeoff.")
                else:
                    self.arm_vehicle() 
            
            case "TAKEOFF":
                if not self.flight_check_passed or self.failsafe_active:
                    self.fsm_current_state = "IDLE"
                    self.get_logger().warn(f"MAV {self.target_system_id}: Takeoff aborted: Pre-flight checks failed or failsafe active.")
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                    self.fsm_current_state = "LOITER"
                    self.get_logger().info(f"MAV {self.target_system_id}: Vehicle is taking off. Waiting for LOITER state.")
                else:
                    self.takeoff_vehicle() 
            
            case "LOITER": 
                if not self.flight_check_passed or self.failsafe_active:
                    self.fsm_current_state = "IDLE"
                    self.get_logger().warn(f"MAV {self.target_system_id}: Loiter aborted: Pre-flight checks failed or failsafe active.")
                elif self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.fsm_current_state = "OFFBOARD"
                    self.fsm_counter = 0 
                    self.get_logger().info(f"MAV {self.target_system_id}: Vehicle in Loiter. Attempting Offboard control.")
                self.arm_vehicle() 

            case "OFFBOARD":
                if not self.flight_check_passed or self.arm_state != VehicleStatus.ARMING_STATE_ARMED or self.failsafe_active:
                    self.fsm_current_state = "IDLE"
                    self.get_logger().warn(f"MAV {self.target_system_id}: Offboard mode exited: Conditions not met. Returning to IDLE.")
                else:
                    self.set_offboard_mode() 
        self.fsm_counter += 1 

    def cmdloop_callback(self):
        """
        Main control loop for publishing offboard control modes and trajectory setpoints.
        Handles waypoint progression and looping logic. Only active when in OFFBOARD mode.
        """
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and 
            self.arm_state == VehicleStatus.ARMING_STATE_ARMED):

            target_idx_for_command = self.current_waypoint_index

            if self.current_waypoint_index >= self.num_waypoints:
                if self.loop_waypoints:
                    self.current_waypoint_index = 0
                    target_idx_for_command = 0
                    self.get_logger().info(f"MAV {self.target_system_id}: All waypoints visited. Looping back to the first waypoint.", throttle_duration_sec=1)
                else:
                    self.current_waypoint_index = self.num_waypoints - 1 
                    target_idx_for_command = self.num_waypoints - 1
                    self.get_logger().info(f"MAV {self.target_system_id}: All waypoints visited. Hovering at the last waypoint.", throttle_duration_sec=5)

            target_waypoint_enu = self.waypoints[target_idx_for_command]

            # --- Perform ENU to NED Conversion for Target Waypoint ---
            # X_NED = Y_ENU (North in NED is Y in ENU)
            # Y_NED = X_ENU (East in NED is X in ENU)
            # Z_NED = -Z_ENU (Down in NED is negative Up in ENU)
            target_x_ned = target_waypoint_enu[1] # Y_ENU -> X_NED (North)
            target_y_ned = target_waypoint_enu[0] # X_ENU -> Y_NED (East)
            target_z_ned = -target_waypoint_enu[2] # Z_ENU (Up) -> Z_NED (Down)

            # --- Calculate Distance to Waypoint (in NED coordinates) ---
            distance_to_waypoint = np.sqrt(
                (self.current_x - target_x_ned)**2 + 
                (self.current_y - target_y_ned)**2 + 
                (self.current_z - target_z_ned)**2             
            )

            if distance_to_waypoint < self.acceptance_threshold:
                self.get_logger().info(f"MAV {self.target_system_id}: Waypoint {self.current_waypoint_index} reached. Distance: {distance_to_waypoint:.2f}m. Preparing for next waypoint.")
                
                self.current_waypoint_index += 1 
                
                if self.current_waypoint_index >= self.num_waypoints:
                    if self.loop_waypoints:
                        self.current_waypoint_index = 0 
                        self.get_logger().info(f"MAV {self.target_system_id}: Starting new loop cycle.")
                    else:
                        self.current_waypoint_index = self.num_waypoints - 1 
                        self.get_logger().info(f"MAV {self.target_system_id}: All waypoints completed. Maintaining hover at final position.")
           
            # --- Publish Trajectory Setpoint (in NED coordinates) ---
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            
            trajectory_msg.position[0] = float(target_x_ned) # X_NED (North)
            trajectory_msg.position[1] = float(target_y_ned) # Y_NED (East)
            trajectory_msg.position[2] = float(target_z_ned) # Z_NED (Down)

            self.publisher_trajectory.publish(trajectory_msg)

        else:
            self.get_logger().info(f"MAV {self.target_system_id}: Waiting for Offboard mode and Armed state (Main loop). Current NAV: {self.nav_state}, ARMING: {self.arm_state}", throttle_duration_sec=5)


def main(args=None):
    rclpy.init(args=args)
    waypoint_offboard_control = WaypointOffboardControl()
    rclpy.spin(waypoint_offboard_control)
    waypoint_offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
