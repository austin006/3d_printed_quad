#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus


class SquareOffboardControl(Node):
    """
    ROS2 node to control a quadcopter to fly in a square trajectory.
    It publishes OffboardControlMode and TrajectorySetpoint messages.
    """

    def __init__(self):
        super().__init__('square_offboard_control') # Renamed the node
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)
        
        # Publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # Timer for the control loop
        timer_period = 0.02  # seconds (50 Hz)
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period # Time step

        # Declare parameters for square trajectory
        self.declare_parameter('side_length', 10.0) # Side length of the square in meters
        self.declare_parameter('speed', 1.0)       # Speed along each side in m/s
        self.declare_parameter('altitude', 5.0)    # Altitude of the square path in meters

        # Get parameter values
        self.side_length = self.get_parameter('side_length').value
        self.speed = self.get_parameter('speed').value
        self.altitude = self.get_parameter('altitude').value

        # Initialize navigation and arming states
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        # Square trajectory state variables
        # Waypoints defining the corners of the square (x, y) relative to (0,0)
        # Starting from top-right, moving counter-clockwise
        self.waypoints = [
            (self.side_length / 2.0, self.side_length / 2.0),   # P0 (Top Right)
            (-self.side_length / 2.0, self.side_length / 2.0),  # P1 (Top Left)
            (-self.side_length / 2.0, -self.side_length / 2.0), # P2 (Bottom Left)
            (self.side_length / 2.0, -self.side_length / 2.0)    # P3 (Bottom Right)
        ]
        self.num_waypoints = len(self.waypoints)
        self.current_waypoint_index = 0 # Index of the current target waypoint in self.waypoints
        self.segment_time_elapsed = 0.0 # Time elapsed since starting the current segment
        
        # Time required to traverse one side of the square
        self.time_per_segment = self.side_length / self.speed if self.speed > 0 else 1.0 # Avoid division by zero

        self.get_logger().info(f"Square trajectory parameters: side_length={self.side_length}, speed={self.speed}, altitude={self.altitude}")
        self.get_logger().info(f"Time per segment: {self.time_per_segment:.2f} seconds")


    def vehicle_status_callback(self, msg):
        """Callback for vehicle status messages."""
        self.get_logger().info(f"NAV_STATUS: {msg.nav_state}, ARMING_STATE: {msg.arming_state}", throttle_duration_sec=5)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def cmdloop_callback(self):
        """
        Main control loop for publishing offboard control modes and trajectory setpoints.
        Generates a square trajectory.
        """
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # Only publish trajectory setpoints if in offboard mode and armed
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and 
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            # Increment time elapsed for the current segment
            self.segment_time_elapsed += self.dt

            # Check if current segment is complete
            if self.segment_time_elapsed >= self.time_per_segment:
                self.segment_time_elapsed = 0.0 # Reset time for the new segment
                self.current_waypoint_index = (self.current_waypoint_index + 1) % self.num_waypoints
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}. Starting new segment.")


            # Determine the start and end points for the current segment
            start_point_idx = (self.current_waypoint_index - 1 + self.num_waypoints) % self.num_waypoints
            start_point = self.waypoints[start_point_idx]
            end_point = self.waypoints[self.current_waypoint_index]

            # Calculate interpolation factor for current position along the segment
            interp_factor = self.segment_time_elapsed / self.time_per_segment

            # Linearly interpolate between the start and end points
            target_x = start_point[0] + (end_point[0] - start_point[0]) * interp_factor
            target_y = start_point[1] + (end_point[1] - start_point[1]) * interp_factor
            
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = target_x
            trajectory_msg.position[1] = target_y
            trajectory_msg.position[2] = -self.altitude # PX4 uses NED convention, so negative for altitude
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            self.publisher_trajectory.publish(trajectory_msg)

            # self.get_logger().info(f"Target position: x={target_x:.2f}, y={target_y:.2f}, z={-self.altitude:.2f}", throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)

    square_offboard_control = SquareOffboardControl()

    rclpy.spin(square_offboard_control)

    square_offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()