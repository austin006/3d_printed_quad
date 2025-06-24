#!/usr/bin/env python3

import numpy as np
import functools # Import functools for partial application
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
# Import the Time message type for accurate timestamp handling
from builtin_interfaces.msg import Time

# Helper function to convert position and attitude to PoseStamped message
# The timestamp_ros_msg argument should be a builtin_interfaces.msg.Time object
def vector2PoseMsg(frame_id: str, timestamp_ros_msg: Time, position: np.ndarray, attitude: np.ndarray) -> PoseStamped:
    pose_msg = PoseStamped()
    # Use the provided ROS message timestamp directly
    pose_msg.header.stamp = timestamp_ros_msg
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.orientation.w = attitude[0]
    pose_msg.pose.orientation.x = attitude[1]
    pose_msg.pose.orientation.y = attitude[2]
    pose_msg.pose.orientation.z = attitude[3]
    pose_msg.pose.position.x = position[0]
    pose_msg.pose.position.y = position[1]
    pose_msg.pose.position.z = position[2]
    return pose_msg

class PX4Visualizer(Node):
    """
    ROS2 Node for visualizing multiple PX4 quadcopters in RViz.
    It subscribes to namespaced PX4 topics and publishes visualization messages
    for each vehicle (pose, velocity, and paths).
    """
    def __init__(self):
        super().__init__('px4_visualizer')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Declare and get the number of vehicles to visualize
        self.declare_parameter('num_vehicles', 1)
        self.num_vehicles = self.get_parameter('num_vehicles').value
        
        self.get_logger().info(f"PX4 Visualizer initialized for {self.num_vehicles} vehicles.")

        # Dictionaries to store state and publishers/subscribers for each vehicle
        self.vehicle_attitudes = {}
        self.vehicle_local_positions = {}
        self.vehicle_local_velocities = {}
        self.setpoint_positions = {}
        self.vehicle_path_msgs = {}
        self.setpoint_path_msgs = {}

        self.vehicle_pose_pubs = {}
        self.vehicle_vel_pubs = {}
        self.vehicle_path_pubs = {}
        self.setpoint_path_pubs = {}

        # Loop to create subscribers and publishers for each vehicle
        for i in range(1, self.num_vehicles + 1):
            vehicle_id = str(i)
            namespace = f"/px4_{vehicle_id}"

            # Initialize state for this vehicle
            self.vehicle_attitudes[vehicle_id] = np.array([1.0, 0.0, 0.0, 0.0]) # w, x, y, z
            self.vehicle_local_positions[vehicle_id] = np.array([0.0, 0.0, 0.0]) # x, y, z
            self.vehicle_local_velocities[vehicle_id] = np.array([0.0, 0.0, 0.0]) # vx, vy, vz
            self.setpoint_positions[vehicle_id] = np.array([0.0, 0.0, 0.0]) # x, y, z

            # Initialize Path messages for this vehicle
            self.vehicle_path_msgs[vehicle_id] = Path()
            self.vehicle_path_msgs[vehicle_id].header.frame_id = 'map'
            self.setpoint_path_msgs[vehicle_id] = Path()
            self.setpoint_path_msgs[vehicle_id].header.frame_id = 'map'

            # Create Subscribers for this vehicle
            # Use functools.partial to pass the vehicle_id to the callback
            self.create_subscription(
                VehicleAttitude,
                f'{namespace}/fmu/out/vehicle_attitude',
                functools.partial(self.vehicle_attitude_callback, vehicle_id=vehicle_id),
                qos_profile
            )
            self.create_subscription(
                VehicleLocalPosition,
                f'{namespace}/fmu/out/vehicle_local_position',
                functools.partial(self.vehicle_local_position_callback, vehicle_id=vehicle_id),
                qos_profile
            )
            self.create_subscription(
                TrajectorySetpoint,
                f'{namespace}/fmu/in/trajectory_setpoint',
                functools.partial(self.trajectory_setpoint_callback, vehicle_id=vehicle_id),
                qos_profile
            )

            # Create Publishers for this vehicle's visualization topics
            self.vehicle_pose_pubs[vehicle_id] = self.create_publisher(PoseStamped, f'/px4_visualizer/vehicle_{vehicle_id}/pose', 10)
            self.vehicle_vel_pubs[vehicle_id] = self.create_publisher(Marker, f'/px4_visualizer/vehicle_{vehicle_id}/velocity', 10)
            self.vehicle_path_pubs[vehicle_id] = self.create_publisher(Path, f'/px4_visualizer/vehicle_{vehicle_id}/path', 10)
            self.setpoint_path_pubs[vehicle_id] = self.create_publisher(Path, f'/px4_visualizer/vehicle_{vehicle_id}/setpoint_path', 10)

        # Timer for the main visualization loop
        timer_period = 0.05  # seconds (20 Hz)
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

    def vehicle_attitude_callback(self, msg, vehicle_id):
        """Callback for VehicleAttitude messages for a specific vehicle."""
        # Convert PX4's NED quaternion (w, x, y, z) to ROS ENU (w, x, -y, -z)
        # Assuming original is FLU (Front, Left, Up) and then converted to NED:
        # qx -> x, qy -> y, qz -> z, qw -> w.
        # PX4 is typically ENU attitude in its VehicleAttitude message,
        # but the local position frame (x,y,z) is NED.
        # Let's verify and apply necessary transformations.
        # Common PX4 NED to ROS ENU attitude conversion:
        # q_enu.w = q_ned.w
        # q_enu.x = q_ned.x
        # q_enu.y = -q_ned.y
        # q_enu.z = -q_ned.z
        self.vehicle_attitudes[vehicle_id][0] = msg.q[0] # w
        self.vehicle_attitudes[vehicle_id][1] = msg.q[1] # x
        self.vehicle_attitudes[vehicle_id][2] = -msg.q[2] # -y
        self.vehicle_attitudes[vehicle_id][3] = -msg.q[3] # -z

    def vehicle_local_position_callback(self, msg, vehicle_id):
        """Callback for VehicleLocalPosition messages for a specific vehicle."""
        # Convert PX4's NED (North, East, Down) position/velocity to ROS ENU (East, North, Up)
        # x_ENU = y_NED (East)
        # y_ENU = x_NED (North)
        # z_ENU = -z_NED (Up)

        self.vehicle_local_positions[vehicle_id][0] = msg.y # NED East -> ENU X
        self.vehicle_local_positions[vehicle_id][1] = msg.x # NED North -> ENU Y
        self.vehicle_local_positions[vehicle_id][2] = -msg.z # NED Down -> ENU Z (altitude)

        self.vehicle_local_velocities[vehicle_id][0] = msg.vy # NED East vel -> ENU X vel
        self.vehicle_local_velocities[vehicle_id][1] = msg.vx # NED North vel -> ENU Y vel
        self.vehicle_local_velocities[vehicle_id][2] = -msg.vz # NED Down vel -> ENU Z vel (climb rate)

    def trajectory_setpoint_callback(self, msg, vehicle_id):
        """Callback for TrajectorySetpoint messages for a specific vehicle.
        Converts TrajectorySetpoint (which is in NED from waypoint_offboard_control)
        to ENU for RViz visualization.
        """
        # The waypoint_offboard_control node converts ENU waypoints to NED
        # before publishing them in TrajectorySetpoint.
        # Therefore, msg.position[0] is NED_X (North), msg.position[1] is NED_Y (East),
        # and msg.position[2] is NED_Z (Down).
        # We need to convert these back to ENU for RViz visualization.

        # ENU X = NED Y
        self.setpoint_positions[vehicle_id][0] = msg.position[1] 
        # ENU Y = NED X
        self.setpoint_positions[vehicle_id][1] = msg.position[0]
        # ENU Z = -NED Z
        self.setpoint_positions[vehicle_id][2] = -msg.position[2] 
    def create_arrow_marker(self, id_base: int, current_timestamp_ros_msg: Time, tail: np.ndarray, vector: np.ndarray, vehicle_id: str) -> Marker:
        """
        Creates a visualization_msgs.msg.Marker of type ARROW.
        id_base is an integer that will be combined with vehicle_id for uniqueness.
        current_timestamp_ros_msg should be a builtin_interfaces.msg.Time object.
        """
        msg = Marker()
        msg.header.stamp = current_timestamp_ros_msg # Use the provided ROS message timestamp
        msg.header.frame_id = 'map'
        msg.ns = f'velocity_arrow_{vehicle_id}' # Unique namespace per vehicle
        msg.id = id_base # Marker ID (unique within its namespace)
        msg.type = Marker.ARROW
        msg.action = Marker.ADD

        # Scale for the arrow
        msg.scale.x = 0.1 # Shaft diameter
        msg.scale.y = 0.2 # Head diameter
        msg.scale.z = 0.0 # Not used for arrow but required

        # Color (e.g., yellow for velocity)
        msg.color.r = 1.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.color.a = 1.0 # Alpha (opacity)

        # Points for the arrow (tail and head)
        dt = 0.3 # Scale the velocity vector for visualization length
        tail_point = Point()
        tail_point.x = tail[0]
        tail_point.y = tail[1]
        tail_point.z = tail[2]
        head_point = Point()
        head_point.x = tail[0] + dt * vector[0]
        head_point.y = tail[1] + dt * vector[1]
        head_point.z = tail[2] + dt * vector[2]
        msg.points = [tail_point, head_point]
        return msg

    def cmdloop_callback(self):
        """
        Main loop for publishing visualization data for all vehicles.
        Runs periodically and updates RViz.
        """
        current_time = self.get_clock().now() # Get current time as rclpy.time.Time object
        current_time_ros_msg = current_time.to_msg() # Convert to builtin_interfaces.msg.Time

        for vehicle_id in range(1, self.num_vehicles + 1):
            vehicle_id_str = str(vehicle_id)

            # Ensure data exists for this vehicle before publishing
            if vehicle_id_str not in self.vehicle_local_positions:
                self.get_logger().warn(f"No data for vehicle {vehicle_id_str} yet. Skipping visualization.", throttle_duration_sec=5)
                continue

            # --- Publish Vehicle Pose ---
            # Pass the correctly converted ROS message timestamp
            vehicle_pose_msg = vector2PoseMsg(
                'map', 
                current_time_ros_msg, 
                self.vehicle_local_positions[vehicle_id_str], 
                self.vehicle_attitudes[vehicle_id_str]
            )
            self.vehicle_pose_pubs[vehicle_id_str].publish(vehicle_pose_msg)

            # --- Publish Vehicle Path ---
            # Append current pose to the path and publish
            # Ensure the path header's timestamp is also updated correctly
            self.vehicle_path_msgs[vehicle_id_str].header.stamp = current_time_ros_msg
            self.vehicle_path_msgs[vehicle_id_str].poses.append(vehicle_pose_msg) 
            self.vehicle_path_pubs[vehicle_id_str].publish(self.vehicle_path_msgs[vehicle_id_str])

            # --- Publish Setpoint Path ---
            # Create a PoseStamped for the setpoint position (attitude can be arbitrary, use vehicle's current)
            # Pass the correctly converted ROS message timestamp
            setpoint_pose_msg = vector2PoseMsg(
                'map', 
                current_time_ros_msg, 
                self.setpoint_positions[vehicle_id_str], 
                self.vehicle_attitudes[vehicle_id_str] # Use current drone attitude for setpoint visualization
            )
            # Ensure the setpoint path header's timestamp is also updated correctly
            self.setpoint_path_msgs[vehicle_id_str].header.stamp = current_time_ros_msg
            self.setpoint_path_msgs[vehicle_id_str].poses.append(setpoint_pose_msg)
            self.setpoint_path_pubs[vehicle_id_str].publish(self.setpoint_path_msgs[vehicle_id_str])

            # --- Publish Vehicle Velocity Arrow Marker ---
            # Use vehicle_id as part of the marker ID to ensure uniqueness across drones
            velocity_marker_id = vehicle_id # Use the integer ID directly for marker ID
            # Pass the correctly converted ROS message timestamp
            velocity_msg = self.create_arrow_marker(
                velocity_marker_id, 
                current_time_ros_msg, 
                self.vehicle_local_positions[vehicle_id_str], 
                self.vehicle_local_velocities[vehicle_id_str],
                vehicle_id_str # Pass vehicle_id_str for namespace
            )
            self.vehicle_vel_pubs[vehicle_id_str].publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)

    px4_visualizer = PX4Visualizer()

    rclpy.spin(px4_visualizer)

    px4_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
