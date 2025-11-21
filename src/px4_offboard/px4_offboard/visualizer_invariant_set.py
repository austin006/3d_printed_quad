#!/usr/bin/env python

from re import M
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

def vector2PoseMsg(frame_id, position, attitude):
    pose_msg = PoseStamped()
    # msg.header.stamp = Clock().now().nanoseconds / 1000
    pose_msg.header.frame_id=frame_id
    pose_msg.pose.orientation.w = attitude[0]
    pose_msg.pose.orientation.x = attitude[1]
    pose_msg.pose.orientation.y = attitude[2]
    pose_msg.pose.orientation.z = attitude[3]
    pose_msg.pose.position.x = position[0]
    pose_msg.pose.position.y = position[1]
    pose_msg.pose.position.z = position[2]
    return pose_msg

class PX4Visualizer(Node):
    def __init__(self):
        super().__init__('px4_visualizer')
        self.get_logger().info("PX4 Visualizer Node has been started.")

        # Declare parameter for epsilon (invariant set calculation)
        self.declare_parameter('epsilon', -0.05)
        self.epsilon: float = float(self.get_parameter('epsilon').value)

        ## Configure subscriptions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback,
            qos_profile)
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.trajectory_setpoint_callback,
            qos_profile)
        
        # Subscribe to lidar data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10)

        self.vehicle_pose_pub = self.create_publisher(PoseStamped, '/px4_visualizer/vehicle_pose', 10)
        self.vehicle_vel_pub = self.create_publisher(Marker, '/px4_visualizer/vehicle_velocity', 10)
        self.vehicle_path_pub = self.create_publisher(Path, '/px4_visualizer/vehicle_path', 10)
        self.setpoint_path_pub = self.create_publisher(Path, '/px4_visualizer/setpoint_path', 10)
        
        # Publisher for transformed lidar scan
        self.lidar_pub = self.create_publisher(LaserScan, '/px4_visualizer/lidar_scan', 10)
        
        # Publisher for invariant set markers
        self.invariant_set_pub = self.create_publisher(MarkerArray, '/px4_visualizer/invariant_set_markers', 10)
        
        # TF broadcaster for lidar frame
        self.tf_broadcaster = TransformBroadcaster(self)

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_path_msg = Path()
        self.setpoint_path_msg = Path()
        self.latest_lidar_msg = None
        self.safe_boundary_points = []  # Store calculated boundary points
        
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

    def vehicle_attitude_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]

    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vx
        self.vehicle_local_velocity[1] = -msg.vy
        self.vehicle_local_velocity[2] = -msg.vz

    def trajectory_setpoint_callback(self, msg):
        self.setpoint_position[0] = msg.position[0]
        self.setpoint_position[1] = -msg.position[1]
        self.setpoint_position[2] = -msg.position[2]

    def lidar_callback(self, msg):
        # Store the latest lidar message
        self.latest_lidar_msg = msg
        
        # Calculate invariant set (but don't publish yet - wait for cmdloop)
        self.calculate_invariant_set(msg)

    def calculate_invariant_set(self, msg: LaserScan):
        """Calculate the invariant set boundary from lidar data"""
        distances = np.array(msg.ranges)
        thetas = np.linspace(msg.angle_min, msg.angle_max, len(distances))

        # Filter out invalid readings
        valid_mask = np.isfinite(distances) & (distances >= msg.range_min) & (distances <= msg.range_max)
        distances = distances[valid_mask]
        thetas = thetas[valid_mask]

        if len(distances) == 0:
            self.get_logger().warn('No valid lidar data received.')
            return

        xs = distances * np.cos(thetas)
        ys = distances * np.sin(thetas)

        data_vectors = np.vstack([xs, ys])

        valid_ps = data_vectors.copy()
        valid_distances = distances.copy()

        safe_boundary_points = []

        while valid_ps.shape[1] > 0:
            min_idx = np.argmin(valid_distances)
            p_closest = valid_ps[:, min_idx].reshape((-1, 1))

            norm = np.linalg.norm(p_closest)
            if norm < 1e-6:
                self.get_logger().warn('Skipping zero-length vector')
                mask_temp = np.ones(valid_ps.shape[1], dtype=bool)
                mask_temp[min_idx] = False
                valid_ps = valid_ps[:, mask_temp]
                valid_distances = valid_distances[mask_temp]
                continue

            a_hat = p_closest / norm

            parallel_vector = np.array([[-a_hat[1, 0]], [a_hat[0, 0]]])
            safe_boundary_points.append((p_closest.copy(), parallel_vector.copy()))

            b = np.dot(a_hat.T, p_closest) + self.epsilon

            safe_region_mask = (valid_ps.T @ a_hat < b).flatten()
            safe_region_mask[min_idx] = False

            valid_ps = valid_ps[:, safe_region_mask]
            valid_distances = valid_distances[safe_region_mask]

        self.get_logger().info(f'Generated {len(safe_boundary_points)} boundary lines')
        # Store the boundary points instead of publishing immediately
        self.safe_boundary_points = safe_boundary_points

    def publish_invariant_set_markers(self, safe_boundary_points, timestamp):
        """Publish the invariant set boundary as markers for RViz"""
        marker = Marker()
        marker.header.stamp = timestamp
        marker.header.frame_id = 'lidar_frame'  # Use lidar_frame for proper transformation
        marker.ns = "invariant_set"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02  # Line thickness
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for point, slope in safe_boundary_points:
            if np.any(np.isnan(point)) or np.any(np.isnan(slope)):
                self.get_logger().warn('Skipping NaN boundary point')
                continue

            temp1 = point + slope * 10
            temp2 = point - slope * 10

            point1 = Point()
            point1.x = float(temp1[0, 0])
            point1.y = float(temp1[1, 0])
            point1.z = 0.0

            point2 = Point()
            point2.x = float(temp2[0, 0])
            point2.y = float(temp2[1, 0])
            point2.z = 0.0

            marker.points.append(point1)
            marker.points.append(point2)

        # self.get_logger().info(f'Publishing invariant set marker with {len(marker.points)} points')

        markers = MarkerArray()
        markers.markers.append(marker)
        self.invariant_set_pub.publish(markers)

    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = 'map'
        # msg.header.stamp = Clock().now().nanoseconds / 1000
        msg.ns = 'arrow'
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1
        msg.scale.y = 0.2
        msg.scale.z = 0.0
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0
        dt = 0.3
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
        vehicle_pose_msg = vector2PoseMsg('map', self.vehicle_local_position, self.vehicle_attitude)
        self.vehicle_pose_pub.publish(vehicle_pose_msg)

        # Publish time history of the vehicle path
        self.vehicle_path_msg.header = vehicle_pose_msg.header
        self.vehicle_path_msg.poses.append(vehicle_pose_msg) 
        self.vehicle_path_pub.publish(self.vehicle_path_msg)

        # Publish time history of the vehicle path
        setpoint_pose_msg = vector2PoseMsg('map', self.setpoint_position, self.vehicle_attitude)
        self.setpoint_path_msg.header = setpoint_pose_msg.header
        self.setpoint_path_msg.poses.append(setpoint_pose_msg)
        self.setpoint_path_pub.publish(self.setpoint_path_msg)

        # Publish arrow markers for velocity
        velocity_msg = self.create_arrow_marker(1, self.vehicle_local_position, self.vehicle_local_velocity)
        self.vehicle_vel_pub.publish(velocity_msg)
        
        # Get current timestamp for TF and markers
        current_time = self.get_clock().now().to_msg()
        
        # Broadcast TF transform for lidar frame
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'map'
        t.child_frame_id = 'lidar_frame'
        t.transform.translation.x = self.vehicle_local_position[0]
        t.transform.translation.y = self.vehicle_local_position[1]
        t.transform.translation.z = self.vehicle_local_position[2]
        t.transform.rotation.w = self.vehicle_attitude[0]
        t.transform.rotation.x = self.vehicle_attitude[1]
        t.transform.rotation.y = self.vehicle_attitude[2]
        t.transform.rotation.z = self.vehicle_attitude[3]
        self.tf_broadcaster.sendTransform(t)
        
        # Republish lidar scan with updated frame
        if self.latest_lidar_msg is not None:
            lidar_msg = LaserScan()
            lidar_msg.header.stamp = current_time
            lidar_msg.header.frame_id = 'lidar_frame'
            lidar_msg.angle_min = self.latest_lidar_msg.angle_min
            lidar_msg.angle_max = self.latest_lidar_msg.angle_max
            lidar_msg.angle_increment = self.latest_lidar_msg.angle_increment
            lidar_msg.time_increment = self.latest_lidar_msg.time_increment
            lidar_msg.scan_time = self.latest_lidar_msg.scan_time
            lidar_msg.range_min = self.latest_lidar_msg.range_min
            lidar_msg.range_max = self.latest_lidar_msg.range_max
            lidar_msg.ranges = self.latest_lidar_msg.ranges
            lidar_msg.intensities = self.latest_lidar_msg.intensities
            self.lidar_pub.publish(lidar_msg)
        
        # Publish invariant set markers synchronized with TF
        if len(self.safe_boundary_points) > 0:
            self.publish_invariant_set_markers(self.safe_boundary_points, current_time)

def main(args=None):
    rclpy.init(args=args)

    px4_visualizer = PX4Visualizer()

    rclpy.spin(px4_visualizer)

    px4_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()