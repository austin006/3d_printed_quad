#!/usr/bin/env python

import numpy as np
from scipy.optimize import minimize
import cvxpy as cp

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class CBFSafeController(Node):
    def __init__(self):
        super().__init__('cbf_safe_controller')
        self.get_logger().info('CBF Safe Controller Node has been started.')

        # Parameters
        self.declare_parameter('alpha', 1.0)  # CBF class-K function parameter
        self.declare_parameter('epsilon', 0.3)  # Safety margin (meters)
        self.declare_parameter('max_velocity', 2.0)  # Max velocity (m/s)
        self.declare_parameter('control_dt', 0.05)  # Control loop period
        
        self.alpha = float(self.get_parameter('alpha').value)
        self.epsilon = float(self.get_parameter('epsilon').value)
        self.max_velocity = float(self.get_parameter('max_velocity').value)
        self.control_dt = float(self.get_parameter('control_dt').value)

        # QoS profile for PX4
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile)
        
        self.desired_setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint_raw',  # User's desired (unsafe) trajectory
            self.desired_setpoint_callback,
            qos_profile)
        
        self.invariant_set_sub = self.create_subscription(
            MarkerArray,
            '/px4_visualizer/invariant_set_markers',
            self.invariant_set_callback,
            10)

        # Publishers
        self.safe_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',  # Safe filtered trajectory
            qos_profile)
        
        self.cbf_viz_pub = self.create_publisher(
            MarkerArray,
            '/cbf_controller/visualization',
            10)

        # State variables
        self.current_position = np.array([0.0, 0.0, 0.0])  # NED frame
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.desired_setpoint = None
        self.barrier_constraints = []  # List of (a_hat, p_closest) tuples
        
        # Control loop timer
        self.timer = self.create_timer(self.control_dt, self.control_loop)
        
        self.get_logger().info(f'CBF parameters: alpha={self.alpha}, epsilon={self.epsilon}')

    def position_callback(self, msg):
        """Update current position and velocity"""
        self.current_position = np.array([msg.x, msg.y, msg.z])
        self.current_velocity = np.array([msg.vx, msg.vy, msg.vz])

    def desired_setpoint_callback(self, msg):
        """Receive desired (potentially unsafe) trajectory"""
        self.desired_setpoint = msg

    def invariant_set_callback(self, msg: MarkerArray):
        """Extract barrier function constraints from invariant set visualization"""
        if len(msg.markers) == 0:
            return
        
        marker = msg.markers[0]
        if marker.type != Marker.LINE_LIST or len(marker.points) < 2:
            return
        
        # Extract line segments and convert to barrier constraints
        self.barrier_constraints = []
        
        for i in range(0, len(marker.points), 2):
            if i + 1 >= len(marker.points):
                break
            
            p1 = np.array([marker.points[i].x, marker.points[i].y])
            p2 = np.array([marker.points[i+1].x, marker.points[i+1].y])
            
            # Compute line direction and normal
            line_vec = p2 - p1
            line_length = np.linalg.norm(line_vec)
            
            if line_length < 1e-6:
                continue
            
            # Normal vector pointing inward (toward robot at origin)
            normal = np.array([-line_vec[1], line_vec[0]]) / line_length
            
            # Use midpoint of line segment
            p_mid = (p1 + p2) / 2
            
            # Ensure normal points toward origin (safe side)
            if np.dot(normal, -p_mid) < 0:
                normal = -normal
            
            self.barrier_constraints.append((normal, p_mid))
        
        self.get_logger().info(f'Updated {len(self.barrier_constraints)} barrier constraints', 
                              throttle_duration_sec=1.0)

    def compute_barrier_function(self, position_2d, a_hat, p_closest):
        """
        Compute barrier function value h(x)
        h(x) > 0 means safe
        h(x) = 0 means on boundary
        h(x) < 0 means unsafe
        """
        return np.dot(a_hat, position_2d - p_closest) - self.epsilon

    def compute_barrier_derivative(self, velocity_2d, a_hat):
        """
        Compute time derivative of barrier function
        ḣ(x) = ∇h · ẋ = a_hat^T · v
        """
        return np.dot(a_hat, velocity_2d)

    def solve_cbf_qp(self, desired_velocity_2d, current_position_2d, current_velocity_2d):
        """
        Solve the CBF-QP to find safe control input
        
        minimize    ||v - v_desired||²
        subject to  ḣ_i(x) + α·h_i(x) ≥ 0  for all barriers i
                    ||v|| ≤ v_max
        """
        if len(self.barrier_constraints) == 0:
            # No constraints, return desired velocity
            return desired_velocity_2d
        
        # Use CVXPY for quadratic programming
        v = cp.Variable(2)
        
        # Objective: minimize distance to desired velocity
        objective = cp.Minimize(cp.sum_squares(v - desired_velocity_2d))
        
        # Constraints
        constraints = []
        
        # CBF constraints for each barrier
        for a_hat, p_closest in self.barrier_constraints:
            h = self.compute_barrier_function(current_position_2d, a_hat, p_closest)
            
            # CBF constraint: a_hat^T · v + α·h ≥ 0
            # This ensures ḣ + α·h ≥ 0
            constraints.append(a_hat @ v + self.alpha * h >= 0)
        
        # Velocity magnitude constraint
        constraints.append(cp.norm(v, 2) <= self.max_velocity)
        
        # Solve
        problem = cp.Problem(objective, constraints)
        
        try:
            problem.solve(solver=cp.OSQP, verbose=False)
            
            if problem.status == cp.OPTIMAL:
                return v.value
            else:
                self.get_logger().warn(f'QP solver status: {problem.status}')
                return current_velocity_2d  # Fallback: maintain current velocity
        
        except Exception as e:
            self.get_logger().error(f'QP solver failed: {e}')
            return current_velocity_2d

    def control_loop(self):
        """Main control loop: filter desired setpoint through CBF-QP"""
        if self.desired_setpoint is None:
            return
        
        # Extract desired position (NED frame from PX4)
        desired_pos = np.array([
            self.desired_setpoint.position[0],
            self.desired_setpoint.position[1],
            self.desired_setpoint.position[2]
        ])
        
        # Compute desired velocity (simple proportional controller)
        position_error = desired_pos - self.current_position
        desired_velocity_3d = position_error / self.control_dt
        
        # Limit desired velocity
        desired_speed = np.linalg.norm(desired_velocity_3d)
        if desired_speed > self.max_velocity:
            desired_velocity_3d = desired_velocity_3d / desired_speed * self.max_velocity
        
        # Work in 2D (x-y plane) for now
        current_position_2d = self.current_position[:2]
        current_velocity_2d = self.current_velocity[:2]
        desired_velocity_2d = desired_velocity_3d[:2]
        
        # Solve CBF-QP to get safe velocity
        safe_velocity_2d = self.solve_cbf_qp(
            desired_velocity_2d,
            current_position_2d,
            current_velocity_2d
        )
        
        # Compute safe position setpoint
        safe_position_3d = self.current_position.copy()
        safe_position_3d[:2] += safe_velocity_2d * self.control_dt
        safe_position_3d[2] = desired_pos[2]  # Keep desired altitude
        
        # Publish safe setpoint
        safe_msg = TrajectorySetpoint()
        safe_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        safe_msg.position = [safe_position_3d[0], safe_position_3d[1], safe_position_3d[2]]
        safe_msg.velocity = [safe_velocity_2d[0], safe_velocity_2d[1], desired_velocity_3d[2]]
        
        self.safe_setpoint_pub.publish(safe_msg)
        
        # Visualize CBF status
        self.publish_cbf_visualization(
            desired_velocity_2d,
            safe_velocity_2d,
            current_position_2d
        )

    def publish_cbf_visualization(self, desired_vel, safe_vel, position):
        """Visualize desired vs safe velocity and barrier function values"""
        markers = MarkerArray()
        
        # Desired velocity arrow (blue)
        desired_marker = Marker()
        desired_marker.header.frame_id = 'map'
        desired_marker.header.stamp = self.get_clock().now().to_msg()
        desired_marker.ns = 'cbf_desired'
        desired_marker.id = 0
        desired_marker.type = Marker.ARROW
        desired_marker.action = Marker.ADD
        desired_marker.scale.x = 0.1
        desired_marker.scale.y = 0.2
        desired_marker.scale.z = 0.0
        desired_marker.color.r = 0.0
        desired_marker.color.g = 0.0
        desired_marker.color.b = 1.0
        desired_marker.color.a = 0.7
        
        # Convert NED to ENU for visualization
        pos_enu = np.array([position[0], -position[1], 0.0])
        vel_enu = np.array([desired_vel[0], -desired_vel[1], 0.0])
        
        from geometry_msgs.msg import Point
        tail = Point()
        tail.x, tail.y, tail.z = pos_enu[0], pos_enu[1], pos_enu[2]
        head = Point()
        head.x = pos_enu[0] + vel_enu[0] * 0.5
        head.y = pos_enu[1] + vel_enu[1] * 0.5
        head.z = pos_enu[2] + vel_enu[2] * 0.5
        desired_marker.points = [tail, head]
        
        markers.markers.append(desired_marker)
        
        # Safe velocity arrow (green)
        safe_marker = Marker()
        safe_marker.header.frame_id = 'map'
        safe_marker.header.stamp = self.get_clock().now().to_msg()
        safe_marker.ns = 'cbf_safe'
        safe_marker.id = 1
        safe_marker.type = Marker.ARROW
        safe_marker.action = Marker.ADD
        safe_marker.scale.x = 0.15
        safe_marker.scale.y = 0.25
        safe_marker.scale.z = 0.0
        safe_marker.color.r = 0.0
        safe_marker.color.g = 1.0
        safe_marker.color.b = 0.0
        safe_marker.color.a = 1.0
        
        safe_vel_enu = np.array([safe_vel[0], -safe_vel[1], 0.0])
        head_safe = Point()
        head_safe.x = pos_enu[0] + safe_vel_enu[0] * 0.5
        head_safe.y = pos_enu[1] + safe_vel_enu[1] * 0.5
        head_safe.z = pos_enu[2] + safe_vel_enu[2] * 0.5
        safe_marker.points = [tail, head_safe]
        
        markers.markers.append(safe_marker)
        
        # Barrier function values as text
        text_marker = Marker()
        text_marker.header.frame_id = 'map'
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = 'cbf_values'
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = pos_enu[0]
        text_marker.pose.position.y = pos_enu[1]
        text_marker.pose.position.z = pos_enu[2] + 1.0
        text_marker.scale.z = 0.3
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        # Find minimum barrier value
        min_h = float('inf')
        for a_hat, p_closest in self.barrier_constraints:
            h = self.compute_barrier_function(position, a_hat, p_closest)
            min_h = min(min_h, h)
        
        if min_h != float('inf'):
            text_marker.text = f'Min h: {min_h:.2f}m'
        else:
            text_marker.text = 'No constraints'
        
        markers.markers.append(text_marker)
        
        self.cbf_viz_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    controller = CBFSafeController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()