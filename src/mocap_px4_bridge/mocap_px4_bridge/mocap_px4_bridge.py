#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry, OffboardControlMode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
from numpy import NaN

class MocapPX4Bridge(Node):
    def __init__(self):
        super().__init__('mocap_px4_bridge')
        
        # Create QoS profile matching the VRPN publisher
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber to VRPN mocap data
        self.mocap_sub = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/x650_quad/pose',
            self.mocap_callback,
            qos_profile
        )
        
        # Publisher to PX4 vehicle odometry
        self.odom_pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos_profile
        )


        # Publisher to PX4 offboard control mode
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        # Add a variable to store the latest mocap pose
        self.latest_mocap_pose = None

        # Create a timer to publish odometry at 40 Hz
        odom_timer_period = 1.0 / 40.0  # seconds
        self.odom_timer = self.create_timer(odom_timer_period, self.publish_odometry)

        # Create a timer to publish offboard control mode at 10 Hz
        offboard_timer_period = 0.1  # seconds
        self.offboard_timer = self.create_timer(offboard_timer_period, self.publish_offboard_mode)
                
        self.get_logger().info("Mocap to PX4 bridge initialized")
        self.get_logger().info("Publishing odometry at 40 Hz and offboard control mode at 10 Hz")
        
        # Flag for once-only logging
        self._first_message = True

    def mocap_callback(self, msg):
        """Callback to receive and store the latest mocap data."""
        
        # Store the latest message
        self.latest_mocap_pose = msg
        
        if self._first_message:
            self.get_logger().info("Received first msg from optitrack.")
            self.get_logger().info(f"P: {msg.pose.position.x:.6f}, {msg.pose.position.y:.6f}, {msg.pose.position.z:.6f}")
            self.get_logger().info(f"q: {msg.pose.orientation.w:.6f}, {msg.pose.orientation.x:.6f}, {msg.pose.orientation.y:.6f}, {msg.pose.orientation.z:.6f}")
            self._first_message = False

    def publish_odometry(self):
        """
        Convert the latest mocap pose to PX4 VehicleOdometry format and publish it.
        This function is called by the timer at a fixed rate.
        """
        # Don't publish anything until the first mocap message has been received
        if self.latest_mocap_pose is None:
            return

        # Use the stored message for conversion
        msg = self.latest_mocap_pose
        
        # Create VehicleOdometry message
        odom_msg = VehicleOdometry()
                
        # 1. Set pose frame
        odom_msg.pose_frame = 2 #VehicleOdometry.POSE_FRAME_FRD
        
        # 2. Let PX4 timestamp the data
        odom_msg.timestamp = 0
        odom_msg.timestamp_sample = 0

        # 3. Position with coordinate transformation 
        # # mocap (FUR) to PX4 (NED)
        odom_msg.position = [
            float(msg.pose.position.x),    # X 
            float(msg.pose.position.z),    # Z 
            float(-msg.pose.position.y)   # Y sign flipped  
        ]
        
        # 4. Orientation with coordinate transformation
        odom_msg.q = [
            float(msg.pose.orientation.w),   # w
            float(msg.pose.orientation.x),  # x
            float(msg.pose.orientation.z),   # z 
            float(-msg.pose.orientation.y)  # y 
        ]
        # odom_msg.q = [NaN, NaN, NaN, NaN]
        
        # Velocity (unknown as mocap doesn't provide this)
        odom_msg.velocity_frame = 2 # FRD from px4 message
        odom_msg.velocity = [NaN, NaN, NaN]
        odom_msg.angular_velocity = [NaN, NaN, NaN]
        
        # Variances
        odom_msg.position_variance = [0.01, 0.01, 0.01]
        odom_msg.orientation_variance = [0.01, 0.01, 0.01] 
        # odom_msg.velocity_variance = [0.0, 0.0, 0.0]
        
        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

    def publish_offboard_mode(self):
        """
        Publish offboard control mode message to maintain offboard mode.
        This is the 'proof of life' signal required by PX4.
        """
        msg = OffboardControlMode()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # microseconds
        
        # Set the control mode. Here we enable position control.
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        
        self.offboard_mode_pub.publish(msg)
        self.get_logger().debug("Published OffboardControlMode message")

def main(args=None):
    rclpy.init(args=args)
    mocap_bridge = MocapPX4Bridge()
    
    try:
        rclpy.spin(mocap_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        mocap_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()