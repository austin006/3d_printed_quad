import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
import numpy as np

class OffboardControl(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        # Set up QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriptions and publishers
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # Initialize navigation and arming states
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        # Create timer for command loop
        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period
        
        # Time tracker
        self.t = 0.0

        self.get_logger().info("Offboard control node started")

    def vehicle_status_callback(self, msg):
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def example_path(self, t):
        """
        Calculate 3D position using the time passed and a pre-determined path.
        FRU coordinates: x-forward, y-right, z-up
        """
        
        # This should be replaced with the desired path calculation.
        x = 0
        y = 0
        z = 1.0
        yaw = 0.0

        return x, y, z, yaw

    def figure_eight_path(self, t):
        """
        Calculate 3D position using the time passed and a pre-determined path.
        FRU coordinates: x-forward, y-right, z-up
        
        This function calculates a figure-eight trajectory with the yaw angle tangent to the path.
        """
        # --- Figure-Eight Trajectory Parameters ---
        x_radius = 2.0  # Width of the pattern
        y_radius = 1.0  # Height of the pattern
        altitude = 2.5  # Constant 2.5m altitude
        speed = 1     # Controls the speed of the trajectory
        
        # --- Parametric Equations for Position ---
        # x = A * cos(w*t)
        # y = B * sin(2*w*t)
        x = x_radius * np.cos(speed * t)
        y = y_radius * np.sin(2 * speed * t)
        z = altitude
        
        # --- Calculate Derivatives (Velocity) for Yaw ---
        # The velocity vector is (dx/dt, dy/dt)
        # dx/dt = -A * w * sin(w*t)
        # dy/dt = B * 2*w * cos(2*w*t)
        vx = -x_radius * speed * np.sin(speed * t)
        vy = 2 * y_radius * speed * np.cos(2 * speed * t)
        
        # --- Calculate Yaw Angle ---
        # Yaw is the angle of the velocity vector (vx, vy) in the X-Y plane
        # Use np.arctan2(vy, vx) to get the correct angle in all four quadrants
        # This angle represents the direction the drone should face to be tangent to the path.
        yaw = np.arctan2(vy, vx)

        return x, y, z, yaw

    def rotating_figure_eight_path(self, t):
        """
        Calculate 3D position using the time passed and a pre-determined path.
        FRU coordinates: x-forward, y-right, z-up
        
        This function calculates a figure-eight trajectory. The plane of this
        trajectory constantly rotates around the world's X-axis (rolling).
        The yaw is fixed to 0.0 (always pointing forward).
        """
        
        # --- Figure-Eight Trajectory Parameters ---
        x_radius = 2.0  # Width of the pattern (along X-axis)
        y_radius = 1.0  # Max displacement in Y/Z (on the rotating plane)
        altitude = 2.5  # The altitude at the center of the rotation (z-offset)
        speed = .5     # Controls the speed of the figure-eight (rad/s)
        
        # --- Rotation Parameters ---
        # Speed at which the entire plane of the figure-eight rotates
        # around the X-axis (in rad/s). A positive value causes a "rolling" motion.
        rotation_speed = 0.01
        
        # Current rotation angle of the plane at time t
        rotation_angle = rotation_speed * t
        
        # --- Local coordinates of the figure-eight ---
        # This is the path as if it were "flat" on the X-Y plane
        x_local = x_radius * np.cos(speed * t)
        y_local = y_radius * np.sin(2 * speed * t)
        
        # --- Parametric Equations for 3D Position ---
        # We apply a 3D rotation around the X-axis to the local coordinates
        # and then add the altitude offset.
        #
        # Rotation Matrix (X-axis):
        # [ 1,  0,      0     ]
        # [ 0,  cos(a), -sin(a)]
        # [ 0,  sin(a),  cos(a)]
        #
        # Applying to the point (x_local, y_local, 0):
        # x = x_local
        # y = y_local * cos(a)
        # z = y_local * sin(a)
        
        # The X position is unaffected by rotation around the X-axis
        x = x_local
        
        # The y_local coordinate is "split" between the world y and z axes
        y = y_local * np.cos(rotation_angle)
        
        # The rotation is centered at the z=altitude line
        z = y_local * np.sin(rotation_angle) + altitude
        
        yaw = 0.0

        return x, y, z, yaw

    def rotating_circle_path(self, t):
        """
        Calculate 3D position using the time passed and a pre-determined path.
        FRU coordinates: x-forward, y-right, z-up
        
        This function calculates a circular trajectory. The plane of this
        trajectory constantly rotates around the world's X-axis (rolling).
        The yaw is fixed to 0.0 (always pointing forward).
        """
        
        # --- Circle Trajectory Parameters ---
        radius = 2.0    # Radius of the circle on the local plane
        altitude = 2.5  # The altitude at the center of the rotation (z-offset)
        speed = 0.75     # Controls the speed of travel around the circle (rad/s)
        
        # --- Rotation Parameters ---
        # Speed at which the entire plane of the circle rotates
        # around the X-axis (in rad/s).
        rotation_speed = 0.1 
        
        # Current rotation angle of the plane at time t
        # 't' is the total elapsed time from the node's loop
        rotation_angle = rotation_speed * t
        
        # --- Local coordinates of the circle ---
        # This is the path as if it were "flat" on the X-Y plane
        x_local = radius * np.cos(speed * t)
        y_local = radius * np.sin(speed * t)
        
        # --- Parametric Equations for 3D Position ---
        # We apply the 3D rotation around the X-axis
        
        # The X position is unaffected by rotation around the X-axis
        x = x_local
        
        # The y_local coordinate is "split" between the world y and z axes
        y = y_local * np.cos(rotation_angle)
        
        # The rotation is centered at the z=altitude line
        z = y_local * np.sin(rotation_angle) + altitude
        
        # --- Fixed Yaw ---
        # As requested, the yaw is fixed, always pointing forward (0 radians).
        yaw = 0.0

        return x, y, z, yaw

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)
        
        # Check if in offboard mode and armed -> publish trajectory setpoints
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and 
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            
            # Create trajectory message
            trajectory_msg = TrajectorySetpoint()
            
            # Calculate 3D position
            x, y, z, yaw = self.rotating_circle_path(self.t)
            
            # Set the trajectory position (NED coordinates, hence negative z for altitude)
            trajectory_msg.position[0] = x
            trajectory_msg.position[1] = y
            trajectory_msg.position[2] = -z
            trajectory_msg.yaw = yaw
            
            # Publish trajectory setpoint
            self.publisher_trajectory.publish(trajectory_msg)
            
            # Update time
            self.t = self.t + self.dt

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()