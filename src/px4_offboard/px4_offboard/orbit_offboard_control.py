import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus

class OffboardControl(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period
        
        # Parameters for the 3D orbit
        self.declare_parameter('radius', 0.75)
        self.declare_parameter('omega', 1.5 * np.pi / 5.0)  # Angular velocity of orbit around path
        self.declare_parameter('altitude', 1.2)  # Center altitude
        self.declare_parameter('rotation_omega', .5 * np.pi / 20.0)  # Angular velocity of plane rotation
        self.declare_parameter('tilt_angle', np.radians(25))  # Tilt of orbit plane in radians
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        
        # Time tracker (used instead of separate theta/phi)
        self.t = 0.0
        
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value
        self.rotation_omega = self.get_parameter('rotation_omega').value
        self.tilt_angle = self.get_parameter('tilt_angle').value

    def vehicle_status_callback(self, msg):
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def orbit_position(self, t):
        """
        Calculate 3D position using the rotation formulation from the example.
        This creates an orbit that rotates around a center point with a tilted plane.
        """
        x = (self.radius * np.cos(self.omega * t) * np.cos(self.rotation_omega * t)
             + self.radius * np.sin(self.omega * t) * np.sin(self.tilt_angle) * np.sin(self.rotation_omega * t))
        
        y = self.radius * np.sin(self.omega * t) * np.cos(self.tilt_angle)
        
        z = (-self.radius * np.cos(self.omega * t) * np.sin(self.rotation_omega * t)
             + self.radius * np.sin(self.omega * t) * np.sin(self.tilt_angle) * np.cos(self.rotation_omega * t))
        
        return x, y, z

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)
        
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and 
            self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            
            trajectory_msg = TrajectorySetpoint()
            
            # Calculate 3D position
            x, y, z = self.orbit_position(self.t)
            
            # Set the trajectory position (NED coordinates, hence negative z for altitude)
            trajectory_msg.position[0] = x
            trajectory_msg.position[1] = y
            trajectory_msg.position[2] = -self.altitude + z
            
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