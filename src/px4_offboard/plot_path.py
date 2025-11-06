import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def rotating_circle_path(t):
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
    speed = 2.0     # Controls the speed of travel around the circle (rad/s)
    
    # --- Rotation Parameters ---
    # Speed at which the entire plane of the circle rotates
    # around the X-axis (in rad/s).
    rotation_speed = 0.2 
    
    # Current rotation angle of the plane at time t
    rotation_angle = rotation_speed * t
    
    # --- Local coordinates of the circle ---
    # This is the path as if it were "flat" on the X-Y plane
    # THIS IS THE PRIMARY CHANGE:
    x_local = radius * np.cos(speed * t)
    y_local = radius * np.sin(speed * t)
    
    # --- Parametric Equations for 3D Position ---
    # We apply the same 3D rotation around the X-axis
    
    # The X position is unaffected by rotation around the X-axis
    x = x_local
    
    # The y_local coordinate is "split" between the world y and z axes
    y = y_local * np.cos(rotation_angle)
    
    # The rotation is centered at the z=altitude line
    z = y_local * np.sin(rotation_angle) + altitude
    
    # --- Fixed Yaw ---
    # As requested, the yaw is fixed, always pointing forward (0 radians).
    yaw = 0.0

    return x, y, z

def rotating_figure_eight_path(t):
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
    speed = 2.0     # Controls the speed of the figure-eight (rad/s)
    
    # --- Rotation Parameters ---
    rotation_speed = 0.2 
    rotation_angle = rotation_speed * t
    
    # --- Local coordinates of the figure-eight ---
    x_local = x_radius * np.cos(speed * t)
    y_local = y_radius * np.sin(2 * speed * t)
    
    # --- Parametric Equations for 3D Position ---
    x = x_local
    y = y_local * np.cos(rotation_angle)
    z = y_local * np.sin(rotation_angle) + altitude
    
    return x, y, z

# --- Simulation Parameters ---
T_total = 25.0  # Total animation time in seconds
fps = 30        # Desired frames per second for playback
num_points = int(T_total * fps) # Total number of frames

# Create a time vector
t_vec = np.linspace(0, T_total, num_points)

# --- Generate All Trajectory Data ---
print("Generating trajectory data...")
x_vals, y_vals, z_vals = rotating_circle_path(t_vec)

# --- Set up the Figure and 3D Axis ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X (forward)')
ax.set_ylabel('Y (right)')
ax.set_zlabel('Z (up)')
ax.set_title('Animating 3D Rotating Figure-Eight (Live)')

# --- Set Fixed Axis Limits ---
all_vals = np.concatenate([x_vals, y_vals, z_vals])
max_range = np.ptp(all_vals) * 0.5
mid_x = (x_vals.max() + x_vals.min()) * 0.5
mid_y = (y_vals.max() + y_vals.min()) * 0.5
mid_z = (z_vals.max() + z_vals.min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)
ax.grid(True)

# --- Camera View Parameters ---
start_elevation = 30  # degrees, the "up-down" angle
start_azimuth = 20    # degrees, the starting "left-right" angle
total_camera_rotation = 180 # Total degrees to rotate the camera

# --- Set the initial camera view ---
ax.view_init(elev=start_elevation, azim=start_azimuth)

# --- Initialize the Artists ---
line, = ax.plot([], [], [], 'b-', label='Trajectory')
dot, = ax.plot([], [], [], 'ro', label='Current Position')
ax.legend()

# --- Create the Animation Functions ---

def init():
    """Initializes the animation by setting empty data."""
    line.set_data_3d([], [], [])
    dot.set_data_3d([], [], [])
    return line, dot

def animate(i):
    """
    This function is called for each frame (i) of the animation.
    It updates the data for the 'line' and 'dot' artists
    AND updates the camera view.
    """
    # Update the line (path) data
    line.set_data_3d(x_vals[:i], y_vals[:i], z_vals[:i])
    
    # Update the dot (drone) data
    dot.set_data_3d([x_vals[i]], [y_vals[i]], [z_vals[i]])
    
    # --- Update Camera Angle ---
    # Calculate the new azimuth angle for this frame
    current_azimuth = start_azimuth + (total_camera_rotation * (i / num_points))
    
    # Set the new view
    ax.view_init(elev=start_elevation, azim=current_azimuth)
    
    return line, dot

# --- Create the Animation Object ---

# Calculate the interval (in milliseconds) between frames
# This is (1000 ms / 1 s) / fps
interval_ms = 1000 / fps

ani = FuncAnimation(
    fig, 
    animate, 
    init_func=init, 
    frames=num_points, 
    interval=interval_ms, # Use interval for live playback
    blit=False
)

# --- Show the Animation ---
print("Showing animation...")
plt.show()