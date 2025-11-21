import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation

# Simulation parameters
dt = 0.05  # time step
max_speed = 2.0  # maximum velocity
alpha = 1.0  # CBF class-K function parameter (controls aggressiveness)

# Environment setup
start = np.array([0.0, 0.0])
goal = np.array([10.0, 0.0])
obstacle_center = np.array([5.0, 0.0])
obstacle_radius = 1.5
safety_margin = 0.3  # extra safety buffer

# Initial state
position = start.copy()
trajectory = [position.copy()]

# Control Barrier Function for circular obstacle
def barrier_function(pos, obs_center, obs_radius, margin):
    """
    h(x) = ||x - x_obs||^2 - (r_obs + margin)^2
    The safe set is where h(x) >= 0
    """
    distance_sq = np.sum((pos - obs_center)**2)
    safe_distance_sq = (obs_radius + margin)**2
    return distance_sq - safe_distance_sq

def barrier_constraint(pos, vel, obs_center, obs_radius, margin):
    """
    CBF constraint: dh/dt + alpha * h(x) >= 0
    This ensures the system stays in the safe set
    """
    h = barrier_function(pos, obs_center, obs_radius, margin)
    
    # Gradient of barrier function: dh/dx = 2(x - x_obs)
    grad_h = 2 * (pos - obs_center)
    
    # Lie derivative: dh/dt = grad_h^T * vel
    lie_derivative = np.dot(grad_h, vel)
    
    # CBF condition: lie_derivative + alpha * h >= 0
    return lie_derivative + alpha * h, grad_h

def nominal_controller(pos, goal_pos, max_vel):
    """Simple proportional controller toward goal"""
    direction = goal_pos - pos
    distance = np.linalg.norm(direction)
    if distance < 0.1:
        return np.zeros(2)
    desired_vel = (direction / distance) * max_vel
    return desired_vel

def cbf_qp_controller(pos, nominal_vel, obs_center, obs_radius, margin):
    """
    Solve a simple QP to find the safe control that minimally modifies nominal control
    min ||u - u_nominal||^2
    s.t. CBF constraint is satisfied
    
    For simplicity, we use a closed-form solution for the single constraint case
    """
    cbf_value, grad_h = barrier_constraint(pos, nominal_vel, obs_center, obs_radius, margin)
    
    # If CBF constraint is already satisfied, use nominal control
    if cbf_value >= 0:
        return nominal_vel
    
    # Otherwise, project nominal control to satisfy constraint
    # The minimum correction is along the gradient direction
    grad_norm_sq = np.dot(grad_h, grad_h)
    if grad_norm_sq < 1e-6:
        return nominal_vel
    
    # Correction needed to satisfy constraint
    correction = -(cbf_value / grad_norm_sq) * grad_h
    safe_vel = nominal_vel + correction
    
    return safe_vel

# Simulation loop
print("Running CBF simulation...")
print(f"Start: {start}, Goal: {goal}")
print(f"Obstacle at {obstacle_center} with radius {obstacle_radius}")

max_steps = 500
for step in range(max_steps):
    # Check if goal is reached
    if np.linalg.norm(position - goal) < 0.2:
        print(f"Goal reached at step {step}!")
        break
    
    # Compute nominal control (toward goal)
    u_nominal = nominal_controller(position, goal, max_speed)
    
    # Apply CBF to ensure safety
    u_safe = cbf_qp_controller(position, u_nominal, obstacle_center, obstacle_radius, safety_margin)
    
    # Update position
    position = position + u_safe * dt
    trajectory.append(position.copy())

trajectory = np.array(trajectory)

# Visualization
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# Left plot: Full trajectory
ax1.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, label='Robot Trajectory')
ax1.plot(start[0], start[1], 'go', markersize=12, label='Start')
ax1.plot(goal[0], goal[1], 'r*', markersize=15, label='Goal')

# Draw obstacle
obstacle = Circle(obstacle_center, obstacle_radius, color='red', alpha=0.3, label='Obstacle')
ax1.add_patch(obstacle)
safety_circle = Circle(obstacle_center, obstacle_radius + safety_margin, 
                       color='orange', alpha=0.2, linestyle='--', fill=False, label='Safety Margin')
ax1.add_patch(safety_circle)

ax1.set_xlabel('X position')
ax1.set_ylabel('Y position')
ax1.set_title('CBF-Based Obstacle Avoidance')
ax1.legend()
ax1.grid(True, alpha=0.3)
ax1.axis('equal')
ax1.set_xlim(-1, 11)
ax1.set_ylim(-3, 3)

# Right plot: Barrier function value over time
barrier_values = []
for pos in trajectory:
    h = barrier_function(pos, obstacle_center, obstacle_radius, safety_margin)
    barrier_values.append(h)

ax2.plot(barrier_values, 'b-', linewidth=2)
ax2.axhline(y=0, color='r', linestyle='--', label='Safety Boundary (h=0)')
ax2.set_xlabel('Time Step')
ax2.set_ylabel('Barrier Function Value h(x)')
ax2.set_title('Safety Verification (h ≥ 0 means safe)')
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print(f"\nSimulation completed in {len(trajectory)} steps")
print(f"Minimum barrier function value: {min(barrier_values):.4f}")
if min(barrier_values) >= 0:
    print("✓ Safety maintained throughout trajectory!")
else:
    print("✗ Warning: Safety constraint violated")