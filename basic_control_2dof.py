import math
import matplotlib.pyplot as plt
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# 1) Connect & start
client = RemoteAPIClient()
sim    = client.require('sim')
simIK  = client.require('simIK')
sim.loadScene('/home/mevlanajr/Projects/robot-collision-avoidance/models/simple_RR_planar_obstacle.ttt')

# 2) Get handles
joint1   = sim.getObject('/joint1')
joint2   = sim.getObject('/joint2')
tip      = sim.getObject('/tip')
base     = sim.getObject('/base')
target   = sim.getObject('/target')    # Still useful for visualization
obstacle = sim.getObject('/obstacle')  # <<-- your scene’s obstacle handle

# --- Set joints back to KINEMATIC mode ---
sim.setJointMode(joint1, sim.jointmode_kinematic)
sim.setJointMode(joint2, sim.jointmode_kinematic)
# --------------------------------------------------

# 3) --- SWITCH TO SYNCHRONOUS MODE ---
sim.setStepping(True)
sim.startSimulation()

# 4) --- TRAJECTORY & CONTROLLER PARAMETERS ---
# Get initial positions to calculate workspace
initial_tip_pos = np.array(sim.getObjectPosition(tip, -1))
base_pos        = np.array(sim.getObjectPosition(base, -1))
joint2_pos      = np.array(sim.getObjectPosition(joint2, -1))

# Measure the robot's link lengths
link1_length = np.linalg.norm(joint2_pos - base_pos)
link2_length = np.linalg.norm(initial_tip_pos - joint2_pos)
max_reach    = link1_length + link2_length

# Define a safe trajectory
radius            = max_reach * 0.7
angular_frequency = 0.3  # rad/s
center_xy         = np.array([base_pos[0], base_pos[1]])

# --- PD Controller gains ---
gamma_p = 6.0   # Proportional gain
gamma_d =  0.4   # Derivative gain
lamda   =  0.01  # Damping factor for Jacobian inverse
# -------------------------------------------------------

# --- CBF parameters (joint-space) ---
d_min = 0.5   # minimum safe distance (m)
k_h   = 4.0   # class-K gain: alpha(h) = k_h * h
# -------------------------------------------------------

# 5) --- Build IK group  ---
ikEnv   = simIK.createEnvironment()
ikGroup = simIK.createGroup(ikEnv)
simIK.addElementFromScene(ikEnv, ikGroup, base, tip, target,
                          simIK.constraint_x | simIK.constraint_y)

# 6) --- Data logs for plotting ---
times = []
error_norm = []
ee_path_log = []
desired_path_log = []
barrier_h_log = [] # <<-- ADDED: Log for the barrier function value

# 7) --- SYNCHRONOUS JACOBIAN CONTROL LOOP ---
print("Starting controller...")
duration       = 50    # seconds
start_sim_time = sim.getSimulationTime()
phi0           = math.atan2(initial_tip_pos[1] - center_xy[1],
                            initial_tip_pos[0] - center_xy[0])
dt             = sim.getSimulationTimeStep()

# --- Initialize variables for derivative calculation ---
last_ee_pos = initial_tip_pos[:2]
# ----------------------------------------------------------

while sim.getSimulationTime() - start_sim_time < duration:
    t = sim.getSimulationTime() - start_sim_time

    # --- Trajectory Generation ---
    ang = angular_frequency * t + phi0
    desired_xy = center_xy + np.array([radius * math.cos(ang),
                                        radius * math.sin(ang)])
    desired_velocity_xy = np.array([-radius * angular_frequency * math.sin(ang),
                                     radius * angular_frequency * math.cos(ang)])
    
    # Update the visual target
    sim.setObjectPosition(target, -1, [float(desired_xy[0]),
                                       float(desired_xy[1]),
                                       initial_tip_pos[2]])

    # --- Control Calculations ---
    # Get current state
    ee_pos = np.array(sim.getObjectPosition(tip, -1))[:2]
    
    # Compute Jacobian
    simIK.syncFromSim(ikEnv, [ikGroup])
    jac_flat, _ = simIK.computeGroupJacobian(ikEnv, ikGroup)
    Jxy = np.array(jac_flat).reshape((2, 2))

    # --- NEW: PD Control Law ---
    error         = desired_xy - ee_pos
    ee_velocity   = (ee_pos - last_ee_pos) / dt
    error_dot     = desired_velocity_xy - ee_velocity
    v_cmd         = desired_velocity_xy + gamma_p * error + gamma_d * error_dot
    # -------------------------

    # Solve for joint velocities using Damped Least Squares
    JTJ = Jxy.T @ Jxy + lamda * np.eye(2)
    dq  = np.linalg.solve(JTJ, Jxy.T @ v_cmd)

    # === joint-space CBF filter (Lemma 2) ===
    # read actual obstacle position
    obs_xyz = np.array(sim.getObjectPosition(obstacle, -1))
    obs_pos = obs_xyz[:2]

    # 1) compute h(q) = ||xee - x_obs||^2 - d_min^2
    delta_p = ee_pos - obs_pos
    h_val   = delta_p.dot(delta_p) - d_min**2

    # 2) chain-rule: ∂h/∂q = (∂h/∂x) * J_y,   ∂h/∂x = 2*(xee - x_obs)
    grad_h  = 2.0 * delta_p           # 1×2
    J_h     = grad_h @ Jxy            # 1×2

    # 3) residual Δ = J_h · dq + α(h)
    residual = J_h.dot(dq) + k_h * h_val

    # 4) if Δ < 0, project dq back into safe set
    if residual < 0.0:
        norm2 = J_h.dot(J_h)          # scalar = J_h·J_h^T
        if norm2 > 1e-6: # Avoid division by zero
            dq    = dq - (residual / norm2) * J_h
    # === end CBF filter ===

    # --- Command the Robot ---
    current_q1 = sim.getJointPosition(joint1)
    current_q2 = sim.getJointPosition(joint2)
    q_desired  = np.array([current_q1, current_q2]) + dq * dt
    
    sim.setJointPosition(joint1, q_desired[0])
    sim.setJointPosition(joint2, q_desired[1])
    # ---------------------------------------------------------------------

    # --- Logging and Stepping ---
    times.append(t + start_sim_time)
    desired_path_log.append(desired_xy)
    ee_path_log.append(ee_pos)
    error_norm.append(np.linalg.norm(error))
    barrier_h_log.append(h_val) # <<-- ADDED: Log the barrier value
    
    # Update state for next loop
    last_ee_pos = ee_pos
    
    # Advance the simulation by one step
    sim.step()

# --- Get obstacle info for plotting BEFORE stopping simulation ---
obs_pos_final = sim.getObjectPosition(obstacle, -1)
size, _ = sim.getShapeBB(obstacle)
obs_radius = size[0] / 2
# --------------------------------------------------------------------

# 8) Stop simulation
sim.stopSimulation()
print("Controller finished.")

# 9) Plotting
if times:
    # Create a figure with 3 subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 12))
    
    # Error convergence plot
    ax1.plot(times, error_norm, 'm-')
    ax1.set_xlabel('Time [s]'); ax1.set_ylabel('||error|| [m]'); ax1.set_title('Tracking Error')
    ax1.grid(True)
    
    # Path tracking performance plot
    ee_path      = np.array(ee_path_log)
    desired_path = np.array(desired_path_log)
    ax2.plot(desired_path[:, 0], desired_path[:, 1], 'r--', label='Desired Path')
    ax2.plot(ee_path[:, 0], ee_path[:, 1], 'b-', label='Actual Path')
    # Plot the obstacle
    circle = plt.Circle((obs_pos_final[0], obs_pos_final[1]), obs_radius, color='g', label='Obstacle')
    ax2.add_artist(circle)
    # Plot the safety boundary
    safety_circle = plt.Circle((obs_pos_final[0], obs_pos_final[1]), d_min, color='g', fill=False, linestyle='--', label='Safety Boundary')
    ax2.add_artist(safety_circle)
    ax2.set_xlabel('X [m]'); ax2.set_ylabel('Y [m]'); ax2.set_title('Path Tracking')
    ax2.legend(); ax2.grid(True); ax2.axis('equal')

    # Barrier function plot
    ax3.plot(times, barrier_h_log, 'k-')
    ax3.axhline(y=0, color='r', linestyle='--', label='Safety Boundary (h=0)')
    ax3.set_xlabel('Time [s]'); ax3.set_ylabel('h(q)'); ax3.set_title('Barrier Function Value')
    ax3.legend(); ax3.grid(True)

    plt.tight_layout()
    plt.show()