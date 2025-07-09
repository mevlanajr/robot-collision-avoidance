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
joint1 = sim.getObject('/joint1')
joint2 = sim.getObject('/joint2')
tip     = sim.getObject('/tip')
base    = sim.getObject('/base')
target  = sim.getObject('/target')
# --- NEW: Get handle for the obstacle ---
obstacle = sim.getObject('/obstacle')
# --------------------------------------

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
base_pos = np.array(sim.getObjectPosition(base, -1))
joint2_pos = np.array(sim.getObjectPosition(joint2, -1))

# Measure the robot's link lengths
link1_length = np.linalg.norm(joint2_pos - base_pos)
link2_length = np.linalg.norm(initial_tip_pos - joint2_pos)
max_reach = link1_length + link2_length

# Define a safe trajectory
radius = max_reach * 0.7
angular_frequency = 0.5
center_xy = np.array([base_pos[0], base_pos[1]])

# PD Controller gains
gamma_p = 3.0
gamma_d = 0.2
lamda = 0.01

# --- NEW: CBF Parameters (Tuned for smoother avoidance) ---
safety_distance = 0.4 # Increased buffer
cbf_gamma = 0.3       # Reduced gain to make it less aggressive
# --------------------------------------------------------

# 5) --- Build IK group (for Jacobian calculation) ---
ikEnv   = simIK.createEnvironment()
ikGroup = simIK.createGroup(ikEnv)
simIK.addElementFromScene(ikEnv, ikGroup, base, tip, target,
                          simIK.constraint_x | simIK.constraint_y)

# 6) --- Data logs for plotting ---
times = []
error_norm = []
ee_path_log = []
desired_path_log = []
barrier_h_log = [] # Log for the barrier function value

# 7) --- SYNCHRONOUS JACOBIAN CONTROL LOOP ---
print("Starting controller with CBF...")
duration = 40 # Increased duration to see more of the behavior
start_sim_time = sim.getSimulationTime()
phi0 = math.atan2(initial_tip_pos[1] - center_xy[1], initial_tip_pos[0] - center_xy[0])
dt = sim.getSimulationTimeStep()

last_ee_pos = initial_tip_pos[:2]

while sim.getSimulationTime() - start_sim_time < duration:
    t = sim.getSimulationTime() - start_sim_time
    
    # --- Trajectory Generation (Nominal Controller) ---
    ang = angular_frequency * t + phi0
    desired_xy = center_xy + np.array([radius * math.cos(ang), radius * math.sin(ang)])
    desired_velocity_xy = np.array([-radius * angular_frequency * math.sin(ang),
                                     radius * angular_frequency * math.cos(ang)])
    
    sim.setObjectPosition(target, -1, [float(desired_xy[0]), float(desired_xy[1]), initial_tip_pos[2]])

    # --- Control Calculations ---
    ee_pos = np.array(sim.getObjectPosition(tip, -1))[:2]
    simIK.syncFromSim(ikEnv, [ikGroup])
    jac_flat, _ = simIK.computeGroupJacobian(ikEnv, ikGroup)
    Jxy = np.array(jac_flat).reshape((2, 2))

    # --- PD Control Law (This is our desired command, u_des) ---
    error = desired_xy - ee_pos
    ee_velocity = (ee_pos - last_ee_pos) / dt
    error_dot = desired_velocity_xy - ee_velocity
    v_cmd_pd = desired_velocity_xy + gamma_p * error + gamma_d * error_dot
    JTJ = Jxy.T @ Jxy + lamda * np.eye(2)
    dq_des = np.linalg.solve(JTJ, Jxy.T @ v_cmd_pd) # This is our desired joint velocity

    # --- NEW: CONTROL BARRIER FUNCTION (SAFETY FILTER) ---
    # 1. Define the barrier function h(x)
    dist_data = sim.checkDistance(tip, obstacle)
    distance = dist_data[0]
    h = distance - safety_distance # h >= 0 means we are safe

    # 2. Calculate the Lie derivatives (L_f h and L_g h)
    tip_closest_point = np.array(dist_data[1])[:2]
    obs_closest_point = np.array(dist_data[2])[:2]
    
    if distance > 1e-6:
        # CORRECTED: The gradient must point AWAY from the obstacle.
        # The vector from obstacle to tip gives the correct direction.
        dh_dx = (tip_closest_point - obs_closest_point) / distance
    else:
        dh_dx = np.array([0.0, 0.0])

    # L_g h = (dh/dx) * J(q)
    Lg_h = dh_dx @ Jxy

    # 3. Check the safety condition and solve the QP (using closed-form solution)
    psi = Lg_h @ dq_des + cbf_gamma * h
    
    if psi < 0:
        # The desired command is unsafe. Modify it.
        Lg_h_norm_sq = np.dot(Lg_h, Lg_h)
        if Lg_h_norm_sq > 1e-6:
            # The corrective term must OPPOSE the unsafe direction.
            # The paper's formula is correct, the issue was the gradient sign.
            dq_safe = dq_des - (Lg_h.T * psi) / Lg_h_norm_sq
        else:
            dq_safe = dq_des # Cannot correct if Lg_h is zero
    else:
        # The desired command is already safe. Use it as is.
        dq_safe = dq_des

    # The final command sent to the robot is the safe one
    dq = dq_safe
    # --------------------------------------------------------

    # --- Command the Robot ---
    current_q1 = sim.getJointPosition(joint1)
    current_q2 = sim.getJointPosition(joint2)
    q_desired = np.array([current_q1, current_q2]) + dq * dt
    
    sim.setJointPosition(joint1, q_desired[0])
    sim.setJointPosition(joint2, q_desired[1])

    # --- Logging and Stepping ---
    times.append(t + start_sim_time)
    desired_path_log.append(desired_xy)
    ee_path_log.append(ee_pos)
    error_norm.append(np.linalg.norm(error))
    barrier_h_log.append(h)
    
    last_ee_pos = ee_pos
    sim.step()

# --- Get obstacle info for plotting BEFORE stopping simulation ---
obs_pos = sim.getObjectPosition(obstacle, -1)
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
    ee_path = np.array(ee_path_log)
    desired_path = np.array(desired_path_log)
    ax2.plot(desired_path[:, 0], desired_path[:, 1], 'r--', label='Desired Path')
    ax2.plot(ee_path[:, 0], ee_path[:, 1], 'b-', label='Actual Path')
    # Plot the obstacle
    circle = plt.Circle((obs_pos[0], obs_pos[1]), obs_radius, color='g', label='Obstacle')
    ax2.add_artist(circle)
    ax2.set_xlabel('X [m]'); ax2.set_ylabel('Y [m]'); ax2.set_title('Path Tracking')
    ax2.legend(); ax2.grid(True); ax2.axis('equal')

    # Barrier function plot
    ax3.plot(times, barrier_h_log, 'k-')
    ax3.axhline(y=0, color='r', linestyle='--', label='Safety Boundary (h=0)')
    ax3.set_xlabel('Time [s]'); ax3.set_ylabel('h(x)'); ax3.set_title('Barrier Function Value')
    ax3.legend(); ax3.grid(True)

    plt.tight_layout()
    plt.show()