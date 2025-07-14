import math
import matplotlib.pyplot as plt
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from qpsolvers import solve_qp # --- NEW: Import the QP solver ---

# 1) Connect & start
client = RemoteAPIClient()
sim    = client.require('sim')
simIK  = client.require('simIK')
# --- NEW: Load the multi-obstacle scene ---
sim.loadScene('/home/mevlanajr/Projects/robot-collision-avoidance/models/cbf_6DOF_multi_obstacle.ttt')

# 2) Get handles
joint_names = [
    '/UR5/base_joint', '/UR5/shoulder_joint', '/UR5/elbow_joint',
    '/UR5/wrist_1_joint', '/UR5/wrist_2_joint', '/UR5/wrist_3_joint'
]
joint_handles = [sim.getObject(name) for name in joint_names]
tip      = sim.getObject('/UR5/tip')
base     = sim.getObject('/UR5')
target   = sim.getObject('/target')

# --- NEW: Get handles for multiple obstacles ---
obstacle_names = ['/obstacle1', '/obstacle2']
obstacle_handles = [sim.getObject(name) for name in obstacle_names]
# ---------------------------------------------

# --- Set all 6 joints to KINEMATIC mode ---
for handle in joint_handles:
    sim.setJointMode(handle, sim.jointmode_kinematic)
# --------------------------------------------------

# 3) --- SWITCH TO SYNCHRONOUS MODE ---
sim.setStepping(True)
sim.startSimulation()

# 4) --- TRAJECTORY & CONTROLLER PARAMETERS ---
initial_tip_pos = np.array(sim.getObjectPosition(tip, -1))
base_pos        = np.array(sim.getObjectPosition(base, -1))
radius            = 0.3
angular_frequency = 0.4
center_xyz        = np.array([base_pos[0] + 0.5, base_pos[1], base_pos[2] + 0.4])

# --- PD Controller & CBF gains ---
gamma_p = 6.0
gamma_d = 0.0
lamda   = 0.01
d_min = 0.2
k_h   = 4.0
# -------------------------------------------------------

# 5) --- Build IK group ---
ikEnv   = simIK.createEnvironment()
ikGroup = simIK.createGroup(ikEnv)
simIK.addElementFromScene(ikEnv, ikGroup, base, tip, target,
                          simIK.constraint_x | simIK.constraint_y | simIK.constraint_z)

# 6) --- Data logs for plotting ---
times, error_norm, ee_path_log, desired_path_log = [], [], [], [],
# Log h for each obstacle
barrier_h_logs = {name: [] for name in obstacle_names}

# 7) --- SYNCHRONOUS JACOBIAN CONTROL LOOP ---
print("Starting 6-DOF controller with multi-obstacle CBF...")
duration       = 80
start_sim_time = sim.getSimulationTime()
dt             = sim.getSimulationTimeStep()
last_ee_pos = initial_tip_pos

while sim.getSimulationTime() - start_sim_time < duration:
    t = sim.getSimulationTime() - start_sim_time

    # --- Trajectory Generation ---
    ang = angular_frequency * t
    desired_xyz = center_xyz + np.array([0, radius * math.cos(ang), radius * math.sin(ang)])
    desired_velocity_xyz = np.array([0, -radius * angular_frequency * math.sin(ang), radius * angular_frequency * math.cos(ang)])
    sim.setObjectPosition(target, -1, desired_xyz.tolist())

    # --- Control Calculations ---
    ee_pos = np.array(sim.getObjectPosition(tip, -1))
    simIK.syncFromSim(ikEnv, [ikGroup])
    jac_flat, _ = simIK.computeGroupJacobian(ikEnv, ikGroup)
    Jxyz = np.array(jac_flat).reshape((3, 6))

    # --- PD Control Law (Desired command) ---
    error         = desired_xyz - ee_pos
    if dt > 0:
        ee_velocity   = (ee_pos - last_ee_pos) / dt
    else:
        ee_velocity = np.zeros(3)
    error_dot     = desired_velocity_xyz - ee_velocity
    v_cmd         = desired_velocity_xyz + gamma_p * error + gamma_d * error_dot
    JTJ = Jxyz.T @ Jxyz + lamda * np.eye(6)
    dq_des  = np.linalg.solve(JTJ, Jxyz.T @ v_cmd)

    # --- NEW: CBF as a Quadratic Program (QP) ---
    # Initialize lists for QP constraints Gx <= h
    G = []
    h_constraints = []

    # Loop through each obstacle to build the constraints
    for i, obs_handle in enumerate(obstacle_handles):
        obs_pos = np.array(sim.getObjectPosition(obs_handle, -1))
        delta_p = ee_pos - obs_pos
        h_val   = delta_p.dot(delta_p) - d_min**2
        grad_h  = 2.0 * delta_p
        J_h     = grad_h @ Jxyz
        
        # Add the constraint: -J_h * dq <= k_h * h_val
        G.append(-J_h)
        h_constraints.append(k_h * h_val)
        barrier_h_logs[obstacle_names[i]].append(h_val)

    # Convert lists to numpy arrays
    G = np.array(G)
    h_constraints = np.array(h_constraints)

    # Define the QP problem: min ||dq - dq_des||^2
    P = 2 * np.eye(6) # Multiply by 2 because qpsolvers uses 0.5 * u'Pu
    q = -2 * dq_des

    # Solve the QP to find the safest command
    try:
        dq_safe = solve_qp(P, q, G, h_constraints, solver='osqp')
        if dq_safe is None: # Solver failed
            dq_safe = dq_des
        dq = dq_safe
    except Exception as e:
        # Fallback if solver fails catastrophically
        print(f"QP Solver failed: {e}")
        dq = dq_des
    # --- End CBF QP ---

    # --- Command the Robot ---
    current_q = np.array([sim.getJointPosition(j) for j in joint_handles])
    q_desired  = current_q + dq * dt
    for i in range(6):
        sim.setJointPosition(joint_handles[i], q_desired[i])

    # --- Logging and Stepping ---
    times.append(t + start_sim_time)
    desired_path_log.append(desired_xyz)
    ee_path_log.append(ee_pos)
    error_norm.append(np.linalg.norm(error))
    
    last_ee_pos = ee_pos
    sim.step()

# --- Get obstacle info for plotting ---
obstacle_info = []
for obs_handle in obstacle_handles:
    pos = sim.getObjectPosition(obs_handle, -1)
    size, _ = sim.getShapeBB(obs_handle)
    radius = size[0] / 2
    obstacle_info.append({'pos': pos, 'radius': radius})
# --------------------------------------------------------------------

# 8) Stop simulation
sim.stopSimulation()
print("Controller finished.")

# 9) Plotting
if times:
    def plot_sphere(ax, center, radius, color, alpha=0.2, label=None):
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = radius * np.cos(u) * np.sin(v) + center[0]
        y = radius * np.sin(u) * np.sin(v) + center[1]
        z = radius * np.cos(v) + center[2]
        ax.plot_surface(x, y, z, color=color, alpha=alpha, label=label)

    fig = plt.figure(figsize=(15, 8))
    ax_3d = fig.add_subplot(1, 2, 1, projection='3d')
    ax_err = fig.add_subplot(2, 2, 2)
    ax_bar = fig.add_subplot(2, 2, 4)

    ee_path = np.array(ee_path_log)
    desired_path = np.array(desired_path_log)
    ax_3d.plot(desired_path[:, 0], desired_path[:, 1], desired_path[:, 2], 'r--', label='Desired Path')
    ax_3d.plot(ee_path[:, 0], ee_path[:, 1], ee_path[:, 2], 'b-', label='Actual Path')
    
    # --- Plot multiple obstacles ---
    for i, obs in enumerate(obstacle_info):
        plot_sphere(ax_3d, obs['pos'], obs['radius'], 'g', alpha=0.5)
        plot_sphere(ax_3d, obs['pos'], d_min, 'g', alpha=0.1)

    ax_3d.set_xlabel('X [m]'); ax_3d.set_ylabel('Y [m]'); ax_3d.set_zlabel('Z [m]')
    ax_3d.set_title('Path Tracking')
    from matplotlib.patches import Patch
    legend_elements = [plt.Line2D([0], [0], color='r', linestyle='--', label='Desired Path'),
                       plt.Line2D([0], [0], color='b', label='Actual Path'),
                       Patch(facecolor='g', alpha=0.5, label='Obstacle(s)'),
                       Patch(facecolor='g', alpha=0.1, label='Safety Boundary')]
    ax_3d.legend(handles=legend_elements)

    ax_err.plot(times, error_norm, 'm-')
    ax_err.set_xlabel('Time [s]'); ax_err.set_ylabel('||error|| [m]'); ax_err.set_title('Tracking Error')
    ax_err.grid(True)

    ax_bar.axhline(y=0, color='r', linestyle='--', label='Safety Boundary (h=0)')
    for name, h_log in barrier_h_logs.items():
        ax_bar.plot(times, h_log, label=f'h for {name}')
    ax_bar.set_xlabel('Time [s]'); ax_bar.set_ylabel('h(q)'); ax_bar.set_title('Barrier Function Values')
    ax_bar.legend(); ax_bar.grid(True)
    
    plt.tight_layout()
    plt.show()