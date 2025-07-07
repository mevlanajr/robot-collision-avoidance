import math
import matplotlib.pyplot as plt
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# 1) Connect & start
client = RemoteAPIClient()
sim    = client.require('sim')
simIK  = client.require('simIK')
scene = '/home/mevlanajr/Projects/robot-collision-avoidance/models/simple_RR_planar.ttt'
sim.loadScene(scene)
sim.setStepping(True)
# match physics time-step to control dt
dt = 0.05  # control timestep [s]

sim.startSimulation()

# 2) Handles
joint1 = sim.getObject('/joint1')
joint2 = sim.getObject('/joint2')
tip     = sim.getObject('/tip')
base    = sim.getObject('/base')
target  = sim.getObject('/target')
# drive joints directly, bypassing dynamics
sim.setJointMode(joint1, sim.jointmode_kinematic)
sim.setJointMode(joint2, sim.jointmode_kinematic)

# 3) Build IK group for Jacobian
ikEnv   = simIK.createEnvironment()
ikGroup = simIK.createGroup(ikEnv)
simIK.addElementFromScene(ikEnv, ikGroup, base, tip, target,
                          simIK.constraint_x | simIK.constraint_y)

# 4) Desired set-point is now replaced by Trajectory Parameters
radius = 1.5
angular_frequency = 0.25 # rad/s, controls the speed on the circle
center_xy = np.array([0, 0])

# Get the initial z-position of the target to keep it constant
pos = sim.getObjectPosition(target, -1)

# compute phase offset so the dummy starts at its initial pose
px, py = sim.getObjectPosition(target, -1)[:2]
phi0   = math.atan2(py - center_xy[1], px - center_xy[0])

# 5) Controller params
duration = 100   # total time [s]
steps    = int(duration / dt)
gamma    = 1.5   # control gain

# 6) Data logs
times      = []
error_norm = []
ee_path_log = []
desired_path_log = []

# 7) Trajectory tracking control loop
for i in range(steps):
    t = i * dt

    # Calculate the desired position and velocity for the current time t
    ang = angular_frequency * t + phi0
    desired_xy = center_xy + np.array([
        radius * math.cos(ang),
        radius * math.sin(ang)
    ])
    desired_velocity_xy = np.array([
        -radius * angular_frequency * math.sin(ang),
         radius * angular_frequency * math.cos(ang)
    ])

    # Move the visual target object to follow the trajectory
    sim.setObjectPosition(target, -1,
        [float(desired_xy[0]), float(desired_xy[1]), pos[2]])

    # 7a) current end-effector pos
    M  = np.array(sim.getObjectMatrix(tip, -1)).reshape(3, 4)
    ee = M[:2, 3]

    # 7b) Compute Jacobian J_xy (sync scene → IK first)
    simIK.syncFromSim(ikEnv, [ikGroup])
    jac_flat, _ = simIK.computeGroupJacobian(ikEnv, ikGroup)
    Jxy = np.array(jac_flat).reshape((2, 2))  # only X-Y rows

    # 7c) control law:
    error = desired_xy - ee
    v_cmd = desired_velocity_xy + gamma * error
    lamda = 1e-1       # damping factor for numerical stability
    JTJ = Jxy.T @ Jxy + lamda * np.eye(2)
    dq  = np.linalg.solve(JTJ, Jxy.T @ v_cmd)

    # Integrate to get desired positions for the next step
    current_q1 = sim.getJointPosition(joint1)
    current_q2 = sim.getJointPosition(joint2)
    q_desired = np.array([current_q1, current_q2]) + dq * dt

    sim.setJointPosition(joint1, q_desired[0])
    sim.setJointPosition(joint2, q_desired[1])

    sim.step()

    # 7e) log data
    times.append(t)
    error_norm.append(np.linalg.norm(error))
    ee_path_log.append(ee)
    desired_path_log.append(desired_xy)

# 8) stop sim
sim.stopSimulation()

# 9) plot error convergence and trajectory side by side
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# Error convergence
ax1.plot(times, error_norm, 'm-')
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('||error|| [m]')
ax1.set_title('Tracking Error Convergence')
ax1.grid(True)

# Trajectory tracking performance
ee_path = np.array(ee_path_log)
desired_path = np.array(desired_path_log)
ax2.plot(desired_path[:, 0], desired_path[:, 1], 'r--', label='Desired Path')
ax2.plot(ee_path[:, 0], ee_path[:, 1], 'b-', label='Actual Path')
ax2.set_xlabel('X [m]')
ax2.set_ylabel('Y [m]')
ax2.set_title('Trajectory Tracking Performance')
ax2.legend()
ax2.grid(True)
ax2.axis('equal')

plt.tight_layout()
plt.show()