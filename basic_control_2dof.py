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
sim.startSimulation()

# 2) Handles
joint1 = sim.getObject('/joint1')
joint2 = sim.getObject('/joint2')
tip     = sim.getObject('/tip')
base    = sim.getObject('/base')
target  = sim.getObject('/target')

# 3) Build IK group for Jacobian
ikEnv   = simIK.createEnvironment()
ikGroup = simIK.createGroup(ikEnv)
simIK.addElementFromScene(ikEnv, ikGroup, base, tip, target, simIK.constraint_x | simIK.constraint_y)
simIK.setGroupCalculation(ikEnv, ikGroup, simIK.method_pseudo_inverse, 0, 0)

# 4) Desired set-point (fixed, since base at origin)
desired_xy = np.array([1, 0.5])  # [x, y] in world coords

# — MOVE THE TARGET TO desired_xy RIGHT AWAY, KEEPING ITS Z —
pos = sim.getObjectPosition(target, -1)
sim.setObjectPosition(target, -1,
                      [float(desired_xy[0]),
                       float(desired_xy[1]),
                       pos[2]])

# 5) Controller params
dt       = 0.05  # control timestep [s]
duration = 10   # total time [s]
steps    = int(duration / dt)
gamma    = 10   # feedback gain

# 6) Data logs
times      = []
error_norm = []

# 7) Set-point control loop
for i in range(steps):
    t = i * dt
    # 7a) current end-effector pos
    M  = np.array(sim.getObjectMatrix(tip, sim.handle_world)).reshape(3, 4)
    ee = M[:2, 3]

    # 7b) Compute Jacobian J_xy
    jac_flat, _ = simIK.computeGroupJacobian(ikEnv, ikGroup)
    Jxy = np.array(jac_flat).reshape((2, 2))  # only X-Y rows

    # 7c) control law:
    error = desired_xy - ee 
    v_cmd = gamma * error
    lamda = 1e-5       # damping factor for numerical stability
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

# 8) stop sim
sim.stopSimulation()

# 9) plot error convergence
plt.figure()
plt.plot(times, error_norm, 'm-')
plt.xlabel('Time [s]')
plt.ylabel('||error|| [m]')
plt.title('Set-Point Error Convergence')
plt.grid(True)
plt.show()