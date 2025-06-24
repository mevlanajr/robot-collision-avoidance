import math
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Connect and start simulation
task_client = RemoteAPIClient()
sim = task_client.require('sim')
sim.loadScene(
    '/home/mevlanajr/Projects/robot-collision-avoidance/models/simple_RR_2DOF.ttt'
)
sim.setStepping(True)
sim.startSimulation()

# Retrieve joint handles
joint1 = sim.getObject('/joint1')
joint2 = sim.getObject('/joint2')
assert joint1 >= 0 and joint2 >= 0, 'Joint handles not found'

# Control parameters
kp = 5.0
dt = 0.05
steps = int(20 / dt)

# Data storage
times = []
q1_actual, q1_desired = [], []
q2_actual, q2_desired = [], []

# P‐controller velocity loop
for i in range(steps):
    t = i * dt
    q1_ref = 0.5 * math.sin(0.5 * t)
    q2_ref = 0.3 * math.sin(0.7 * t)
    q1 = sim.getJointPosition(joint1)
    q2 = sim.getJointPosition(joint2)
    times.append(t)
    q1_desired.append(q1_ref)
    q1_actual.append(q1)
    q2_desired.append(q2_ref)
    q2_actual.append(q2)
    sim.setJointTargetVelocity(joint1, kp * (q1_ref - q1))
    sim.setJointTargetVelocity(joint2, kp * (q2_ref - q2))
    sim.step()

# Stop simulation
sim.stopSimulation()

# Plot tracking results
plt.figure()
plt.plot(times, q1_desired, label='Joint 1 desired')
plt.plot(times, q1_actual, label='Joint 1 actual')
plt.xlabel('Time [s]')
plt.ylabel('Angle [rad]')
plt.legend()

plt.figure()
plt.plot(times, q2_desired, label='Joint 2 desired')
plt.plot(times, q2_actual, label='Joint 2 actual')
plt.xlabel('Time [s]')
plt.ylabel('Angle [rad]')
plt.legend()

plt.show()