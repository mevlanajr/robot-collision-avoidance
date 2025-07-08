import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math

# --- Main script ---
# 1) Connect & start
client = RemoteAPIClient()
sim    = client.require('sim')
sim.loadScene('/home/mevlanajr/Projects/robot-collision-avoidance/models/simple_RR_planar.ttt')

# 2) Get handles
target = sim.getObject('/target')
tip = sim.getObject('/tip')
base = sim.getObject('/base')
joint2 = sim.getObject('/joint2')

# 3) --- SWITCH TO SYNCHRONOUS MODE ---
sim.setStepping(True)
sim.startSimulation()

# 4) --- TRAJECTORY PARAMETERS ---
# Get initial positions
initial_tip_pos = np.array(sim.getObjectPosition(tip, -1))
base_pos = np.array(sim.getObjectPosition(base, -1))
joint2_pos = np.array(sim.getObjectPosition(joint2, -1))

# Measure the robot's link lengths to determine its maximum reach
link1_length = np.linalg.norm(joint2_pos - base_pos)
link2_length = np.linalg.norm(initial_tip_pos - joint2_pos)
max_reach = link1_length + link2_length

# --- SET YOUR DESIRED RADIUS HERE ---
desired_radius = 1.9 # <-- You can change this value
# ------------------------------------

# Safety check: ensure the desired radius is reachable
radius = desired_radius
if radius >= max_reach:
    radius = max_reach * 0.95 # Cap at 95% of max reach to be safe

# Define the circle's center and tracking speed
center_xy = np.array([base_pos[0], base_pos[1]])
angular_frequency = 0.5  # rad/s

# 5) --- Move smoothly to the start of the desired circle ---
start_angle = 0 # We will start the circle at angle 0 for simplicity
start_x = center_xy[0] + radius * math.cos(start_angle)
start_y = center_xy[1] + radius * math.sin(start_angle)
circle_start_pos = np.array([start_x, start_y, initial_tip_pos[2]])

# Number of steps for the initial move
move_steps = 100 
for i in range(move_steps + 1):
    # Linearly interpolate between the tip's start and the circle's start
    alpha = i / move_steps
    next_pos = (1 - alpha) * initial_tip_pos + alpha * circle_start_pos
    
    # Set the target's position and step the simulation
    sim.setObjectPosition(target, -1, next_pos.tolist())
    sim.step()
# ----------------------------------------------------------------

# 6) --- Data logs for plotting ---
times = []
error_norm = []
ee_path_log = []
desired_path_log = []

# 7) --- SYNCHRONOUS TRAJECTORY TRACKING LOOP ---
duration = 20   # seconds
start_sim_time = sim.getSimulationTime()
phi0 = start_angle # The starting angle is now what we defined for the circle

while sim.getSimulationTime() - start_sim_time < duration:
    # Calculate the desired position for the current simulation time
    t = sim.getSimulationTime() - start_sim_time
    ang = angular_frequency * t + phi0
    desired_xy = center_xy + np.array([
        radius * math.cos(ang),
        radius * math.sin(ang)
    ])
    
    # Set the target's new position
    target_pos_current = sim.getObjectPosition(target, -1)
    sim.setObjectPosition(target, -1, [float(desired_xy[0]), float(desired_xy[1]), target_pos_current[2]])
    
    # --- Log data directly in the main loop ---
    tip_pos_current = sim.getObjectPosition(tip, -1)
    times.append(t + start_sim_time) # Use absolute simulation time for the x-axis
    desired_path_log.append(desired_xy)
    ee_path_log.append(tip_pos_current[:2])
    error = np.linalg.norm(desired_xy - np.array(tip_pos_current[:2]))
    error_norm.append(error)
    
    # --- Advance the simulation by one step. This replaces time.sleep() ---
    sim.step()

# 8) Stop simulation
sim.stopSimulation()

# 9) Plotting
if times:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    ax1.plot(times, error_norm, 'm-')
    ax1.set_xlabel('Time [s]'); ax1.set_ylabel('||error|| [m]'); ax1.set_title('Tracking Error')
    ax1.grid(True)
    ee_path = np.array(ee_path_log)
    desired_path = np.array(desired_path_log)
    ax2.plot(desired_path[:, 0], desired_path[:, 1], 'r--', label='Desired Path (Target)')
    ax2.plot(ee_path[:, 0], ee_path[:, 1], 'b-', label='Actual Path (End-Effector)')
    ax2.set_xlabel('X [m]'); ax2.set_ylabel('Y [m]'); ax2.set_title('Path Tracking')
    ax2.legend(); ax2.grid(True); ax2.axis('equal')
    plt.tight_layout()
    plt.show()