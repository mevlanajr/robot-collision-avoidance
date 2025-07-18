import math
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# 1) Connect & API setup
client      = RemoteAPIClient()
sim         = client.require('sim')
simOMPL     = client.require('simOMPL')
simIK       = client.require('simIK')

# --- Load scene & get handles ---
sim.loadScene('/home/mevlanajr/Projects/robot-collision-avoidance/models/ompl_benchmark_scene.ttt')
base            = sim.getObject('/UR5')
tip             = sim.getObject('/UR5/tip')
goal            = sim.getObject('/goal')

joint_names     = [
    '/UR5/base_joint','/UR5/shoulder_joint','/UR5/elbow_joint',
    '/UR5/wrist_1_joint','/UR5/wrist_2_joint','/UR5/wrist_3_joint'
]
joint_handles   = [sim.getObject(n) for n in joint_names]
for i,h in enumerate(joint_handles):
    if h == -1:
        raise RuntimeError(f"Joint handle not found: {joint_names[i]}")

obstacle_names   = ['/obstacle1','/obstacle2','/obstacle3','/obstacle4','/obstacle5']
obstacle_handles = [sim.getObject(n) for n in obstacle_names]

# ====================================================================
# --- STAGE 1: PATH PLANNING (OFFLINE) ---
# ====================================================================
print("--- STAGE 1: PATH PLANNING ---")
task = simOMPL.createTask('task')
simOMPL.setStateSpaceForJoints(task, joint_handles, [1,1,1,0,0,0])
links_to_check = joint_handles + [tip]

robot_coll = sim.createCollection()
sim.addItemToCollection(robot_coll, sim.handle_tree, base, 0)

# now avoid that collection against each obstacle
collision_pairs = []
for obs in obstacle_handles:
    collision_pairs += [robot_coll, obs]

simOMPL.setCollisionPairs(task, collision_pairs)

start_q = [sim.getJointPosition(j) for j in joint_handles]
simOMPL.setStartState(task, start_q)

# Find a valid goal configuration using IK
sim.startSimulation(); sim.step()
ikEnv   = simIK.createEnvironment()
ikGroup = simIK.createGroup(ikEnv)
elementID, simToIkMap, _ = simIK.addElementFromScene(
    ikEnv, ikGroup, base, tip, goal, simIK.constraint_position
)
if elementID == -1: raise RuntimeError("IK element creation failed!")
simIK.syncFromSim(ikEnv, [ikGroup])
ik_jhs = [simToIkMap.get(j) for j in joint_handles]
if any(h is None for h in ik_jhs):
    raise RuntimeError("IK handle mapping failed. Check scene hierarchy.")
configs = simIK.findConfigs(ikEnv, ikGroup, ik_jhs, {
    'maxTime': 15.0,  # 15 seconds for IK search
    'maxDist': 1e-1    # 1 cm accuracy
})
simIK.eraseEnvironment(ikEnv)
sim.stopSimulation()

if not configs: raise RuntimeError("No IK solution found for goal pose.")
goal_q = configs[0]
simOMPL.setGoalState(task, goal_q)

# Compute the path using RRT
simOMPL.setAlgorithm(task, simOMPL.Algorithm.RRT)
simOMPL.setup(task)
print("Computing path with RRT...")
result, path = simOMPL.compute(task, 30.0, 1)
simOMPL.destroyTask(task)
if not result: raise RuntimeError("OMPL planning failed.")
print("Path found successfully!")

# ====================================================================
# --- STAGE 2: TRAJECTORY GENERATION & EXECUTION ---
# ====================================================================
print("\n--- STAGE 2: TRAJECTORY GENERATION & EXECUTION ---")

# --- Define robot's dynamic limits for smooth trajectory generation ---
fk_max_vel   = 180 * math.pi / 180  # 180 deg/s
fk_max_accel =  40 * math.pi / 180  # 40 deg/s²

# Build interleaved [min1,max1,min2,max2,...] lists:
vel_limits = [fk_max_vel]*6
acc_limits = [fk_max_accel]*6

min_max_vel   = [v for pair in zip([-v for v in vel_limits],   vel_limits)   for v in pair]
min_max_accel = [a for pair in zip([-a for a in acc_limits], acc_limits) for a in pair]
# ----------------------------------------------------------------------

# --- Generate a time-optimal, smooth trajectory from the path ---
print("Generating smooth trajectory from path...")
path_lengths, _ = sim.getPathLengths(path, 6)

trajectory_pts, trajectory_times, _ = sim.generateTimeOptimalTrajectory(
    path, path_lengths, min_max_vel, min_max_accel
)
print("Smooth trajectory generated.")
# --------------------------------------------------------------------

# Data logs
times, error_norm, actual_path, desired_path_log = [], [], [], []
dist_logs   = {name: [] for name in obstacle_names}
ee_to_goal  = []  # NEW: end-effector → goal distance

# Synchronous mode for the controller
sim.setStepping(True)
sim.startSimulation()

dt = sim.getSimulationTimeStep()
start_sim_time = sim.getSimulationTime()

# --- Main Tracking Loop for the pre-computed trajectory ---
print("Executing smooth trajectory...")
while sim.getSimulationTime() - start_sim_time < trajectory_times[-1]:
    t = sim.getSimulationTime() - start_sim_time
    
    # Get the desired joint configuration from the pre-computed blueprint
    q_desired = sim.getPathInterpolatedConfig(trajectory_pts, trajectory_times, t)
    
    # Command the robot directly to the desired joint configuration
    for i in range(6):
        sim.setJointPosition(joint_handles[i], q_desired[i])
        
    # --- Logging and Stepping ---
    ee_pos = np.array(sim.getObjectPosition(tip, -1))
    
    # For plotting, we calculate the desired EE pos for this q_desired
    temp_start_q = [sim.getJointPosition(j) for j in joint_handles]  # save
    for i in range(6): sim.setJointPosition(joint_handles[i], q_desired[i])
    desired_ee_pos = np.array(sim.getObjectPosition(tip, -1))
    for i in range(6): sim.setJointPosition(joint_handles[i], temp_start_q[i])  # restore
    
    times.append(t)
    actual_path.append(ee_pos.copy())
    desired_path_log.append(desired_ee_pos)
    error_norm.append(np.linalg.norm(desired_ee_pos - ee_pos))
    
    # obstacle distances
    for name, h_obs in zip(obstacle_names, obstacle_handles):
        pos_obs = np.array(sim.getObjectPosition(h_obs, -1))
        dist_logs[name].append(np.linalg.norm(ee_pos - pos_obs))
    
    # NEW: log end-effector → goal distance
    goal_pos = np.array(sim.getObjectPosition(goal, -1))
    ee_to_goal.append(np.linalg.norm(ee_pos - goal_pos))
    
    sim.step()
# -----------------------------------------------------------------

# --- Cleanup ---
sim.stopSimulation()
print("Controller finished.")

# --- Plotting ---
if times:
    fig = plt.figure(figsize=(15,8))
    # 2×2 layout:
    ax1 = fig.add_subplot(2,2,1, projection='3d')
    planned = np.array(desired_path_log)
    actual  = np.array(actual_path)
    ax1.plot(planned[:,0], planned[:,1], planned[:,2], 'r--', label='Desired Trajectory')
    ax1.plot(actual[:,0], actual[:,1], actual[:,2], 'b-',  label='Actual Trajectory')
    ax1.set_title('End-Effector Trajectory'); ax1.legend()

    ax2 = fig.add_subplot(2,2,2)
    ax2.plot(times, error_norm, 'm-')
    ax2.set_title('Tracking Error')
    ax2.set_xlabel('Time [s]'); ax2.set_ylabel('||error|| [m]'); ax2.grid(True)

    ax3 = fig.add_subplot(2,2,3)
    for name, dlist in dist_logs.items():
        ax3.plot(times, dlist, label=f'distance to {name}')
    ax3.set_title('EE–Obstacle Distances')
    ax3.set_xlabel('Time [s]'); ax3.set_ylabel('Distance [m]')
    ax3.legend(); ax3.grid(True)

    ax4 = fig.add_subplot(2,2,4)  # NEW fourth subplot
    ax4.plot(times, ee_to_goal, 'k-')
    ax4.set_title('EE → Goal Distance')
    ax4.set_xlabel('Time [s]'); ax4.set_ylabel('Distance [m]'); ax4.grid(True)

    plt.tight_layout()
    plt.show()