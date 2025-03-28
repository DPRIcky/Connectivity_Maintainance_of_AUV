LOS-Constrained Multi-Robot Coordination Using MLCCST
This repository implements a Control Barrier Function (CBF)-based framework for maintaining Line-of-Sight (LOS) connectivity in a multi-robot system using the Minimum Line-of-Sight Connectivity Constraint Spanning Tree (MLCCST) algorithm. The simulation models a team of Autonomous Underwater Vehicles (AUVs) navigating towards task goals while preserving global and subgroup connectivity in obstacle-filled environments.

<br>
🚀 Features
Implementation of MLCCST-based connectivity maintenance.

CBF-based safety, LOS, and obstacle avoidance constraints.

Dynamic relay assignment to preserve tree-like communication.

Goal-oriented circular convergence formation.

Configurable robot count, communication range, and environment.

Visualization with task zones, robots, connectivity edges, and obstacles.


Getting Started
Prerequisites
MATLAB R2021a or later

Optimization Toolbox (for quadprog)

To Run the Simulation
Single Step Visualization

matlab
Copy
Edit
main.m
Animated Continuous Simulation

matlab
Copy
Edit
main_constant_update.m
This version updates the robot positions in real-time and shows dynamic MLCCST construction, robot relays, and convergence.

<br>
Parameters to Tune
You can change the following parameters in either main file:

N: Number of robots

R_c: Communication range

step_size: Robot movement speed

relay_threshold: Edge length that triggers relay assignment

<br>
Future Extensions
Robust CBF-based obstacle avoidance

Controlled growth/shrinkage of communication trees

Integration with Control Lyapunov Functions (CLFs)

Deployment on underwater robotic platforms via ROS 2 / Gazebo

<br>
📸 Demo Snapshots
