# ME-556-Final-Project-Evan-Watts-MPC-HEDAC-Exploration

## Integrated Exploration and MPC Control with HEDAC Bias

## Overview
This project implements an integrated exploration and control framework for an omnidirectional mobile robot in a partially known 2D environment. The system combines:

- Local Rapidly-exploring Random Tree (RRT) planning  
- Model Predictive Control (MPC) for trajectory tracking  
- Heat-Equation–Driven Area Coverage (HEDAC) field to guide exploration  

The robot explores a cluttered workspace, avoids convex obstacles, and navigates toward a global goal using a unified exploration and control framework. The project is implemented as a self-contained MATLAB simulation.

---

## Key Features
- Omnidirectional mobile robot with velocity-level integrator dynamics  
- Randomly generated convex obstacles with safety buffers  
- Local RRT planning within a finite sensing radius  
- HEDAC heat field discouraging revisits and biasing RRT samples toward unexplored areas  
- MPC-based waypoint tracking with velocity and corridor constraints  
- Heat-aware MPC that scales control inputs to avoid high-temperature regions  
- Real-time visualization of robot motion, heat map, RRT nodes, corridors, and sensing radius  

---

## Code Structure
The main script: `integrated_explore_mpc_selfcontained_with_HEDAC.m`  

Sections include:

1. Environment and robot parameter definition  
2. Heat field (HEDAC) initialization and diffusion  
3. Random convex obstacle generation with buffers  
4. Visualization setup  
5. MPC prediction and cost matrix precomputation  
6. Main exploration and control loop  
7. Local RRT planning with heat-biased sampling  
8. MPC-based waypoint tracking inside convex corridors  
9. Heat updating, diffusion, and numerical result logging  

All helper functions are defined locally within the same MATLAB file.

---

## Core Functions

### Local RRT with HEDAC Bias
```matlab
[path, nodes] = local_rrt(p_start, p_goal, r_s_local, rrt_max_nodes, rrt_step, in_collision_segment, gridSize, HX, HY, heatT, beta, ~)
corridorPolys = path_to_corridor_simple(path, halfwidth)
[x_new, ok] = mpc_drive_to_waypoint(x0, wp_goal, S_big, Phi_big, Qbar, Rbar, H_quad_base, N_mpc, dt_ctrl, tol_waypoint, u_min, u_max, corridorPolys, segIdx, optsQP, HX, HY, heatT, heat_thres)
heatT = apply_heat_source_continuous(heatT, HX, HY, robotPos, r_source, strength_per_sec, dt, cap)
Tnew = diffuse_heat_fast(T, dx, alpha, dt)
```
## Dependencies
- MATLAB R2025a (or later)
- Optimization Toolbox (for quadprog)
- YALMIP (https://yalmip.github.io/)

---
## How to Run

1. Clone or download this repository.  
2. Open MATLAB and navigate to the project directory.  
3. Ensure the script is on the MATLAB path.  
4. Run:

```matlab
integrated_explore_mpc_selfcontained_with_HEDAC.m
```
5. A visualization window will display:
- Obstacles and safety buffers
- HEDAC heat map evolution
- RRT exploration nodes
- Convex corridors
- Robot trajectory and sensing radius

## Reproducibility and Randomness

The robot start position, goal location, and obstacle placement are randomly generated. To ensure reproducible results that work correctly, the simulation uses a fixed random seed:

```matlab
rng(195212115);  % fixed seed for reproducibility
```
Changing or removing the seed (e.g., using ```rng('shuffle')```) will generate different environments each run. Some random configurations may place the goal inside an obstacle, create obstacles too close to the start, or occasionally trigger index errors. Using the provided fixed seed ensures that the example runs reliably, however, for verification of the exploration of entirely random scenarios requires changing the seed.

## Author
### Evan Watts 
ME 556 – Robotics Final Project  
December 2025 
