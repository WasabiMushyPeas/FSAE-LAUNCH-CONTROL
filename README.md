# Cal Poly Racing EV 2026 | Launch Control & Vehicle Dynamics Simulation

![MATLAB](https://img.shields.io/badge/MATLAB-R2025B-orange)
![Simulink](https://img.shields.io/badge/Simulink-Required-blue)
![Status](https://img.shields.io/badge/Status-Active%20Development-brightgreen)
![Team](https://img.shields.io/badge/Team-Cal%20Poly%20Racing%20EV-red)

> Simulation and controls workflow for optimizing launch performance in the Formula SAE EV acceleration event.

## Table of Contents
- [Cal Poly Racing EV 2026 | Launch Control \& Vehicle Dynamics Simulation](#cal-poly-racing-ev-2026--launch-control--vehicle-dynamics-simulation)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Objectives](#objectives)
  - [Repository Structure](#repository-structure)
    - [File Summary](#file-summary)
  - [Simulink Architecture](#simulink-architecture)
    - [Aero Block](#aero-block)
    - [Weight Transfer](#weight-transfer)
    - [Tire Model](#tire-model)
    - [Motor Torque](#motor-torque)
    - [Controller](#controller)
  - [How to Run](#how-to-run)
    - [Requirements](#requirements)
    - [Workflow](#workflow)
    - [Performance Metrics](#performance-metrics)
    - [Time-History Outputs](#time-history-outputs)
  - [Model Assumptions and Scope](#model-assumptions-and-scope)
  - [Current Limitations](#current-limitations)
  - [Results and Validation](#results-and-validation)

## Overview
This repository contains the **Simulink model** and **MATLAB live scripts** used to simulate, analyze, and optimize the launch control strategy for the **Cal Poly Racing EV 2026** Formula SAE car in the **0-75 m acceleration event**.

The project is built around a **rear-wheel-drive electric vehicle model** and focuses on minimizing launch time through the combination of:

- a custom **PID-based traction controller**
- a **time-based baseline throttle strategy**
- a simplified but highly practical **tire force model**

The result is a simulation workflow intended to support a near-production controls process: parameterize the vehicle, run the launch model, inspect system behavior, controller gains, and drivetrain assumptions until the car launches harder, cleaner, and more consistently.

## Objectives
The primary objective of this project is to reduce the vehicle's **0-75 meter acceleration time** while preserving controllability and limiting excessive wheel slip.

Secondary goals include:

- improving **0-60 mph performance predictions**
- identifying an effective **final drive ratio**
- tuning a **traction control PID loop** around a target slip ratio
- understanding how **weight transfer, tire loading, and drivetrain compliance** influence launch performance
- creating a repeatable simulation workflow for future team members and design reviews

## Repository Structure

### File Summary
| File | Purpose |
|---|---|
| `.gitattributes` | Git attributes configuration for normalizing repository file handling. |
| `FINAL_DRIVE_RATIO.mlx` | Iterative script used to sweep final drive ratios and identify the best mechanical advantage for acceleration performance. |
| `FINAL_DRIVE_RATIO_OLD.mlx` | Previous iteration of the final drive sweep workflow retained for reference and comparison. |
| `PARAMS.mlx` | Main script used to configure variables, initialize model parameters, run setup logic, and graph simulation results. |
| `PID_TUNER.mlx` | Script used for faster controller gain sweeps and traction-control tuning studies. |
| `README.md` | Project documentation covering repository structure, model architecture, workflow, and assumptions. |
| `SIM-PICTURE.pdf` | Reference PDF showing a model or simulation diagram used for documentation. |
| `sweep_results.csv` | Exported results from a parameter sweep study for quick review outside MATLAB. |
| `TC_PID_sweep_results.mat` | Saved MATLAB data file containing PID sweep results for later analysis. |
| `TC_SIM.slx` | Core Simulink model for launch control, traction control, drivetrain response, and vehicle longitudinal dynamics. |
| `TC_SIM.slx.original` | Backup copy of the Simulink model preserved for comparison or recovery. |
| `TC_SIM.slxc` | Simulink generated cache or compiled artifact for the main model. |
| `TC_SIM_FAST.slx` | Streamlined Simulink model paired with `PID_TUNER.mlx` for rapid PID tuning iterations. |
| `figures/` | Directory for generated plots, images, or other visual outputs used in the project. |
| `slprj/` | Simulink generated build and cache directory created during model compilation and execution. |


## Simulink Architecture
The `TC_SIM.slx` model is organized around a set of subsystems that together simulate the launch event from throttle command to tire force generation and vehicle motion.

### Aero Block
The aero subsystem computes the aerodynamic forces acting on the vehicle during the run, including:

- **drag**, which opposes acceleration
- **downforce**, which increases tire normal load and can improve traction at speed

While the acceleration event is short, these effects still matter as vehicle speed rises and can influence both force balance and rear tire loading.

### Weight Transfer
This subsystem calculates **longitudinal weight transfer** under acceleration and updates the **dynamic normal load on the driven rear axle**.

This is critical because available tire force depends strongly on vertical load. During launch, rearward load transfer can increase traction potential at the driven wheels, making this block one of the most important links between chassis behavior and controller performance.

### Tire Model
The tire model calculates **slip ratio** and uses a simplified Pacejka-style relationship to estimate longitudinal tire force.

The current implementation uses hardcoded coefficients and a grip-scaling factor:

```matlab
% --- Hardcoded Model Coefficients ---
B = coder.const(10.4);
C = coder.const(1.58);
D1 = coder.const(3.02);
D2 = coder.const(0.0008 / 4.448221615);    % per N instead of per lbf 

% --- Apply the Simplified Pacejka Formula ---
mu = (D1 - (D2) .* (normalForce/2)) * Grip_Fact;

% Calculate the peak force (D_factor), which is mu * Fz
D_factor = mu .* (normalForce/2);

% Apply the simplified Pacejka sine formula
Fx = (D_factor .* sin(C * atan(B * slipRatio)));
```

At a high level, the model:

- computes a load-sensitive friction coefficient `mu`
- scales grip using `Grip_Fact`
- converts normal load into peak longitudinal force potential
- applies a simplified sine-arctangent Pacejka relationship to estimate tire force

This provides a practical balance between physical realism and simulation speed for launch-control studies.

### Motor Torque
The motor subsystem applies a **hardcoded rpm vs. torque curve** to represent available torque over the launch event.


### Controller

The control strategy consists of:

- a **time-based lookup table** that defines a baseline target throttle or torque percentage during launch
- a **PID controller** that outputs a **torque correction** relative to that baseline command
- a control objective of maintaining a **target slip ratio of `0.13`**
- a blending strategy that transitions smoothly toward the primary throttle input at higher vehicle speeds

This structure allows the system to launch aggressively while still correcting for excessive wheel slip when the tire approaches or moves beyond peak traction.

## How to Run
### Requirements
- **MATLAB R2025B**
- **Simulink**

### Workflow
1. Open `PARAMS.mlx`.
2. Configure the variables in `PARAMS.mlx` as desired.
3. Run `PARAMS.mlx` to initialize workspace variables and setup parameters.
4. Click **Run** in Matlab Live Editor.
5. Inspect generated plots, logged signals, and workspace outputs.


### Performance Metrics
- **0-75 m time**
- **0-60 mph time**

### Time-History Outputs
The following signals are tracked or plotted versus time:

- **PID Output**
- **Slip Ratio**
- **Error**
- **Distance**
- **Velocity**
- **Acceleration**
- **Normal Force on Rear Axle**
- **Target Motor Torque %**
- **Tractive Force**
- **Mu**

These outputs are intended to support both performance optimization and controls debugging.

## Model Assumptions and Scope
This model is intentionally focused on the FSAE acceleration event and uses a fixed set of vehicle, tire, aerodynamic, and controller parameters to keep the workflow fast, interpretable, and useful for iteration.

The current model setup assumes:

- **straight-line rear-wheel-drive EV acceleration** for the **0-75 m event**
- **vehicle mass `Mv = 285.763 kg`**, **wheel radius `r = 0.203 m`**, **CG height `h_cg = 0.25273 m`**, and **wheelbase `W = 1.53035 m`**
- **static CG location `cg_f = 0.734568 m`** measured from the front axle
- **drivetrain represented with lumped rotational inertias** using **`J = 1.2 kg*m^2`**, **`J_Motor = 0.25 kg*m^2`**, and **`J_Wheel = 0.165 kg*m^2`**
- **half-shaft compliance modeled as a torsional spring-damper** with **stiffness `K = 2552 Nm/rad`** and **damping `C = 30 Nms/rad`**
- **fixed final drive ratio `fd = 4.5`** and **drivetrain efficiency `Drive_Train = 0.89`**
- **simplified longitudinal tire behavior** with **`Grip_Fact = 1.0`** and **`Grip_Influence = 1.2`** used to scale available grip and table sensitivity
- **aerodynamics represented with constant coefficients** using **frontal area `A = 1 m^2`**, **air density `rho = 1.225 kg/m^3`**, **drag coefficient `Cd = 1`**, **lift coefficient `Cl = 3.8`**, and **rear aero load distribution `Cp = 0.53`**
- **traction control targeting slip ratio `Slip_Target = 0.13`** with **initial driver torque request `T_request = 150 Nm`**
- **blended controller scheduling between `3.0 m/s` and `8.0 m/s`** using **`Start_Blend = 3.0`** and **`End_Blend = 8.0`**
- **actuator and speed limits** of **`Max_Wheel_Omega = 183.3 rad/s`** and **`Max_Motor_Torque = 150 Nm`**
- **gravity fixed at `9.80665 m/s^2`**
- **PID slip controller gains `P = 1.75`, `I = 4`, `D = 0`**, with **sample time `Ts = 0.01 s`**
- **simulation-first development**, with no real-vehicle correlation completed yet

## Current Limitations
This repository is actively under development, and several known areas still require refinement:

- **half-shaft oscillation behavior appears to be imperfectly matched**, suggesting the compliance model or damping assumptions may need further tuning
- the **PID loop is somewhat unstable** in parts of the launch event and may require additional tuning or control-structure improvements
- the **baseline lookup table is not yet fully optimized**
- model performance has **not yet been validated against real vehicle data**

These limitations do not reduce the value of the repository as a design and tuning tool, but they should be kept in mind when interpreting results.

## Results and Validation
This section is reserved for future simulation summaries, screenshots, and validation comparisons.

Planned additions include:

- best simulated **0-75 m** and **0-60 mph** results
- optimal **final drive ratio** findings
- representative **PID tuning comparisons**
- example **plots and screenshots** from MATLAB and Simulink
- eventual comparison between **simulation predictions and real vehicle data**

---
