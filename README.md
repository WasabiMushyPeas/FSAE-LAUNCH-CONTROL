# Cal Poly Racing EV 2026 | Launch Control & Vehicle Dynamics Simulation

![MATLAB](https://img.shields.io/badge/MATLAB-R2025B-orange)
![Simulink](https://img.shields.io/badge/Simulink-Required-blue)
![Status](https://img.shields.io/badge/Status-Active%20Development-brightgreen)
![Team](https://img.shields.io/badge/Team-Cal%20Poly%20Racing%20EV-red)

> Simulation and controls workflow for optimizing launch performance in the Formula SAE EV acceleration event.

## Table of Contents
- [Overview](#overview)
- [Objectives](#objectives)
- [Tech Stack](#tech-stack)
- [Features](#features)
- [Repository Structure](#repository-structure)
- [Simulink Architecture](#simulink-architecture)
  - [Aero Block](#aero-block)
  - [Weight Transfer](#weight-transfer)
  - [Tire Model](#tire-model)
  - [Motor Torque](#motor-torque)
  - [Controller](#controller)
  - [Vehicle and Drivetrain Dynamics](#vehicle-and-drivetrain-dynamics)
- [How to Run](#how-to-run)
- [Simulation Outputs](#simulation-outputs)
- [Model Assumptions and Scope](#model-assumptions-and-scope)
- [Current Limitations](#current-limitations)
- [Results and Validation](#results-and-validation)

## Overview
This repository contains the **Simulink model** and **MATLAB live scripts** used to simulate, analyze, and optimize the launch control strategy for the **Cal Poly Racing EV 2026** Formula SAE car in the **0-75 m acceleration event**.

The project is built around a **rear-wheel-drive electric vehicle model** and focuses on minimizing launch time through the combination of:

- a custom **PID-based traction controller**
- a **time-based baseline throttle strategy**
- a simplified but highly practical **tire force model**
- an **advanced drivetrain representation** that captures half-shaft compliance and oscillatory behavior

The result is a simulation workflow intended to support a near-production controls process: parameterize the vehicle, run the launch model, inspect system behavior, and iterate on gearing, controller gains, and drivetrain assumptions until the car launches harder, cleaner, and more consistently.

## Objectives
The primary objective of this project is to reduce the vehicle's **0-75 meter acceleration time** while preserving controllability and limiting excessive wheel slip.

Secondary goals include:

- improving **0-60 mph performance predictions**
- identifying an effective **final drive ratio**
- tuning a **traction control PID loop** around a target slip ratio
- understanding how **weight transfer, tire loading, and drivetrain compliance** influence launch performance
- creating a repeatable simulation workflow for future team members and design reviews

## Tech Stack
- **MATLAB R2025B**
- **Simulink**

## Features
- **Launch-control-focused vehicle simulation** for the FSAE acceleration event
- **Rear-wheel-drive EV drivetrain model** tailored for straight-line performance analysis
- **Custom PID-based slip controller** targeting a defined optimal slip ratio
- **Baseline throttle lookup table** for feedforward launch shaping
- **Dynamic rear axle normal force estimation** through longitudinal weight transfer
- **Simplified Pacejka-based tire force model** with grip-factor scaling
- **Half-shaft spring-mass-damper model** to capture drivetrain oscillation and resonance effects
- **Final drive ratio sweep tools** for mechanical optimization
- **Fast PID tuning workflow** for rapid iteration
- **MATLAB post-processing and plotting** for time-history analysis and performance comparison

## Repository Structure

### File Summary
| File | Purpose |
|---|---|
| `PARAMS.mlx` | Main script used to configure variables, initialize model parameters, run setup logic, and graph simulation results. |
| `TC_SIM.slx` | Core Simulink model for launch control, traction control, drivetrain response, and vehicle longitudinal dynamics. |
| `FINAL_DRIVE_RATIO.mlx` | Iterative script used to sweep final drive ratios and identify the best mechanical advantage for acceleration performance. |
| `FINAL_DRIVE_RATIO_OLD.mlx` | Previous iteration of the final drive sweep workflow retained for reference and comparison. |
| `PID_TUNER.mlx` | Script used for faster controller gain sweeps and traction-control tuning studies. |
| `TC_SIM_FAST.slx` | Streamlined Simulink model paired with `PID_TUNER.mlx` for rapid PID tuning iterations. |

### Repository Tree
```text
.
├── PARAMS.mlx
├── TC_SIM.slx
├── FINAL_DRIVE_RATIO.mlx
├── FINAL_DRIVE_RATIO_OLD.mlx
├── PID_TUNER.mlx
└── TC_SIM_FAST.slx
```

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
The motor subsystem applies a **hardcoded EV power/torque curve** to represent available propulsion over the launch event.

This block acts as the powertrain force source and works with the controller and drivetrain model to determine how much torque ultimately reaches the tire contact patch.

### Controller
The controller combines **feedforward launch shaping** with **closed-loop slip correction**.

The control strategy consists of:

- a **time-based lookup table** that defines a baseline target throttle or torque percentage during launch
- a **PID controller** that outputs a **torque correction** relative to that baseline command
- a control objective of maintaining a **target slip ratio of `0.13`**
- a blending strategy that transitions smoothly toward the primary throttle input at higher vehicle speeds

This structure allows the system to launch aggressively while still correcting for excessive wheel slip when the tire approaches or moves beyond peak traction.

### Vehicle and Drivetrain Dynamics
This subsystem models the longitudinal response of the car and includes drivetrain dynamics beyond a simple rigid connection.

Key behaviors represented include:

- **vehicle acceleration and velocity propagation**
- **distance integration** for event-time measurement
- a **0.02 s electrical/motor response delay**
- an advanced **half-shaft spring-mass-damper model**

The half-shaft model uses:

- wheel-side inertia
- motor-side inertia
- half-shaft stiffness
- half-shaft damping

This enables the model to simulate **drivetrain resonance, torque windup, and oscillatory behavior** that can meaningfully affect early-launch response and traction-controller stability.

## How to Run
### Requirements
- **MATLAB R2025B**
- **Simulink**

### Workflow
1. Open `PARAMS.mlx`.
2. Configure the variables in `PARAMS.mlx` as desired.
3. Run `PARAMS.mlx` to initialize workspace variables and setup parameters.
4. Open `TC_SIM.slx`.
5. Click **Run** in Simulink.
6. Inspect generated plots, logged signals, and workspace outputs.

### Recommended Starting Point
The simulation is intended to run reasonably well with the existing setup, so new users can usually start with the default configuration before tuning parameters.

## Simulation Outputs
The workflow is set up to evaluate headline launch metrics as well as key internal controller and vehicle states.

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
This model is intentionally focused on the FSAE acceleration event and uses a controlled set of assumptions to keep the workflow fast, interpretable, and useful for iteration.

Current scope and assumptions include:

- **straight-line acceleration only**
- **rear-wheel-drive EV configuration**
- **launch optimization for the 0-75 m event**
- **high-level drivetrain compliance modeling** rather than full multibody drivetrain simulation
- **simplified tire behavior** using a reduced Pacejka-style longitudinal force formulation
- **surface grip effects represented through a tunable grip factor**
- **controller target slip ratio fixed at `0.13`** in the present implementation
- **simulation-first workflow**, with no real-vehicle correlation completed yet

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

Built for internal development, simulation-driven controls design, and knowledge transfer within **Cal Poly Racing EV**.
