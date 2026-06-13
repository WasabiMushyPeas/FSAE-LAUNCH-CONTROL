# Cal Poly Racing EV 2026 | Launch Control & Vehicle Dynamics Simulation

![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-orange)
![Simulink](https://img.shields.io/badge/Simulink-Required-blue)
![Status](https://img.shields.io/badge/Status-Active%20Development-brightgreen)
![Team](https://img.shields.io/badge/Team-Cal%20Poly%20Racing%20EV-red)
![Real World Best](https://img.shields.io/badge/Real%20World%20Best%200--75m-4.7s-yellow)

> Simulation and controls workflow for optimizing launch performance in the Formula SAE EV acceleration event.

## Table of Contents
- [Overview](#overview)
- [Objectives](#objectives)
- [Repository Structure](#repository-structure)
- [Simulink Architecture](#simulink-architecture)
  - [Aerodynamics (Areo1)](#aerodynamics-areo1)
  - [Tire Model](#tire-model)
  - [Max Torque (EMRAX 208 Curve)](#max-torque-emrax-208-curve)
  - [Launch Controller](#launch-controller)
  - [Half-Car Suspension Normal Load](#half-car-suspension-normal-load)
  - [Longitudinal Load Transfer (Legacy, Disabled)](#longitudinal-load-transfer-legacy-disabled)
  - [Vehicle & Drivetrain Dynamics](#vehicle--drivetrain-dynamics)
- [Scripts and Tooling](#scripts-and-tooling)
- [How to Run](#how-to-run)
  - [Requirements](#requirements)
  - [Workflow](#workflow)
  - [Alternative Entry Points](#alternative-entry-points)
  - [Performance Metrics](#performance-metrics)
  - [Time-History Outputs](#time-history-outputs)
- [Model Parameters](#model-parameters)
- [Model Assumptions and Scope](#model-assumptions-and-scope)
- [Current Limitations](#current-limitations)
- [Results and Validation](#results-and-validation)

## Overview
This repository contains the **Simulink model** and **MATLAB scripts** used to simulate, analyze, and optimize the launch control strategy for the **Cal Poly Racing EV 2026** Formula SAE car in the **0–75 m acceleration event**.

The project is built around a **rear-wheel-drive electric vehicle model** and focuses on minimizing launch time through the combination of:

- a **fixed-point launch controller** that mirrors the car's embedded firmware (feedforward launch table plus a discrete slip-control PID)
- a **time-based launch (feedforward) table** that scales the available motor torque
- a simplified but practical **Pacejka-style tire force model**
- a **4-DOF half-car suspension model** that supplies the dynamic rear-axle normal load

The result is a controls workflow intended to support a near-production process: parameterize the vehicle in `PARAMS.m`, run the launch model, and inspect system behavior, controller response, and drivetrain assumptions until the car launches harder, cleaner, and more consistently.

The team's **best real-world 0–75 m time is 4.7 s**, which is the benchmark this simulation is being tuned and validated against.

## Objectives
The primary objective is to reduce the vehicle's **0–75 m acceleration time** while preserving controllability and limiting excessive wheel slip.

Secondary goals include:

- improving **0–60 mph (26.8224 m/s)** performance predictions
- identifying an effective **final drive ratio**
- tuning the **launch-control slip loop** around a target slip ratio
- understanding how **weight transfer, tire loading, aerodynamics, and drivetrain compliance** influence launch performance
- correlating the simulation against the **4.7 s real-world benchmark**
- creating a repeatable simulation workflow for future team members and design reviews

## Repository Structure

The table below lists the version-controlled files. Generated artifacts (`figures/`, `*.csv`, `*.slxc`, `slprj/`, and the temporary `_tc_sim_unpack*/` and `_tc_sim_zip.zip` inspection dumps) are produced at runtime and are intentionally **git-ignored**.

| File | Purpose |
|---|---|
| `PARAMS.m` | **Main entry point.** Defines all vehicle, tire, aero, drivetrain, half-car, and launch-control parameters; generates the sigmoid launch table; runs `TC_SIM.slx`; logs signals; and exports plots/data. |
| `PARAMS_IDEAL_LAUNCH_TABLE.m` | Alternative entry point. Generates an *ideal feedforward* launch table by inverting the longitudinal vehicle + tire model at a target slip ratio, then optionally runs `TC_SIM.slx` open-loop with that table. |
| `THROTTLE_LOOKUP_GENERATOR.m` | Coordinate-descent optimizer that tunes a time-varying throttle lookup table against a slip-tracking objective and exports `throttle_lookup_table.csv` / `.mat`. |
| `FINAL_DRIVE_GRIP_SWEEP_75M.m` | Parallel (`parsim`) 2-D sweep of final drive ratio × tire grip factor, plotting simulated 75 m time for each grip level. |
| `PID_TUNER.m` | Parallel `P`/`I` gain sweep run against the reduced `TC_SIM_FAST.slx` model for fast controller-tuning studies. |
| `TC_SIM.slx` | **Core Simulink model:** launch control, tire forces, half-car normal load, aerodynamics, drivetrain/half-shaft compliance, and longitudinal dynamics. |
| `TC_SIM_FAST.slx` | Reduced model paired with `PID_TUNER.m`. Uses a single discrete-transfer-function PID (`P`, `I`, `D`, `Ts`) for rapid sweeps. |
| `throttle_lookup_table.mat` | Saved output of `THROTTLE_LOOKUP_GENERATOR.m` (optimized table, optimization history, and metrics). |
| `ideal_launch_outputs/` | PNG outputs from `PARAMS_IDEAL_LAUNCH_TABLE.m` (ideal command, required torque, acceleration, and sim comparisons). |
| `README.md` | This document. |
| `.gitattributes`, `.gitignore` | Repository configuration. |

## Simulink Architecture
`TC_SIM.slx` is organized into top-level subsystems that simulate the launch event from torque command to tire force generation and vehicle motion. The launch controller and its I/O are sampled at **`LC_Ts_sec = 0.01 s` (100 Hz)** through Zero-Order-Hold blocks, matching the car's real launch-control loop rate.

### Aerodynamics (Areo1)
Computes the aerodynamic forces acting on the vehicle from vehicle speed:

- **drag**, which opposes acceleration, fed into the vehicle dynamics
- **downforce**, which is passed to the half-car model and increases rear normal load (and therefore grip) at speed

While the acceleration event is short, these effects still matter as speed rises.

### Tire Model
The tire subsystem contains three pieces:

- **Slip Ratio** (`compute_slip`): `slip = (V_wheel − V_velocity) / max(|V_velocity|, |V_wheel|, 0.1)`, with a 0.1 m/s denominator floor to keep the launch numerically well-behaved.
- **Friction Coefficient Calc** (`calculatePacejkaFx`): a simplified, load-sensitive Pacejka relationship.
- **Tire relaxation** (`tire_relaxation_derivative`): a first-order lag (`tau_tire`) so tire force does not respond instantaneously to slip.

The Pacejka block uses hardcoded coefficients and a grip-scaling factor:

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

At a high level, the model computes a load-sensitive friction coefficient `mu`, scales grip with `Grip_Fact`, converts normal load into peak longitudinal force potential, and applies the simplified sine–arctangent Pacejka relationship. The rear-axle normal load (`normalForce`) comes from the half-car suspension model.

### Max Torque (EMRAX 208 Curve)
A 1-D lookup table maps **motor RPM → maximum available motor torque** using the EMRAX 208 curve (≈150 Nm plateau at low/mid speed, tapering to ≈102 Nm near 7000 rpm). This rpm-dependent ceiling is the **base torque** the launch controller scales.

### Launch Controller
The controller (`car_launch_control_active`) is a **fixed-point integer reimplementation of the car's embedded launch-control firmware** (`%#codegen`). It takes simulation time, the available base torque, slip ratio, and vehicle speed, and produces the final motor torque command.

Its strategy:

- a **feedforward launch table** (`launchTimeMs` / `launchCmd`, in permille of available torque) interpolated by time and scaled by a grip factor
- a **discrete PID slip-correction loop** in integer math, with error `= LC_SLIP_TARGET − slip` (slip in permyriad, so `1300 = 13.00%`)
- **conditional-integration anti-windup**: the correction state only updates when the output is unsaturated, or when saturated in the direction that the current error would relieve
- a **speed-based blend** (`alpha`, 0→1 between `LC_START_BLEND` and `LC_END_BLEND`) that fades the PID correction in as the car accelerates
- final command = grip-scaled feedforward + blended correction, clamped to `[LC_MIN_CMD, LC_MAX_CMD]` (0–1000 permille), then applied as a fraction of the available motor torque: `torqueNm = baseTorqueNm × finalCmd / 1000`

The discrete PID is the standard `P + I/s + D·s` structure discretized to the recursive `a0·e[k] + a1·e[k-1] + a2·e[k-2]` form, with the coefficients built from the fixed-point gains `LC_KP`, `LC_KI_STEP`, and `LC_KD_STEP` at sample time `LC_TS`. In the current parameter set the derivative term is zero (`LC_KD_STEP = 0`), so the loop runs as a **PI controller**. Diagnostic outputs include the slip error, blend `alpha`, feedforward scale, and PID-correction scale (logged as `error`, `alpha`, and `pid_correction`).

> Note: `TC_SIM_FAST.slx` (used only by `PID_TUNER.m`) keeps an *older* controller representation — a single `DiscreteTransferFcn` PID parameterized by floating-point `P`, `I`, `D`, `Ts` — for fast gain sweeps. The production `TC_SIM.slx` uses the firmware-style fixed-point controller described above.

### Half-Car Suspension Normal Load
A **4-DOF half-car suspension model** (8-state) is the active source of dynamic axle normal loads. Driven by longitudinal acceleration (inertial pitch moment through the CG height) and aerodynamic downforce (split front/rear by `Cp`), it integrates sprung-mass heave and pitch plus front and rear axle hop, and outputs **dynamic front/rear normal loads, suspension forces, dynamic tire forces, and pitch angle**. Its **rear normal load (`Fz_rear`) feeds the tire model**, making the link between chassis behavior and available traction physically explicit.

### Longitudinal Load Transfer (Legacy, Disabled)
A simpler **second-order longitudinal load-transfer** block (parameterized by `f_load`, `zeta_load`) also exists in the model but is **commented out / inactive** — it has been superseded by the half-car model. Its parameters remain in `PARAMS.m` for reference.

### Vehicle & Drivetrain Dynamics
Resolves net longitudinal force (tractive force − drag) into acceleration, velocity, and distance. A first-order **motor/controller torque lag** (`tau_motor`) sits ahead of the wheel dynamics, and the **rear half-shaft pair is modeled as a torsional spring–damper** (`K`, `C`) coupling motor/diff and wheel inertias (`J`, `J_Motor`, `J_Wheel`). Motor RPM out of this subsystem drives the Max Torque lookup.

## Scripts and Tooling
- **`PARAMS.m`** — the standard run. Builds the sigmoid launch table via `generate_sigmoid_launch_table`, runs the model, and exports every logged signal as PDF + PNG + CSV into `figures/`.
- **`PARAMS_IDEAL_LAUNCH_TABLE.m`** — inverts the longitudinal + tire model to compute the motor torque needed to hold a target slip exactly, producing an "ideal" feedforward table. Includes reflected inertia and optional motor-lag compensation. Intended to test the feedforward table open-loop (it can zero the LC PID gains).
- **`THROTTLE_LOOKUP_GENERATOR.m`** — coordinate-descent search over an adaptive time grid that minimizes a slip-tracking objective, exporting an optimized throttle table.
- **`FINAL_DRIVE_GRIP_SWEEP_75M.m`** and **`PID_TUNER.m`** — parallel `parsim` sweeps (Parallel Computing Toolbox) for final-drive/grip and PID-gain studies respectively.

> The standalone sweep/optimizer scripts carry their own embedded copies of the vehicle parameters, some of which are slightly older than `PARAMS.m` (e.g. `Mv = 285.763 kg`, `h_cg = 0.25273 m`, `End_Blend = 8.0 m/s`). `PARAMS.m` is the authoritative current parameter set.

## How to Run
### Requirements
- **MATLAB R2025b**
- **Simulink**
- **Parallel Computing Toolbox** (only for the `*_SWEEP` / `PID_TUNER` scripts)

### Workflow
1. Open `PARAMS.m`.
2. Adjust parameters at the top of the file as desired.
3. Run `PARAMS.m`. It initializes the workspace, generates the launch table, runs `TC_SIM.slx` for 10 s, and reports the 0–75 m and 0–60 mph times in the Command Window.
4. Inspect the generated plots and the exported PDF/PNG/CSV files written to `figures/`.

### Alternative Entry Points
- Run `PARAMS_IDEAL_LAUNCH_TABLE.m` instead of `PARAMS.m` to generate and test the model-inverted ideal launch table.
- Run `THROTTLE_LOOKUP_GENERATOR.m` to optimize a throttle lookup table.
- Run `FINAL_DRIVE_GRIP_SWEEP_75M.m` or `PID_TUNER.m` for sweep studies.

### Performance Metrics
- **0–75 m time**
- **0–60 mph time** (speed target `26.8224 m/s`)

### Time-History Outputs
The following signals are logged and plotted versus time (and exported as PDF/PNG/CSV):

- Launch throttle table
- PID correction
- Slip ratio
- Slip error
- Distance, Velocity, Acceleration
- Downforce
- Half-car front/rear normal load (`Fz_front`, `Fz_rear`)
- Half-car front/rear suspension force
- Half-car front/rear dynamic tire force
- Half-car pitch angle
- Target motor torque
- Tractive force (and tractive force vs. slip ratio)
- Friction coefficient `mu`
- Blend `alpha`
- Wheel-torque comparison (commanded vs. tractive vs. max available, at the wheel)

These outputs support both performance optimization and controls debugging.

## Model Parameters
Current values from `PARAMS.m`:

| Group | Parameter | Value |
|---|---|---|
| Vehicle | Mass `Mv` | 278.10 kg |
| | Wheel radius `r` | 0.203 m |
| | Final drive `fd` | 4.5 |
| | CG height `h_cg` | 0.254 m |
| | Wheelbase `W` | 1.53035 m |
| | Front-axle-to-CG `halfcar_a` (`cg_f`) | 29.402 in ≈ 0.7468 m |
| | CG-to-rear-axle `halfcar_b` | 30.848 in ≈ 0.7835 m |
| | Drivetrain efficiency `Drive_Train` | 0.89 |
| Drivetrain | Inertias `J`, `J_Motor`, `J_Wheel` | 0.51151, 0.34661, 0.16490 kg·m² |
| | Half-shaft stiffness `K` | 5105.49 Nm/rad |
| | Half-shaft damping `C` | 16.56 Nms/rad |
| Aero | `A`, `rho`, `Cd`, `Cl`, `Cp` | 1.0 m², 1.225 kg/m³, 1.0, 3.8, 0.53 |
| Motor | Max torque `Max_Motor_Torque` | 150 Nm |
| | Max speed `Max_Motor_RPM` | 7000 rpm (EMRAX 208) |
| | Torque lag `tau_motor` | 0.03 s |
| Launch Control | Sample time `LC_TS` | 10 ms (`LC_Ts_sec = 0.01 s`) |
| | Gains `LC_KP`, `LC_KI_STEP`, `LC_KD_STEP` | 700, 25, 0 |
| | Slip target `LC_SLIP_TARGET` (`Slip_Target`) | 1300 permyriad (0.13) |
| | Blend speeds `LC_START_BLEND`→`LC_END_BLEND` | 300→500 cm/s (3.0→5.0 m/s) |
| | Controller influence | 100% |
| Half-Car | Unsprung mass per axle | 32 lb (≈14.5 kg) each |
| | Pitch inertia (total) | 110 slug·ft² |
| | Suspension stiffness (front/rear) | 400 / 425 lbf/in |
| | Tire stiffness (per tire) | 640 lbf/in |
| | Suspension damping (front/rear) | 150.36 lbf/(ft/s) |
| Other | Gravity | 9.80665 m/s² |

## Model Assumptions and Scope
This model is intentionally focused on the FSAE acceleration event and assumes:

- **straight-line rear-wheel-drive EV acceleration** for the **0–75 m event**
- the **launch controller is the car's firmware algorithm** (fixed-point, 100 Hz), so simulation behavior reflects production control logic
- **rear-axle normal load from the 4-DOF half-car model**, driven by longitudinal load transfer and aero downforce
- **drivetrain represented with lumped rotational inertias** and a **torsional half-shaft spring–damper**
- **simplified longitudinal tire behavior** (load-sensitive simplified Pacejka with a first-order relaxation lag)
- **aerodynamics with constant coefficients**
- a **time-based feedforward launch table** generated as a sigmoid in `PARAMS.m`
- **simulation-first development**, with formal correlation against the 4.7 s real-world result still in progress

## Current Limitations
This repository is under active development, and several areas still require refinement:

- **half-shaft oscillation behavior** may need further tuning of the compliance/damping assumptions
- the **half-car suspension model** was recently added and its parameters are not yet fully validated
- the **launch (feedforward) table** is not yet fully optimized for the current vehicle/tire parameters
- model performance has **not yet been formally correlated** against the 4.7 s real-world acceleration data

These limitations do not reduce the value of the repository as a design and tuning tool, but they should be kept in mind when interpreting results.

## Results and Validation
**Real-world benchmark:** the team's best measured **0–75 m time is 4.7 s**. This is the primary validation target for the simulation.

Planned additions to this section:

- best simulated **0–75 m** and **0–60 mph** results alongside the 4.7 s real-world time
- optimal **final drive ratio** findings from the sweep studies
- representative **launch-table and controller tuning** comparisons
- example **plots and screenshots** from MATLAB and Simulink
- a documented **simulation-vs-real-vehicle correlation** once data alignment is complete

---
