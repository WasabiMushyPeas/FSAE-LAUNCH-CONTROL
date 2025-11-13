# FSAE-LAUNCH-CONTROL
**PID traction control for EV launch — Cal Poly Racing**

> Regulates wheel slip to maximize tractive force during launch using a PID loop driving a Magic‑Formula‑like tire model and a simple longitudinal vehicle model.

<!--
TODO (delete after filling):
- Tested MATLAB & Simulink versions (e.g., R2023b).
- Confirm required toolboxes beyond Simulink + Parallel Computing (e.g., Control System Toolbox?).
- CI/badges actually used (if any).
- “Results summary” headline: best fd and 0–75 m time at Grip_Fact = 0.6.
- Maintainer contact (email/URL) for License section.
-->

---

## Overview

This repository contains a Simulink® model and MATLAB® Live Scripts for a traction‑control system tailored to Formula SAE Electric (FSAE EV) launches. The controller drives **target slip → PID torque request → wheel/vehicle dynamics**, closing the loop on measured slip to maximize longitudinal tractive force.

- **Model:** `TC_SIM.slx`
- **Parameters & single‑run plots:** `PARAMS.mlx`
- **Sweeps:** `FINAL_DRIVE_RATIO.mlx` (multi‑threaded FDR × grip grid) and `FINAL_DRIVE_RATIO_OLD.mlx` (legacy)
- **Artifacts:** plots saved to `/figures`, sweep table to `sweep_results.csv`

A high‑level model picture lives in [`SIM-PICTURE.pdf`](./SIM-PICTURE.pdf). Governing equations and block mapping come from **Traction Control Simulink Equations.pdf** (see “How it works”).

> ⚠️ **Caution:** Simulation ≠ vehicle. Validate on‑car and enforce safety limits before enabling any closed‑loop torque control.

---

## Repository tree

```text
FSAE-LAUNCH-CONTROL/
├─ PARAMS.mlx
├─ TC_SIM.slx
├─ FINAL_DRIVE_RATIO.mlx
├─ FINAL_DRIVE_RATIO_OLD.mlx
├─ sweep_results.csv
├─ SIM-PICTURE.pdf
└─ figures/
   ├─ Final-Drive-Ratio-vs-Accel.png
   ├─ Final-Drive-Ratio-vs-Score.png
   ├─ Old-Final-Drive-Ratio-vs-Accel.png
   ├─ acceleration.png
   ├─ distance.png
   ├─ error.png
   ├─ mu.png
   ├─ normal_force.png
   ├─ slip_ratio.png
   ├─ target_motor_torque.png
   ├─ torque_comparison_atw.png
   ├─ tractive_force.png
   ├─ tractive_force_vs_slip_ratio.png
   └─ velocity.png
```

---

## Quickstart

### Requirements
- MATLAB® + Simulink®
- Parallel Computing Toolbox™ (for the multi‑threaded sweep)
- (Optional) Control System Toolbox™ if you use advanced PID blocks—core model uses standard Simulink PID.

**Tested versions:** _TBD (please confirm)_.

### Setup
1. **Clone**
   ```bash
   git clone https://github.com/<your-org>/FSAE-LAUNCH-CONTROL.git
   cd FSAE-LAUNCH-CONTROL
   ```
2. **Open MATLAB** and add the repo to your path.
3. **Run a single simulation & generate figures**
   - Open and run [`PARAMS.mlx`](./PARAMS.mlx).
   - This produces plots under [`figures/`](./figures).
4. **Reproduce sweeps (parallel)**
   - Ensure Parallel Computing Toolbox is available.
   - Start a pool sized to your machine:
     ```matlab
     parpool('local', feature('numcores'));  % or parpool('local', <n>)
     ```
     > Note: `FINAL_DRIVE_RATIO.mlx` uses **16 workers by default**. Adjust if your machine has fewer cores.
   - Run [`FINAL_DRIVE_RATIO.mlx`](./FINAL_DRIVE_RATIO.mlx). Outputs:
     - `sweep_results.csv`
     - `figures/Final-Drive-Ratio-vs-Accel.png`
     - `figures/Final-Drive-Ratio-vs-Score.png`

---

## Key parameters

### Vehicle
| Parameter | Symbol | Value | Units |
|---|---:|---:|:--|
| Vehicle mass | `Mv` | 285.763 | kg |
| Wheel radius | `r` | 0.203 | m |
| Total rotational inertia | `J` | 1.2 | kg·m² |
| Final drive ratio (motor:tire) | `fd` | 4.5 | — |
| CG height | `h_cg` | 0.26035 | m |
| Wheelbase | `W` | 1.53035 | m |
| Tire grip factor | `Grip_Fact` | 0.6 | — |
| Drivetrain efficiency | `Drive_Train` | 0.89 | — |

### Aero
| Parameter | Symbol | Value | Units |
|---|---:|---:|:--|
| Frontal area | `A` | 1 | m² |
| Air density | `rho` | 1.225 | kg/m³ |
| Drag coefficient | `Cd` | 1 | — |
| Downforce coefficient | `Cl` | 3.8 | — |
| Rear aero distribution | `Cp` | 0.53 | — |

### Static geometry
| Parameter | Symbol | Value | Units |
|---|---:|---:|:--|
| Front axle ↔ CG | `cg_f` | 0.734568 | m |
| Rear axle ↔ CG | `cg_r` | 0.795782 | m |

### Control
| Parameter | Symbol | Value | Units |
|---|---:|---:|:--|
| Initial torque request | `T_request` | 150 | Nm |
| Slip target | `Slip_Target` | 0.13 | — |
| Max wheel speed | `Max_Wheel_Omega` | 183.3 | rad/s |
| Max motor torque | `Max_Motor_Torque` | 150 | Nm |
| Gravity | `gravity` | 9.80665 | m/s² |
| PID gains | `P`,`I`,`D`,`N` | 5000, 30, 0, 0 | — |

### Simulation
| Parameter | Symbol | Value | Units |
|---|---:|---:|:--|
| Time horizon | `timedomain` | 10 | s |
| Model file | — | [`TC_SIM.slx`](./TC_SIM.slx) | — |
| Sweep grips | `grips` | `0.5:0.1:0.9` | — |
| Sweep FDRs | `fdrs` | `linspace(2,6,49)` | — |

---

## How it works

**Control loop.** The controller tracks a **slip setpoint** (e.g., 0.13) and drives a torque request via PID. Slip is computed from wheel angular speed and vehicle speed, and the requested torque is saturated by drivetrain limits. The wheel/vehicle states (ω, v, x) evolve from applied torques and forces.

**Vehicle & tire model.** The longitudinal model consists of:
- **Aerodynamics**: drag \(D = \tfrac{1}{2}\rho A C_d V^2\), downforce \(L = C_p\,\rho A C_l V^2\).
- **Load transfer** (rear normal force \(N_z\)) from gravity, aero and acceleration, based on CG height and axle distances.
- **Slip ratio** \(Slip = \frac{\omega_s - V}{\max(\omega_s, V)}\).
- **Tractive force** \(F_t = G_t \big((D_1 + D_2/1000)\,N_z\big) N_z \sin\!\big(C \tan^{-1}(B\,Slip)\big)\) (Magic‑Formula‑like).
- **Wheel dynamics**: \(\dot{\omega}_s = \tfrac{1}{J}\big(PID_t\cdot fd - F_t \, r\big)\), limited to `Max_Wheel_Omega`.
- **Vehicle dynamics**: \(a = (F_t - D)/M_v\), \(\dot V = a\), \(\dot x = V\).
- **Motor RPM**: \(RPM_m = \frac{\omega_s}{fd}\frac{60}{2\pi}\).

These are implemented as Simulink subsystems (Aerodynamics, Load Transfer, Tire, Wheel, Vehicle, PID). See [`SIM-PICTURE.pdf`](./SIM-PICTURE.pdf) for the block overview and **Traction Control Simulink Equations.pdf** for the governing equations and symbols.

**PID details.** The PID input is slip error \(e = Slip_{target} - Slip\); the output scales an initial torque request and is clamped by the motor’s torque capability (function of speed) and `Max_Motor_Torque`.

> Major assumptions: rigid wheel (no relaxation), no actuator lag/torque slew limits by default, constant drivetrain efficiency, straight‑line launch (no yaw), flat track, static aero distribution (no CoP migration).

---

## Experiments

### 1) Multi‑threaded sweep — `FINAL_DRIVE_RATIO.mlx`
- **Varies:** Final drive ratio (`fd`) over `linspace(2,6,49)` and grip factor (`Grip_Fact`) over `0.5:0.1:0.9`.
- **Outputs:**  
  - `sweep_results.csv` with columns: `Grip_Fact`, `fd`, `time75_s`, `score`  
  - Plots:  
    - [`figures/Final-Drive-Ratio-vs-Accel.png`](./figures/Final-Drive-Ratio-vs-Accel.png)  
    - [`figures/Final-Drive-Ratio-vs-Score.png`](./figures/Final-Drive-Ratio-vs-Score.png)

**Interpreting the plots:**
- *Accel vs FDR*: lower time (faster 0–75 m) is better; look for the minimum across FDR at each grip.
- *Score vs FDR*: uses the competition‑style scoring below to turn time into points; look for the FDR maximizing points per grip.

### 2) Legacy sweep — `FINAL_DRIVE_RATIO_OLD.mlx`
- **Varies:** legacy single‑parameter sweep over final drive.
- **Output:** [`figures/Old-Final-Drive-Ratio-vs-Accel.png`](./figures/Old-Final-Drive-Ratio-vs-Accel.png)

### Scoring
Let
- \(TimeMin = 3.7\) s, \(TimeMax = 1.5\cdot TimeMin\).
- For a measured time \(t\) to 75 m, the score is
  $$
  score = 95.5 \cdot \frac{\left(\frac{TimeMax}{t}\right)-1}{\left(\frac{TimeMax}{TimeMin}\right)-1} + 4.5,
  $$
  then **clamped at 0**.

Units: seconds for \(t\) and meters for the 75 m distance.

---

## Reproducing figures

| Script | Figures written (→ `figures/`) |
|---|---|
| `PARAMS.mlx` | [`acceleration.png`](./figures/acceleration.png), [`distance.png`](./figures/distance.png), [`error.png`](./figures/error.png), [`mu.png`](./figures/mu.png), [`normal_force.png`](./figures/normal_force.png), [`slip_ratio.png`](./figures/slip_ratio.png), [`target_motor_torque.png`](./figures/target_motor_torque.png), [`torque_comparison_atw.png`](./figures/torque_comparison_atw.png), [`tractive_force.png`](./figures/tractive_force.png), [`tractive_force_vs_slip_ratio.png`](./figures/tractive_force_vs_slip_ratio.png), [`velocity.png`](./figures/velocity.png) |
| `FINAL_DRIVE_RATIO.mlx` | [`Final-Drive-Ratio-vs-Accel.png`](./figures/Final-Drive-Ratio-vs-Accel.png), [`Final-Drive-Ratio-vs-Score.png`](./figures/Final-Drive-Ratio-vs-Score.png), plus [`sweep_results.csv`](./sweep_results.csv) |
| `FINAL_DRIVE_RATIO_OLD.mlx` | [`Old-Final-Drive-Ratio-vs-Accel.png`](./figures/Old-Final-Drive-Ratio-vs-Accel.png) |

---

## Results summary

A compact slice of `sweep_results.csv`:

| Grip_Fact | fd  | time75_s | score |
|---:|---:|---:|---:|
| 0.5 | 3.8 | — | — |
| 0.6 | 4.5 | — | — |
| 0.7 | 4.2 | — | — |

> _Placeholder — please insert headline_: **At Grip_Fact = 0.6, the best final drive was `fd = __` giving `time75_s = __` s and `score = __` points.**

---

## Benchmarks (optional)

_Placeholder — to be filled with team or published FSAE‑EV 0–75 m references for context (e.g., typical ranges and state‑of‑the‑art)._

---

## Limitations & next steps

**Simplifications in the baseline model**
- No actuator dynamics or torque rate limits (may cause overshoot/oscillation in reality).
- No tire relaxation length or slip dynamics.
- Constant drivetrain efficiency; no thermal or battery current limits.
- Static aero distribution; no center‑of‑pressure migration.
- Straight‑line, flat road; no grade, camber, or yaw effects.
- Sensor noise not injected by default.

**Possible extensions**
- Add **torque slew rate** and **motor/controller delay** blocks; retune PID (or use PIDF, gain scheduling).
- Include **tire relaxation** (first‑order slip dynamics) and temperature dependence.
- Model **battery/motor limits**: torque‑speed curve, inverter current and thermal limits.
- Implement **state estimation** (wheel speed filters, slip observers) and noise models for robustness.
- Extend to **closed‑loop launch + traction** over low‑µ transitions and uneven surfaces.

---

## FAQ / Troubleshooting

**Q: Figures didn’t generate.**  
A: Ensure you ran `PARAMS.mlx` to completion and that MATLAB can write to the `figures/` folder. Check the Command Window for errors.

**Q: `parpool` errors or hangs.**  
A: Reduce worker count to match available logical cores: `parpool('local', <n>)`. If the pool auto‑starts to 16 and you have fewer cores, explicitly set a smaller `<n>`.

**Q: Simulink version mismatch when opening `TC_SIM.slx`.**  
A: Open with the version used to save the model, or allow Simulink to upgrade. If blocks are missing, install the required toolbox(es).

**Q: PID saturates at max torque.**  
A: Verify `Max_Motor_Torque`, slip target, and initial `T_request`. If saturation is expected, consider anti‑windup and adding torque‑rate limits.

**Q: How is slip computed and where is the limit enforced?**  
A: Slip is \( (\omega_s - V)/\max(\omega_s,V) \). Wheel speed is limited to `Max_Wheel_Omega = 183.3 rad/s` in the wheel dynamics. See the governing equations document.

---

## Team & Maintainers

- **Maintainer/Author:** **Jackson** — Vehicle Dynamics, Cal Poly Racing  
- **Team:** Cal Poly Racing

---

## Contributing

Issues and PRs are welcome. Please:
1. Open an issue describing the change.
2. For modeling changes, attach a brief rationale and any validation plots.
3. Keep Live Script outputs deterministic (set RNG seeds if used) and save figures to `figures/`.

---

## License

**No license.** All rights reserved by default.  
For permissions (use, distribution, or reuse), please contact the maintainers (email/URL **TBD**).

---

## References

- **Traction Control Simulink Equations.pdf** — variable definitions and governing equations used by this model.
