%% PARAMS_IDEAL_LAUNCH_TABLE.m
% Generates an ideal feedforward launch table by inverting the longitudinal
% vehicle + tire model at a target slip ratio.
%
% This is intended to be run instead of PARAMS.m. It keeps the same core
% parameters, computes launchCmd from model inversion, then optionally runs
% TC_SIM.slx using that ideal table.
%
% Important limitation:
% This is a model-inversion feedforward table. It ignores closed-loop PID,
% and it assumes the drivetrain can follow the desired wheel acceleration.
% It includes reflected rotating inertia and optional first-order motor lag
% compensation, but it does not exactly invert half-shaft twist transients.

clear; clc; close all;

%% --- Vehicle Parameters ---
Mv = 285.763;                   % Vehicle mass [kg]
r = 0.203;                      % Wheel rolling radius [m]
fd = 9;                         % Final drive, motor speed / wheel speed
h_cg = 0.25273;                 % CG height [m]
W = 1.53035;                    % Wheelbase [m]
Grip_Fact = 1.0;                % Tire grip scaling factor
Grip_Influence = 1.2;           % Legacy table tuning scalar
Drive_Train = 0.89;             % Drivetrain efficiency [0..1]
gravity = 9.80665;              % Gravity [m/s^2]

%% --- Drivetrain Inertias ---
J = 0.51151;                    % Total wheel-side rotating inertia [kg*m^2]
J_Motor = 0.34661;              % Inboard motor/diff inertia, wheel-side referenced
J_Wheel = 0.16490;              % Two driven rear wheel assemblies [kg*m^2]

%% --- Half-Shaft Parameters ---
K = 5105.49;                    % Rear half-shaft pair stiffness [Nm/rad]
C = 16.56;                      % Half-shaft damping [Nms/rad]

%% --- Aerodynamics ---
A = 1.0;                        % Frontal area [m^2]
rho = 1.225;                    % Air density [kg/m^3]
Cd = 1.0;                       % Drag coefficient
Cl = 3.8;                       % Downforce coefficient
Cp = 0.53;                      % Fraction of aero downforce on rear axle
Crr = 0.015;                    % Rolling resistance coefficient

%% --- Static Loads ---
cg_f = 0.734568;                % Distance from front axle to CG [m]
rear_static_frac = cg_f/W;      % Rear static load fraction
front_static_frac = 1 - rear_static_frac;

%% --- Motor / Control Params ---
Slip_Target = 0.13;             % Target slip ratio [decimal]
T_request = 150;                % Driver torque request [Nm]
Start_Blend = 3.0;              % Legacy blend start [m/s]
End_Blend = 8.0;                % Legacy blend end [m/s]
Max_Motor_RPM = 7000;           % EMRAX 208 speed limit [rpm]
Max_Wheel_Omega = (Max_Motor_RPM * 2*pi/60) / fd;  % Wheel speed limit [rad/s]
tau_motor = 0.03;               % Motor/controller torque lag [s]
tau_tire = 0.03;                % Tire relaxation delay [s]
tau_load = 0.03;                % Load transfer lag [s]
Max_Motor_Torque = 150;         % Maximum motor torque [Nm]

%% --- Launch Control Params From Car Code ---
CONTROLLER_INFLUENCE = int32(100);

LC_TABLE_LENGTH = int32(40);
LC_TS = int32(10);              % Launch-control sample time [ms]
LC_Ts_sec = double(LC_TS)/1000; % Launch-control sample time [s]

LC_KP = int32(700);
LC_KI_STEP = int32(25);
LC_KD_STEP = int32(0);

LC_SLIP_TARGET = int32(1300);   % 1300 = 13.00% slip

LC_START_BLEND = int32(300);    % cm/s = 3 m/s
LC_END_BLEND = int32(500);      % cm/s = 5 m/s

LC_GRIP_FACTOR = int32(1000);
LC_GRIP_INFLUENCE = int32(1500);
LC_MIN_GRIP_SCALE = int32(0);
LC_MAX_GRIP_SCALE = int32(1500);

LC_MIN_CMD = int32(0);
LC_MAX_CMD = int32(1000);

LC_MIN_CORR = int32(-1000);
LC_MAX_CORR = int32(1000);

launchTimeMs = int32([ ...
    0,16,33,51,72,96,121,148,177,207,239,271,302,337,376,420,469,523, ...
    583,650,724,806,899,1000,1406,1906,2440,2972,3554,4138,4724,5310, ...
    5895,6481,7067,7654,8240,8827,9413,10000]);

% Current PARAMS.m table kept for reference only.
launchCmd_original = int32([ ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000]);

%% --- Tire Model Coefficients Used By The Inversion ---
% Rear-axle convention used by this ideal table:
% normalForce/FzRear is the total rear axle normal force.
% Fz_tire = FzRear/2
% mu = (D1 - D2*Fz_tire)*Grip_Fact
% Fx_total_rear_axle = 2*(mu*Fz_tire*sin(C*atan(B*slip)))
%
% In the Simulink Tire Model / Friction Coefficient Calc block, make sure
% the final force output also represents the full rear axle, not one tire:
% Fx = 2*Fx_tire;
Tire_B = 10.4;
Tire_C = 1.58;
Tire_D1 = 3.02;
Tire_D2 = 0.0008 / 4.448221615;    % per N instead of per lbf

%% --- Ideal Launch Table Settings ---
Ideal_Slip_Target = 0.13;       % 13 percent slip, decimal
ideal_dt = 0.001;               % internal inversion integration step [s]
ideal_t_end = double(launchTimeMs(end))/1000.0;
ideal_compensate_motor_lag = true;
ideal_smooth_command = true;
runSimWithIdealTable = true;
makeIdealPlots = true;
useIdealTableOpenLoopInSim = true;  % sets LC PID gains to zero before sim

%% --- Generate Ideal Launch Table ---
tableTimes_s = double(launchTimeMs(:)) / 1000.0;

ideal = make_ideal_launch_table( ...
    tableTimes_s, ideal_dt, ideal_t_end, Ideal_Slip_Target, ...
    Mv, r, J, fd, Drive_Train, Max_Motor_Torque, tau_motor, ...
    gravity, h_cg, W, cg_f, ...
    A, rho, Cd, Cl, Cp, Crr, Grip_Fact, ...
    Tire_B, Tire_C, Tire_D1, Tire_D2, ...
    ideal_compensate_motor_lag, ideal_smooth_command);

% Use the clipped command as the actual car-format launch table.
launchCmd_ideal_unclipped = int32(round(ideal.cmdPermilleUnclipped));
launchCmd_ideal = int32(round(ideal.cmdPermilleClipped));

% Overwrite launchCmd so TC_SIM uses the ideal table.
launchCmd = launchCmd_ideal(:).';
Time_pts = double(launchTimeMs)/1000.0;
Throttle_pts = double(launchCmd)/1000.0;

% This file is meant to test the feedforward table by itself. If your
% CONTROLLER block still contains the car PID correction, disable it here so
% the sim uses only the model-inverted launch table.
if useIdealTableOpenLoopInSim
    LC_KP = int32(0);
    LC_KI_STEP = int32(0);
    LC_KD_STEP = int32(0);
end

fprintf('\n--- Ideal Launch Table Generated ---\n');
fprintf('Target slip: %.2f %%\n', 100*Ideal_Slip_Target);
fprintf('Max available motor torque: %.1f Nm\n', Max_Motor_Torque);
fprintf('Required actual motor torque range: %.1f to %.1f Nm\n', ...
    min(ideal.motorTorqueActualReq_Nm), max(ideal.motorTorqueActualReq_Nm));
fprintf('Required commanded motor torque range: %.1f to %.1f Nm\n', ...
    min(ideal.motorTorqueCmdReq_Nm), max(ideal.motorTorqueCmdReq_Nm));
fprintf('Clipped command range: %d to %d permille\n', min(launchCmd_ideal), max(launchCmd_ideal));

if all(launchCmd_ideal >= 1000)
    fprintf(['WARNING: launchCmd_ideal is all 1000. With this tire/load model, ' ...
        '150 Nm is not enough to force 13%% slip open-loop.\n']);
end

fprintf('\nPaste this into the car-style launch table if desired:\n');
print_int32_table('launchCmd_ideal', launchCmd_ideal);

% Save the table data next to the script.
outdir = fullfile(pwd, 'ideal_launch_outputs');
if ~exist(outdir, 'dir')
    mkdir(outdir);
end

Tideal = table( ...
    tableTimes_s(:), ...
    double(launchTimeMs(:)), ...
    double(launchCmd_original(:)), ...
    double(launchCmd_ideal(:)), ...
    ideal.cmdPermilleUnclipped(:), ...
    ideal.motorTorqueActualReq_Nm(:), ...
    ideal.motorTorqueCmdReq_Nm(:), ...
    ideal.vehicleSpeed_mps(:), ...
    ideal.accel_mps2(:), ...
    ideal.Fx_N(:), ...
    ideal.FzRear_N(:), ...
    'VariableNames', { ...
    'Time_s', 'Time_ms', 'Original_Cmd_permille', 'Ideal_Cmd_Clipped_permille', ...
    'Ideal_Cmd_Unclipped_permille', 'Ideal_Actual_Motor_Torque_Nm', ...
    'Ideal_Commanded_Motor_Torque_Nm', 'Ideal_Vehicle_Speed_mps', ...
    'Ideal_Accel_mps2', 'Ideal_Fx_N', 'Ideal_FzRear_N'});

writetable(Tideal, fullfile(outdir, 'ideal_launch_table.csv'));

if makeIdealPlots
    plot_ideal_table(outdir, tableTimes_s, launchCmd_original, launchCmd_ideal, ideal, Max_Motor_Torque);
end

%% --- Simulink Run Using Ideal Table ---
timedomain = 10;

if runSimWithIdealTable
    fprintf('\nRunning TC_SIM.slx with ideal launch table...\n');
    simout = sim('TC_SIM.slx', timedomain);

    % Basic performance prints. These are intentionally robust to different
    % signal sample times.
    if isprop(simout, 'v_distance') && isprop(simout, 'v_velocity')
        Dist_Time = simout.v_distance.Time(:);
        Dist = squeeze(simout.v_distance.Data); Dist = Dist(:);
        Vel_Time = simout.v_velocity.Time(:);
        Vel = squeeze(simout.v_velocity.Data); Vel = Vel(:);

        target_distance = 75;
        idxDist = find(Dist >= target_distance, 1, 'first');
        if ~isempty(idxDist)
            fprintf('Time when distance reaches 75 m: %.4f seconds\n', Dist_Time(idxDist));
        else
            fprintf('Distance of 75 m was not reached during the simulation.\n');
        end

        target_speed = 26.8224;
        idxVel = find(Vel >= target_speed, 1, 'first');
        if ~isempty(idxVel)
            fprintf('Time when speed reaches 26.8224 m/s: %.4f seconds\n', Vel_Time(idxVel));
        else
            fprintf('Speed of 26.8224 m/s was not reached during the simulation.\n');
        end
    end

    % Optional quick plots from the sim if the expected signals exist.
    try
        plot_sim_results_with_own_timebases(simout, outdir, Grip_Fact, fd, r, Drive_Train, Max_Motor_Torque);
    catch ME
        warning('Ideal launch sim completed, but plotting failed: %s', ME.message);
    end
end

%% ========================================================================
% Local helper functions
% ========================================================================

function ideal = make_ideal_launch_table( ...
    tableTimes, dt, tEnd, slipTarget, ...
    Mv, r, J_total, fd, Drive_Train, Max_Motor_Torque, tau_motor, ...
    gravity, h_cg, W, cg_f, ...
    A, rho, Cd, Cl, Cp, Crr, Grip_Fact, ...
    B, C, D1, D2, compensateMotorLag, smoothCommand)

% Forward-integrates an idealized launch where tire slip is held exactly at
% slipTarget. The table is generated from the torque required at the motor.
% This uses the Simulink driven-wheel positive-slip convention:
% slip = (V_wheel - V_vehicle) / max(abs(V_vehicle), abs(V_wheel), 0.1).
% During the 0.1 m/s startup floor, V_wheel = V_vehicle + slip*0.1.
% Away from that floor, positive slip gives
% V_wheel = V_vehicle/(1 - slip).

if slipTarget < 0.0 || slipTarget >= 1.0
    error('slipTarget must be in [0, 1) for this positive-slip launch inversion.');
end

slipDenominatorFloor_mps = 0.1;

nSteps = floor(tEnd/dt) + 1;
tGrid = (0:nSteps-1).' * dt;

vGrid = zeros(nSteps,1);
aGrid = zeros(nSteps,1);
FxGrid = zeros(nSteps,1);
FzGrid = zeros(nSteps,1);
TactualGrid = zeros(nSteps,1);
TcmdGrid = zeros(nSteps,1);
cmdPermilleActualGrid = zeros(nSteps,1);
cmdPermilleCmdGrid = zeros(nSteps,1);

v = 0.0;

for k = 1:nSteps
    [Fx, accel, FzRear] = ideal_force_at_slip( ...
        v, slipTarget, Mv, gravity, h_cg, W, cg_f, ...
        A, rho, Cd, Cl, Cp, Crr, Grip_Fact, B, C, D1, D2);

    % Match Simulink's slip denominator, including the low-speed floor.
    if v < slipDenominatorFloor_mps * (1.0 - slipTarget)
        wheelSurfaceAccelTarget = accel;
    else
        wheelSurfaceAccelTarget = accel / (1.0 - slipTarget);
    end
    omegaDotTarget = wheelSurfaceAccelTarget / r;

    % Wheel-side drive torque needed for tire force plus rotating inertia.
    T_wheel_drive_req = Fx*r + J_total*omegaDotTarget;

    % Motor torque after final drive and drivetrain efficiency.
    T_motor_actual_req = T_wheel_drive_req / (Drive_Train * fd);

    vGrid(k) = v;
    aGrid(k) = accel;
    FxGrid(k) = Fx;
    FzGrid(k) = FzRear;
    TactualGrid(k) = T_motor_actual_req;
    cmdPermilleActualGrid(k) = 1000.0*T_motor_actual_req/Max_Motor_Torque;

    v = max(0.0, v + accel*dt);
end

if compensateMotorLag
    % The motor lag is approximately tau*dT_actual/dt + T_actual = T_command.
    dTdt = gradient(TactualGrid, dt);
    TcmdGrid = TactualGrid + tau_motor*dTdt;
else
    TcmdGrid = TactualGrid;
end

TcmdGrid = max(TcmdGrid, 0.0);

if smoothCommand
    % Light smoothing to avoid table point noise from numerical inversion.
    win = max(3, 2*floor(0.010/dt/2) + 1); % around 10 ms window, odd length
    TcmdGrid = movmean(TcmdGrid, win);
end

cmdPermilleCmdGrid = 1000.0*TcmdGrid/Max_Motor_Torque;

ideal.motorTorqueActualReq_Nm = interp1(tGrid, TactualGrid, tableTimes, 'linear', 'extrap');
ideal.motorTorqueCmdReq_Nm = interp1(tGrid, TcmdGrid, tableTimes, 'linear', 'extrap');
ideal.cmdPermilleUnclipped = interp1(tGrid, cmdPermilleCmdGrid, tableTimes, 'linear', 'extrap');
ideal.cmdPermilleClipped = min(max(ideal.cmdPermilleUnclipped, 0.0), 1000.0);
ideal.vehicleSpeed_mps = interp1(tGrid, vGrid, tableTimes, 'linear', 'extrap');
ideal.accel_mps2 = interp1(tGrid, aGrid, tableTimes, 'linear', 'extrap');
ideal.Fx_N = interp1(tGrid, FxGrid, tableTimes, 'linear', 'extrap');
ideal.FzRear_N = interp1(tGrid, FzGrid, tableTimes, 'linear', 'extrap');

% Full-resolution data for diagnostics.
ideal.tGrid = tGrid;
ideal.vGrid = vGrid;
ideal.aGrid = aGrid;
ideal.FxGrid = FxGrid;
ideal.FzGrid = FzGrid;
ideal.TactualGrid = TactualGrid;
ideal.TcmdGrid = TcmdGrid;
ideal.cmdPermilleActualGrid = cmdPermilleActualGrid;
ideal.cmdPermilleCmdGrid = cmdPermilleCmdGrid;

end

function [Fx, accel, FzRear] = ideal_force_at_slip( ...
    v, slipTarget, Mv, gravity, h_cg, W, cg_f, ...
    A, rho, Cd, Cl, Cp, Crr, Grip_Fact, B, C, D1, D2)

% Solves the implicit equation:
% Fx = tire_force(slipTarget, FzRear)
% FzRear = static + longitudinal load transfer + rear aero
% accel = (Fx - drag - rolling resistance)/Mv

drag = 0.5*rho*Cd*A*v*abs(v);
downforce = 0.5*rho*Cl*A*v^2;
roll = Crr*Mv*gravity*tanh(v/0.5);

Fz_static_rear = Mv*gravity*cg_f/W;

% Load transfer term can be written using Fx directly:
% M*a*h/W = h/W*(Fx - drag - roll)
beta = h_cg/W;
A0 = Fz_static_rear + Cp*downforce - beta*(drag + roll);

shape = sin(C * atan(B * slipTarget));

force_balance = @(FxGuess) Grip_Fact * shape * ...
    tire_force_from_rear_fz(A0 + beta*FxGuess, D1, D2) - FxGuess;

lo = 0.0;
hi = 20000.0;
f_lo = force_balance(lo);
f_hi = force_balance(hi);

if f_lo*f_hi > 0
    Fx = max(0.0, min(hi, Grip_Fact * shape * tire_force_from_rear_fz(A0, D1, D2)));
else
    for i = 1:80
        mid = 0.5*(lo + hi);
        f_mid = force_balance(mid);

        if f_lo*f_mid <= 0
            hi = mid;
            f_hi = f_mid; %#ok<NASGU>
        else
            lo = mid;
            f_lo = f_mid;
        end
    end
    Fx = 0.5*(lo + hi);
end

FzRear = max(A0 + beta*Fx, 0.0);
accel = (Fx - drag - roll) / Mv;

end

function FxNoShape = tire_force_from_rear_fz(FzRear, D1, D2)

% Full rear axle version of the simplified Pacejka model:
% Fz_tire = FzRear/2
% mu = D1 - D2*Fz_tire
% Fx_total / shape = 2*mu*Fz_tire = D1*FzRear - 0.5*D2*FzRear^2

FzRear = max(FzRear, 0.0);
FxNoShape = D1*FzRear - 0.5*D2*FzRear^2;
FxNoShape = max(FxNoShape, 0.0);

end

function print_int32_table(name, values)
values = values(:).';
fprintf('%s = int32([ ...\n    ', name);
for i = 1:numel(values)
    if i < numel(values)
        fprintf('%d,', values(i));
    else
        fprintf('%d', values(i));
    end

    if mod(i, 10) == 0 && i < numel(values)
        fprintf(' ...\n    ');
    end
end
fprintf(']);\n');
end

function plot_ideal_table(outdir, tableTimes, launchCmdOriginal, launchCmdIdeal, ideal, MaxMotorTorque)

figure('Color','white');
hold on;
plot(tableTimes, double(launchCmdOriginal), 'LineWidth', 2);
plot(tableTimes, double(launchCmdIdeal), 'LineWidth', 2);
plot(tableTimes, ideal.cmdPermilleUnclipped, '--', 'LineWidth', 1.5);
hold off;
grid on;
xlabel('Time (s)');
ylabel('Launch command (permille)');
title('Original vs Ideal Launch Table');
legend('Original', 'Ideal clipped', 'Ideal unclipped', 'Location', 'southeast');
exportgraphics(gcf, fullfile(outdir, 'ideal_launch_command.png'), 'Resolution', 300, 'BackgroundColor','white');

figure('Color','white');
hold on;
plot(tableTimes, ideal.motorTorqueActualReq_Nm, 'LineWidth', 2);
plot(tableTimes, ideal.motorTorqueCmdReq_Nm, 'LineWidth', 2);
yline(MaxMotorTorque, '--', 'Max motor torque');
hold off;
grid on;
xlabel('Time (s)');
ylabel('Motor torque (Nm)');
title('Ideal Required Motor Torque');
legend('Actual torque required', 'Command torque with lag compensation', 'Location', 'southeast');
exportgraphics(gcf, fullfile(outdir, 'ideal_required_motor_torque.png'), 'Resolution', 300, 'BackgroundColor','white');

figure('Color','white');
plot(tableTimes, ideal.accel_mps2, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Ideal 13 Percent Slip Acceleration');
exportgraphics(gcf, fullfile(outdir, 'ideal_acceleration.png'), 'Resolution', 300, 'BackgroundColor','white');

end

function plot_sim_results_with_own_timebases(simout, outdir, Grip_Fact, fd, r, Drive_Train, Max_Motor_Torque)

% Pull common signals only if they exist. This avoids breaking if a log name
% changes while still giving useful plots for the current TC_SIM model.

if isprop(simout, 'slip_ratio')
    t = simout.slip_ratio.Time(:);
    y = squeeze(simout.slip_ratio.Data); y = y(:);
    figure('Color','white');
    plot(t, y, 'LineWidth', 2);
    grid on;
    xlabel('Time (s)'); ylabel('Slip ratio');
    title(sprintf('Sim Slip Ratio with Ideal Table (%.2f Grip Factor)', Grip_Fact));
    exportgraphics(gcf, fullfile(outdir, 'sim_slip_ratio.png'), 'Resolution', 300, 'BackgroundColor','white');
end

if isprop(simout, 'T_motor')
    t = simout.T_motor.Time(:);
    y = squeeze(simout.T_motor.Data); y = y(:);
    figure('Color','white');
    plot(t, y, 'LineWidth', 2);
    yline(Max_Motor_Torque, '--', 'Max motor torque');
    grid on;
    xlabel('Time (s)'); ylabel('Motor torque request (Nm)');
    title('Sim Motor Torque Request with Ideal Table');
    exportgraphics(gcf, fullfile(outdir, 'sim_motor_torque.png'), 'Resolution', 300, 'BackgroundColor','white');
end

if isprop(simout, 'Longitudinal_Force') && isprop(simout, 'T_motor') && isprop(simout, 'Max_Tor')
    Trac_Time = simout.Longitudinal_Force.Time(:);
    Trac = squeeze(simout.Longitudinal_Force.Data); Trac = Trac(:);
    Motor_Time = simout.T_motor.Time(:);
    Motor_Tor = squeeze(simout.T_motor.Data); Motor_Tor = Motor_Tor(:);
    Max_Time = simout.Max_Tor.Time(:);
    Max_Tor = squeeze(simout.Max_Tor.Data); Max_Tor = Max_Tor(:);

    T_from_trac_atw = Trac*r;
    T_motor_atw = interp_signal(Motor_Time, Motor_Tor*fd*Drive_Train, Trac_Time);
    T_max_atw = interp_signal(Max_Time, Max_Tor*fd*Drive_Train, Trac_Time);

    figure('Color','white');
    hold on;
    plot(Trac_Time, T_motor_atw, 'LineWidth', 2);
    plot(Trac_Time, T_from_trac_atw, 'LineWidth', 2);
    plot(Trac_Time, T_max_atw, '--', 'LineWidth', 2);
    hold off;
    grid on;
    xlabel('Time (s)'); ylabel('Wheel torque (Nm)');
    title('Wheel Torque Comparison with Ideal Table');
    legend('Commanded motor torque ATW', 'Tractive force ATW', 'Max motor torque ATW', 'Location', 'southeast');
    exportgraphics(gcf, fullfile(outdir, 'sim_wheel_torque_comparison.png'), 'Resolution', 300, 'BackgroundColor','white');
end

end

function yq = interp_signal(t, y, tq)
t = t(:);
y = y(:);
tq = tq(:);
valid = isfinite(t) & isfinite(y);
t = t(valid);
y = y(valid);

if isempty(t)
    yq = nan(size(tq));
    return;
end

[tUnique, ia] = unique(t, 'stable');
yUnique = y(ia);

if numel(tUnique) == 1
    yq = repmat(yUnique(1), size(tq));
else
    yq = interp1(tUnique, yUnique, tq, 'linear', 'extrap');
end
end
