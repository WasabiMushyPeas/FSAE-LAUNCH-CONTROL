%% ---------------- Units ----------------
lb_to_kg = 0.45359237;         % Pound mass to kilogram [kg/lb]
in_to_m = 0.0254;              % Inch to meter [m/in]
lbf_in_to_N_m = 175.126835;    % Spring rate conversion [N/m per lbf/in]
lbf_ft_s_to_N_s_m = 14.593903; % Damping conversion [N*s/m per lbf/(ft/s)]
slug_ft2_to_kg_m2 = 1.35581795;% Pitch inertia conversion [kg*m^2 per slug*ft^2]

%% ---------------- Vehicle Parameters ----------------
Mv = 278.10;                   % Vehicle mass [kg]
r = 0.203;                     % Wheel rolling radius [m]
fd = 4.5;                      % Final drive ratio, motor speed / wheel speed [-]
h_cg = 0.254;                  % CG height [m]
W = 1.53035;                   % Wheelbase [m]
gravity = 9.80665;             % Gravity [m/s^2]

Grip_Fact = 1.0;               % Tire grip scale [-]
Grip_Influence = 1.2;          % Legacy lookup-table grip influence [-]
Drive_Train = 0.89;            % Drivetrain efficiency [0..1]

%% ---------------- Vehicle Geometry ----------------
halfcar_a = 29.402 * in_to_m;  % Front axle to CG [m]
halfcar_b = 30.848 * in_to_m;  % CG to rear axle [m]

cg_f = halfcar_a;              % Front axle to CG for legacy load model [m]
cg_r = halfcar_b;              % CG to rear axle [m]
wheelbase_from_vds = halfcar_a + halfcar_b; % VDS wheelbase check [m]

if abs(W - wheelbase_from_vds) > 1e-4
    warning('PARAMS:WheelbaseMismatch', ...
        'W does not match halfcar_a + halfcar_b from VDS geometry.');
end

%% ---------------- Drivetrain Inertias ----------------
J = 0.51151;                   % Total wheel-side rotating inertia [kg*m^2]
J_Motor = 0.34661;             % Motor/diff inertia at wheel side [kg*m^2]
J_Wheel = 0.16490;             % Two driven rear wheel assemblies [kg*m^2]

%% ---------------- Half-Shaft Parameters ----------------
K = 5105.49;                   % Rear half-shaft pair stiffness [Nm/rad]
C = 16.56;                     % Half-shaft damping, coupled vehicle model [Nms/rad]

%% ---------------- Aerodynamics ----------------
A = 1.0;                       % Frontal area [m^2]
rho = 1.225;                   % Air density [kg/m^3]
Cd = 1.0;                      % Drag coefficient [-]
Cl = 3.8;                      % Downforce coefficient [-]
Cp = 0.53;                     % Rear aero load fraction [-]

%% ---------------- Motor And Shared Control Parameters ----------------
Slip_Target = 0.13;            % Slip target [decimal]
T_request = 150;               % Initial driver torque request [Nm]
Max_Motor_RPM = 7000;          % EMRAX 208 speed limit [rpm]
Max_Wheel_Omega = (Max_Motor_RPM * 2*pi/60) / fd;  % Wheel speed limit [rad/s]
tau_motor = 0.03;              % Motor/controller torque lag [s]
tau_tire = 0.03;               % Legacy tire relaxation time constant [s]
tau_load = 0.03;               % Legacy load-transfer lag [s]
Max_Motor_Torque = 150;        % Maximum motor torque [Nm]

%% ---------------- Static Loads ----------------
Fz_front_static = Mv*gravity*cg_r/W; % Static front axle normal load [N]
Fz_rear_static  = Mv*gravity*cg_f/W; % Static rear axle normal load [N]
Fz_static_rear = Fz_rear_static;     % Legacy rear static load alias [N]

%% ---------------- Rear Normal-Load Suspension Dynamics ----------------
f_load = 6.45;                 % Load-transfer natural frequency [Hz]
zeta_load = 0.35;              % Load-transfer damping ratio [-]
omega_load = 2*pi*f_load;      % Load-transfer natural frequency [rad/s]

load_tf_num = omega_load^2;    % Load-transfer numerator coefficient [rad^2/s^2]
load_tf_den = [1, 2*zeta_load*omega_load, omega_load^2]; % Load-transfer denominator coefficients [-, rad/s, rad^2/s^2]

%% ---------------- Half-Car Suspension Parameters ----------------
halfcar_muf = 32.0 * lb_to_kg; % Front unsprung axle mass [kg]
halfcar_mur = 32.0 * lb_to_kg; % Rear unsprung axle mass [kg]

halfcar_ms = Mv - halfcar_muf - halfcar_mur; % Sprung mass [kg]

halfcar_Iyy_total = 110.0 * slug_ft2_to_kg_m2; % Total-car pitch inertia from VDS [kg*m^2]
halfcar_Iyy = halfcar_Iyy_total ... % Sprung pitch inertia [kg*m^2]
    - halfcar_muf*halfcar_a^2 ...
    - halfcar_mur*halfcar_b^2;

MR_heave_F = 1.0;             % Front heave motion ratio [-]
MR_heave_R = 1.0;             % Rear heave motion ratio [-]

K_susp_F_lbf_in = 400.0 * MR_heave_F^2; % Front axle suspension heave stiffness [lbf/in]
K_susp_R_lbf_in = 425.0 * MR_heave_R^2; % Rear axle suspension heave stiffness [lbf/in]

halfcar_Ksf = K_susp_F_lbf_in * lbf_in_to_N_m; % Front suspension stiffness [N/m]
halfcar_Ksr = K_susp_R_lbf_in * lbf_in_to_N_m; % Rear suspension stiffness [N/m]

K_tire_each_lbf_in = 640.0;  % One tire vertical stiffness [lbf/in]
K_tire_axle_lbf_in = 2.0*K_tire_each_lbf_in; % Axle pair tire stiffness [lbf/in]

halfcar_Ktf = K_tire_axle_lbf_in * lbf_in_to_N_m; % Front axle tire stiffness [N/m]
halfcar_Ktr = K_tire_axle_lbf_in * lbf_in_to_N_m; % Rear axle tire stiffness [N/m]

halfcar_Ctf = 0.0;            % Front tire vertical damping [N*s/m]
halfcar_Ctr = 0.0;            % Rear tire vertical damping [N*s/m]

C_heave_F_lbf_ft_s = 150.36; % Front suspension damping [lbf/(ft/s)]
C_heave_R_lbf_ft_s = 150.36; % Rear suspension damping [lbf/(ft/s)]

halfcar_Csf = C_heave_F_lbf_ft_s * lbf_ft_s_to_N_s_m; % Front suspension damping [N*s/m]
halfcar_Csr = C_heave_R_lbf_ft_s * lbf_ft_s_to_N_s_m; % Rear suspension damping [N*s/m]

halfcar_x0 = zeros(8,1);      % Half-car initial state vector [mixed]

K_heave_eff_F_lbf_in = ... % Front ride rate [lbf/in]
    (K_susp_F_lbf_in*K_tire_axle_lbf_in)/(K_susp_F_lbf_in + K_tire_axle_lbf_in);

K_heave_eff_R_lbf_in = ... % Rear ride rate [lbf/in]
    (K_susp_R_lbf_in*K_tire_axle_lbf_in)/(K_susp_R_lbf_in + K_tire_axle_lbf_in);

%% ---------------- Launch Control Params From Car Code ----------------
CONTROLLER_INFLUENCE = int32(100); % PID correction influence [%]

LC_TS = int32(10);             % Launch-control sample time [ms]
LC_Ts_sec = double(LC_TS)/1000;% Launch-control sample time [s]

LC_KP = int32(700);            % Fixed-point proportional gain [-]
LC_KI_STEP = int32(25);        % Fixed-point integral gain per step [-]
LC_KD_STEP = int32(0);         % Fixed-point derivative gain per step [-]

LC_SLIP_TARGET = int32(round(10000*Slip_Target)); % 1300 = 13.00% slip

LC_START_BLEND = int32(300);   % PID blend start speed [cm/s]
LC_END_BLEND = int32(500);     % PID blend end speed [cm/s]
LC_START_BLEND_MPS = double(LC_START_BLEND)/100; % PID blend start speed [m/s]
LC_END_BLEND_MPS = double(LC_END_BLEND)/100;     % PID blend end speed [m/s]

Start_Blend = LC_START_BLEND_MPS; % Legacy PID blend start speed [m/s]
End_Blend = LC_END_BLEND_MPS;     % Legacy PID blend end speed [m/s]

LC_GRIP_FACTOR = int32(round(1000*Grip_Fact)); % Fixed-point grip factor [permille]
LC_GRIP_INFLUENCE = int32(1500); % 1500 = 1.5x grip correction influence
LC_MIN_GRIP_SCALE = int32(0);  % Minimum grip scale [permille]
LC_MAX_GRIP_SCALE = int32(1500); % Maximum grip scale [permille]

LC_MIN_CMD = int32(0);         % Minimum torque command [permille]
LC_MAX_CMD = int32(1000);      % Maximum torque command [permille]

LC_MIN_CORR = int32(-1000);    % Minimum PID correction [permille]
LC_MAX_CORR = int32(1000);     % Maximum PID correction [permille]

launchTimeMs = int32([ ...
    0,16,33,51,72,96,121,148,177,207,239,271,302,337,376,420,469,523, ...
    583,650,724,806,899,1000,1406,1906,2440,2972,3554,4138,4724,5310, ...
    5895,6481,7067,7654,8240,8827,9413,10000]);

launchCmd = int32([ ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000]);

if numel(launchTimeMs) ~= numel(launchCmd)
    error('PARAMS:LaunchTableLengthMismatch', ...
        'launchTimeMs and launchCmd must have the same number of entries.');
end

if any(diff(double(launchTimeMs)) <= 0)
    error('PARAMS:LaunchTableTimeOrder', ...
        'launchTimeMs must be strictly increasing.');
end

LC_TABLE_LENGTH = int32(numel(launchTimeMs));

Time_pts = double(launchTimeMs)/1000;
Throttle_pts = double(launchCmd)/1000;


%% ---------------- Simulation ----------------
timedomain = 10;
simout = sim("TC_SIM.slx", timedomain);

%% ---------------- Logged Signals ----------------
Wheel_Speed_Time = simout.wheel_speed.Time(:);
Wheel_Speed = squeeze(simout.wheel_speed.Data);
Wheel_Speed = Wheel_Speed(:);

try
    [Downforce_Time, Downforce] = logged_signal(simout, 'v_down_force');
catch
    [Downforce_Time, Downforce] = logged_signal(simout, 'v_lift');
end

Drag_Time = simout.v_drag.Time(:);
Drag = squeeze(simout.v_drag.Data);
Drag = Drag(:);

Trac_Time = simout.Longitudinal_Force.Time(:);
Trac = squeeze(simout.Longitudinal_Force.Data);
Trac = Trac(:);

Accel_Time = simout.v_accel.Time(:);
Accel = squeeze(simout.v_accel.Data);
Accel = Accel(:);

Vel_Time = simout.v_velocity.Time(:);
Vel = squeeze(simout.v_velocity.Data);
Vel = Vel(:);
Time = Vel_Time;

Dist_Time = simout.v_distance.Time(:);
Dist = squeeze(simout.v_distance.Data);
Dist = Dist(:);

Motor_Tor_Time = simout.T_motor.Time(:);
Motor_Tor = squeeze(simout.T_motor.Data);
Motor_Tor = Motor_Tor(:);

[Fz_Front_Time, Fz_Front] = logged_signal(simout, 'Fz_front');
[Fz_Rear_Time, Fz_Rear] = logged_signal(simout, 'Fz_rear');
[F_Susp_Front_Time, F_Susp_Front] = logged_signal(simout, 'F_susp_front');
[F_Susp_Rear_Time, F_Susp_Rear] = logged_signal(simout, 'F_susp_rear');
[F_Tire_Front_Dyn_Time, F_Tire_Front_Dyn] = logged_signal(simout, 'F_tire_front_dyn');
[F_Tire_Rear_Dyn_Time, F_Tire_Rear_Dyn] = logged_signal(simout, 'F_tire_rear_dyn');
[Theta_Out_Time, Theta_Out] = logged_signal(simout, 'theta_out');

Error_Time = simout.error.Time(:);
Error = squeeze(simout.error.Data);
Error = Error(:);

Mu_Time = simout.mu.Time(:);
Mu = squeeze(simout.mu.Data);
Mu = Mu(:);

Alpha_Time = simout.alpha.Time(:);
Alpha = squeeze(simout.alpha.Data);
Alpha = Alpha(:);

Slip_Ratio_Time = simout.slip_ratio.Time(:);
Slip_Ratio = squeeze(simout.slip_ratio.Data);
Slip_Ratio = Slip_Ratio(:);

Max_Motor_Tor_Time = simout.Max_Tor.Time(:);
Max_Motor_Tor = squeeze(simout.Max_Tor.Data);
Max_Motor_Tor = Max_Motor_Tor(:);

PID_Time = simout.pid_correction.Time(:);
PID = squeeze(simout.pid_correction.Data);
PID = PID(:);


%% ---------------- Performance Metrics ----------------
target_distance = 75;
time_at_target_distance = Dist_Time(Dist >= target_distance);

if ~isempty(time_at_target_distance)
    fprintf('Time when distance reaches 75 m: %.4f seconds\n', time_at_target_distance(1));
else
    fprintf('Distance of 75 m was not reached during the simulation.\n');
end

target_speed = 26.8224;
time_at_target_speed = Vel_Time(Vel >= target_speed);

if ~isempty(time_at_target_speed)
    fprintf('Time when speed reaches 26.8224 m/s: %.4f seconds\n', time_at_target_speed(1));
else
    fprintf('Speed of 26.8224 m/s was not reached during the simulation.\n');
end

%% ---------------- Figure Export Settings ----------------
exportAxesFontSize = 16;
exportLabelFontSize = 18;
exportTitleFontSize = 20;
exportLegendFontSize = 16;

set(groot, ...
    'defaultFigureColor',   'white', ...
    'defaultAxesColor',     'white', ...
    'defaultAxesXColor',    'black', ...
    'defaultAxesYColor',    'black', ...
    'defaultAxesZColor',    'black', ...
    'defaultTextColor',     'black', ...
    'defaultAxesGridColor', 'black', ...
    'defaultAxesFontSize',  exportAxesFontSize, ...
    'defaultTextFontSize',  exportLabelFontSize, ...
    'defaultLegendFontSize', exportLegendFontSize);

outdir = fullfile(pwd, 'figures');
if ~exist(outdir, 'dir')
    mkdir(outdir);
end

save_fig = @(base, X, Y, xname, ynames) export_pdf_png_and_csv( ...
    gcf, outdir, base, X, Y, xname, ynames, ...
    exportAxesFontSize, exportLabelFontSize, exportTitleFontSize, exportLegendFontSize);

%% ---------------- Plots ----------------
% PID
figure;
plot(PID_Time, PID, 'k', 'LineWidth', 2);
title('PID Output');
xlabel('Time (s)');
ylabel('PID');
xlim([0, 1.8]);
grid on;
save_fig('pid', PID_Time, PID, 'Time_s', {'PID'});

% Slip Ratio
figure;
plot(Slip_Ratio_Time, Slip_Ratio, 'k', 'LineWidth', 2);
title(sprintf('Slip Ratio (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Slip Ratio');
xlim([0, 10]);
grid on;
save_fig('slip_ratio', Slip_Ratio_Time, Slip_Ratio, 'Time_s', {'Slip_Ratio'});

% Error
figure;
plot(Error_Time, Error, 'k', 'LineWidth', 2);
title(sprintf('Error (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Error');
xlim([0, 1.8]);
grid on;
save_fig('error', Error_Time, Error, 'Time_s', {'Error'});

% Distance
figure;
plot(Dist_Time, Dist, 'k', 'LineWidth', 2);
title(sprintf('Distance (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Distance (m)');
grid on;
save_fig('distance', Dist_Time, Dist, 'Time_s', {'Distance_m'});

% Velocity
figure;
plot(Vel_Time, Vel, 'k', 'LineWidth', 2);
title(sprintf('Velocity (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;
save_fig('velocity', Vel_Time, Vel, 'Time_s', {'Velocity_mps'});

% Acceleration
figure;
plot(Accel_Time, Accel, 'k', 'LineWidth', 2);
title(sprintf('Acceleration (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Accel (m/s^2)');
grid on;
save_fig('acceleration', Accel_Time, Accel, 'Time_s', {'Acceleration_mps2'});

% Downforce
figure;
plot(Downforce_Time, Downforce, 'k', 'LineWidth', 2);
title(sprintf('Downforce (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Downforce (N)');
grid on;
save_fig('downforce', Downforce_Time, Downforce, 'Time_s', {'Downforce_N'});

% Half-car front vertical load
figure;
plot(Fz_Front_Time, Fz_Front, 'k', 'LineWidth', 2);
title(sprintf('Half-Car Fz Front (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Fz Front (N)');
grid on;
save_fig('fz_front', Fz_Front_Time, Fz_Front, 'Time_s', {'Fz_Front_N'});

% Half-car rear vertical load
figure;
plot(Fz_Rear_Time, Fz_Rear, 'k', 'LineWidth', 2);
title(sprintf('Half-Car Fz Rear (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Fz Rear (N)');
grid on;
save_fig('fz_rear', Fz_Rear_Time, Fz_Rear, 'Time_s', {'Fz_Rear_N'});

% Half-car front suspension force
figure;
plot(F_Susp_Front_Time, F_Susp_Front, 'k', 'LineWidth', 2);
title(sprintf('Half-Car Front Suspension Force (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Front Suspension Force (N)');
grid on;
save_fig('f_susp_front', F_Susp_Front_Time, F_Susp_Front, 'Time_s', {'F_Susp_Front_N'});

% Half-car rear suspension force
figure;
plot(F_Susp_Rear_Time, F_Susp_Rear, 'k', 'LineWidth', 2);
title(sprintf('Half-Car Rear Suspension Force (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Rear Suspension Force (N)');
grid on;
save_fig('f_susp_rear', F_Susp_Rear_Time, F_Susp_Rear, 'Time_s', {'F_Susp_Rear_N'});

% Half-car front dynamic tire force
figure;
plot(F_Tire_Front_Dyn_Time, F_Tire_Front_Dyn, 'k', 'LineWidth', 2);
title(sprintf('Half-Car Front Dynamic Tire Force (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Front Dynamic Tire Force (N)');
grid on;
save_fig('f_tire_front_dyn', F_Tire_Front_Dyn_Time, F_Tire_Front_Dyn, 'Time_s', {'F_Tire_Front_Dyn_N'});

% Half-car rear dynamic tire force
figure;
plot(F_Tire_Rear_Dyn_Time, F_Tire_Rear_Dyn, 'k', 'LineWidth', 2);
title(sprintf('Half-Car Rear Dynamic Tire Force (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Rear Dynamic Tire Force (N)');
grid on;
save_fig('f_tire_rear_dyn', F_Tire_Rear_Dyn_Time, F_Tire_Rear_Dyn, 'Time_s', {'F_Tire_Rear_Dyn_N'});

% Half-car pitch angle
figure;
plot(Theta_Out_Time, Theta_Out, 'k', 'LineWidth', 2);
title(sprintf('Half-Car Pitch Angle (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Pitch Angle (rad)');
grid on;
save_fig('theta_out', Theta_Out_Time, Theta_Out, 'Time_s', {'Theta_Out_rad'});

% Target Motor Torque
figure;
plot(Motor_Tor_Time, Motor_Tor, 'k', 'LineWidth', 2);
title(sprintf('Target Motor Torque (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Motor Torque Request (Nm)');
ylim([0, 1.1*Max_Motor_Torque]);
grid on;
save_fig('target_motor_torque_PID_2', Motor_Tor_Time, Motor_Tor, 'Time_s', {'Motor_Torque_Nm'});

% Tractive Force
figure;
plot(Trac_Time, Trac, 'k', 'LineWidth', 2);
title(sprintf('Tractive Force (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Tractive Force (N)');
grid on;
save_fig('tractive_force', Trac_Time, Trac, 'Time_s', {'Tractive_Force_N'});

% Mu
figure;
plot(Mu_Time, Mu, 'k', 'LineWidth', 2);
title(sprintf('Mu (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('mu');
grid on;
save_fig('mu', Mu_Time, Mu, 'Time_s', {'Mu'});

% Alpha
figure;
plot(Alpha_Time, Alpha, 'k', 'LineWidth', 2);
title(sprintf('Blend Alpha (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Alpha');
ylim([0, 1]);
grid on;
save_fig('alpha', Alpha_Time, Alpha, 'Time_s', {'Alpha'});

% Tractive Force vs Slip Ratio
Trac_on_Slip_Time = interp_signal(Trac_Time, Trac, Slip_Ratio_Time);

figure;
plot(Slip_Ratio, Trac_on_Slip_Time, 'k', 'LineWidth', 2);
title(sprintf('Tractive Force vs. Slip Ratio (%.2f Grip Factor)', Grip_Fact));
xlabel('Slip Ratio');
ylabel('Tractive Force (N)');
xlim([0.0, 0.18]);
grid on;
save_fig('tractive_force_vs_slip_ratio', Slip_Ratio, Trac_on_Slip_Time, 'Slip_Ratio', {'Tractive_Force_N'});

% Wheel Torque
T_motor_atw     = Motor_Tor(:) * fd;
T_from_trac_atw = Trac(:) * r;
max_avail_atw   = Max_Motor_Tor(:) * fd;

figure;
hold on;
p1 = plot(Motor_Tor_Time,     T_motor_atw,     'LineWidth', 2);
p2 = plot(Trac_Time,          T_from_trac_atw, 'LineWidth', 2);
p3 = plot(Max_Motor_Tor_Time, max_avail_atw,   'g--', 'LineWidth', 2);
hold off;
grid on;

title(sprintf('Wheel Torque Comparison (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Wheel Torque (N*m)');
legend([p1 p2 p3], ...
    {'Target torque ATW', 'Tractive force -> torque ATW', 'Max available motor torque ATW'}, ...
    'Location','northeast');

allY = [T_motor_atw; T_from_trac_atw; max_avail_atw];
allY = allY(~isnan(allY));

if ~isempty(allY)
    ylim([0, 1.1*max(allY)]);
end

Torque_Time = Trac_Time;
T_motor_atw_on_Torque_Time = interp_signal(Motor_Tor_Time, T_motor_atw, Torque_Time);
max_avail_atw_on_Torque_Time = interp_signal(Max_Motor_Tor_Time, max_avail_atw, Torque_Time);

save_fig('torque_comparison_atw', Torque_Time, ...
    [T_motor_atw_on_Torque_Time, T_from_trac_atw, max_avail_atw_on_Torque_Time], ...
    'Time_s', {'Target_Torque_ATW_Nm','From_Tractive_Force_ATW_Nm','Max_Available_ATW_Nm'});

%% ---------------- Local Functions ----------------

function [t, y] = logged_signal(simout, signalName)

[found, sig] = get_simout_variable(simout, signalName);

if ~found
    [found, sig] = get_simout_variable(simout, ['out.' signalName]);
end

if ~found
    [hasOut, outContainer] = get_simout_variable(simout, 'out');
    if hasOut
        [found, sig] = get_named_signal(outContainer, signalName);
    end
end

if ~found
    [hasYout, youtContainer] = get_simout_variable(simout, 'yout');
    if hasYout
        [found, sig] = get_named_signal(youtContainer, signalName);
    end
end

if ~found
    availableNames = get_simout_names(simout);
    if isempty(availableNames)
        availableText = '(none)';
    else
        availableText = strjoin(availableNames, ', ');
    end

    error('Logged signal "%s" was not found. Available SimulationOutput variables: %s', ...
        signalName, availableText);
end

[t, y] = signal_to_time_data(sig, signalName);

end


function [found, value] = get_simout_variable(simout, variableName)

found = false;
value = [];

try
    names = get_simout_names(simout);
    if any(strcmp(names, variableName))
        value = simout.get(variableName);
        found = true;
        return;
    end
catch
end

try
    value = simout.(variableName);
    found = true;
catch
end

end


function names = get_simout_names(simout)

names = {};

try
    names = simout.who;
    if ischar(names)
        names = cellstr(names);
    end
catch
end

end


function [found, sig] = get_named_signal(container, signalName)

found = false;
sig = [];

if isstruct(container)
    if isfield(container, signalName)
        sig = container.(signalName);
        found = true;
        return;
    end

    if isfield(container, 'signals')
        [found, sig] = get_named_signal(container.signals, signalName);
        if found
            return;
        end
    end
end

if isa(container, 'Simulink.SimulationData.Dataset')
    try
        sig = container.get(signalName);
        found = ~isempty(sig);
        if found
            return;
        end
    catch
    end

    try
        sig = getElement(container, signalName);
        found = true;
        return;
    catch
    end
end

if isa(container, 'Simulink.SimulationData.Signal')
    try
        found = strcmp(container.Name, signalName);
        if found
            sig = container;
            return;
        end
    catch
    end
end

if iscell(container)
    for i = 1:numel(container)
        [found, sig] = get_named_signal(container{i}, signalName);
        if found
            return;
        end
    end
end

end


function [t, y] = signal_to_time_data(sig, signalName)

if isa(sig, 'Simulink.SimulationData.Signal')
    sig = sig.Values;
end

if isa(sig, 'timeseries')
    t = sig.Time(:);
    y = squeeze(sig.Data);
    y = y(:);
    return;
end

if isstruct(sig)
    if isfield(sig, 'Time') && isfield(sig, 'Data')
        t = sig.Time(:);
        y = squeeze(sig.Data);
        y = y(:);
        return;
    end

    if isfield(sig, 'time') && isfield(sig, 'signals')
        t = sig.time(:);
        if isfield(sig.signals, 'values')
            y = squeeze(sig.signals.values);
            y = y(:);
            return;
        end
    end
end

error('Logged signal "%s" was found but is not a supported timeseries-like type.', signalName);

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

[t_unique, ia] = unique(t, 'stable');
y_unique = y(ia);

if numel(t_unique) == 1
    yq = repmat(y_unique(1), size(tq));
else
    yq = interp1(t_unique, y_unique, tq, 'linear', 'extrap');
end

end


function export_pdf_png_and_csv(hfig, outdir, base, X, Y, xname, ynames, ...
    axesFontSize, labelFontSize, titleFontSize, legendFontSize)

X = X(:);

if isvector(Y)
    Y = Y(:);
end

n = min(numel(X), size(Y,1));
X = X(1:n);
Y = Y(1:n, :);

varNames = [{xname}, ynames(:)'];
varNames = matlab.lang.makeValidName(varNames);
T = array2table([X, Y], 'VariableNames', varNames);

writetable(T, fullfile(outdir, [base '.csv']));

apply_export_font_sizes(hfig, axesFontSize, labelFontSize, titleFontSize, legendFontSize);

exportgraphics(hfig, fullfile(outdir, [base '.pdf']), ...
    'BackgroundColor','white', 'ContentType','vector');

exportgraphics(hfig, fullfile(outdir, [base '.png']), ...
    'BackgroundColor','white', 'Resolution', 300);

end


function apply_export_font_sizes(hfig, axesFontSize, labelFontSize, titleFontSize, legendFontSize)

axesList = findall(hfig, 'Type', 'axes');

for i = 1:numel(axesList)
    ax = axesList(i);
    ax.FontSize = axesFontSize;
    ax.XLabel.FontSize = labelFontSize;
    ax.YLabel.FontSize = labelFontSize;
    ax.ZLabel.FontSize = labelFontSize;
    ax.Title.FontSize = titleFontSize;

    if isprop(ax, 'Toolbar')
        ax.Toolbar.Visible = 'off';
    end
end

legendList = findall(hfig, 'Type', 'legend');

for i = 1:numel(legendList)
    legendList(i).FontSize = legendFontSize;
end

end
