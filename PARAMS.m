% --- Vehicle Parameters ---
Mv = 285.763;                   % Vehicle Weight                     (kg)
r = 0.203;                      % Wheel Radius                       (m)
fd = 4.5;                       % Final Drive motor:tire             (Ratio)
h_cg = 0.25273;                 % Height of Center of Gravity        (m)
W = 1.53035;                    % Wheelbase                          (m)
Grip_Fact = 1.0;                % Grip Factor of the Tire
Grip_Influence = 1.2;           % Grip Factor Impact on Table
Drive_Train = 0.89;             % Drivetain loss perecent

% --- Drivetrain Inertias ---
J = 0.51151;                    % Total reflected rotating inertia     (kg*m^2)
J_Motor = 0.34661;              % Inboard motor/diff inertia, wheel-side referenced
J_Wheel = 0.16490;              % Two driven rear wheel assemblies     (kg*m^2)

% --- Half-Shaft Parameters ---
K = 5105.49;                    % Rear half-shaft pair stiffness       (Nm/rad)

% Damping chosen from approx zeta = 0.20 with vehicle mass coupled in.
% Use C = 9.55 if testing drivetrain alone with no vehicle body coupling.
C = 16.56;                      % Half-shaft damping                   (Nms/rad)


% --- Aerodynamics ---
A = 1;                          % Frontal Area                       (m^2)
rho = 1.225;                    % Air Density                        (kg/m^3)
Cd = 1;                         % Drag Coefficient
Cl = 3.8;                       % Lift Coefficient
Cp = 0.53;                      % Load Percent on Rear Wheels Aero

% -- Static Loads --
cg_f = 0.734568;                % Front Axel to CG                   (m)

% -- Control Params --
Slip_Target = 0.13;             % Target Slip Ratio
T_request = 150;                % Initial Driver Torque Request      (Nm)
Start_Blend = 3.0;              % Velocity Low Start to Change PIDs  (m/s)
End_Blend = 8.0;                % Velocity High End Changing PIDs    (m/s)
Max_Motor_RPM = 7000;           % EMRAX 208 speed limit              (rpm)
Max_Wheel_Omega = (Max_Motor_RPM * 2*pi/60) / fd;  % Maximum wheel angular velocity  (rad/s)
tau_motor = 0.020;              % Motor/controller torque lag         (s)
Max_Motor_Torque = 150;         % Maximum Motor Torque                (Nm)
gravity = 9.80665;              % Accel due to Gravity Used           (m/s^2)


% -- PID Params Slip Ratio --
% P = 1.75;                       % Proportional in PID
% I = 4;                          % Integral in PID
% D = 0;                          % Derivative in PID
% Ts = 0.01;                      % Sample Time                        (s)


% -- Launch Control Params From Car Code --
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

% launchCmd = int32([ ...
%     0,364,600,830,903,920,917,903,906,895,886,882,873,883,876,888,889, ...
%     890,905,879,863,878,887,907,990,1000,1000,1000,1000,1000,1000, ...
%     1000,1000,1000,1000,1000,1000,1000,1000,1000]);

launchCmd = int32([ ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000]);

% Optional double versions for normal Simulink lookup tables / plotting.
Time_pts = double(launchTimeMs)/1000;   % [s]
Throttle_pts = double(launchCmd)/1000;  % 0 to 1 scale


% -- Simulink --
timedomain = 10;                % Simulation Time (s)
simout = sim("TC_SIM.slx", timedomain);

% -- Time and Data --
% Pull each logged signal with its own time vector. Some signals are logged at
% the variable-step solver times, while the LC controller signals are logged
% at the discrete 10 ms controller sample time. Plot each signal against its
% own time vector to avoid dimension mismatch errors.

Wheel_Speed_Time = simout.wheel_speed.Time(:);
Wheel_Speed = squeeze(simout.wheel_speed.Data);
Wheel_Speed = Wheel_Speed(:);   % Wheel Speed (m/s)

Lift_Time = simout.v_lift.Time(:);
Lift = squeeze(simout.v_lift.Data);
Lift = Lift(:);                 % Vehicle Lift / Downforce (N)

Drag_Time = simout.v_drag.Time(:);
Drag = squeeze(simout.v_drag.Data);
Drag = Drag(:);                 % Vehicle Drag (N)

Trac_Time = simout.Longitudinal_Force.Time(:);
Trac = squeeze(simout.Longitudinal_Force.Data);
Trac = Trac(:);                 % Tractive Force (N)

Accel_Time = simout.v_accel.Time(:);
Accel = squeeze(simout.v_accel.Data);
Accel = Accel(:);               % Vehicle Accel (m/s^2)

Vel_Time = simout.v_velocity.Time(:);
Vel = squeeze(simout.v_velocity.Data);
Vel = Vel(:);                   % Vehicle Velocity (m/s)
Time = Vel_Time;                % Main vehicle time vector

Dist_Time = simout.v_distance.Time(:);
Dist = squeeze(simout.v_distance.Data);
Dist = Dist(:);                 % Vehicle Distance (m)

Motor_Tor_Time = simout.T_motor.Time(:);
Motor_Tor = squeeze(simout.T_motor.Data);
Motor_Tor = Motor_Tor(:);       % Motor Torque (Nm)

Normal_Force_Time = simout.normal_force.Time(:);
Normal_Force = squeeze(simout.normal_force.Data);
Normal_Force = Normal_Force(:); % Rear axle normal force (N)

Error_Time = simout.error.Time(:);
Error = squeeze(simout.error.Data);
Error = Error(:);               % LC slip error

Mu_Time = simout.mu.Time(:);
Mu = squeeze(simout.mu.Data);
Mu = Mu(:);                     % Friction Coefficient

Alpha_Time = simout.alpha.Time(:);
Alpha = squeeze(simout.alpha.Data);
Alpha = Alpha(:);               % LC blend factor

Slip_Ratio_Time = simout.slip_ratio.Time(:);
Slip_Ratio = squeeze(simout.slip_ratio.Data);
Slip_Ratio = Slip_Ratio(:);     % Slip Ratio

Max_Motor_Tor_Time = simout.Max_Tor.Time(:);
Max_Motor_Tor = squeeze(simout.Max_Tor.Data);
Max_Motor_Tor = Max_Motor_Tor(:); % Max Motor torque (Nm)

PID_Time = simout.pid_correction.Time(:);
PID = squeeze(simout.pid_correction.Data);
PID = PID(:);                   % LC PID correction


% -- Print Time when Distance is 75 m --
target_distance = 75; % Target distance (m)
time_at_target_distance = Dist_Time(Dist >= target_distance);

if ~isempty(time_at_target_distance)
    fprintf('Time when distance reaches 75 m: %.4f seconds\n', time_at_target_distance(1));
else
    fprintf('Distance of 75 m was not reached during the simulation.\n');
end

% -- Print Time when Speed is 26.8224 m/s --
target_speed = 26.8224; % Target speed (m/s)
time_at_target_speed = Vel_Time(Vel >= target_speed);

if ~isempty(time_at_target_speed)
    fprintf('Time when speed reaches 26.8224 m/s: %.4f seconds\n', time_at_target_speed(1));
else
    fprintf('Speed of 26.8224 m/s was not reached during the simulation.\n');
end

% --- Black-on-white figure theme ---
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

% -- Figure output settings --
outdir = fullfile(pwd, 'figures');
if ~exist(outdir, 'dir')
    mkdir(outdir);
end

save_fig = @(base, X, Y, xname, ynames) export_pdf_png_and_csv( ...
    gcf, outdir, base, X, Y, xname, ynames, ...
    exportAxesFontSize, exportLabelFontSize, exportTitleFontSize, exportLegendFontSize);

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

% Normal Force
figure;
plot(Normal_Force_Time, Normal_Force, 'k', 'LineWidth', 2);
title(sprintf('Normal Force on Rear Axle (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Normal Force (N)');
ylim([0, 3500]);
grid on;
save_fig('normal_force', Normal_Force_Time, Normal_Force, 'Time_s', {'Normal_Force_N'});

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
% Interpolate tractive force onto the slip-ratio time base before making the XY plot.
Trac_on_Slip_Time = interp_signal(Trac_Time, Trac, Slip_Ratio_Time);

figure;
plot(Slip_Ratio, Trac_on_Slip_Time, 'k', 'LineWidth', 2);
title(sprintf('Tractive Force vs. Slip Ratio (%.2f Grip Factor)', Grip_Fact));
xlabel('Slip Ratio');
ylabel('Tractive Force (N)');
xlim([0.0, 0.18]);
grid on;
save_fig('tractive_force_vs_slip_ratio', Slip_Ratio, Trac_on_Slip_Time, 'Slip_Ratio', {'Tractive_Force_N'});

% --- Combined plot: all series as torque at the wheel ATW ---
T_motor_atw     = Motor_Tor(:) * fd;      % motor target -> wheel torque (Nm)
T_from_trac_atw = Trac(:) * r;            % tractive force -> wheel torque (Nm)
max_avail_atw   = Max_Motor_Tor(:) * fd;  % max motor torque -> wheel torque (Nm)

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

% Save the combined CSV on one common time base.
Torque_Time = Trac_Time;
T_motor_atw_on_Torque_Time = interp_signal(Motor_Tor_Time, T_motor_atw, Torque_Time);
max_avail_atw_on_Torque_Time = interp_signal(Max_Motor_Tor_Time, max_avail_atw, Torque_Time);

save_fig('torque_comparison_atw', Torque_Time, ...
    [T_motor_atw_on_Torque_Time, T_from_trac_atw, max_avail_atw_on_Torque_Time], ...
    'Time_s', {'Target_Torque_ATW_Nm','From_Tractive_Force_ATW_Nm','Max_Available_ATW_Nm'});


function yq = interp_signal(t, y, tq)
% Interpolate a logged signal onto a requested time base.
% This avoids plot/CSV dimension errors when some signals are continuous and
% others are discrete at the launch-control sample time.

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
% Ensure shapes

X = X(:);

if isvector(Y)
    Y = Y(:);
end

n = min(numel(X), size(Y,1));
X = X(1:n);
Y = Y(1:n, :);

% Build table with clean variable names
varNames = [{xname}, ynames(:)'];
varNames = matlab.lang.makeValidName(varNames);
T = array2table([X, Y], 'VariableNames', varNames);

% Write CSV
writetable(T, fullfile(outdir, [base '.csv']));

% Make saved plot text readable
apply_export_font_sizes(hfig, axesFontSize, labelFontSize, titleFontSize, legendFontSize);

% Export vector PDF
exportgraphics(hfig, fullfile(outdir, [base '.pdf']), ...
    'BackgroundColor','white', 'ContentType','vector');

% Export PNG
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
