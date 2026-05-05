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
%Switch = 3.0;                  % Velocity to Change Controllers     (m/s)
Start_Blend = 3.0;              % Velocity Low Start to Change PIDs  (m/s)
End_Blend = 8.0;                % Velocity High End Changing PIDs    (m/s)
Max_Motor_RPM = 7000;           % EMRAX 208 speed limit                (rpm)
Max_Wheel_Omega = (Max_Motor_RPM * 2*pi/60) / fd;  % Maximum wheel angular velocity  (rad/s)
tau_motor = 0.020;              % Motor/controller torque lag          (s)
Max_Motor_Torque = 150;         % Maximum Motor Torque               (rad/s)
gravity = 9.80665;              % Accel due to Gravity Used          (m/s^2)

% -- Look Up Table --
Time_pts = [0.000, 0.016, 0.033, 0.051, 0.072, 0.096, 0.121, 0.148, 0.177, 0.207, 0.239, 0.271, 0.302, 0.337, 0.376, 0.420, 0.469, 0.523, 0.583, 0.650, 0.724, 0.806, 0.899, 1.000, 1.406, 1.906, 2.440, 2.972, 3.554, 4.138, 4.724, 5.310, 5.895, 6.481, 7.067, 7.654, 8.240, 8.827, 9.413, 10.000];
Throttle_pts = [0.000, 0.364, 0.600, 0.830, 0.903, 0.920, 0.917, 0.903, 0.906, 0.895, 0.886, 0.882, 0.873, 0.883, 0.876, 0.888, 0.889, 0.890, 0.905, 0.879, 0.863, 0.878, 0.887, 0.907, 0.990, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000];


% -- PID Params Slip Ratio --
P = 1.75;                       % Proportional in PID
I = 4;                          % Integral in PID
D = 0;                          % Derivative in PID
Ts = 0.01;                      % Sample Time                        (s)


% -- Simulink --
timedomain = 10;                % Simulation Time (s)
simout = sim("TC_SIM.slx", timedomain);

% -- Time and Data --
Time = simout.v_velocity.Time;           % Time Vector                   (s)
Wheel_Speed = simout.wheel_speed.Data;   % Wheel Speed                   (m/s)
Lift = simout.v_lift.Data;               % Vechile Lift                  (N)
Drag = simout.v_drag.Data;               % Vechile Drag                  (N)
Trac = simout.Longitudinal_Force.Data;   % Tractive Force                (N)
Accel = simout.v_accel.Data;             % Vechicle Accel                (m/s^2)
Vel = simout.v_velocity.Data;            % Vechicle Velocity             (m/s)
Dist = simout.v_distance.Data;           % Vechicle Distance             (m)
Motor_Tor = simout.T_motor.Data;         % Motor Torque                  (Nm)
Normal_Force = simout.normal_force.Data; % Normal Force on the rear axel (N)
Error = simout.error.Data;               % PID Error
Mu = simout.mu.Data;                     % Friction Coefficient
Slip_Ratio = simout.slip_ratio.Data;     % Slip Ratio
Max_Motor_Tor = simout.Max_Tor.Data;     % Max Motor torque              (Nm)
PID = simout.pid_correction.Data;        % The PID Correction



% -- Print Time when Distance is 75 m --
target_distance = 75; % Target distance (m)
time_at_target_distance = Time(Dist >= target_distance); % Find times when distance is greater than or equal to 75 m

if ~isempty(time_at_target_distance)
    fprintf('Time when distance reaches 75 m: %.4f seconds\n', time_at_target_distance(1)); %[output:8f33bcb4]
else
    fprintf('Distance of 75 m was not reached during the simulation.\n');
end

% -- Print Time when Speed is 26.8224 m/s --
target_speed = 26.8224; % Target speed (m/s)
time_at_target_speed = Time(Vel >= target_speed); % Find times when speed is greater than or equal to 26.8224 m/s

if ~isempty(time_at_target_speed)
    fprintf('Time when speed reaches 26.8224 m/s: %.4f seconds\n', time_at_target_speed(1));
else
    fprintf('Speed of 26.8224 m/s was not reached during the simulation.\n');
end

% --- Black-on-white figure theme (applies to all figures in this session) ---
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
if ~exist(outdir, 'dir'); mkdir(outdir); end

% Helper to export the current figure and write CSV of the series used.
save_fig = @(base, X, Y, xname, ynames) export_pdf_png_and_csv( ...
    gcf, outdir, base, X, Y, xname, ynames, ...
    exportAxesFontSize, exportLabelFontSize, exportTitleFontSize, exportLegendFontSize);

% PID
figure;
plot(Time, PID, 'k', 'LineWidth', 2);
title('PID Output');
xlabel('Time (s)'); ylabel('PID');
xlim([0, 1.8]);
grid on;
save_fig('pid', Time, PID, 'Time_s', {'PID'});
% Slip Ratio
figure;
plot(Time, Slip_Ratio, 'k', 'LineWidth', 2);
title(sprintf('Slip Ratio (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)'); ylabel('Slip Ratio');
%ylim([0, 0.2]);
xlim([0, 10]);
grid on;
save_fig('slip_ratio', Time, Slip_Ratio, 'Time_s', {'Slip_Ratio'});


% Error
figure;
plot(Time, Error, 'k', 'LineWidth', 2);
title(sprintf('Error (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)'); ylabel('Error');
xlim([0, 1.8]);
grid on;
save_fig('error', Time, Error, 'Time_s', {'Error'});


% Distance
figure;
plot(Time, Dist, 'k', 'LineWidth', 2);
title(sprintf('Distance (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)'); ylabel('Distance (m)');
grid on;
save_fig('distance', Time, Dist, 'Time_s', {'Distance_m'});

% Velocity
figure;
plot(Time, Vel, 'k', 'LineWidth', 2);
title(sprintf('Velocity (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)'); ylabel('Velocity (m/s)');
grid on;
save_fig('velocity', Time, Vel, 'Time_s', {'Velocity_mps'});

% Acceleration
figure;
plot(Time, Accel, 'k', 'LineWidth', 2);
title(sprintf('Acceleration (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)'); ylabel('Accel (m/s^2)');
%ylim([0, 12]);
grid on;
save_fig('acceleration', Time, Accel, 'Time_s', {'Acceleration_mps2'});




% Normal Force
figure;
plot(Time, Normal_Force, 'k', 'LineWidth', 2);
title(sprintf('Normal Force on Rear Axle (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)'); ylabel('Normal Force (N)');
ylim([0, 3500]);
grid on;
save_fig('normal_force', Time, Normal_Force, 'Time_s', {'Normal_Force_N'});

% Target Motor Torque
figure;
plot(Time, Motor_Tor, 'k', 'LineWidth', 2);
title(sprintf('Target Motor Torque PID (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)'); ylabel('Torque %');
ylim([0, 1.1]);
grid on;
save_fig('target_motor_torque_PID_2', Time, Motor_Tor, 'Time_s', {'Motor_Torque_Nm'});


% Tractive Force (Time)
figure;
plot(Time, Trac, 'k', 'LineWidth', 2);
title(sprintf('Tractive Force (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)'); ylabel('Tractive Force (N)');
grid on;
save_fig('tractive_force', Time, Trac, 'Time_s', {'Tractive_Force_N'});

% Mu (Time)
figure;
plot(Time, Mu, 'k', 'LineWidth', 2);
title(sprintf('Mu (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)'); ylabel('mu');
grid on;
save_fig('mu', Time, Mu, 'Time_s', {'Mu'});

% Tractive Force vs Slip Ratio (XY)
figure;
plot(Slip_Ratio, Trac, 'k', 'LineWidth', 2);
title(sprintf('Tractive Force vs. Slip Ratio (%.2f Grip Factor)', Grip_Fact));
xlabel('Slip Ratio'); ylabel('Tractive Force (N)');
xlim([0.0, 0.18]);
grid on;
save_fig('tractive_force_vs_slip_ratio', Slip_Ratio, Trac, 'Slip_Ratio', {'Tractive_Force_N'});

% --- Combined plot: all series as torque at the wheel (ATW) ---
T_motor_atw    = Motor_Tor(:) * fd;      % motor target -> wheel torque (Nm)
T_from_trac_atw= Trac(:) * r;            % tractive force -> wheel torque (Nm)
max_avail_atw  = Max_Motor_Tor(:) * fd;  % max motor torque -> wheel torque (Nm)

figure; hold on;
p1 = plot(Time, T_motor_atw,     'LineWidth', 2);
p2 = plot(Time, T_from_trac_atw, 'LineWidth', 2);
p3 = plot(Time, max_avail_atw,   'g--', 'LineWidth', 2);
hold off; grid on;

title(sprintf('Wheel Torque Comparison (%.2f Grip Factor)', Grip_Fact));
xlabel('Time (s)');
ylabel('Wheel Torque (N·m)');
legend([p1 p2 p3], ...
    {'Target torque ATW', 'Tractive force \rightarrow torque ATW', 'Max available motor torque ATW'}, ...
    'Location','northeast');

allY = [T_motor_atw; T_from_trac_atw; max_avail_atw];
allY = allY(~isnan(allY));
if ~isempty(allY)
    ylim([0, 1.1*max(allY)]);
end

save_fig('torque_comparison_atw', Time, [T_motor_atw, T_from_trac_atw, max_avail_atw], ...
    'Time_s', {'Target_Torque_ATW_Nm','From_Tractive_Force_ATW_Nm','Max_Available_ATW_Nm'});


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

% Write CSV (same basename)
writetable(T, fullfile(outdir, [base '.csv']));

% Make saved plot text readable in reports and presentations.
apply_export_font_sizes(hfig, axesFontSize, labelFontSize, titleFontSize, legendFontSize);

% Export vector PDF (same basename)
exportgraphics(hfig, fullfile(outdir, [base '.pdf']), ...
    'BackgroundColor','white', 'ContentType','vector');

% (Optional) also keep a PNG for quick viewing
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
