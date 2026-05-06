%% FINAL_DRIVE_GRIP_SWEEP_75M.m
% Sweeps final drive ratio and tire grip factor, then plots simulated
% 75 m time versus final drive ratio for each grip factor.
%
% The 2D sweep is flattened into independent SimulationInput objects so
% parsim can spread the runs across multiple MATLAB workers.

clear; clc; close all;

%% ---------------- Sweep Settings ----------------
model = "TC_SIM";
timedomain = 10;                % Simulation stop time [s]
target_distance = 75;           % Distance target [m]

fd_vals = 2.5:0.1:12.5;         % Final drive ratios to test
Grip_Fact_vals = 0.50:0.10:1.0; % Tire grip factors to test

requestedWorkers = [];          % [] = automatic. Set to a number to force it.
batchSize = 128;                % Lower this if RAM becomes tight
useFastRestart = true;
fastRestartOption = onOffString(useFastRestart);

saveOutputs = true;
outdir = fullfile(pwd, 'figures');

%% ---------------- Vehicle Parameters ----------------
params.Mv = 285.763;            % Vehicle mass [kg]
params.r = 0.203;               % Wheel radius [m]
params.fd = 4.5;                % Final drive, motor speed / wheel speed
params.h_cg = 0.25273;          % CG height [m]
params.W = 1.53035;             % Wheelbase [m]
params.Grip_Fact = 1.0;         % Tire grip factor
params.Grip_Influence = 1.2;    % Grip factor impact on table
params.Drive_Train = 0.89;      % Drivetrain efficiency [0..1]

%% ---------------- Drivetrain Inertias ----------------
params.J = 0.51151;             % Total reflected rotating inertia [kg*m^2]
params.J_Motor = 0.34661;       % Inboard motor/diff inertia, wheel-side referenced
params.J_Wheel = 0.16490;       % Two driven rear wheel assemblies [kg*m^2]

%% ---------------- Half-Shaft Parameters ----------------
params.K = 5105.49;             % Rear half-shaft pair stiffness [Nm/rad]
params.C = 16.56;               % Half-shaft damping [Nms/rad]

%% ---------------- Aerodynamics ----------------
params.A = 1;                   % Frontal area [m^2]
params.rho = 1.225;             % Air density [kg/m^3]
params.Cd = 1;                  % Drag coefficient
params.Cl = 3.8;                % Lift/downforce coefficient
params.Cp = 0.53;               % Rear aero load fraction

%% ---------------- Static Loads ----------------
params.cg_f = 0.734568;         % Front axle to CG [m]

%% ---------------- Control Params ----------------
params.Slip_Target = 0.13;      % Target slip ratio
params.T_request = 150;         % Driver torque request [Nm]
params.Start_Blend = 3.0;       % Velocity low start to change PIDs [m/s]
params.End_Blend = 8.0;         % Velocity high end changing PIDs [m/s]
params.Max_Motor_RPM = 7000;    % EMRAX 208 speed limit [rpm]
params.tau_motor = 0.020;       % Motor/controller torque lag [s]
params.Max_Motor_Torque = 150;  % Maximum motor torque [Nm]
params.gravity = 9.80665;       % Gravity [m/s^2]
params.Max_Wheel_Omega = (params.Max_Motor_RPM * 2*pi/60) / params.fd;

%% ---------------- Launch Control Params From Car Code ----------------
params.CONTROLLER_INFLUENCE = int32(100);

params.LC_TABLE_LENGTH = int32(40);
params.LC_TS = int32(10);       % Launch-control sample time [ms]
params.LC_Ts_sec = double(params.LC_TS)/1000;

params.LC_KP = int32(700);
params.LC_KI_STEP = int32(25);
params.LC_KD_STEP = int32(0);

params.LC_SLIP_TARGET = int32(1300);

params.LC_START_BLEND = int32(300);
params.LC_END_BLEND = int32(500);

params.LC_GRIP_FACTOR = int32(1000);
params.LC_GRIP_INFLUENCE = int32(1500);
params.LC_MIN_GRIP_SCALE = int32(0);
params.LC_MAX_GRIP_SCALE = int32(1500);

params.LC_MIN_CMD = int32(0);
params.LC_MAX_CMD = int32(1000);

params.LC_MIN_CORR = int32(-1000);
params.LC_MAX_CORR = int32(1000);

params.launchTimeMs = int32([ ...
    0,16,33,51,72,96,121,148,177,207,239,271,302,337,376,420,469,523, ...
    583,650,724,806,899,1000,1406,1906,2440,2972,3554,4138,4724,5310, ...
    5895,6481,7067,7654,8240,8827,9413,10000]);

params.launchCmd = int32([ ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000, ...
    1000,1000,1000,1000,1000,1000,1000,1000,1000,1000]);

params.Time_pts = double(params.launchTimeMs)/1000;
params.Throttle_pts = double(params.launchCmd)/1000;

%% ---------------- Build 2D Sweep Grid ----------------
[fd_grid, grip_grid] = ndgrid(fd_vals(:), Grip_Fact_vals(:));
fd_sweep = fd_grid(:);
grip_sweep = grip_grid(:);
nRuns = numel(fd_sweep);

fprintf('Final drive / grip sweep\n');
fprintf('Model: %s\n', model);
fprintf('Total simulations: %d\n', nRuns);
fprintf('Final drive values: %.2f to %.2f (%d values)\n', ...
    fd_vals(1), fd_vals(end), numel(fd_vals));
fprintf('Grip factor values: %.2f to %.2f (%d values)\n\n', ...
    Grip_Fact_vals(1), Grip_Fact_vals(end), numel(Grip_Fact_vals));

%% ---------------- Start Parallel Pool ----------------
pool = startParallelPool(requestedWorkers, nRuns);
fprintf('Using %d parallel workers.\n\n', pool.NumWorkers);

%% ---------------- Run Simulations ----------------
load_system(model);

time_75m = nan(nRuns, 1);
simError = strings(nRuns, 1);
numBatches = ceil(nRuns / batchSize);
tStart = tic;

for b = 1:numBatches
    idxStart = (b-1)*batchSize + 1;
    idxEnd = min(b*batchSize, nRuns);
    idxBatch = idxStart:idxEnd;
    nThis = numel(idxBatch);

    fprintf('Running batch %d / %d (cases %d to %d)\n', ...
        b, numBatches, idxStart, idxEnd);

    in = repmat(Simulink.SimulationInput(model), nThis, 1);

    for k = 1:nThis
        ii = idxBatch(k);
        runParams = params;
        runParams.fd = fd_sweep(ii);
        runParams.Grip_Fact = grip_sweep(ii);
        runParams.Max_Wheel_Omega = ...
            (runParams.Max_Motor_RPM * 2*pi/60) / runParams.fd;

        in(k) = Simulink.SimulationInput(model);
        in(k) = setVariablesFromStruct(in(k), runParams);
        in(k) = in(k).setModelParameter('StopTime', num2str(timedomain));
    end

    out = parsim(in, ...
        'ShowProgress', 'on', ...
        'UseFastRestart', fastRestartOption);

    for k = 1:nThis
        ii = idxBatch(k);

        errMsg = "";
        try
            errMsg = string(out(k).ErrorMessage);
        catch
        end

        if strlength(errMsg) > 0
            simError(ii) = errMsg;
            continue;
        end

        try
            [tVec, dVec] = getDistanceTrace(out(k));
            time_75m(ii) = crossingTimeLinearInterp(tVec, dVec, target_distance);
        catch ME
            simError(ii) = string(ME.message);
            time_75m(ii) = nan;
        end
    end

    clear in out
end

elapsedTime = toc(tStart);

%% ---------------- Assemble Results ----------------
time_75m_grid = reshape(time_75m, numel(fd_vals), numel(Grip_Fact_vals));

results = table( ...
    fd_sweep, ...
    grip_sweep, ...
    time_75m, ...
    ~isnan(time_75m), ...
    simError, ...
    'VariableNames', {'FinalDrive', 'Grip_Fact', 'Time_75m_s', 'Reached75m', 'ErrorMessage'});

validResults = results(results.Reached75m, :);
validResults = sortrows(validResults, 'Time_75m_s', 'ascend');

fprintf('\n====================================================\n');
fprintf('Sweep complete in %.2f seconds\n', elapsedTime);
fprintf('Successful runs reaching %.1f m: %d / %d\n', ...
    target_distance, height(validResults), nRuns);
fprintf('====================================================\n\n');

if isempty(validResults)
    fprintf('No simulations reached %.1f m.\n', target_distance);
else
    fprintf('Best result:\n');
    disp(validResults(1, {'FinalDrive','Grip_Fact','Time_75m_s'}));
end

%% ---------------- Plot ----------------
figure('Color', 'white');
hold on;
colors = lines(numel(Grip_Fact_vals));

for g = 1:numel(Grip_Fact_vals)
    plot(fd_vals, time_75m_grid(:, g), '-o', ...
        'Color', colors(g, :), ...
        'LineWidth', 2, ...
        'MarkerSize', 6, ...
        'DisplayName', sprintf('Grip\\_Fact = %.2f', Grip_Fact_vals(g)));
end

hold off;
grid on;
xlabel('Final Drive Ratio');
ylabel('Simulated 75 m Time [s]');
title('Final Drive Ratio vs 75 m Time by Tire Grip Factor');
legend('Location', 'best');

%% ---------------- Save Outputs ----------------
if saveOutputs
    if ~exist(outdir, 'dir')
        mkdir(outdir);
    end

    save(fullfile(outdir, 'final_drive_grip_sweep_75m.mat'), ...
        'fd_vals', 'Grip_Fact_vals', 'time_75m_grid', ...
        'results', 'validResults', 'elapsedTime');

    writetable(results, fullfile(outdir, 'final_drive_grip_sweep_75m_long.csv'));

    wideResults = array2table(time_75m_grid, ...
        'VariableNames', gripColumnNames(Grip_Fact_vals));
    wideResults.FinalDrive = fd_vals(:);
    wideResults = movevars(wideResults, 'FinalDrive', 'Before', 1);
    writetable(wideResults, fullfile(outdir, 'final_drive_grip_sweep_75m_wide.csv'));

    exportgraphics(gcf, fullfile(outdir, 'final_drive_grip_sweep_75m.png'), ...
        'Resolution', 300);
end

%% ---------------- Local Functions ----------------
function pool = startParallelPool(requestedWorkers, nRuns)
if isempty(ver('parallel'))
    error(['Parallel Computing Toolbox is required for this script because ' ...
        'the sweep is intended to use more than one core.']);
end

if nRuns < 2
    error('At least two sweep points are required to use more than one core.');
end

numCores = feature('numcores');
autoWorkers = min(max(2, numCores - 1), nRuns);

if isempty(requestedWorkers)
    numWorkers = autoWorkers;
else
    numWorkers = min(requestedWorkers, nRuns);
    if numWorkers < 2
        error('requestedWorkers must be at least 2 for a multicore sweep.');
    end
end

pool = gcp('nocreate');
if isempty(pool)
    pool = parpool('local', numWorkers);
elseif pool.NumWorkers ~= numWorkers
    delete(pool);
    pool = parpool('local', numWorkers);
end
end

function simIn = setVariablesFromStruct(simIn, params)
names = fieldnames(params);
for n = 1:numel(names)
    simIn = simIn.setVariable(names{n}, params.(names{n}));
end
end

function [tVec, dVec] = getDistanceTrace(simOut)
try
    distSig = simOut.v_distance;
catch
    try
        distSig = simOut.get('v_distance');
    catch
        error('Simulation output does not contain v_distance.');
    end
end

if isa(distSig, 'timeseries')
    tVec = distSig.Time(:);
    dVec = squeeze(distSig.Data);
    dVec = dVec(:);
    return;
end

if isstruct(distSig) && isfield(distSig, 'Time') && isfield(distSig, 'Data')
    tVec = distSig.Time(:);
    dVec = squeeze(distSig.Data);
    dVec = dVec(:);
    return;
end

error('Could not extract time/distance from v_distance.');
end

function tCross = crossingTimeLinearInterp(tVec, yVec, targetValue)
idx = find(yVec >= targetValue, 1, 'first');

if isempty(idx)
    tCross = nan;
    return;
end

if idx == 1
    tCross = tVec(1);
    return;
end

t1 = tVec(idx-1);
t2 = tVec(idx);
y1 = yVec(idx-1);
y2 = yVec(idx);

if y2 == y1
    tCross = t2;
else
    tCross = t1 + (targetValue - y1) * (t2 - t1) / (y2 - y1);
end
end

function names = gripColumnNames(gripVals)
names = cell(1, numel(gripVals));
for k = 1:numel(gripVals)
    label = sprintf('Grip_Fact_%.2f', gripVals(k));
    names{k} = strrep(label, '.', 'p');
end
end

function value = onOffString(flag)
if flag
    value = 'on';
else
    value = 'off';
end
end
