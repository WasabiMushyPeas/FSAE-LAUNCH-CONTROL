function lookupTable = THROTTLE_LOOKUP_GENERATOR()
% THROTTLE_LOOKUP_GENERATOR
% Builds a time-varying throttle lookup table for TC_SIM.slx using the same
% baseline setup as PARAMS.m and a local coordinate-descent optimization.

% -- Command Window --
clc;

% -- Model Setup --
model = "TC_SIM";              % Simulink model name                 (-)
modelFile = "TC_SIM.slx";      % Simulink model file                (-)

% -- Base Parameter Set --
params = buildBaseParams();
Time_pts = [0.000, 0.016, 0.033, 0.051, 0.072, 0.096, 0.121, 0.148, 0.177, 0.207, 0.239, 0.271, 0.302, 0.337, 0.376, 0.420, 0.469, 0.523, 0.583, 0.650, 0.724, 0.806, 0.899, 1.000, 1.406, 1.906, 2.440, 2.972, 3.554, 4.138, 4.724, 5.310, 5.895, 6.481, 7.067, 7.654, 8.240, 8.827, 9.413, 10.000];
Throttle_pts = [1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000]; % Baseline lookup table throttle points  (-)
baselineTimePts = Time_pts;     % Baseline lookup table time vector   (s)
baselineThrottlePts = Throttle_pts; % Baseline lookup table throttle vector (-)

% -- Generator Settings --
settings.stopTime_s = 10;                                     % Simulation stop time                 (s)
settings.targetDistance_m = 75;                               % Reporting distance target            (m)
settings.numRows = 40;                                        % Target lookup-table row count        (-)
settings.minTimeSpacing_s = 0.015;                            % Minimum spacing between time points  (s)
settings.earlyFocusTime_s = 1.0;                              % Early-time region with extra density (s)
settings.earlyRowFraction = 0.60;                             % Fraction of rows placed early        (-)
settings.outputDecimals = 3;                                  % Exported lookup-table precision      (-)
settings.passRadius = [0.09, 0.03, 0.01];                     % Throttle search radius per pass      (-)
settings.passStep = [0.03, 0.01, 0.005];                      % Throttle search step per pass        (-)
settings.objectiveStart_s = 0.05;                             % Slip objective start time            (s)
settings.objectiveWindow_s = 1.50;                            % Slip-tracking objective window       (s)
settings.maxOptimizationWindow_s = 1.50;                      % Last time actively optimized         (s)
settings.csvPath = fullfile(pwd, "throttle_lookup_table.csv"); % CSV export path                     (-)
settings.matPath = fullfile(pwd, "throttle_lookup_table.mat"); % MAT export path                     (-)

% -- Baseline Simulation --
fprintf("Running baseline simulation from PARAMS.m values...\n");
baselineResult = runLaunchSimulation( ...
    model, params, Time_pts, Throttle_pts, ...
    settings.stopTime_s, settings.targetDistance_m, settings.objectiveStart_s, settings.objectiveWindow_s, false, true);

% -- Adaptive Lookup Time Grid --
timeGrid = buildAdaptiveTimeGrid( ...
    baselineResult.time, baselineResult.slip, baselineResult.accel, ...
    Time_pts, Throttle_pts, settings.numRows, ...
    settings.stopTime_s, settings.minTimeSpacing_s, ...
    settings.earlyFocusTime_s, settings.earlyRowFraction);

% -- Initial Throttle Guess --
throttleInitial = clamp01(interp1( ...
    Time_pts(:), Throttle_pts(:), timeGrid, "linear", "extrap"));
throttleInitial(1) = Throttle_pts(1);
throttleInitial(end) = 1.0;

% -- Optimization Window Selection --
if isnan(baselineResult.metrics.TimeTo75m_s)
    optimizationWindow_s = settings.maxOptimizationWindow_s;
else
    optimizationWindow_s = min( ...
        settings.stopTime_s, ...
        max(settings.maxOptimizationWindow_s, baselineResult.metrics.TimeTo75m_s + 0.75));
end

activeIdx = find(timeGrid > 0 & timeGrid < optimizationWindow_s);

% -- Optimization Report --
fprintf("Adaptive grid built with %d rows.\n", numel(timeGrid));
fprintf("Optimizing %d lookup points through %.3f s...\n", numel(activeIdx), optimizationWindow_s);

% -- Model Load / Fast Restart Management --
load_system(modelFile);
cleanupObj = onCleanup(@() disableFastRestart(model));

% -- Initial Best Candidate --
bestThrottle = throttleInitial(:);
bestResult = runLaunchSimulation( ...
    model, params, timeGrid, bestThrottle, settings.stopTime_s, ...
    settings.targetDistance_m, settings.objectiveStart_s, settings.objectiveWindow_s, true, false);

% -- Optimization History Log --
history = table( ...
    zeros(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), ...
    zeros(0, 1), zeros(0, 1), zeros(0, 1), ...
    'VariableNames', { ...
    'Pass', 'Point_Index', 'Time_s', 'Old_Throttle', ...
    'New_Throttle', 'Score', 'TimeTo75m_s'});

% -- Coordinate-Descent Optimization --
for passIdx = 1:numel(settings.passRadius)
    radius = settings.passRadius(passIdx); % Candidate search radius         (-)
    step = settings.passStep(passIdx);     % Candidate search resolution     (-)

    fprintf("\nPass %d/%d  (radius = %.3f, step = %.3f)\n", ...
        passIdx, numel(settings.passRadius), radius, step);

    % Evaluate one lookup-table point at a time.
    for n = 1:numel(activeIdx)
        idx = activeIdx(n);                                              % Active lookup table index            (-)
        currentThrottle = bestThrottle(idx);                             % Current throttle value               (-)
        candidateValues = buildCandidateValues(currentThrottle, radius, step); % Nearby throttle candidates    (-)

        localBestThrottle = currentThrottle;       % Best local throttle so far         (-)
        localBestResult = bestResult;              % Best local simulation result       (-)
        localBestScore = bestResult.metrics.Score; % Best local objective value         (-)

        % Sweep nearby throttle values at this single time point.
        for k = 1:numel(candidateValues)
            candidateThrottle = candidateValues(k); % Trial throttle command            (-)

            if abs(candidateThrottle - currentThrottle) < 1e-12
                candidateResult = bestResult;
            else
                throttleTrial = bestThrottle;        % Trial lookup-table throttle vector (-)
                throttleTrial(idx) = candidateThrottle; % Trial value at active index   (-)

                candidateResult = runLaunchSimulation( ...
                    model, params, timeGrid, throttleTrial, ...
                    settings.stopTime_s, settings.targetDistance_m, settings.objectiveStart_s, settings.objectiveWindow_s, true, false);
            end

            if candidateResult.metrics.Score < (localBestScore - 1e-9)
                localBestScore = candidateResult.metrics.Score;
                localBestThrottle = candidateThrottle;
                localBestResult = candidateResult;
            end
        end

        % Commit the locally improved value if it beats the current best.
        if abs(localBestThrottle - currentThrottle) > 1e-12
            bestThrottle(idx) = localBestThrottle;
            bestResult = localBestResult;

            history = [history; { ...
                passIdx, idx, timeGrid(idx), currentThrottle, ...
                localBestThrottle, bestResult.metrics.Score, ...
                bestResult.metrics.TimeTo75m_s}]; %#ok<AGROW>

            fprintf( ...
                "  t = %6.3f s : %.3f -> %.3f | score %.6f | RMS err %.5f | max err %.5f | T75 %.4f s\n", ...
                timeGrid(idx), currentThrottle, localBestThrottle, ...
                bestResult.metrics.Score, bestResult.metrics.RmsSlipError, ...
                bestResult.metrics.MaxAbsSlipError, bestResult.metrics.TimeTo75m_s);
        end
    end
end

% -- Tail Cleanup --
tailAdjustedThrottle = enforceFullThrottleTail(bestThrottle);
if any(abs(tailAdjustedThrottle - bestThrottle) > 1e-12)
    tailAdjustedResult = runLaunchSimulation( ...
        model, params, timeGrid, tailAdjustedThrottle, settings.stopTime_s, ...
        settings.targetDistance_m, settings.objectiveStart_s, settings.objectiveWindow_s, true, false);

    if tailAdjustedResult.metrics.Score <= (bestResult.metrics.Score + 1e-9)
        bestThrottle = tailAdjustedThrottle;
        bestResult = tailAdjustedResult;
        fprintf("\nApplied non-decreasing tail after the first full-throttle point.\n");
    end
end

% -- Final Lookup Table --
lookupTable = table( ...
    timeGrid(:), clamp01(bestThrottle(:)), ...
    'VariableNames', {'Time', 'Throttle_Percent'});
lookupTable = roundLookupTable(lookupTable, settings.outputDecimals);

% -- Export Files --
writeLookupTableCsv(lookupTable, settings.csvPath, settings.outputDecimals);

baselineMetrics = baselineResult.metrics; % Baseline performance summary     (struct)
finalMetrics = bestResult.metrics;        % Optimized performance summary    (struct)
save(settings.matPath, ...
    'lookupTable', 'history', 'baselineMetrics', 'finalMetrics', ...
    'baselineTimePts', 'baselineThrottlePts', 'settings');

% -- Console Summary --
fprintf("\nSaved CSV: %s\n", settings.csvPath);
fprintf("Saved MAT: %s\n", settings.matPath);
fprintf("\nBaseline ISE:          %.6f\n", baselineResult.metrics.IntegratedSquaredError);
fprintf("Optimized ISE:         %.6f\n", bestResult.metrics.IntegratedSquaredError);
fprintf("Baseline RMS error:    %.5f\n", baselineResult.metrics.RmsSlipError);
fprintf("Optimized RMS error:   %.5f\n", bestResult.metrics.RmsSlipError);
fprintf("Baseline max abs err:  %.5f\n", baselineResult.metrics.MaxAbsSlipError);
fprintf("Optimized max abs err: %.5f\n", bestResult.metrics.MaxAbsSlipError);
fprintf("Baseline final abs err:  %.5f\n", baselineResult.metrics.FinalAbsSlipError);
fprintf("Optimized final abs err: %.5f\n", bestResult.metrics.FinalAbsSlipError);
fprintf("Baseline peak slip:    %.4f\n", baselineResult.metrics.PeakSlip);
fprintf("Optimized peak slip:   %.4f\n", bestResult.metrics.PeakSlip);
fprintf("Baseline T75 (info):   %.4f s\n", baselineResult.metrics.TimeTo75m_s);
fprintf("Optimized T75 (info):  %.4f s\n\n", bestResult.metrics.TimeTo75m_s);

% -- Lookup Table Figure --
plotLookupTable(lookupTable, baselineTimePts, baselineThrottlePts, settings.stopTime_s);

disp(lookupTable);
printLookupArrays(lookupTable);

% -- Cleanup --
clear cleanupObj
disableFastRestart(model);

end

function params = buildBaseParams()
% buildBaseParams
% -- Parameter block mirrored from PARAMS.m --

% --- Vehicle Parameters ---
params.Mv = 285.763;            % Vehicle Weight                     (kg)
params.r = 0.203;               % Wheel Radius                       (m)
params.fd = 8;                  % Final Drive motor:tire             (Ratio)
params.h_cg = 0.25273;          % Height of Center of Gravity        (m)
params.W = 1.53035;             % Wheelbase                          (m)
params.Grip_Fact = 1.0;         % Grip Factor of the Tire            (-)
params.Grip_Influence = 1.2;    % Grip Factor Impact on Table        (-)
params.Drive_Train = 0.89;      % Drivetrain loss percent            (-)

% --- Drivetrain Inertias ---
params.J = 0.51151;             % Total reflected rotating inertia    (kg*m^2)
params.J_Motor = 0.34661;       % Inboard motor/diff inertia, wheel-side referenced
params.J_Wheel = 0.16490;       % Two driven rear wheel assemblies    (kg*m^2)

% --- Half-Shaft Parameters ---
params.K = 5105.49;             % Rear half-shaft pair stiffness      (Nm/rad)
params.C = 16.56;               % Half-shaft damping                  (Nms/rad)

% --- Aerodynamics ---
params.A = 1;                   % Frontal Area                       (m^2)
params.rho = 1.225;             % Air Density                        (kg/m^3)
params.Cd = 1;                  % Drag Coefficient                   (-)
params.Cl = 3.8;                % Lift Coefficient                   (-)
params.Cp = 0.53;               % Load Percent on Rear Wheels Aero   (-)

% -- Static Loads --
params.cg_f = 0.734568;         % Front Axel to CG                   (m)

% -- Control Params --
params.Slip_Target = 0.13;      % Target Slip Ratio                  (-)
params.T_request = 150;         % Initial Driver Torque Request      (Nm)
params.Start_Blend = 3.0;       % Velocity Low Start to Change PIDs  (m/s)
params.End_Blend = 8.0;         % Velocity High End Changing PIDs    (m/s)
params.Max_Motor_RPM = 7000;    % EMRAX 208 speed limit              (rpm)
params.Max_Wheel_Omega = (params.Max_Motor_RPM * 2*pi/60) / params.fd; % Maximum Wheel Angular Velocity (rad/s)
params.tau_motor = 0.03;        % Motor/controller torque lag        (s)
params.tau_tire = 0.03;         % Tire relaxation delay              (s)
params.tau_load = 0.03;         % Load transfer lag                  (s)
params.Max_Motor_Torque = 150;  % Maximum Motor Torque               (Nm)
params.gravity = 9.80665;       % Accel due to Gravity Used          (m/s^2)

% -- Launch Control Params From Car Code --
params.CONTROLLER_INFLUENCE = int32(100);

params.LC_TABLE_LENGTH = int32(40);
params.LC_TS = int32(10);       % Launch-control sample time         (ms)
params.LC_Ts_sec = double(params.LC_TS)/1000; % Launch-control sample time (s)

params.LC_KP = int32(700);
params.LC_KI_STEP = int32(25);
params.LC_KD_STEP = int32(0);

params.LC_SLIP_TARGET = int32(1300); % 1300 = 13.00% slip

params.LC_START_BLEND = int32(300);  % cm/s = 3 m/s
params.LC_END_BLEND = int32(500);    % cm/s = 5 m/s

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

% -- PID Params Slip Ratio --
params.P = 1.75;                % Proportional in PID                (-)
params.I = 4;                   % Integral in PID                    (-)
params.D = 0;                   % Derivative in PID                  (-)
params.Ts = 0.01;               % Sample Time                        (s)

end

function result = runLaunchSimulation( ...
    model, params, timePts, throttlePts, stopTime_s, targetDistance_m, objectiveStart_s, objectiveWindow_s, useFastRestart, keepSignals)
% runLaunchSimulation
% Runs one TC_SIM simulation and returns signals plus summary metrics.

% -- Simulation Input Object --
simIn = Simulink.SimulationInput(model);

% -- Push Base Parameters Into Model Workspace --
paramNames = fieldnames(params);
for i = 1:numel(paramNames)
    name = paramNames{i};                      % Parameter field name            (-)
    simIn = simIn.setVariable(name, params.(name));
end

% -- Push Lookup Table and Stop Time --
simIn = simIn.setVariable("Time_pts", reshape(timePts, 1, []));
simIn = simIn.setVariable("Throttle_pts", reshape(throttlePts, 1, []));
simIn = simIn.setModelParameter("StopTime", num2str(stopTime_s));

% -- Optional Fast Restart --
if useFastRestart
    simIn = simIn.setModelParameter("FastRestart", "on");
end

% -- Run Simulink Model --
simOut = sim(simIn);

% -- Extract Logged Signals --
[time, velocity] = extractSignal(simOut, "v_velocity");
[~, distance] = extractSignal(simOut, "v_distance");
[~, slip] = extractSignal(simOut, "slip_ratio");
[~, accel] = extractSignal(simOut, "v_accel");

% -- Control Error Signal --
if isfield(simOut, "error") || isprop(simOut, "error")
    [~, controlError] = extractSignal(simOut, "error");
else
    controlError = params.Slip_Target - slip; % Fallback slip-tracking error (-)
end

% -- Compute Performance Metrics --
metrics = computeLaunchMetrics( ...
    time, distance, velocity, slip, controlError, accel, ...
    targetDistance_m, objectiveStart_s, objectiveWindow_s);

% -- Assemble Output Struct --
result.metrics = metrics;

% -- Optional Signal Retention --
if keepSignals
    result.time = time;
    result.distance = distance;
    result.velocity = velocity;
    result.slip = slip;
    result.error = controlError;
    result.accel = accel;
end

end

function [time, data] = extractSignal(simOut, signalName)
% extractSignal
% Returns time and data vectors from a logged simulation output signal.

% -- Requested Logged Signal --
signal = simOut.(signalName); % Logged Simulink signal object          (-)

% -- Timeseries Format --
if isa(signal, "timeseries")
    time = signal.Time(:);
    data = signal.Data(:);
    return
end

% -- Structure-With-Time Format --
if isstruct(signal) && isfield(signal, "Time") && isfield(signal, "Data")
    time = signal.Time(:);
    data = signal.Data(:);
    return
end

error("Could not extract signal '%s' from simulation output.", signalName);

end

function metrics = computeLaunchMetrics(time, distance, velocity, slip, controlError, accel, targetDistance_m, objectiveStart_s, objectiveWindow_s)
% computeLaunchMetrics
% Converts time histories into a scalar objective plus summary metrics.

% -- Key Launch Metrics --
timeTo75m_s = crossingTimeLinearInterp(time, distance, targetDistance_m); % Interpolated 75 m time         (s)
finalDistance_m = distance(end);                                          % Final distance at stop time    (m)
finalVelocity_mps = velocity(end);                                        % Final velocity at stop time    (m/s)
peakSlip = max(slip);                                                     % Maximum slip ratio             (-)

% -- Slip-Tracking Objective Window --
windowMask = time >= objectiveStart_s & time <= objectiveWindow_s; % Samples used by slip objective (-)
windowTime = time(windowMask);                                 % Time vector inside objective      (s)
windowError = controlError(windowMask);                        % Error vector inside objective     (-)

% -- Scalar Objective Function --
if numel(windowTime) < 2
    integratedSquaredError = inf;
    rmsSlipError = inf;
    meanAbsSlipError = inf;
    maxAbsSlipError = inf;
    finalAbsSlipError = inf;
else
    windowDuration_s = max(windowTime(end) - windowTime(1), eps);      % Objective window duration         (s)
    integratedSquaredError = trapz(windowTime, windowError.^2);        % Integrated squared slip error     (-^2*s)
    rmsSlipError = sqrt(integratedSquaredError / windowDuration_s);    % RMS slip error                    (-)
    meanAbsSlipError = trapz(windowTime, abs(windowError)) / windowDuration_s; % Mean absolute slip error (-)
    maxAbsSlipError = max(abs(windowError));                           % Maximum absolute slip error       (-)
    finalAbsSlipError = abs(windowError(end));                         % Final absolute slip error         (-)
end

score = rmsSlipError ...                  % Primary RMS slip-tracking error          (-)
    + 0.35 * maxAbsSlipError ...          % Penalize large instantaneous deviations  (-)
    + 0.15 * meanAbsSlipError ...         % Penalize average offset from target      (-)
    + 0.10 * finalAbsSlipError;           % Encourage settling near zero error       (-)

% -- Output Metric Struct --
metrics.Reached75m = ~isnan(timeTo75m_s);
metrics.TimeTo75m_s = timeTo75m_s;
metrics.FinalDistance_m = finalDistance_m;
metrics.FinalVelocity_mps = finalVelocity_mps;
metrics.PeakSlip = peakSlip;
metrics.IntegratedSquaredError = integratedSquaredError;
metrics.RmsSlipError = rmsSlipError;
metrics.MeanAbsSlipError = meanAbsSlipError;
metrics.MaxAbsSlipError = maxAbsSlipError;
metrics.FinalAbsSlipError = finalAbsSlipError;
metrics.ObjectiveStart_s = objectiveStart_s;
metrics.ObjectiveWindow_s = objectiveWindow_s;
metrics.Score = score;

% -- Optional Smoothness Metric (reserved for future weighting) --
jerkPenalty = trapz(time, gradient(accel, time).^2); %#ok<NASGU> % Integrated squared jerk         (-)

end

function timeGrid = buildAdaptiveTimeGrid( ...
    simTime, slip, accel, baselineTimePts, baselineThrottlePts, ...
    numRows, stopTime_s, minTimeSpacing_s, earlyFocusTime_s, earlyRowFraction)
% buildAdaptiveTimeGrid
% Creates a non-uniform lookup-table time grid with extra resolution in
% the first second and more spread-out points later in the launch.

% -- Baseline Throttle Trace --
baselineThrottleTrace = interp1( ...
    baselineTimePts(:), baselineThrottlePts(:), simTime(:), "linear", "extrap");

% -- Dynamic Activity Signals --
slipRate = abs(gradient(slip(:), simTime(:)));                           % Slip-rate magnitude             (-/s)
accelRate = abs(gradient(accel(:), simTime(:)));                         % Acceleration-rate magnitude     (m/s^3)
throttleRate = abs(gradient(baselineThrottleTrace(:), simTime(:)));      % Throttle-rate magnitude         (-/s)

% -- Adaptive Density Weighting --
activity = 0.25 ...
    + 3.0 * normalizeSignal(slipRate) ...
    + 1.5 * normalizeSignal(accelRate) ...
    + 0.75 * normalizeSignal(throttleRate) ...
    + 2.0 * exp(-simTime(:) / 0.40);

activity = max(activity, 1e-6);
splitTime_s = min(max(earlyFocusTime_s, 5 * minTimeSpacing_s), stopTime_s - 5 * minTimeSpacing_s); % Early / late split time  (s)
numEarlyRows = max(6, min(numRows - 3, round(numRows * earlyRowFraction)));                          % Rows allocated early     (-)
numLateRows = numRows - numEarlyRows + 1;                                                            % Rows allocated late      (-)

% -- Early and Late Segment Grids --
earlyGrid = buildAdaptiveSegmentGrid( ...
    simTime(:), activity, 0, splitTime_s, numEarlyRows);
lateGrid = buildAdaptiveSegmentGrid( ...
    simTime(:), activity, splitTime_s, stopTime_s, numLateRows);

% -- Combined Time Grid --
timeGrid = [earlyGrid(1:end-1); lateGrid];

% -- Minimum Spacing Enforcement --
timeGrid = enforceMinimumSpacing(timeGrid(:), minTimeSpacing_s, stopTime_s);

end

function timeGrid = buildAdaptiveSegmentGrid(simTime, activity, startTime_s, endTime_s, numRows)
% buildAdaptiveSegmentGrid
% Generates one adaptive time segment from a weighted activity trace.

% -- Segment Support Data --
supportTime = unique([startTime_s; simTime(simTime >= startTime_s & simTime <= endTime_s); endTime_s]); % Segment support times   (s)
supportActivity = interp1(simTime, activity, supportTime, "linear", "extrap");                           % Segment activity values (-)

% -- Uniform Spacing In Cumulative Activity Space --
cumActivity = cumtrapz(supportTime, supportActivity);                 % Cumulative activity             (-)
targetCum = linspace(cumActivity(1), cumActivity(end), numRows);     % Evenly spaced cumulative bins   (-)

% -- Invert Back To Time --
timeGrid = interp1(cumActivity, supportTime, targetCum, "linear").'; % Adaptive segment time points    (s)
timeGrid(1) = startTime_s;
timeGrid(end) = endTime_s;

end

function x = normalizeSignal(x)
% normalizeSignal
% Scales a vector onto [0, 1] while preserving zeros.

% -- Input Magnitude --
x = abs(x(:));
denom = max(x); % Maximum magnitude used for normalization (-)

% -- Safe Normalization --
if denom <= 0
    x = zeros(size(x));
else
    x = x ./ denom;
end

end

function timeGrid = enforceMinimumSpacing(timeGrid, minSpacing_s, stopTime_s)
% enforceMinimumSpacing
% Makes sure adjacent lookup-table points do not collapse together.

% -- Boundary Conditions --
timeGrid = timeGrid(:);
timeGrid(1) = 0;
timeGrid(end) = stopTime_s;

% -- Degenerate Case --
if numel(timeGrid) < 2
    return
end

% -- Forward Pass --
for i = 2:numel(timeGrid)-1
    lowerBound = timeGrid(i-1) + minSpacing_s;                       % Earliest allowed time           (s)
    upperBound = stopTime_s - ((numel(timeGrid) - i) * minSpacing_s); % Latest allowed time            (s)
    timeGrid(i) = min(max(timeGrid(i), lowerBound), upperBound);
end

% -- Backward Pass --
for i = numel(timeGrid)-1:-1:2
    lowerBound = (i - 1) * minSpacing_s;    % Earliest allowed time after backward pass (s)
    upperBound = timeGrid(i+1) - minSpacing_s; % Latest allowed time after backward pass (s)
    timeGrid(i) = min(max(timeGrid(i), lowerBound), upperBound);
end

end

function values = buildCandidateValues(currentValue, radius, step)
% buildCandidateValues
% Creates a bounded candidate sweep around the current throttle value.

% -- Candidate Sweep Vector --
values = clamp01((currentValue - radius):step:(currentValue + radius)); % Raw candidate throttle values   (-)
values = unique([currentValue, values], "stable");                      % Unique candidates including current value (-)

end

function tCross = crossingTimeLinearInterp(time, signal, targetValue)
% crossingTimeLinearInterp
% Returns the first threshold crossing time using linear interpolation.

% -- First Crossing Index --
idx = find(signal >= targetValue, 1, "first"); % First sample above threshold (-)

% -- Never Reached --
if isempty(idx)
    tCross = NaN;
    return
end

% -- Crossed At First Sample --
if idx == 1
    tCross = time(1);
    return
end

% -- Bracketing Samples --
t1 = time(idx - 1);      % Lower time bracket                  (s)
t2 = time(idx);          % Upper time bracket                  (s)
y1 = signal(idx - 1);    % Lower signal bracket                (-)
y2 = signal(idx);        % Upper signal bracket                (-)

% -- Linear Interpolation --
if y2 == y1
    tCross = t2;
else
    tCross = t1 + (targetValue - y1) * (t2 - t1) / (y2 - y1);
end

end

function x = clamp01(x)
% clamp01
% Saturates a scalar or vector onto the interval [0, 1].

x = min(max(x, 0), 1);

end

function throttle = enforceFullThrottleTail(throttle)
% enforceFullThrottleTail
% Prevents the optimized table from dipping after it first reaches full
% throttle, which keeps the late-run command table cleaner.

% -- Column Format --
throttle = throttle(:);
firstFullIdx = find(throttle >= 0.999, 1, "first"); % First effectively-full-throttle index (-)

% -- No Full-Throttle Segment --
if isempty(firstFullIdx)
    return
end

% -- Force Non-Decreasing Tail --
for i = firstFullIdx+1:numel(throttle)
    throttle(i) = max(throttle(i), throttle(i-1));
end

end

function printLookupArrays(lookupTable)
% printLookupArrays
% Prints MATLAB-ready lookup arrays for direct copy into PARAMS.m.

% -- Printable Array Text --
timeText = strjoin(compose("%.3f", lookupTable.Time.'), ", ");                     % Time array as formatted text      (-)
throttleText = strjoin(compose("%.3f", lookupTable.Throttle_Percent.'), ", ");    % Throttle array as formatted text  (-)

fprintf("Time_pts = [%s];\n", timeText);
fprintf("Throttle_pts = [%s];\n", throttleText);

end

function plotLookupTable(lookupTable, baselineTimePts, baselineThrottlePts, stopTime_s)
% plotLookupTable
% Graphs the optimized throttle lookup table and overlays the baseline
% lookup table for quick visual comparison.

% -- Figure --
figure;
hold on;

% -- Baseline Table --
plot( ...
    baselineTimePts(:), baselineThrottlePts(:), ...
    '--', 'Color', [0.45, 0.45, 0.45], 'LineWidth', 1.5);

% -- Optimized Table --
plot( ...
    lookupTable.Time, lookupTable.Throttle_Percent, ...
    '-o', 'Color', [0.00, 0.00, 0.00], ...
    'LineWidth', 2, 'MarkerSize', 4, ...
    'MarkerFaceColor', [0.00, 0.00, 0.00]);

% -- Figure Formatting --
hold off;
grid on;
box on;
xlim([0, stopTime_s]);
ylim([0, 1.05]);
title('Optimal Throttle Lookup Table');
xlabel('Time (s)');
ylabel('Throttle Percent');
legend('Baseline Table', 'Optimized Table', 'Location', 'southeast');

end

function lookupTable = roundLookupTable(lookupTable, numDecimals)
% roundLookupTable
% Rounds the exported lookup-table values to the requested precision.

% -- Rounded Export Values --
lookupTable.Time = round(lookupTable.Time, numDecimals);                         % Rounded lookup-table time values      (s)
lookupTable.Throttle_Percent = round(lookupTable.Throttle_Percent, numDecimals); % Rounded lookup-table throttle values  (-)

end

function writeLookupTableCsv(lookupTable, csvPath, numDecimals)
% writeLookupTableCsv
% Writes the lookup table to CSV using fixed decimal formatting.

% -- CSV Format String --
rowFormat = sprintf('%%.%df,%%.%df\\n', numDecimals, numDecimals); % Fixed-width numeric CSV row format (-)

% -- CSV File Output --
fid = fopen(csvPath, 'w');
if fid == -1
    error('Could not open CSV path for writing: %s', csvPath);
end

cleanupObj = onCleanup(@() fclose(fid));

fprintf(fid, 'Time,Throttle_Percent\n');
for i = 1:height(lookupTable)
    fprintf(fid, rowFormat, lookupTable.Time(i), lookupTable.Throttle_Percent(i));
end

clear cleanupObj

end

function disableFastRestart(model)
% disableFastRestart
% Safely turns Fast Restart off if the model is still loaded.

% -- Loaded Model Check --
if bdIsLoaded(model)
    try
        set_param(model, "FastRestart", "off");
    catch
    end
end

end
