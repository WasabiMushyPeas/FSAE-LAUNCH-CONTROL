%% TC PID Sweep on 32 Cores
clear; clc;

%% ---------------- Vehicle Parameters ----------------
Mv = 285.763;                   % Vehicle Weight                     (kg)
r = 0.203;                      % Wheel Radius                       (m)
fd = 4.5;                       % Final Drive motor:tire             (Ratio)
h_cg = 0.25273;                 % Height of Center of Gravity        (m)
W = 1.53035;                    % Wheelbase                          (m)
Grip_Fact = 1.0;                % Grip Factor of the Tire
Grip_Influence = 1.2;           % Grip Factor Impact on Table
Drive_Train = 0.89;             % Drivetrain loss percent

%% ---------------- Drivetrain Inertias ----------------
J = 0.51151;                    % Total reflected rotating inertia   (kg*m^2)
J_Motor = 0.34661;              % Inboard motor/diff inertia, wheel-side referenced
J_Wheel = 0.16490;              % Two driven rear wheel assemblies   (kg*m^2)

%% ---------------- Half-Shaft Parameters ----------------
K = 5105.49;                    % Rear half-shaft pair stiffness     (Nm/rad)
C = 16.56;                      % Half-shaft damping                 (Nms/rad)

%% ---------------- Aerodynamics ----------------
A = 1;                          % Frontal area                       (m^2)
rho = 1.225;                    % Air density                        (kg/m^3)
Cd = 1;                         % Drag coefficient                   (-)
Cl = 3.8;                       % Lift coefficient                   (-)
Cp = 0.53;                      % Rear aero load fraction            (-)

%% ---------------- Static Loads ----------------
cg_f = 0.734568;                % Front axle to CG                   (m)

%% ---------------- Control Params ----------------
Slip_Target = 0.13;             % Target slip ratio
T_request = 150;                % Driver torque request              (Nm)
Start_Blend = 3.0;              % Velocity Low Start to Change PIDs  (m/s)
End_Blend = 8.0;                % Velocity High End Changing PIDs    (m/s)
Max_Motor_RPM = 7000;           % EMRAX 208 speed limit              (rpm)
Max_Wheel_Omega = (Max_Motor_RPM * 2*pi/60) / fd;  % Maximum wheel angular velocity  (rad/s)
tau_motor = 0.020;              % Motor/controller torque lag        (s)
Max_Motor_Torque = 150;         % Maximum motor torque               (Nm)
gravity = 9.80665;              % Gravity                            (m/s^2)

%% ---------------- Look Up Table ----------------
Time_pts = [0.000, 0.016, 0.033, 0.051, 0.072, 0.096, 0.121, 0.148, 0.177, 0.207, 0.239, 0.271, 0.302, 0.337, 0.376, 0.420, 0.469, 0.523, 0.583, 0.650, 0.724, 0.806, 0.899, 1.000, 1.406, 1.906, 2.440, 2.972, 3.554, 4.138, 4.724, 5.310, 5.895, 6.481, 7.067, 7.654, 8.240, 8.827, 9.413, 10.000];
Throttle_pts = [0.000, 0.364, 0.600, 0.830, 0.903, 0.920, 0.917, 0.903, 0.906, 0.895, 0.886, 0.882, 0.873, 0.883, 0.876, 0.888, 0.889, 0.890, 0.905, 0.879, 0.863, 0.878, 0.887, 0.907, 0.990, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000, 1.000];

%% ---------------- PID Setup ----------------
D = 0;                          % Derivative gain fixed
Ts = 0.01;                      % Sample time                        (s)

P_vals = 0:0.01:2;              % 201 values
I_vals = 0:1:100;               % 101 values

%% ---------------- Simulation Setup ----------------
model = "TC_SIM_FAST";
timedomain = 10;                % Simulation stop time (s)
target_distance = 75;           % Distance target (m)

numWorkers = 16;                % Requested workers
batchSize = 256;                % Conservative for 32 GB RAM
topN = 10;

%% Build sweep grid
[P_grid, I_grid] = ndgrid(P_vals, I_vals);
P_sweep = P_grid(:);
I_sweep = I_grid(:);
nRuns = numel(P_sweep);

fprintf('Total simulations to run: %d\n', nRuns); %[output:59e86ebf]
fprintf('P sweep: %.2f to %.2f in %.2f steps\n', P_vals(1), P_vals(end), P_vals(2)-P_vals(1)); %[output:5977c0f9]
fprintf('I sweep: %.0f to %.0f in %.0f steps\n\n', I_vals(1), I_vals(end), I_vals(2)-I_vals(1)); %[output:780ee398]

%% Start / reset parallel pool
pool = gcp('nocreate');
if isempty(pool)
    parpool('local', numWorkers);
elseif pool.NumWorkers ~= numWorkers
    delete(pool);
    parpool('local', numWorkers);
end

%% Load model
load_system(model);

%% Preallocate results
time_75m = nan(nRuns, 1);
simError = strings(nRuns, 1);

numBatches = ceil(nRuns / batchSize);

tStart = tic;

for b = 1:numBatches %[output:group:6b54d772]
    idxStart = (b-1)*batchSize + 1;
    idxEnd   = min(b*batchSize, nRuns);
    idxBatch = idxStart:idxEnd;
    nThis    = numel(idxBatch);

    fprintf('Running batch %d / %d  (cases %d to %d)\n', b, numBatches, idxStart, idxEnd); %[output:2047b74b] %[output:023ce4a3] %[output:66bad25c] %[output:76504f99] %[output:09c840fa] %[output:5fb9abfc] %[output:0ce42669] %[output:5cb30556] %[output:6276fc45] %[output:1bae5ef7] %[output:505d6d9e] %[output:45f5f06d] %[output:3d35634b] %[output:1781b634] %[output:1e429a86] %[output:047a46af] %[output:77b67692] %[output:8f729505] %[output:63a1cf15] %[output:2355e09f] %[output:23845919] %[output:7c240902] %[output:317e0dfa] %[output:9fe649b9] %[output:9fe807e2] %[output:5b1e4182] %[output:462efaa0] %[output:498e8404] %[output:0576212d] %[output:5575a976] %[output:04031884] %[output:33037721] %[output:78f91cf0] %[output:74e52527] %[output:50c067bc] %[output:62f515c6] %[output:50b982bb] %[output:12c4609b] %[output:7111b0f4] %[output:171b8f7b] %[output:01631a1d] %[output:081567ad] %[output:15c83f93] %[output:322db268] %[output:1f29abb3] %[output:6e82c9f1] %[output:362b789c] %[output:98b029c7] %[output:441b8570] %[output:16620f81] %[output:9f8d4748] %[output:8c8c7cbd] %[output:489e6a19] %[output:6de2b623] %[output:4f879926] %[output:5d8e9a37] %[output:03c59e89] %[output:417b9e7f] %[output:895d154a] %[output:0ea8e4e3] %[output:4d019e63] %[output:4adaa2df] %[output:6e28a6db] %[output:01a8d64e] %[output:24e7a57b] %[output:51f64713] %[output:01c3aadc] %[output:36714251] %[output:4ad193c6] %[output:3aae8384] %[output:65984de6] %[output:1aa9c1cd] %[output:80c5326c] %[output:1b1ecdbd] %[output:9e4a5377] %[output:4bb248e7] %[output:7f370406] %[output:0b7b17aa] %[output:65306fb4] %[output:8d389c67]

    % Preallocate SimulationInput array
    in(nThis,1) = Simulink.SimulationInput(model);

    for k = 1:nThis
        ii = idxBatch(k);

        in(k) = Simulink.SimulationInput(model);
        in(k) = in(k).setVariable('P', P_sweep(ii));
        in(k) = in(k).setVariable('I', I_sweep(ii));
        in(k) = in(k).setVariable('D', D);

        % Stop time per run
        in(k) = in(k).setModelParameter('StopTime', num2str(timedomain));

        % Optional speed-up:
        % Uncomment this only if your model behaves correctly in accelerator mode
        % in(k) = in(k).setModelParameter('SimulationMode', 'accelerator');
    end

    % Run this batch in parallel
    out = parsim(in, ... %[output:2092655f] %[output:07ab29b2] %[output:318d5a40] %[output:6e2bd2ba] %[output:6e6c8d87] %[output:14dad609] %[output:30c4dc17] %[output:75eac396] %[output:745c1566] %[output:1e7db223] %[output:1e42f40f] %[output:786ff781] %[output:55263708] %[output:2abd8942] %[output:5e0b9d80] %[output:5a9aa6e5] %[output:9eb5424e] %[output:749330ed] %[output:093610bc] %[output:910c5af8] %[output:3195b599] %[output:22bc523d] %[output:1e797c70] %[output:2199dc7c] %[output:555efdea] %[output:6a2f085f] %[output:1ccdfade] %[output:04c1645f] %[output:63fba9d0] %[output:5bbc8888] %[output:0d622a7e] %[output:9e4273a7] %[output:67369e79] %[output:8a9a5254] %[output:04d42ca7] %[output:37f122c5] %[output:63c5e80d] %[output:62c841bf] %[output:552840ff] %[output:369924b2] %[output:552b87ab] %[output:08b86e46] %[output:9dd2507d] %[output:6522ab10] %[output:1f7255a1] %[output:25f901aa] %[output:3b7e3b61] %[output:716c4285] %[output:1e369efc] %[output:3f549eac] %[output:18333a85] %[output:6399e7e2] %[output:7648cf4b] %[output:19e4e80f] %[output:86b1b219] %[output:24d777fb] %[output:86b2cee4] %[output:14ffcb46] %[output:41eb35e3] %[output:8353a090] %[output:6870d137] %[output:752b6b05] %[output:206aab1d] %[output:10784a83] %[output:2c9f8cb2] %[output:9c00cb3e] %[output:34886850] %[output:2b6a8ff1] %[output:66bc998e] %[output:3d0ffc8d] %[output:4094026b] %[output:0c5300a0] %[output:0ad2700a] %[output:2e16dc94] %[output:6fc225e1] %[output:7f2e254c] %[output:39169c01] %[output:7bd457a2] %[output:44489666] %[output:4cacffe4] %[output:18bc44d2] %[output:42191f5d] %[output:6a90c6eb] %[output:89daf99d] %[output:27049b3d]
        'ShowProgress', 'on', ... %[output:2092655f] %[output:07ab29b2] %[output:318d5a40] %[output:6e2bd2ba] %[output:6e6c8d87] %[output:14dad609] %[output:30c4dc17] %[output:75eac396] %[output:745c1566] %[output:1e7db223] %[output:1e42f40f] %[output:786ff781] %[output:55263708] %[output:2abd8942] %[output:5e0b9d80] %[output:5a9aa6e5] %[output:9eb5424e] %[output:749330ed] %[output:093610bc] %[output:910c5af8] %[output:3195b599] %[output:22bc523d] %[output:1e797c70] %[output:2199dc7c] %[output:555efdea] %[output:6a2f085f] %[output:1ccdfade] %[output:04c1645f] %[output:63fba9d0] %[output:5bbc8888] %[output:0d622a7e] %[output:9e4273a7] %[output:67369e79] %[output:8a9a5254] %[output:04d42ca7] %[output:37f122c5] %[output:63c5e80d] %[output:62c841bf] %[output:552840ff] %[output:369924b2] %[output:552b87ab] %[output:08b86e46] %[output:9dd2507d] %[output:6522ab10] %[output:1f7255a1] %[output:25f901aa] %[output:3b7e3b61] %[output:716c4285] %[output:1e369efc] %[output:3f549eac] %[output:18333a85] %[output:6399e7e2] %[output:7648cf4b] %[output:19e4e80f] %[output:86b1b219] %[output:24d777fb] %[output:86b2cee4] %[output:14ffcb46] %[output:41eb35e3] %[output:8353a090] %[output:6870d137] %[output:752b6b05] %[output:206aab1d] %[output:10784a83] %[output:2c9f8cb2] %[output:9c00cb3e] %[output:34886850] %[output:2b6a8ff1] %[output:66bc998e] %[output:3d0ffc8d] %[output:4094026b] %[output:0c5300a0] %[output:0ad2700a] %[output:2e16dc94] %[output:6fc225e1] %[output:7f2e254c] %[output:39169c01] %[output:7bd457a2] %[output:44489666] %[output:4cacffe4] %[output:18bc44d2] %[output:42191f5d] %[output:6a90c6eb] %[output:89daf99d] %[output:27049b3d]
        'TransferBaseWorkspaceVariables', 'on', ... %[output:2092655f] %[output:07ab29b2] %[output:318d5a40] %[output:6e2bd2ba] %[output:6e6c8d87] %[output:14dad609] %[output:30c4dc17] %[output:75eac396] %[output:745c1566] %[output:1e7db223] %[output:1e42f40f] %[output:786ff781] %[output:55263708] %[output:2abd8942] %[output:5e0b9d80] %[output:5a9aa6e5] %[output:9eb5424e] %[output:749330ed] %[output:093610bc] %[output:910c5af8] %[output:3195b599] %[output:22bc523d] %[output:1e797c70] %[output:2199dc7c] %[output:555efdea] %[output:6a2f085f] %[output:1ccdfade] %[output:04c1645f] %[output:63fba9d0] %[output:5bbc8888] %[output:0d622a7e] %[output:9e4273a7] %[output:67369e79] %[output:8a9a5254] %[output:04d42ca7] %[output:37f122c5] %[output:63c5e80d] %[output:62c841bf] %[output:552840ff] %[output:369924b2] %[output:552b87ab] %[output:08b86e46] %[output:9dd2507d] %[output:6522ab10] %[output:1f7255a1] %[output:25f901aa] %[output:3b7e3b61] %[output:716c4285] %[output:1e369efc] %[output:3f549eac] %[output:18333a85] %[output:6399e7e2] %[output:7648cf4b] %[output:19e4e80f] %[output:86b1b219] %[output:24d777fb] %[output:86b2cee4] %[output:14ffcb46] %[output:41eb35e3] %[output:8353a090] %[output:6870d137] %[output:752b6b05] %[output:206aab1d] %[output:10784a83] %[output:2c9f8cb2] %[output:9c00cb3e] %[output:34886850] %[output:2b6a8ff1] %[output:66bc998e] %[output:3d0ffc8d] %[output:4094026b] %[output:0c5300a0] %[output:0ad2700a] %[output:2e16dc94] %[output:6fc225e1] %[output:7f2e254c] %[output:39169c01] %[output:7bd457a2] %[output:44489666] %[output:4cacffe4] %[output:18bc44d2] %[output:42191f5d] %[output:6a90c6eb] %[output:89daf99d] %[output:27049b3d]
        'UseFastRestart', 'on'); %[output:2092655f] %[output:07ab29b2] %[output:318d5a40] %[output:6e2bd2ba] %[output:6e6c8d87] %[output:14dad609] %[output:30c4dc17] %[output:75eac396] %[output:745c1566] %[output:1e7db223] %[output:1e42f40f] %[output:786ff781] %[output:55263708] %[output:2abd8942] %[output:5e0b9d80] %[output:5a9aa6e5] %[output:9eb5424e] %[output:749330ed] %[output:093610bc] %[output:910c5af8] %[output:3195b599] %[output:22bc523d] %[output:1e797c70] %[output:2199dc7c] %[output:555efdea] %[output:6a2f085f] %[output:1ccdfade] %[output:04c1645f] %[output:63fba9d0] %[output:5bbc8888] %[output:0d622a7e] %[output:9e4273a7] %[output:67369e79] %[output:8a9a5254] %[output:04d42ca7] %[output:37f122c5] %[output:63c5e80d] %[output:62c841bf] %[output:552840ff] %[output:369924b2] %[output:552b87ab] %[output:08b86e46] %[output:9dd2507d] %[output:6522ab10] %[output:1f7255a1] %[output:25f901aa] %[output:3b7e3b61] %[output:716c4285] %[output:1e369efc] %[output:3f549eac] %[output:18333a85] %[output:6399e7e2] %[output:7648cf4b] %[output:19e4e80f] %[output:86b1b219] %[output:24d777fb] %[output:86b2cee4] %[output:14ffcb46] %[output:41eb35e3] %[output:8353a090] %[output:6870d137] %[output:752b6b05] %[output:206aab1d] %[output:10784a83] %[output:2c9f8cb2] %[output:9c00cb3e] %[output:34886850] %[output:2b6a8ff1] %[output:66bc998e] %[output:3d0ffc8d] %[output:4094026b] %[output:0c5300a0] %[output:0ad2700a] %[output:2e16dc94] %[output:6fc225e1] %[output:7f2e254c] %[output:39169c01] %[output:7bd457a2] %[output:44489666] %[output:4cacffe4] %[output:18bc44d2] %[output:42191f5d] %[output:6a90c6eb] %[output:89daf99d] %[output:27049b3d]

    % Extract 75 m time from each result
    for k = 1:nThis
        ii = idxBatch(k);

        % Capture simulation error if present
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

    % Clear batch outputs to keep RAM under control
    clear in out
end %[output:group:6b54d772]

elapsedTime = toc(tStart);

%% Assemble results table
results = table( ...
    P_sweep, ...
    I_sweep, ...
    repmat(D, nRuns, 1), ...
    time_75m, ...
    simError, ...
    'VariableNames', {'P', 'I', 'D', 'Time_75m_s', 'ErrorMessage'});

results.Reached75m = ~isnan(results.Time_75m_s);

validResults = results(results.Reached75m, :);
validResults = sortrows(validResults, 'Time_75m_s', 'ascend');

%% Print best 10
fprintf('\n====================================================\n'); %[output:7649d423]
fprintf('Sweep complete in %.2f seconds\n', elapsedTime); %[output:2ec82639]
fprintf('Successful runs reaching 75 m: %d / %d\n', height(validResults), nRuns); %[output:3b098e85]
fprintf('====================================================\n\n'); %[output:6f40a10f]

if isempty(validResults) %[output:group:470325bb]
    fprintf('No simulations reached 75 m.\n');
else
    nPrint = min(topN, height(validResults));
    fprintf('Best %d 0-75 m times:\n\n', nPrint); %[output:7444107e]
    disp(validResults(1:nPrint, {'P','I','D','Time_75m_s'})); %[output:3c58799c]
end %[output:group:470325bb]

%% Save full results
save('TC_PID_sweep_results.mat', 'results', 'validResults', 'elapsedTime');

%% ---------------- Local Functions ----------------
function [tVec, dVec] = getDistanceTrace(simOut)
% Tries a few common Simulink output formats for v_distance

distSig = simOut.v_distance;

if isa(distSig, 'timeseries')
    tVec = distSig.Time(:);
    dVec = distSig.Data(:);
    return;
end

% Structure with time
if isstruct(distSig) && isfield(distSig, 'Time') && isfield(distSig, 'Data')
    tVec = distSig.Time(:);
    dVec = distSig.Data(:);
    return;
end

% Fallback to the format used in your original script
if isprop(simOut, 'v_velocity') || isfield(simOut, 'v_velocity')
    tVec = simOut.v_velocity.Time(:);
    dVec = simOut.v_distance.Data(:);
    return;
end

error('Could not extract time/distance from simOut.');
end

function tCross = crossingTimeLinearInterp(tVec, yVec, targetValue)
% Returns interpolated first crossing time, NaN if never reached

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
