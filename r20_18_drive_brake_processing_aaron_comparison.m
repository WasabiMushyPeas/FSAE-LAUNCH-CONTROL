%% TTC TIRE DATA PROCESSNG: DRIVE | BRAKE - COMPARISON
% TIRE: 43100 18.0x6.0-10 R20 (7 in RIM) 
% ROUND: 9 
% BY: THOMAS PIERCE
% DATE CREATED: 10/7/2025
% DATE MODIFED: 10/7/2025

close all;
clc;
clear;
    
% Load data

% CHANGE FILE LOCATION TO YOURS<<>>>>>
data = load(".\A2356run72.mat");

slip_ratio = data.SR;
fx = data.FX;
fz = abs(data.FZ);
fy = data.FY;

tire_temp_in = data.TSTI;
tire_temp_center = data.TSTC;
tire_temp_out = data.TSTO;
tire_pressure = data.P;


data_length = length(slip_ratio);
count_vector = 1:data_length;

%% Contour Plot
figure(1);
scatter(slip_ratio, fx, 15, fz, 'filled');  
colormap("turbo");
colorbar;
xlabel('Slip Ratio');
ylabel('Fx (lbf)');
title('Fx vs Slip Ratio (Colored by Fz)');
grid on;


%% plot_temperature

figure(2)

subplot(6,1,1)
plot(count_vector, tire_temp_out);
hold on
plot(count_vector, tire_temp_center);
plot(count_vector, tire_temp_in);
legend('outside temp','center','inside');
hold off
title('temp')

subplot(6,1,2)
plot(count_vector, tire_pressure);
title('pressure')

subplot(6,1,3)
plot(count_vector,fz);
title('normal load')


subplot(6,1,4)
plot(count_vector,fx);
title('long. force')

subplot(6,1,5);
plot(count_vector, slip_ratio);
title('slipratio')

subplot(6,1,6);
plot(count_vector, fy);
title('fy');


%% Pacjka Calibration 
% 8 PSI shift
% slip_ratio_shift = 0.03;
% fx_shift = -20;

% 12 PSI shift
 slip_ratio_shift = 0.025;
 fx_shift = 5;

% ______________________________________________________

% 8 PSI, SWITCH COMMENT TO TURN ON AND OFF
% start_idx = 2000;
% end_idx   = 9000;


% 12 PSI
 start_idx = 3500;
 end_idx   = 9000;

%_________________________________________________

sr_sub = slip_ratio(start_idx:end_idx) + slip_ratio_shift;
fx_sub = fx(start_idx:end_idx) + fx_shift;
fz_sub = fz(start_idx:end_idx);

figure(3);
scatter(sr_sub, fx_sub, 15, fz_sub, 'filled');
hold on;
colormap("turbo");
colorbar;
xlabel('Slip Ratio');
xlim([-0.25 0.25]);
ylabel('Fx (lbf)');
title('Fx vs Slip Ratio, contour fz (12 PSI)');
grid on;
hold off; % separate curve fit or not

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Coefficents for cal. 

% B = cornering stiffness factor (more + is more steep)
% C = curve factor or mu decay (more - is more drop off)
% D1 = D = peak factor (more - is higher peak)
% D2 = normal load sensitivity (more + is more NLS)
% No E coeffficient but C takes care of this

% THOMAS COEFFICENTS (12 PSI FIT, 18.0x6.0-10 R20 7 in RIM)
 B = 9.6;
 C = -1.75;
 D1 = -2.40;
 D2 = 0.50;

% AARON COEFFICENTS (ASSUME DESIRED 12 PSI FIT, 16x7.5-10 R20 7 in RIM)
 B_a  = 11.357;
 C_a  = -1.408;
 D1_a = -2.615;
 D2_a = -1.402;

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

figure(4);
xlabel('Slip Ratio');
xlim([-0.25 0.25]);
ylabel('Fx (lbf)');
title('Fx vs Slip Ratio - 12 PSI R20 16x7.5-10 7 in Rim Pacejka4 Comparison Pre-Regression');
grid on;
hold on;

sr_model = linspace(min(sr_sub), max(sr_sub), 200);

fz_vector = [250, 200, 150, 100, 50];

cmap = turbo(256);
fz_min = min(fz_sub);
fz_max = max(fz_sub);
mapColor = @(Fz) cmap( round( (Fz - fz_min)/(fz_max - fz_min) * (size(cmap,1)-1) + 1 ), : );

for i = 1:length(fz_vector)
    fz_model = fz_vector(i);
    thisColor = mapColor(fz_model);

    % THOMAS PACEJKA FIT
    Fx_model = (D1 + D2 / 1000 * fz_model) * fz_model .* sin(C * atan(B * sr_model));
   
    plot(sr_model, Fx_model, '-', ...
        'Color', thisColor, 'LineWidth', 2.0, ...
        'DisplayName', sprintf('Current 18x6 Fit Fz = %d lbf', fz_model));

    % AARON PACEJKA FIT
    Fx_model_a = (D1_a + D2_a / 1000 * fz_model) * fz_model .* sin(C_a * atan(B_a * sr_model));
   
    plot(sr_model, Fx_model_a, '--', ...
        'Color', thisColor, 'LineWidth', 2.0, ...
        'DisplayName', sprintf('Old 16x7.5 Fit Fz = %d lbf', fz_model));

end

colorbar;
colormap(turbo);
clim([fz_min fz_max]);
legend('Location', 'northwest');

% Add RMS corelation , or add RSM, calculate corelation % 

%% Extra Plots

% figure(4);
% plot(count_vector,fy);
% xlabel('time');
% xlim([start_idx end_idx]);
% ylabel('Fy (lbf)');
% title('Fy bigger');
% grid on;
