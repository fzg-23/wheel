clc; clear;
% 删除(findall(0, '类型', '数字')); % 这将删除 uifigure
format long;

addpath("lib\");
addpath("log_plot\");
addpath("VICON_data\");

load('dynamic_properties.mat');
load('dynamics_functions.mat');

% ============================ Adjustable Parameters ===================
filename_VICON = '20250209_VICON_Test_EKF_Hard.csv';
filename_Pow = '20250209_EKFTunning_hardterrain.csv';

% === VICON ===
VICON_FPS = 250; % Hz
% h_偏移量=0.2； % height (m) - 相对于 VICON 设置中指定的 IMU 的偏移
h_offset = -0.104535; % 高度 (m) - VICON 设置中指定的 IMU 偏移

% === System (Pow) ===
Ts = 0.008; % Sampling time

% === EKF ===
Case_num = 0;
P_init = eye(4)*1; % 初始估计误差协方差矩阵
R_cov = []; % Sensor noise Covariance Matrix
Q_cov = []; % Process noise Covariance Matrix

if Case_num == 0
    % Consider all measurements
    R_cov = diag([1e-1, 1, 1e-1,...
        1e-3, 4.12642e-6, 1,...
        0, 0]);
    Q_cov = diag([0, 1, 1, 1]);

elseif Case_num == 1
    % Consider only dominant measurements / without Coriolis
    R_cov = diag([7e-2, 7e-2, 4.12642e-6, 0, 0]);
    Q_cov = diag([0, 1e-6, 1e-2, 1e-2]);

elseif Case_num == 2
    % Consider only dominant measurements / with Coriolis
    R_cov = diag([7e-2, 7e-2, 4.12642e-6, 0, 0]);
    Q_cov = diag([0, 1e-6, 1e-2, 1e-2]);

elseif Case_num == 3
    % Consider only dominant measurements / use Pseudo measurement
    R_cov = diag([1e-2, 1e-4, 0, 0]);
    Q_cov = diag([0, 1, 1, 1]);
end


%
x_lim_VICON = [1, 30];
x_lim_Pow = [1, 30];

% =========================================================================

% Extract true states from VICON
rawData = readcell(filename_VICON);
[timeStamp_VICON, x_trues, hs] = extract_truestates_from_VICON(rawData, VICON_FPS, h_offset);

x_lim_VICON(1) = max(min(timeStamp_VICON),x_lim_VICON(1));
x_lim_VICON(2) = min(max(timeStamp_VICON),x_lim_VICON(2));

% Extract data from System(Pow)
data_Pow = readtable(filename_Pow);
timeStamp_Pow = data_Pow.TimeStamp * 1e-3;

x_lim_Pow(1) = max(min(timeStamp_Pow),x_lim_Pow(1));
x_lim_Pow(2) = min(max(timeStamp_Pow),x_lim_Pow(2));

h_d_log = data_Pow.h_d;

x_d_log = [data_Pow.theta_d';
    zeros(1,length(timeStamp_Pow))
    data_Pow.v_d'
    data_Pow.psi_dot_d'];

x_hat_log = [data_Pow.theta_hat'
    data_Pow.theta_dot_hat'
    data_Pow.v_hat'
    data_Pow.psi_dot_hat'];

z_m =  [data_Pow.acc_x'
    data_Pow.acc_y'
    data_Pow.acc_z'
    data_Pow.gyr_x'
    data_Pow.gyr_y'
    data_Pow.gyr_z'
    data_Pow.theta_dot_RW'
    data_Pow.theta_dot_LW'];

% Apply IMU bias
% z_m(1,:) = z_m(1,:) + 0.5 * 7/13;
% z_m(3,:) = z_m(3,:) - 1.002969875*7/11;

z = [];
if Case_num == 0
    z = z_m;
elseif Case_num == 1
    z = z_m([1,3,5,7,8],:);
elseif Case_num == 2
    z = z_m([1,3,5,7,8],:);
elseif Case_num == 3
    z = [atan2(-z_m(1,:), z_m(3,:))
        z_m([5,7,8], :)];
end

u = [data_Pow.tau_RW'
    data_Pow.tau_LW'];

% Prepare Estimator
x_hat_init = [0; 0; 0; 0];
x_hat = x_hat_init;

model = Pol(properties, dynamic_functions);
obs_model = Observe_model(properties);
Estimator = EKF_c(x_hat_init, P_init, Q_cov, R_cov, model, Ts);

x_hat_cal = zeros(4,length(timeStamp_Pow));
x_pred = x_hat_cal;
P_pred_log = x_hat_cal;
P_log = x_hat_cal;
for i = 1:length(timeStamp_Pow)
    if i <= 2
        u_prevprev = [0;0];
    else
        u_prevprev = u(:,i-2);
        % u_prevprev = [0;0];
    end
    h = h_d_log(i);

    Estimator.predict(u_prevprev, h);
    x_pred(:,i) = Estimator.x;
    P_pred_log(:,i) = sqrt(abs([Estimator.P(1,1); Estimator.P(2,2); Estimator.P(3,3); Estimator.P(4,4)]));

    if Case_num == 0
        Estimator.update(z(:,i));
    else
        Estimator.update_test(z(:,i), obs_model, Case_num);
    end

    x_hat_cal(:,i) = Estimator.x;
    P_log(:,i) = sqrt(abs([Estimator.P(1,1); Estimator.P(2,2); Estimator.P(3,3); Estimator.P(4,4)]));

    if Case_num == 0
        h_obs(:,i) = Estimator.h_obs;
    elseif Case_num == 1
        h_obs(:,i) = [Estimator.h_obs(1); 0; Estimator.h_obs(2); 0; Estimator.h_obs(3); 0; ...
            Estimator.h_obs(4); Estimator.h_obs(5)];
        h_obs(2,i) = 2*h*cos(x_pred(1,i))*x_pred(4,i)^2+x_pred(3,i)*x_pred(4,i);
    elseif Case_num == 2
        h_obs(:,i) = [Estimator.h_obs(1); 0; Estimator.h_obs(2); 0; Estimator.h_obs(3); 0; ...
            Estimator.h_obs(4); Estimator.h_obs(5)];
        h_obs(2,i) = 2*h*cos(x_pred(1,i))*x_pred(4,i)^2+x_pred(3,i)*x_pred(4,i);
    elseif Case_num == 3
        h_obs(:,i) = [-model.g*sin(Estimator.h_obs(1)); 0; model.g*cos(Estimator.h_obs(1)); 0; Estimator.h_obs(2); 0; ...
            Estimator.h_obs(3); Estimator.h_obs(4)];
    end
end

% Synchronization
% hs和h_d_log之间的同步操作
hs_sync_idx = find(hs < 0.195, 1, 'first');
h_d_log_sync_idx = find(h_d_log < 0.195, 1, 'first');

hs_sync_time = timeStamp_VICON(hs_sync_idx);
h_d_log_sync_time = timeStamp_Pow(h_d_log_sync_idx);

timeStamp_Pow = timeStamp_Pow - (h_d_log_sync_time - hs_sync_time) - 0.064 -0.055;


% ========================================================================
% ==================== Plot Start ========================================
% ========================================================================
screenSize = get(0, 'ScreenSize'); %获取屏幕尺寸

% 创建uifigure（设置为全屏尺寸）
fig = uifigure('Name', " VICON_Filter_Comparision", 'Position', [0, 0, screenSize(3), screenSize(4)*0.97]);
tabGroup = uitabgroup(fig, 'Position', [0, 0, fig.Position(3), fig.Position(4)]);

% ========================== Figure 1 ============================
figure_title = "VICON_true states";
Line_width = 1.5;

tab1 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab1, 2, 2, "TileSpacing", "compact");
title(layout,figure_title, 'FontSize', 14, 'FontWeight', 'bold')

% ==== Fig.1 - 1/4 ====
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp_VICON, rad2deg(x_trues(1,:)), 'r', 'DisplayName', '\theta_{ true}', 'LineWidth', Line_width);
title(ax, '\theta_{ true}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
xlim(ax, x_lim_VICON);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.1 - 2/4 ====
ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp_VICON, rad2deg(x_trues(2,:)), 'r', 'DisplayName', '\theta^{ dot}_{ true}', 'LineWidth', Line_width); 
title(ax, '\theta^{ dot}_{ true}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim_VICON);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.1 - 3/4 ====
ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp_VICON, x_trues(3,:), 'r', 'DisplayName', 'v_{ true}', 'LineWidth', Line_width); 
title(ax, 'v_{ true}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, x_lim_VICON);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.1 - 4/4 ====
ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp_VICON, rad2deg(x_trues(4,:)), 'r', 'DisplayName', '\psi^{ dot}_{ true}', 'LineWidth', Line_width); 
title(ax, '\psi^{ dot}_{ true}');
xlabel(ax, 'TimeStamp');
ylabel(ax, 'deg/s');
xlim(ax, x_lim_VICON);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");

% ========================== Figure 2 ============================
figure_title = "Estimated States from Log Data";
Line_width = 1.5;

tab2 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab2, 2, 2, "TileSpacing", "compact");
title(layout, figure_title, 'FontSize', 14, 'FontWeight', 'bold')

% ==== Fig.2 - 1/4 ====
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp_Pow, rad2deg(x_hat_log(1,:)), 'r', 'DisplayName', '\theta_{ estimated}', 'LineWidth', Line_width);
title(ax, '\theta_{ estimated}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
xlim(ax, x_lim_Pow);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 2/4 ====
ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp_Pow, rad2deg(x_hat_log(2,:)), 'r', 'DisplayName', '\theta^{ dot}_{ estimated}', 'LineWidth', Line_width);
title(ax, '\theta^{ dot}_{ estimated}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim_Pow);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 3/4 ====
ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp_Pow, x_hat_log(3,:), 'r', 'DisplayName', 'v_{ estimated}', 'LineWidth', Line_width);
title(ax, 'v_{ estimated}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, x_lim_Pow);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 4/4 ====
ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp_Pow, rad2deg(x_hat_log(4,:)), 'r', 'DisplayName', '\psi^{ dot}_{ estimated}', 'LineWidth', Line_width);
title(ax, '\psi^{ dot}_{ estimated}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'deg/s');
xlim(ax, x_lim_Pow);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");


% ========================== Figure 3 ============================
figure_title = "Height Comparison";
Line_width = 1.5;

tab3 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab3, 2, 1, "TileSpacing", "compact");
title(layout, figure_title, 'FontSize', 14, 'FontWeight', 'bold')

% ==== Fig.3 - 1/2: Estimated Height (VICON Time) ====
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp_VICON, hs, 'r', 'DisplayName', 'Estimated Height (h_s)', 'LineWidth', Line_width);
plot(ax, timeStamp_Pow, h_d_log, 'b', 'DisplayName', 'Measured Height (h_d)', 'LineWidth', Line_width);
title(ax, 'Height Comparision');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Height (m)');
xlim(ax, x_lim_VICON);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ========================== Figure 4 ============================
figure_title = "Comparision between VICON and log";
Line_width = 1.5;

tab4 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab4, 2, 2, "TileSpacing", "compact");
title(layout, figure_title, 'FontSize', 14, 'FontWeight', 'bold')

% ==== Fig.2 - 1/4 ====
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp_VICON, rad2deg(x_trues(1,:)), 'r', 'DisplayName', '\theta_{ true}', 'LineWidth', Line_width);
plot(ax, timeStamp_Pow, rad2deg(x_hat_log(1,:)), 'b', 'DisplayName', '\theta_{ estimated}', 'LineWidth', Line_width);
title(ax, '\theta');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
xlim(ax, x_lim_VICON);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 2/4 ====
ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp_VICON, rad2deg(x_trues(2,:)), 'r', 'DisplayName', '\theta^{ dot}_{ true}', 'LineWidth', Line_width); 
plot(ax, timeStamp_Pow, rad2deg(x_hat_log(2,:)), 'b', 'DisplayName', '\theta^{ dot}_{ estimated}', 'LineWidth', Line_width);
title(ax, '\theta^{ dot}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim_Pow);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 3/4 ====
ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp_VICON, x_trues(3,:), 'r', 'DisplayName', 'v_{ true}', 'LineWidth', Line_width); 
plot(ax, timeStamp_Pow, x_hat_log(3,:), 'b', 'DisplayName', 'v_{ estimated}', 'LineWidth', Line_width);
title(ax, 'v');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, x_lim_Pow);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 4/4 ====
ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp_VICON, rad2deg(x_trues(4,:)), 'r', 'DisplayName', '\psi^{ dot}_{ true}', 'LineWidth', Line_width); 
plot(ax, timeStamp_Pow, rad2deg(x_hat_log(4,:)), 'b', 'DisplayName', '\psi^{ dot}_{ estimated}', 'LineWidth', Line_width);
title(ax, '\psi^{ dot}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'deg/s');
xlim(ax, x_lim_Pow);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ========================== Figure 5 ============================
figure_title = "Comparision between VICON and calculated EKF";
Line_width = 1.5;

tab5 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab5, 2, 2, "TileSpacing", "compact");
title(layout, figure_title, 'FontSize', 14, 'FontWeight', 'bold')

% ==== Fig.5 - 1/4 ====
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp_VICON, rad2deg(x_trues(1,:)), 'r', 'DisplayName', '\theta_{ true}', 'LineWidth', Line_width);
plot(ax, timeStamp_Pow, rad2deg(x_hat_cal(1,:)), 'b', 'DisplayName', '\theta_{ estimated}', 'LineWidth', Line_width);
title(ax, '\theta');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
xlim(ax, x_lim_VICON);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.5 - 2/4 ====
ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp_VICON, rad2deg(x_trues(2,:)), 'r', 'DisplayName', '\theta^{ dot}_{ true}', 'LineWidth', Line_width); 
plot(ax, timeStamp_Pow, rad2deg(x_hat_cal(2,:)), 'b', 'DisplayName', '\theta^{ dot}_{ estimated}', 'LineWidth', Line_width);
title(ax, '\theta^{ dot}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim_Pow);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.5 - 3/4 ====
ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp_VICON, x_trues(3,:), 'r', 'DisplayName', 'v_{ true}', 'LineWidth', Line_width); 
plot(ax, timeStamp_Pow, x_hat_cal(3,:), 'b', 'DisplayName', 'v_{ estimated}', 'LineWidth', Line_width);
title(ax, 'v');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, x_lim_Pow);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.5 - 4/4 ====
ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp_VICON, rad2deg(x_trues(4,:)), 'r', 'DisplayName', '\psi^{ dot}_{ true}', 'LineWidth', Line_width); 
plot(ax, timeStamp_Pow, rad2deg(x_hat_cal(4,:)), 'b', 'DisplayName', '\psi^{ dot}_{ estimated}', 'LineWidth', Line_width);
title(ax, '\psi^{ dot}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'deg/s');
xlim(ax, x_lim_Pow);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");
