clc; clear;
% 删除(findall(0, '类型', '数字')); % 这将删除 uifigure
format long;

addpath("lib\");
addpath("log_plot\");
addpath("VICON_data\");

load('dynamic_properties.mat');
load('dynamics_functions.mat');

% ============================ Adjustable Parameters ===================
filename = '20250117_SystemID_HardTerrain.csv';

Ts = 0.008; % Sampling time
x_lim = [5, 6];

% =========================================================================

% Extract data from System(Pow)
data = readtable(filename);

x_lim_idx = [find(data.TimeStamp > x_lim(1)*1e3, 1, 'first'), ...
             find(data.TimeStamp < x_lim(2)*1e3, 1, 'last')];

data = data(x_lim_idx(1):x_lim_idx(2), :);

timeStamp = data.TimeStamp * 1e-3;

h_d = data.h_d;

x_d = [data.theta_d';
    zeros(1,length(timeStamp))
    data.v_d'
    data.psi_dot_d'];

x_hat = [data.theta_hat'
    data.theta_dot_hat'
    data.v_hat'
    data.psi_dot_hat'];

z =  [data.acc_x'
    data.acc_y'
    data.acc_z'
    data.gyr_x'
    data.gyr_y'
    data.gyr_z'
    data.theta_dot_RW'
    data.theta_dot_LW'];

u = [data.tau_RW'
    data.tau_LW'];

% =====================================================

% 广告矩阵
Ad = [1.002502658560470,  0.008006672643349,  0, 0;
    0.625925480414487,  1.002502658560470,  0, 0;
   -0.078674798143939, -0.000314568048800,  1, 0;
   -0.013760979018869, -0.000055020977768,  0, 1];

% BD矩阵
Bd = [0.001916871323186, -0.001865138996260;
    0.479417617252981, -0.466479143704869;
   -0.098643170860809,  0.095626546851669;
   -0.493984618992856, -0.461885479931035];


x_hat(1,:) = x_hat(1,:) - x_d(1,:);

As = [NaN, NaN, 0, 0
      NaN, NaN, 0, 0
      NaN, NaN, 1, 0
      NaN, NaN, 0, 1];
Bs = [NaN, NaN; NaN, NaN; NaN, NaN; NaN, NaN];

Cs = eye(4,4);
Ds = zeros(4,2);
Ks = zeros(4,4);

% ms = idss(A,B,C,D, 'Ts', Ts);
ms = idss(As,Bs,Cs,Ds,Ks);
ms.Structure.C.Free = false;
ms.Structure.K.Free = false;


ms.StateName = {'\theta', '\theta^{ dot}', 'v', '\psi^{ dot}'};
ms.InputName = {'\tau_{ RW}', '\tau_{ LW}'};
ms.StateUnit = {'rad', 'rad/s', 'm/s', 'rad/s'};
ms.InputUnit = {'Nm', 'Nm'};
% ms.InputDelay = 1;

% Prepare data for Subspace Identification
Y = x_hat'; % Transpose for compatibility (samples x states)
U = u'; % Transpose for compatibility (samples x inputs)

% Subspace method using n4sid
data_id = iddata(Y, U, Ts); % Create identification dataset

opt = ssestOptions('display','on', 'Focus', 'simulation', 'N4weight', 'ssARX');
m2 = pem(data_id,ms, opt);

figure;
compare(data_id, m2, sys);

x_pred = zeros(size(x_hat));
x_pred(:,1:2) = x_hat(:,1:2);
% x_pred = Ad*x_hat(:,2:end) + Bd*u(:,1:end-1);
% for i = 2:length(timeStamp)-1
%     x_pred(:,i+1) = Ad*x_pred(:,i) + Bd*u(:,i-1);
% end
for i = 2:length(timeStamp)-1
    x_pred(:,i+1) = Ad*x_hat(:,i) + Bd*u(:,i);
end

% ========================================================================
% ==================== Plot Start ========================================
% ========================================================================
screenSize = get(0, 'ScreenSize'); %获取屏幕尺寸

% 创建uifigure（设置为全屏尺寸）
fig = uifigure('Name', " VICON_Filter_Comparision", 'Position', [0, 0, screenSize(3), screenSize(4)*0.97]);
tabGroup = uitabgroup(fig, 'Position', [0, 0, fig.Position(3), fig.Position(4)]);

% ========================== Figure 1 ============================
figure_title = "Estimated States from Log Data";
Line_width = 1.5;

tab1 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab1, 2, 2, "TileSpacing", "compact");
title(layout, figure_title, 'FontSize', 14, 'FontWeight', 'bold')

% ==== Fig.2 - 1/4 ====
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(x_hat(1,:)), 'r', 'DisplayName', '\theta_{ estimated}', 'LineWidth', Line_width);
plot(ax, timeStamp, rad2deg(x_pred(1,:)), 'bo', 'DisplayName', '\theta_{ pred}', 'LineWidth', Line_width);
title(ax, '\theta_{ estimated}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
xlim(ax, x_lim);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 2/4 ====
ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(x_hat(2,:)), 'r', 'DisplayName', '\theta^{ dot}_{ estimated}', 'LineWidth', Line_width);
plot(ax, timeStamp, rad2deg(x_pred(2,:)), 'bo', 'DisplayName', '\theta^{ dot}_{ pred}', 'LineWidth', Line_width);
title(ax, '\theta^{ dot}_{ estimated}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 3/4 ====
ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, x_hat(3,:), 'r', 'DisplayName', 'v_{ estimated}', 'LineWidth', Line_width);
plot(ax, timeStamp, x_pred(3,:), 'bo', 'DisplayName', 'v_{ pred}', 'LineWidth', Line_width);
title(ax, 'v_{ estimated}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, x_lim);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 4/4 ====
ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(x_hat(4,:)), 'r', 'DisplayName', '\psi^{ dot}_{ estimated}', 'LineWidth', Line_width);
plot(ax, timeStamp, rad2deg(x_pred(4,:)), 'bo', 'DisplayName', '\psi^{ dot}_{ pred}', 'LineWidth', Line_width);
title(ax, '\psi^{ dot}_{ estimated}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'deg/s');
xlim(ax, x_lim);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");
