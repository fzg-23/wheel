clc; clear;
% 删除(findall(0, '类型', '数字')); % 这将删除 uifigure
format long;

% 读取 CSV 文件
filename = '20250210_maxyawrate.csv'; % CSV 文件名
time_cutter = [0,1000];
angle_unit = 1;
% 0 : degree, 1: rad

data = readtable(filename);

% ============ Extract data ===========================
timeStamp = (data.TimeStamp-min(data.TimeStamp)) / 1000; % (s)
acc_x_imu = data.acc_x; % (m/s^2)
acc_y_imu = data.acc_y; % (m/s^2)
acc_z_imu = data.acc_z; % (m/s^2)
gyr_x_imu = data.gyr_x; % (rad/s)
gyr_y_imu = data.gyr_y; % (rad/s)
gyr_z_imu = data.gyr_z; % (rad/s)

cal_time = data.cal_time; % (ms)

current_LW = data.current_LW; % (A)
current_RW = data.current_RW; % (A)

h_d = data.h_d;             % (m)
theta_d = data.theta_d;     % (rad)
v_d = data.v_d;             % (m/s)
psi_dot_d = data.psi_dot_d; % (rad/s)

psi_dot_hat = data.psi_dot_hat;
theta_hat = data.theta_hat;
theta_dot_hat = data.theta_dot_hat;
v_hat = data.v_hat;

theta_dot_LW = data.theta_dot_LW;
theta_dot_RW = data.theta_dot_RW;

tau_LW = data.tau_LW;
tau_RW = data.tau_RW;

% 计算采样时间（假设TimeStamp以毫秒为单位）
Ts = round(mean(diff(timeStamp*1000)))/1000;
fprintf("Sampling time : %d (ms) \n", Ts*1000);

% 使用速度计算加速度
theta_ddot_LW = gradient(theta_dot_LW)/Ts;
theta_ddot_RW = gradient(theta_dot_RW)/Ts;

% 计算总加速度
total_acc = sqrt(acc_x_imu.^2+acc_y_imu.^2+acc_z_imu.^2);

% 时间切割器最大设置

time_cutter(2) = min(time_cutter(2), max(timeStamp));
cutted_idx = round(time_cutter/Ts);
cutted_idx(1) = max(cutted_idx(1), 1);

% ==================== Plot Start ========================================
screenSize = get(0, 'ScreenSize'); %获取屏幕尺寸

% 创建uifigure（设置为全屏尺寸）
fig = uifigure('Name', filename + "/ data_plotter", 'Position', [0, 0, screenSize(3), screenSize(4)*0.97]);
tabGroup = uitabgroup(fig, 'Position', [0, 0, fig.Position(3), fig.Position(4)]);

% =================================== Figure 1 ============================
figure_title = "Estimated states and inputs";
Line_width = 1.5;

tab1 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab1, 2, 3, "TileSpacing", "compact");
title(layout,figure_title, 'FontSize', 14, 'FontWeight', 'bold')

ax = nexttile(layout, 1);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(theta_hat), 'r', 'DisplayName', '\theta_{hat}', 'LineWidth', Line_width);
    plot(ax, timeStamp, rad2deg(theta_d), 'k', 'DisplayName', '\theta_{d}');
    ylabel(ax, 'Angle (deg)');
elseif angle_unit == 1
    plot(ax, timeStamp, theta_hat, 'r', 'DisplayName', '\theta_{hat}', 'LineWidth', Line_width);
    plot(ax, timeStamp, theta_d, 'k', 'DisplayName', '\theta_{d}');
    ylabel(ax, 'Angle (rad)');
end
title(ax, '\theta_{hat}');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 2);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(theta_dot_hat), 'r', 'DisplayName', '\theta^{dot}_{hat}', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (deg/s)');
elseif angle_unit == 1
    plot(ax, timeStamp, theta_dot_hat, 'r', 'DisplayName', '\theta^{dot}_{hat}', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (rad/s)');
end
plot(ax, timeStamp, zeros(length(timeStamp),1), 'k', 'DisplayName', '\theta^{dot}_{d}');
title(ax, '\theta^{dot}_{hat}');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp, v_hat, 'r', 'DisplayName', 'v_{hat}', 'LineWidth', Line_width);
plot(ax, timeStamp, v_d, 'k', 'DisplayName', 'v_{d}');
title(ax, 'v_{hat}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 5);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(psi_dot_hat), 'r', 'DisplayName', '\psi^{dot}_{hat}', 'LineWidth', Line_width);
    plot(ax, timeStamp, rad2deg(psi_dot_d), 'k', 'DisplayName', '\psi^{dot}_{d}');
    ylabel(ax, 'Angular rate (deg/s)');
elseif angle_unit == 1
    plot(ax, timeStamp, psi_dot_hat, 'r', 'DisplayName', '\psi^{dot}_{hat}', 'LineWidth', Line_width);
    plot(ax, timeStamp, psi_dot_d, 'k', 'DisplayName', '\psi^{dot}_{d}');
    ylabel(ax, 'Angular rate (rad/s)');
end
title(ax, '\psi^{dot}_{hat}');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, tau_RW, 'g', 'DisplayName', '\tau_{RW}', 'LineWidth', Line_width);
plot(ax, timeStamp, current_RW * 0.000857902 / (3.3/2048), 'r', 'DisplayName', 'I_{RW}');
title(ax, 'Right Wheel Input');
xlabel(ax, 'TimeStamp');
ylabel(ax, 'Nm');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 6);
hold(ax, 'on');
plot(ax, timeStamp, tau_LW, 'g', 'DisplayName', '\tau_{LW}', 'LineWidth', Line_width);
plot(ax, timeStamp, current_LW * 0.001043224 / (3.3/2048), 'r', 'DisplayName', 'I_{LW}');
title(ax, 'Left Wheel Input');
xlabel(ax, 'TimeStamp');
ylabel(ax, 'Nm');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% =================================== Figure 2 ============================
figure_title = "Estimated states specific";
Line_width = 1.5;

tab2 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab2, 2, 2, "TileSpacing", "compact");
title(layout,figure_title, 'FontSize', 14, 'FontWeight', 'bold')

ax = nexttile(layout, 1);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(theta_hat), 'r', 'DisplayName', '\theta_{hat}', 'LineWidth', Line_width);
    ylabel(ax, 'Angle (deg)');
elseif angle_unit == 1
    plot(ax, timeStamp, theta_hat, 'r', 'DisplayName', '\theta_{hat}', 'LineWidth', Line_width);
    ylabel(ax, 'Angle (rad)');
end
title(ax, '\theta_{hat}');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 2);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(theta_dot_hat), 'r', 'DisplayName', '\theta^{dot}_{hat}', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (deg/s)');
elseif angle_unit == 1
    plot(ax, timeStamp, theta_dot_hat, 'r', 'DisplayName', '\theta^{dot}_{hat}', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (rad/s)');
end
title(ax, '\theta^{dot}_{hat}');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, v_hat, 'r', 'DisplayName', 'v_{hat}', 'LineWidth', Line_width);
title(ax, 'v_{hat}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 4);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(psi_dot_hat), 'r', 'DisplayName', '\psi^{dot}_{hat}', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (deg/s)');
elseif angle_unit == 1
    plot(ax, timeStamp, psi_dot_hat, 'r', 'DisplayName', '\psi^{dot}_{hat}', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (rad/s)');
end
title(ax, '\psi^{dot}_{hat}');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% =================================== Figure 3 ============================
figure_title = "Measurements All";
Line_width = 0.5;

tab3 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab3, 2, 3, "TileSpacing", "compact");
title(layout,figure_title, 'FontSize', 14, 'FontWeight', 'bold')

ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp, acc_x_imu, 'r', 'DisplayName', 'a_{x}', 'LineWidth', Line_width);
plot(ax, timeStamp, acc_y_imu, 'g', 'DisplayName', 'a_{y}', 'LineWidth', Line_width);
plot(ax, timeStamp, acc_z_imu, 'b', 'DisplayName', 'a_{z}', 'LineWidth', Line_width);
plot(ax, timeStamp, total_acc, 'k', 'DisplayName', 'a_{total}', 'LineWidth', Line_width);
title(ax, 'IMU Accelerometer Measurements');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Acceleration (m/s^2)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 4);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(gyr_x_imu), 'r', 'DisplayName', 'w_x', 'LineWidth', Line_width);
    plot(ax, timeStamp, rad2deg(gyr_y_imu), 'g', 'DisplayName', 'w_y', 'LineWidth', Line_width);
    plot(ax, timeStamp, rad2deg(gyr_z_imu), 'b', 'DisplayName', 'w_z', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (deg/s)');
elseif angle_unit == 1
    plot(ax, timeStamp, gyr_x_imu, 'r', 'DisplayName', 'w_x', 'LineWidth', Line_width);
    plot(ax, timeStamp, gyr_y_imu, 'g', 'DisplayName', 'w_y', 'LineWidth', Line_width);
    plot(ax, timeStamp, gyr_z_imu, 'b', 'DisplayName', 'w_z', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (rad/s)');
end
title(ax, 'IMU Gyroscope Measurements');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 2);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(theta_dot_RW), 'r', 'DisplayName', '\theta^{dot}_{RW}', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (deg/s)');
elseif angle_unit == 1
    plot(ax, timeStamp, theta_dot_RW, 'r', 'DisplayName', '\theta^{dot}_{RW}', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (rad/s)');
end
title(ax, 'Right Wheel speed');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 5);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(theta_dot_LW), 'r', 'DisplayName', '\theta^{dot}_{LW}', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (deg/s)');
elseif angle_unit == 1
    plot(ax, timeStamp, theta_dot_LW, 'r', 'DisplayName', '\theta^{dot}_{LW}', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (rad/s)');
end
title(ax, 'Left Wheel speed');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, current_RW, 'b', 'DisplayName', 'I_{RW}', 'LineWidth', Line_width);
title(ax, 'Right Wheel Current');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Current (A)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 6);
hold(ax, 'on');
plot(ax, timeStamp, current_LW, 'g', 'DisplayName', 'I_{LW}', 'LineWidth', Line_width);
title(ax, 'Left Wheel Current');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Current (A)');
xlim(ax, time_cutter);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% =================================== Figure 4 ============================
figure_title = "IMU Specific";
Line_width = 1.5;

tab4 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab4, 2, 3, "TileSpacing", "compact");
title(layout,figure_title, 'FontSize', 14, 'FontWeight', 'bold')

ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp, acc_x_imu, 'r', 'DisplayName', 'a_{x}', 'LineWidth', Line_width);
title(ax, 'a_{IMU, x}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Acceleration (m/s^2)');
xlim(ax, time_cutter);
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp, acc_y_imu, 'g', 'DisplayName', 'a_{y}', 'LineWidth', Line_width);
title(ax, 'a_{IMU, y}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Acceleration (m/s^2)');
xlim(ax, time_cutter);
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, acc_z_imu, 'b', 'DisplayName', 'a_{z}', 'LineWidth', Line_width);
title(ax, 'a_{IMU, z}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Acceleration (m/s^2)');
xlim(ax, time_cutter);
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 4);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(gyr_x_imu), 'r', 'DisplayName', 'w_x', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (deg/s)');
elseif angle_unit == 1
    plot(ax, timeStamp, gyr_x_imu, 'r', 'DisplayName', 'w_x', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (rad/s)');
end
title(ax, 'w_{IMU, x}');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 5);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(gyr_y_imu), 'g', 'DisplayName', 'w_y', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (deg/s)');
elseif angle_unit == 1
    plot(ax, timeStamp, gyr_y_imu, 'g', 'DisplayName', 'w_y', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (rad/s)');
end
title(ax, 'w_{IMU, y}');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
grid(ax, "on");
grid(ax, "minor");

ax = nexttile(layout, 6);
hold(ax, 'on');
if angle_unit == 0
    plot(ax, timeStamp, rad2deg(gyr_z_imu), 'b', 'DisplayName', 'w_z', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (deg/s)');
elseif angle_unit == 1
    plot(ax, timeStamp, gyr_z_imu, 'b', 'DisplayName', 'w_z', 'LineWidth', Line_width);
    ylabel(ax, 'Angular rate (rad/s)');
end
title(ax, 'w_{IMU, z}');
xlabel(ax, 'Time (s)');
xlim(ax, time_cutter);
grid(ax, "on");
grid(ax, "minor");

% =================================== Figure 5 ============================
figure_title = "Frequency Analysis of IMU Data";
Line_width = 1.5;
font_size = 12;

tab5 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab5, 2, 3, "TileSpacing", "compact");
title(layout,figure_title, 'FontSize', 14, 'FontWeight', 'bold');

titles = {"a_{ x}", "a_{ y}", "a_{ z}", "w_{ x}", "w_{ y}", "w_{ z}"};
y_labels = {"Magnitude (m/s^2)", "Magnitude (m/s^2)", "Magnitude (m/s^2)", ...
    "Magnitude (deg/s)", "Magnitude (deg/s)", "Magnitude (deg/s)"};

data = {acc_x_imu, acc_y_imu, acc_z_imu, ...
    gyr_x_imu, gyr_y_imu, gyr_z_imu};
for i = 1:6
    ax = nexttile(layout, i);
    hold(ax, 'on');

    [f, P] = get_fft(data{i}(cutted_idx(1):cutted_idx(2)), Ts);
    if (i <= 3)
        f = f(4:end);
        P = P(4:end);
    end
    plot(ax, f, P, 'LineWidth', Line_width, 'Color', [0, 0.447, 0.741]); % Use MATLAB default blue color

    % Highlight peak frequency
    [~, peak_idx] = max(P);
    peak_freq = f(peak_idx);
    peak_mag = P(peak_idx);
    plot(ax, peak_freq, peak_mag, 'ro', 'MarkerSize', 8, 'LineWidth', Line_width);
    text(ax, peak_freq, peak_mag, sprintf('  %.1f Hz', peak_freq), ...
        'FontSize', font_size, 'Color', 'red', 'VerticalAlignment', 'bottom');

    xlabel(ax, "Frequency (Hz)", 'FontSize', font_size);
    ylabel(ax, y_labels{i}, 'FontSize', font_size);
    title(ax, titles{i}, 'FontSize', font_size + 2, 'FontWeight', 'bold');
    xlim(ax, [0, max(f)]); % Optional: Limit frequency range to useful values
end


% =================================== Figure 6 ============================
figure_title = "Frequency Analysis of Estimated States and Inputs";
Line_width = 1.5;
font_size = 12;

tab6 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab6, 2, 3, "TileSpacing", "compact");
title(layout,figure_title, 'FontSize', 14, 'FontWeight', 'bold');

titles = {"\theta_{hat}", "\theta^{dot}_{hat}", "\tau_{RW}", "v_{hat}", "\psi^{dot}_{hat}", "\tau_{LW}"};
y_labels = {"Magnitude (deg)", "Magnitude (deg/s)", "Magnitude (Nm)", ...
    "Magnitude (m/s)", "Magnitude (deg/s)", "Magnitude (Nm)"};

data = {rad2deg(theta_hat), rad2deg(theta_dot_hat), tau_RW, ...
    v_hat, rad2deg(psi_dot_hat), tau_LW};

for i = 1:6
    ax = nexttile(layout, i);
    hold(ax, 'on');

    [f, P] = get_fft(data{i}(cutted_idx(1):cutted_idx(2)), Ts);
    % if (i <= 3)
    %     f = f(4:end);
    %     P = P(4:end);
    % end
    plot(ax, f, P, 'LineWidth', Line_width, 'Color', [0, 0.447, 0.741]); % Use MATLAB default blue color

    % Highlight peak frequency
    [~, peak_idx] = max(P);
    peak_freq = f(peak_idx);
    peak_mag = P(peak_idx);
    plot(ax, peak_freq, peak_mag, 'ro', 'MarkerSize', 8, 'LineWidth', Line_width);
    text(ax, peak_freq, peak_mag, sprintf('  %.1f Hz', peak_freq), ...
        'FontSize', font_size, 'Color', 'red', 'VerticalAlignment', 'bottom');

    xlabel(ax, "Frequency (Hz)", 'FontSize', font_size);
    ylabel(ax, y_labels{i}, 'FontSize', font_size);
    title(ax, titles{i}, 'FontSize', font_size + 2, 'FontWeight', 'bold');
    xlim(ax, [0, max(f)]); % Optional: Limit frequency range to useful values
end

% =================================== Figure 7 ============================
figure_title = "etc.";
Line_width = 1.5;

tab7 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab7, 2, 3, "TileSpacing", "compact");
title(layout,figure_title, 'FontSize', 14, 'FontWeight', 'bold')

ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp, cal_time, 'r', 'DisplayName', 'cal_time', 'LineWidth', Line_width);
title(ax, 'Calculation time');
xlabel(ax, 'Time (s)');
ylabel(ax, 'calculation time (ms)');
xlim(ax, time_cutter);
grid(ax, "on");
grid(ax, "minor");

