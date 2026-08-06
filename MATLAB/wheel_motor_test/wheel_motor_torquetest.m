clc; clear;
format long;

% 读取 CSV 文件
filename = '20241229_logdata_wheelmotor_torquetest_3ms_Kp200_Ki255.csv'; % CSV 文件名
% 文件名='20241227_logdata_wheelmotor_torquetest_3ms.csv'; % CSV 文件名

data = readtable(filename);

% time_cut_idx = 5588;
% time_cut_idx = 2700;
time_cut_idx = 350;

% extract data 
timeStamp = data.TimeStamp(1:time_cut_idx);
acc_x_imu = data.acc_x(1:time_cut_idx); % m/s^2
acc_y_imu = data.acc_y(1:time_cut_idx); % m/s^2
acc_z_imu = data.acc_z(1:time_cut_idx); % m/s^2
gyr_x_imu = data.gyr_x(1:time_cut_idx); % rad/s
gyr_y_imu = data.gyr_y(1:time_cut_idx); % rad/s
gyr_z_imu = data.gyr_z(1:time_cut_idx); % rad/s

iq_LW_output = data.iq_LW(1:time_cut_idx); % (LSD)
iq_RW_output = data.iq_RW(1:time_cut_idx); % (LSD)

iq_LW_input = data.tau_LW(1:time_cut_idx);  % (LSD)
iq_RW_input = data.tau_RW(1:time_cut_idx);  % (LSD)

theta_dot_LW = data.theta_dot_LW(1:time_cut_idx); % rad/s
theta_dot_RW = data.theta_dot_RW(1:time_cut_idx); % rad/s

%转换常数
iq_to_actual = 3.3 / 2048;       % Iq 转换系数（原始电流至实际电流）
encoder_to_degrees = 2*pi/65536;
torque_constant = 0.7;         % Torque Constant (Nm/A)

% Inertia
left_Inertia = 0.00072399902807526; % kg*m^2
right_Inertia = 0.00064074279507983; % kg*m^2

% left_Inertia = 0.00072399902807526 + 87235.791*1e-9 ; % kg*m^2
% right_Inertia = 0.00064074279507983 + 87235.791*1e-9 ; % kg*m^2

% 计算采样时间（假设TimeStamp以毫秒为单位）
dt = diff(timeStamp) / 1000; % 转换为秒

% 使用速度计算加速度
theta_ddot_LW = diff(theta_dot_LW) ./ dt; 
theta_ddot_RW = diff(theta_dot_RW) ./ dt;

tau_LW = left_Inertia * theta_ddot_LW;
tau_RW = right_Inertia * theta_ddot_RW;

% 时间轴调整
timeStamp_acceleration = timeStamp(1:end-1); %加速时间轴
% 
% 提取速度绝对值大于或等于32的索引。
LW_speed_idx = find(abs(theta_dot_LW) >= 20);
RW_speed_idx = find(abs(theta_dot_RW) >= 20);

% 从 motor1_input 中排除该索引
iq_LW_input_filtered = iq_LW_input(1:end-1);
tau_LW_filtered = tau_LW;
iq_LW_output_filtered = iq_LW_output(2:end);
iq_LW_input_filtered(LW_speed_idx(1:end-1)) = [];
tau_LW_filtered(LW_speed_idx(1:end-1)) = [];
iq_LW_output_filtered(LW_speed_idx(1:end-1)) = [];

% 从 motor1_input 中排除该索引
iq_RW_input_filtered = iq_RW_input(1:end-1);
tau_RW_filtered = tau_RW;
iq_RW_output_filtered = iq_RW_output(2:end);
iq_RW_input_filtered(RW_speed_idx(1:end-1)) = [];
tau_RW_filtered(RW_speed_idx(1:end-1)) = [];
iq_RW_output_filtered(RW_speed_idx(1:end-1)) = [];


% 绘图数据比较
figure;

% 角加速度比较
subplot(3, 2, 1);
plot(timeStamp_acceleration, tau_LW, 'r', 'DisplayName', '\tau_{LW}'); hold on;
% plot(timeStamp_acceleration, iq_LW_input(1:end-1)*iq_to_actual*torque_constant, 'k', 'DisplayName', '\tau_{LW,in}');
plot(timeStamp_acceleration, iq_LW_input(1:end-1)*0.001043224, 'k', 'DisplayName', '\tau_{LW,in}');
title('Left Wheel torque');
xlabel('TimeStamp');
ylabel('torque (Nm)');
legend('show'); hold off;

subplot(3, 2, 2);
plot(timeStamp_acceleration, tau_RW, 'b', 'DisplayName', '\tau_{RW}'); hold on;
% plot(timeStamp_acceleration, iq_RW_input(1:end-1)*iq_to_actual*torque_constant, 'k', 'DisplayName', '\tau_{RW,in}');
plot(timeStamp_acceleration, iq_RW_input(1:end-1)*0.000857902, 'k', 'DisplayName', '\tau_{RW,in}');

title('Right Wheel torque');
xlabel('TimeStamp');
ylabel('torque (Nm)');
legend('show'); hold off;


% 速度对比
subplot(3, 2, 3);
plot(timeStamp, theta_dot_LW, 'r', 'DisplayName', 'd\theta_{LW}');
title('Speed');
xlabel('TimeStamp');
ylabel('Speed(rad/s)');
legend('show'); hold off;

subplot(3, 2, 4);
plot(timeStamp, theta_dot_RW, 'b', 'DisplayName', 'd\theta_{RW}');
title('Speed');
xlabel('TimeStamp');
ylabel('Speed(rad/s)');
legend('show'); hold off;

% 智商电流比较
subplot(3, 2, 5);
plot(timeStamp, iq_LW_input, 'k', 'DisplayName', 'iq_{in}'); hold on;
plot(timeStamp, iq_LW_output, 'r', 'DisplayName', 'iq_{out}');
title('Left Wheel current');
xlabel('TimeStamp');
ylabel('current(LSD)');
legend('show'); hold off;

subplot(3, 2, 6);
plot(timeStamp, iq_RW_input, 'k', 'DisplayName', 'iq_{in}'); hold on;
plot(timeStamp, iq_RW_output, 'b', 'DisplayName', 'iq_{out}');
title('Right Wheel current');
xlabel('TimeStamp');
ylabel('current(LSD)');
legend('show'); hold off;

% figure;
% 
% subplot(3, 2, 1);
% % plot(iq_LW_output(2:end), tau_LW, '.', 'Color', 'r', 'DisplayName', 'LW');
% plot(iq_LW_output_filtered, tau_LW_filtered, '.', 'Color', 'r', 'DisplayName', 'LW');
% title('Left Wheel: IQ Current vs. Torque');
% xlabel('IQ Current (LSD)');
% ylabel('Torque (Nm)');
% legend('show'); hold off;
% 
% subplot(3, 2, 2);
% % plot(iq_RW_output(2:end), tau_RW, '.', 'Color', 'b', 'DisplayName', 'RW');
% plot(iq_RW_output_filtered, tau_RW_filtered, '.', 'Color', 'b', 'DisplayName', 'RW');
% title('Right Wheel: IQ Current vs. Torque');
% xlabel('IQ Current (LSD)');
% ylabel('Torque (Nm)');
% legend('show'); hold off;
% 
% subplot(3, 2, 3);
% % plot(iq_LW_input(1:end-1), tau_LW, '.', 'Color', 'r', 'DisplayName', 'LW');
% plot(iq_LW_input_filtered, tau_LW_filtered, '.', 'Color', 'r', 'DisplayName', 'LW');
% title('Left Wheel: Input vs. Torque');
% xlabel('Motor Input (LSD)');
% ylabel('Torque (Nm)');
% legend('show'); hold off;
% 
% subplot(3, 2, 4);
% % plot(iq_RW_input(1:end-1), tau_RW, '.', 'Color', 'b', 'DisplayName', 'RW');
% plot(iq_RW_input_filtered, tau_RW_filtered, '.', 'Color', 'b', 'DisplayName', 'RW');
% title('Right Wheel: Input vs. Torque');
% xlabel('Motor Input (LSD)');
% ylabel('Torque (Nm)');
% legend('show'); hold off;
% 
% subplot(3, 2, 5);
% % plot(iq_LW_input(1:end-1), iq_LW_output(2:end), '.', 'Color', 'r', 'DisplayName', 'LW');
% plot(iq_LW_input_filtered, iq_LW_output_filtered, '.', 'Color', 'r', 'DisplayName', 'LW');
% title('Left Wheel: Input vs. Output');
% xlabel('Motor Input (LSD)');
% ylabel('Torque (Nm)');
% legend('show'); hold off;
% 
% subplot(3, 2, 6);
% % plot(iq_RW_input(1:end-1), iq_RW_output(2:end), '.', 'Color', 'b', 'DisplayName', 'RW');
% plot(iq_RW_input_filtered, iq_RW_output_filtered, '.', 'Color', 'b', 'DisplayName', 'RW');
% title('Right Wheel: Input vs. Output');
% xlabel('Motor Input (LSD)');
% ylabel('Torque (Nm)');
% legend('show'); hold off;

figure;

% Left Wheel: IQ Current vs. Torque
subplot(3, 2, 1);
plot(iq_LW_output_filtered, tau_LW_filtered, '.', 'Color', 'r', 'DisplayName', 'LW');
title('Left Wheel: IQ Current vs. Torque');
xlabel('IQ Current (LSD)');
ylabel('Torque (Nm)');
hold on;

% Perform linear regression for Left Wheel (IQ Current vs. Torque)
coeffs = polyfit(iq_LW_output_filtered, tau_LW_filtered, 1); % Linear regression
x_fit = linspace(min(iq_LW_output_filtered), max(iq_LW_output_filtered), 100); % X range for line
y_fit = polyval(coeffs, x_fit); % Calculate y values for the regression line
plot(x_fit, y_fit, '-g', 'LineWidth', 2, 'DisplayName', 'Linear Fit LW'); % Plot regression line
grid on;
legend('show', 'Location', 'best'); hold off;

% Right Wheel: IQ Current vs. Torque
subplot(3, 2, 2);
plot(iq_RW_output_filtered, tau_RW_filtered, '.', 'Color', 'b', 'DisplayName', 'RW');
title('Right Wheel: IQ Current vs. Torque');
xlabel('IQ Current (LSD)');
ylabel('Torque (Nm)');
hold on;

% Perform linear regression for Right Wheel (IQ Current vs. Torque)
coeffs = polyfit(iq_RW_output_filtered, tau_RW_filtered, 1);
x_fit = linspace(min(iq_RW_output_filtered), max(iq_RW_output_filtered), 100);
y_fit = polyval(coeffs, x_fit);
plot(x_fit, y_fit, '-g', 'LineWidth', 2, 'DisplayName', 'Linear Fit RW');
grid on;
legend('show', 'Location', 'best'); hold off;

% Left Wheel: Input vs. Torque
subplot(3, 2, 3);
plot(iq_LW_input_filtered, tau_LW_filtered, '.', 'Color', 'r', 'DisplayName', 'LW');
title('Left Wheel: Input vs Torque');
xlabel('Motor Input (LSD)');
ylabel('Torque (Nm)');
hold on;

% Perform linear regression for Left Wheel (Input vs. Torque)
coeffs = polyfit(iq_LW_input_filtered, tau_LW_filtered, 1)
x_fit = linspace(min(iq_LW_input_filtered), max(iq_LW_input_filtered), 100);
y_fit = polyval(coeffs, x_fit);
plot(x_fit, y_fit, '-g', 'LineWidth', 2, 'DisplayName', 'Linear Fit LW');
grid on;
legend('show', 'Location', 'best'); hold off;

% Right Wheel: Input vs. Torque
subplot(3, 2, 4);
plot(iq_RW_input_filtered, tau_RW_filtered, '.', 'Color', 'b', 'DisplayName', 'RW');
title('Right Wheel: Input vs Torque');
xlabel('Motor Input (LSD)');
ylabel('Torque (Nm)');
hold on;

% Perform linear regression for Right Wheel (Input vs. Torque)
coeffs = polyfit(iq_RW_input_filtered, tau_RW_filtered, 1)
x_fit = linspace(min(iq_RW_input_filtered), max(iq_RW_input_filtered), 100);
y_fit = polyval(coeffs, x_fit);
plot(x_fit, y_fit, '-g', 'LineWidth', 2, 'DisplayName', 'Linear Fit RW');
grid on;
legend('show', 'Location', 'best'); hold off;

% Left Wheel: Input vs. Output
subplot(3, 2, 5);
plot(iq_LW_input_filtered, iq_LW_output_filtered, '.', 'Color', 'r', 'DisplayName', 'LW');
title('Left Wheel: Input vs. Output');
xlabel('Motor Input (LSD)');
ylabel('Motor Output (LSD)');
hold on;

% Perform linear regression for Left Wheel (Input vs. Output)
coeffs = polyfit(iq_LW_input_filtered, iq_LW_output_filtered, 1);
x_fit = linspace(min(iq_LW_input_filtered), max(iq_LW_input_filtered), 100);
y_fit = polyval(coeffs, x_fit);
plot(x_fit, y_fit, '-g', 'LineWidth', 2, 'DisplayName', 'Linear Fit LW');
legend('show', 'Location', 'best'); hold off;

% Right Wheel: Input vs. Output
subplot(3, 2, 6);
plot(iq_RW_input_filtered, iq_RW_output_filtered, '.', 'Color', 'b', 'DisplayName', 'RW');
title('Right Wheel: Input vs. Output');
xlabel('Motor Input (LSD)');
ylabel('Motor Output (LSD)');
hold on;

% Perform linear regression for Right Wheel (Input vs. Output)
coeffs = polyfit(iq_RW_input_filtered, iq_RW_output_filtered, 1);
x_fit = linspace(min(iq_RW_input_filtered), max(iq_RW_input_filtered), 100);
y_fit = polyval(coeffs, x_fit);
plot(x_fit, y_fit, '-g', 'LineWidth', 2, 'DisplayName', 'Linear Fit RW');
legend('show', 'Location', 'best'); hold off;





