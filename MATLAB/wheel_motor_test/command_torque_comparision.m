clc; clear; close all;

% 读取 CSV 文件
filename = '20241207_0409_logdata_withwheel_torque_speed_iqvalue_offset10_sampletime0.2s_10to1000.csv'; % CSV 文件名
rawData = readmatrix(filename); % 将 CSV 数据读取为矩阵

%转换常数
iq_to_actual = 33 / 2048; % Iq 转换系数（原始电流至实际电流）
speed_to_rads = pi / 1800; % 1 dps/LSB -> rad/s 转换系数
encoder_to_degrees = 2*pi/65536;

% 数据分离
timeStamp = rawData(:, 1);  % 第一列：时间戳

% 电机1数据
motor1_input = rawData(:, 4);     % Motor 1 TorqueValue
motor1_totalAngle = rawData(:, 5) * encoder_to_degrees; % 电机 1 总角度转换为实际值
motor1_iqcurrent = rawData(:, 6);  % 将电机 1 iqcurrentValue 转换为实际值
motor1_speed = rawData(:, 7)* speed_to_rads;       % 将电机 1 速度转换为 rad/s

% 电机2数据
motor2_input = rawData(:, 10);     % Motor 2 TorqueValue
motor2_totalAngle = rawData(:, 11) * encoder_to_degrees; % 将电机 2 TotalAngle 转换为实际值
motor2_iqcurrent = rawData(:, 12); % 将电机 2 iqcurrentValue 转换为实际值
motor2_speed = rawData(:, 13)* speed_to_rads;      % 将电机 2 速度转换为 rad/s

% Torque Constant (Nm/A)
torque_constant = 0.07;

% 计算采样时间（假设TimeStamp以毫秒为单位）
dt = diff(timeStamp) / 1000; % 转换为秒

% Inertia
left_Inertia = 0.00072399902807526; % kg*m^2
right_Inertia = 0.00064074279507983; % kg*m^2

% 使用速度计算加速度
motor1_acceleration = diff(motor1_speed) ./ dt; % Motor 1 Acceleration from Speed
motor2_acceleration = diff(motor2_speed) ./ dt; % Motor 2 Acceleration from Speed

inertia_motor1_torque = left_Inertia * motor1_acceleration;
inertia_motor2_torque = right_Inertia * motor2_acceleration;

% 时间轴调整
timeStamp_acceleration = timeStamp(1:end-1); %加速时间轴

% 从电机 1 中提取速度绝对值大于或等于 32 的索引。
motor1_speed_idx = find(abs(motor1_speed) >= 32);

% 从电机2中提取速度绝对值大于或等于32的索引
motor2_speed_idx = find(abs(motor2_speed) >= 32);

% 从 motor1_input 中排除该索引
motor1_input_filtered = motor1_input;
inertia_motor1_torque_filtered = inertia_motor1_torque;
motor1_iqcurrent_filtered = motor1_iqcurrent;
motor1_input_filtered(motor1_speed_idx) = [];
inertia_motor1_torque_filtered(motor1_speed_idx(1:end-1)) = [];
motor1_iqcurrent_filtered(motor1_speed_idx) = [];

% 从 motor2_input 中排除该索引
motor2_input_filtered = motor2_input;
inertia_motor2_torque_filtered = inertia_motor2_torque;
motor2_iqcurrent_filtered = motor2_iqcurrent;
motor2_input_filtered(motor2_speed_idx) = [];
inertia_motor2_torque_filtered(motor2_speed_idx(1:end-1)) = [];
motor2_iqcurrent_filtered(motor2_speed_idx) = [];



% 绘图数据比较
figure;

% 角加速度比较
subplot(3, 2, 1);
plot(timeStamp_acceleration, motor1_acceleration, 'r', 'DisplayName', 'Motor 1'); hold on;
plot(timeStamp_acceleration, motor2_acceleration, 'b', 'DisplayName', 'Motor 2');
title('Angular Acceleration');
xlabel('TimeStamp');
ylabel('Angular Acceleration (rad/s^2)');
legend('show'); hold off;

% 速度对比
subplot(3, 2, 2);
plot(timeStamp, motor1_speed, 'r', 'DisplayName', 'Motor 1'); hold on;
plot(timeStamp, motor2_speed, 'b', 'DisplayName', 'Motor 2');
title('Speed');
xlabel('TimeStamp');
ylabel('Speed(rad/s)');
legend('show'); hold off;

% 扭矩比较
subplot(3, 2, 3);
plot(timeStamp, motor1_input, 'k', 'DisplayName', 'Command'); hold on;
plot(timeStamp, motor1_iqcurrent, 'r', 'DisplayName', 'current');
title('Left Wheel current');
xlabel('TimeStamp');
ylabel('current(LSD)');
legend('show'); hold off;

% 比较 iqcurrent 值
subplot(3, 2, 4);
plot(timeStamp, motor2_input, 'k', 'DisplayName', 'Command'); hold on;
plot(timeStamp, motor2_iqcurrent, 'b', 'DisplayName', 'current');
title('Right Wheel current');
xlabel('TimeStamp');
ylabel('current(LSD)');
legend('show'); hold off;

% 计算出的扭矩比较（基于 IqCurrent）
subplot(3, 2, 5);
plot(timeStamp, motor1_input*iq_to_actual*torque_constant, 'k', 'DisplayName', 'Command'); hold on;
plot(timeStamp_acceleration, inertia_motor1_torque, 'r', 'DisplayName', 'Motor 1');
title('inertia Torque');
xlabel('TimeStamp');
ylabel('Torque (Nm)');
legend('show'); hold off;

% 计算出的扭矩比较（基于 IqCurrent）
subplot(3, 2, 6);
plot(timeStamp, motor2_input*iq_to_actual*torque_constant, 'k', 'DisplayName', 'Command'); hold on;
plot(timeStamp_acceleration, inertia_motor2_torque, 'b', 'DisplayName', 'Motor 2');
title('inertia Torque');
xlabel('TimeStamp');
ylabel('Torque (Nm)');
legend('show'); hold off;

figure;

subplot(2, 2, 1);
% plot(motor1_iqcurrent(1:end-1), inertia_motor1_torque, '.', 'Color', 'r', 'DisplayName', 'Motor 1');
plot(motor1_iqcurrent_filtered, inertia_motor1_torque_filtered, '.', 'Color', 'r', 'DisplayName', 'Motor 1');
title('Left Wheel: Motor 1 IQ Current vs. Torque');
xlabel('IQ Current (LSD)');
ylabel('Torque (Nm)');
legend('show'); hold off;

subplot(2, 2, 2);
plot(motor2_iqcurrent_filtered, inertia_motor2_torque_filtered, '.', 'Color', 'r', 'DisplayName', 'Motor 2');
title('Right Wheel: Motor 2 IQ Current vs. Torque');
xlabel('IQ Current (LSD)');
ylabel('Torque (Nm)');
legend('show'); hold off;

% subplot(2, 2, 3);
% plot(motor1_input(1:end-1), inertia_motor1_torque, '.', 'Color', 'r', 'DisplayName', 'Motor 1');
% title('Left Wheel: Filtered Motor 1 Input vs. Torque');
% xlabel('Motor Input (LSD)');
% ylabel('Torque (Nm)');
% legend('show'); hold off;
% 
% subplot(2, 2, 4);
% plot(motor2_input(1:end-1), inertia_motor2_torque, '.', 'Color', 'b', 'DisplayName', 'Motor 2');
% title('Right Wheel: Filtered Motor 2 Input vs. Torque');
% xlabel('Motor Input (LSD)');
% ylabel('Torque (Nm)');
% legend('show'); hold off;

subplot(2, 2, 3);
plot(motor1_input_filtered, inertia_motor1_torque_filtered, '.', 'Color', 'r', 'DisplayName', 'Motor 1');
title('Left Wheel: Filtered Motor 1 Input vs. Torque');
xlabel('Motor Input (LSD)');
ylabel('Torque (Nm)');
legend('show'); hold off;

subplot(2, 2, 4);
plot(motor2_input_filtered, inertia_motor2_torque_filtered, '.', 'Color', 'b', 'DisplayName', 'Motor 2');
title('Right Wheel: Filtered Motor 2 Input vs. Torque');
xlabel('Motor Input (LSD)');
ylabel('Torque (Nm)');
legend('show'); hold off;



