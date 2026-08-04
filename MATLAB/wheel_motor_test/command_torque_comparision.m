clc; clear; close all;

% CSV 파일 읽기
filename = '20241207_0409_logdata_withwheel_torque_speed_iqvalue_offset10_sampletime0.2s_10to1000.csv'; % CSV 파일 이름
rawData = readmatrix(filename); % CSV 데이터를 행렬로 읽기

% 변환 상수
iq_to_actual = 33 / 2048; % Iq 변환 계수 (raw to actual current)
speed_to_rads = pi / 1800; % 1 dps/LSB -> rad/s 변환 계수
encoder_to_degrees = 2*pi/65536;

% 데이터 분리
timeStamp = rawData(:, 1);  % 첫 번째 열: TimeStamp

% Motor 1 데이터
motor1_input = rawData(:, 4);     % Motor 1 TorqueValue
motor1_totalAngle = rawData(:, 5) * encoder_to_degrees; % Motor 1 TotalAngle 실제 값으로 변환
motor1_iqcurrent = rawData(:, 6);  % Motor 1 iqcurrentValue를 실제 값으로 변환
motor1_speed = rawData(:, 7)* speed_to_rads;       % Motor 1 Speed를 rad/s로 변환

% Motor 2 데이터
motor2_input = rawData(:, 10);     % Motor 2 TorqueValue
motor2_totalAngle = rawData(:, 11) * encoder_to_degrees; % Motor 2 TotalAngle을 실제 값으로 변환
motor2_iqcurrent = rawData(:, 12); % Motor 2 iqcurrentValue를 실제 값으로 변환
motor2_speed = rawData(:, 13)* speed_to_rads;      % Motor 2 Speed를 rad/s로 변환

% Torque Constant (Nm/A)
torque_constant = 0.07;

% Sampling time 계산 (TimeStamp는 밀리초 단위라고 가정)
dt = diff(timeStamp) / 1000; % 초 단위로 변환

% Inertia
left_Inertia = 0.00072399902807526; % kg*m^2
right_Inertia = 0.00064074279507983; % kg*m^2

% Speed를 사용하여 Acceleration 계산
motor1_acceleration = diff(motor1_speed) ./ dt; % Motor 1 Acceleration from Speed
motor2_acceleration = diff(motor2_speed) ./ dt; % Motor 2 Acceleration from Speed

inertia_motor1_torque = left_Inertia * motor1_acceleration;
inertia_motor2_torque = right_Inertia * motor2_acceleration;

% 시간축 조정
timeStamp_acceleration = timeStamp(1:end-1); % Acceleration 시간축

% Motor 1에서 speed의 절대값이 32 이상인 인덱스를 추출
motor1_speed_idx = find(abs(motor1_speed) >= 32);

% Motor 2에서 speed의 절대값이 32 이상인 인덱스를 추출
motor2_speed_idx = find(abs(motor2_speed) >= 32);

% motor1_input에서 해당 인덱스를 제외
motor1_input_filtered = motor1_input;
inertia_motor1_torque_filtered = inertia_motor1_torque;
motor1_iqcurrent_filtered = motor1_iqcurrent;
motor1_input_filtered(motor1_speed_idx) = [];
inertia_motor1_torque_filtered(motor1_speed_idx(1:end-1)) = [];
motor1_iqcurrent_filtered(motor1_speed_idx) = [];

% motor2_input에서 해당 인덱스를 제외
motor2_input_filtered = motor2_input;
inertia_motor2_torque_filtered = inertia_motor2_torque;
motor2_iqcurrent_filtered = motor2_iqcurrent;
motor2_input_filtered(motor2_speed_idx) = [];
inertia_motor2_torque_filtered(motor2_speed_idx(1:end-1)) = [];
motor2_iqcurrent_filtered(motor2_speed_idx) = [];



% Plot 데이터 비교
figure;

% Angular Acceleration 비교
subplot(3, 2, 1);
plot(timeStamp_acceleration, motor1_acceleration, 'r', 'DisplayName', 'Motor 1'); hold on;
plot(timeStamp_acceleration, motor2_acceleration, 'b', 'DisplayName', 'Motor 2');
title('Angular Acceleration');
xlabel('TimeStamp');
ylabel('Angular Acceleration (rad/s^2)');
legend('show'); hold off;

% Speed 비교
subplot(3, 2, 2);
plot(timeStamp, motor1_speed, 'r', 'DisplayName', 'Motor 1'); hold on;
plot(timeStamp, motor2_speed, 'b', 'DisplayName', 'Motor 2');
title('Speed');
xlabel('TimeStamp');
ylabel('Speed(rad/s)');
legend('show'); hold off;

% Torque 비교
subplot(3, 2, 3);
plot(timeStamp, motor1_input, 'k', 'DisplayName', 'Command'); hold on;
plot(timeStamp, motor1_iqcurrent, 'r', 'DisplayName', 'current');
title('Left Wheel current');
xlabel('TimeStamp');
ylabel('current(LSD)');
legend('show'); hold off;

% iqcurrentValue 비교
subplot(3, 2, 4);
plot(timeStamp, motor2_input, 'k', 'DisplayName', 'Command'); hold on;
plot(timeStamp, motor2_iqcurrent, 'b', 'DisplayName', 'current');
title('Right Wheel current');
xlabel('TimeStamp');
ylabel('current(LSD)');
legend('show'); hold off;

% Calculated Torque 비교 (IqCurrent 기반)
subplot(3, 2, 5);
plot(timeStamp, motor1_input*iq_to_actual*torque_constant, 'k', 'DisplayName', 'Command'); hold on;
plot(timeStamp_acceleration, inertia_motor1_torque, 'r', 'DisplayName', 'Motor 1');
title('inertia Torque');
xlabel('TimeStamp');
ylabel('Torque (Nm)');
legend('show'); hold off;

% Calculated Torque 비교 (IqCurrent 기반)
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



