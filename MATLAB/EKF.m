clc;clear;close all;
addpath("lib\");
format long;
load('dynamic_properties.mat');
load('dynamics_functions.mat');
properties.IG_matrices();

Ts = 0.008;
Ts_lqr = 0.008;
dt = 0.0001; % simulation dt

h = 0.2; % (m)
phi = 0;

% desired command
v_d = 2;
dpsi_d = 3.14;

% LQR Weights
Q_ = diag([100 0 20 5]);  % 状态权重
R_ = diag([150 150]);     % 输入权重

% EKF Parameters
P_init = eye(4); % 初始估计误差协方差矩阵
R_cov = diag([4e-1, 4e1, 4e-1,...
              1e-2, 1e-4, 1e-2,...
              0, 0]); % Sensor noise Covariance Matrix
Q_cov = diag([0, 1, 1, 1]); % Processor noise Covariance Matrix

% Estimator
x_hat_init = [0; 0; 0; 0];
x_hat = x_hat_init;

model = Pol(properties, dynamic_functions);
model.setState(zeros(4,1), zeros(2,1),h);

Estimator = EKF_c(x_hat_init, P_init, Q_cov, R_cov, model, Ts);

theta_eq = model.get_theta_eq(h, phi);

x_eq = [theta_eq; 0; 0; 0];
u_eq = zeros(2,1);

model.setState(x_eq, u_eq, h);
model.calculateDynamics();
model.calculateJacobian();

B = model.B;
M = model.M;
nle = model.nle;
dM_dtheta = model.dM_dtheta;
dnle_dtheta = model.dnle_dtheta;
dnle_dqdot = model.dnle_dqdot;

A_ = calculate_fx(M, dM_dtheta, nle, dnle_dtheta, dnle_dqdot);
B_ = calculate_fu(M, B);

% 计算可控性矩阵
C = ctrb(A_, B_);  % ctrb 函数是 MATLAB 内置函数，用于计算可控性矩阵。
if rank(C) == length(A_)  % 根据A_的大小确定系统的状态维数
    disp('System is controllable.'); 
else
    disp('System is NOT controllable.');
end

% 离散系统转换
sys_c = ss(A_, B_, eye(4, 4), []);        % 持续的系统创建
% Ac = sys_c.A
% Bc = sys_c.B
sys_d = c2d(sys_c, Ts_lqr, 'zoh');    % 使用 ZOH 方法进行离散化
Ad = sys_d.A                    % 离散 A 矩阵
Bd = sys_d.B                    % 离散 B 矩阵

A_delay = [Ad, Bd; zeros(2,6)];
B_delay = [zeros(4,2);eye(2)];
Q_delay = diag([10 10 5000 100 0 0]);  % 状态权重
[K_delay, S_delay, P_delay] = dlqr(A_delay, B_delay, Q_delay, R_);
K_delay;

% 离散 LQR 增益计算
[K_d, S_d, P_d] = dlqr(Ad, Bd, Q_, R_);
K_d


% 连续 LQR 增益计算
[K_c, S_c, P_c] = lqr(A_, B_, Q_, R_);

% 结果输出（离散系统）
disp('Discrete-time Closed-loop poles:');
disp(P_d);

% 结果输出（连续系统）
disp('Continuous-time Closed-loop poles:');
disp(P_c);

% Visualize discretized system closed loop poles
figure;
scatter(real(P_d), imag(P_d), 'filled', 'DisplayName', 'Poles');hold on;
xlabel('Real Part');
ylabel('Imaginary Part');
title('Closed-Loop Pole and Zero Locations (Discrete System)');
grid on;
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), '--r', 'LineWidth', 1, 'DisplayName', 'Unit Circle'); %单位圆
legend;
axis equal;

% Visualize continuous system closed loop poles
figure;
scatter(real(P_c), imag(P_c), 'filled', 'DisplayName', 'Poles');hold on;
xlabel('Real Part');
ylabel('Imaginary Part');
title('Closed-Loop Pole and Zero Locations (Continuous System)');
grid on;
xline(0, '--r', 'LineWidth', 1, 'DisplayName', 'Stability Boundary'); % 稳定区基线（真实收缩）
legend;
axis equal;

% 设置波特图选项
opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz'; % 设置频率（以 Hz 为单位）
opts.Grid = 'on';

% 设置输入名称
input_labels = {'\theta_d', 'd\theta_d', 'v_d', 'd\psi_d'};
syscl_c.InputName = input_labels;
output_labels = {'\theta', 'd\theta', 'v', 'd\psi'};
syscl_c.outputName = output_labels;

% 创建波特图
% figure();
% bodeplot(syscl_c, opts);

% 初始设置
x_eq = [theta_eq; 0; 0; 0];
x = x_eq + [0 * pi/180; 0; 0; 0];
x_prev = x;
z = zeros(8,1);

x_d = [theta_eq; 0; v_d; dpsi_d];
xi = zeros(3,1) + [x(1);0;0];

u_prev = [0; 0];
u = [0; 0];
z_log = zeros(8, 1);
x_log = zeros(4, 1);
x_hat_log = zeros(4,1);
u_log = zeros(2, 1);
xi_log = zeros(3,1);

% 模拟
idx = 1;
for t = 0:dt:3
    % Input updates based on sampling time
    if mod(idx, floor(Ts / dt)) == 0
        
        % 测量矢量（示例）
        % 通过真值提取传感器值
        model.calculateJacobian();
        [~, f, ~] = model.predict_state(dt);
        [z, ~] = model.predict_measurement(f, x);

        acc_noise_std = [0.039273; 0.043964; 0.048661];
        gyro_noise_std = [0.0017645; 0.0020313; 0.0023176];
        mspeed_precision = 0.1*pi/180;
        mspeed_noise_std = [0.1*pi/180; 0.1*pi/180];
        % 向输入添加噪声
        acc_noise = acc_noise_std .* randn(3,1);
        gyro_noise = gyro_noise_std .* randn(3,1);
        % mspeed_noise = mspeed_noise_std .* randn(2,1);
        
        z(1:3) = z(1:3) + acc_noise;
        z(4:6) = z(4:6) + gyro_noise;
        z(7:8) = round(z(7:8)/mspeed_precision)*mspeed_precision;
        % z(7:8) = z(7:8) + mspeed_noise;
        
        Estimator.predict(u, h);
        Estimator.update(z);
        x_hat = Estimator.x;
        
        % 定义范围
        % u_min = -1.128;
        % u_max = 1.128;
        u_min = -0.48;
        u_max = 0.48;
        
        u_prev = u;
        % u 计算
        u = -K_d * (x_hat - x_d);
        % u = -K_delay * ([x_hat; u_prev] - [x_d; 0; 0]);

        % 应用饱和度
        % u = max(min(u, u_max), u_min);
    end
    noise_std = 0.01;
    noise_mean = 0;
    % 向输入添加噪声
    noise = noise_std * randn(size(u)) + noise_mean;
    % u = u + noise;

    u_real = u_prev + noise;
    if(abs(u_real(1)) < 0.034)
        u_real(1) = 0;
    end
    if(abs(u_real(2)) < 0.034)
        u_real(2) = 0;
    end
    
    % 计算下一个状态
    x_prev = x;
    model.setState(x,u,h);
    model.calculateDynamics();
    [x, ~, ~] = model.predict_state(dt);

    xi = xi + x(2:4)*dt;


    % 数据存储
    z_log(:, idx) = z;
    x_log(:, idx) = x_prev;
    x_hat_log(:, idx) = x_hat;
    u_log(:, idx) = u_real;
    xi_log(:, idx) = xi;
    idx = idx + 1;
end
% 转换角度单位 (rad → deg)
x_log(1:2, :) = x_log(1:2, :) * 180 / pi;
x_log(4, :) = x_log(4, :) * 180 / pi;
x_hat_log(1:2, :) = x_hat_log(1:2, :) * 180 / pi;
x_hat_log(4, :) = x_hat_log(4, :) * 180 / pi;
x_d(1:2) = x_d(1:2) * 180 / pi;
x_d(4) = x_d(4) * 180 / pi;
xi_log(1,:) = xi_log(1,:) * 180 / pi;
xi_log(3,:) = xi_log(3,:) * 180 / pi;



% 创建时间向量
time = 0:dt:(size(x_log, 2) - 1) * dt;

% Plotting state variables (\theta)
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,3,1);
plot(time, x_log(1, :), 'b', 'LineWidth', 1.5); hold on;
plot(time, x_hat_log(1, :), 'k:', 'LineWidth', 1.5);
yline(x_d(1), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\theta (deg)');
grid on;
title('State Variable: \theta');
legend('x', 'x_{hat}' ,'x_{d}', 'Location', 'best');
ylim([-30, 30]); % Adjust as needed

% Plotting state variables (\dot{\theta})
subplot(2,3,2);
plot(time, x_log(2, :), 'b', 'LineWidth', 1.5); hold on;
plot(time, x_hat_log(2, :), 'k:', 'LineWidth', 1.5);
yline(x_d(2), 'r--', 'LineWidth', 1.5);
plot(time, xi_log(1,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('d\theta (deg/s)');
grid on;
title('State Variable: d\theta');
legend('x', 'x_{hat}', 'x_{d}', 'x_i', 'Location', 'best');
ylim([-100, 100]); % Adjust as needed

% Plotting state variables (v)
subplot(2,3,4);
plot(time, x_log(3, :), 'b', 'LineWidth', 1.5); hold on;
plot(time, x_hat_log(3, :), 'k:', 'LineWidth', 1.5);
yline(x_d(3), 'r--', 'LineWidth', 1.5);
plot(time, xi_log(2,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('v (m/s)');
grid on;
title('State Variable: v');
legend('x', 'x_{hat}', 'x_{d}', 'x_i', 'Location', 'best');
ylim([-4, 4]); % Adjust as needed

% Plotting state variables (\dot{\psi})
subplot(2,3,5);
plot(time, x_log(4, :), 'b', 'LineWidth', 1.5); hold on;
plot(time, x_hat_log(4, :), 'k:', 'LineWidth', 1.5);
yline(x_d(4), 'r--', 'LineWidth', 1.5);
plot(time, xi_log(3,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('d\psi (deg/s)');
grid on;
title('State Variable: d\psi');
legend('x', 'x_{hat}', 'x_{d}', 'x_i', 'Location', 'best');
ylim([-360, 360]); % Adjust as needed

% 绘制输入变量 (u_1)
subplot(2,3,3);
plot(time, u_log(1, :), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('u_1');
grid on;
title('Control Input: u_1');
legend('u', 'Location', 'best');
ylim([-2, 2]); % Adjust as needed

% 绘制输入变量 (u_2)
subplot(2,3,6);
plot(time, u_log(2, :), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('u_2');
grid on;
title('Control Input: u_2');
legend('u', 'Location', 'best');
ylim([-2, 2]); % Adjust as needed



% IMU acceleration plotting
acc_total = sqrt(z_log(1,:).^2+z_log(2,:).^2+z_log(3,:).^2);
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,3,1);
plot(time, z_log(1, :), 'r', 'LineWidth', 1.5); hold on;
plot(time, z_log(2, :), 'g', 'LineWidth', 1.5);
plot(time, z_log(3, :), 'b', 'LineWidth', 1.5);
plot(time, acc_total, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
grid on;
title('IMU acc');
legend('acc_x', 'acc_y' ,'acc_z', 'Location', 'best');
% ylim([-30, 30]); % Adjust as needed

% IMU gyro plotting
subplot(2,3,4);
plot(time, rad2deg(z_log(4, :)), 'r', 'LineWidth', 1.5); hold on;
plot(time, rad2deg(z_log(5, :)), 'g', 'LineWidth', 1.5);
plot(time, rad2deg(z_log(6, :)), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
grid on;
title('IMU gyro');
legend('gyro_x', 'gyro_y' ,'gyro_z', 'Location', 'best');
% ylim([-30, 30]); % Adjust as needed


% Wheel speed
subplot(2,3,2);
plot(time, rad2deg(z_log(7, :)), 'r', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Speed (deg/s)');
grid on;
title('Right Wheel speed');
% ylim([-4, 4]); % Adjust as needed

subplot(2,3,5);
plot(time, rad2deg(z_log(8, :)), 'b', 'LineWidth', 1.5); hold on;
xlabel('Time (s)');
ylabel('Speed (deg/s)');
grid on;
title('Left Wheel speed');
% ylim([-4, 4]); % Adjust as needed

