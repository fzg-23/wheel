clc;
% clear;
close all;
format long;
load('dynamic_properties.mat');
load('dynamics_functions.mat');
addpath("lib\");

h = 0.06; % (m)
phi = 0;

[mass_total, r_total, I_total] = calculate_body_total_property(h, phi, properties);
m_B = mass_total;
p_bcom = r_total;
I_B_B = I_total;
L = properties.L;
R = properties.R;
m_RW =mass_of_Wheel_Right * 1e-3;
m_LW =mass_of_Wheel * 1e-3;

theta_eq = atan(-r_total(1) / (h + r_total(3)));
% disp('theta_eq(deg) : ');
% disp(theta_eq * 180 / pi);

B = B_f(L, R);

M = M_f(I_B_B(1,1),I_B_B(2,1),I_B_B(2,2),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_LW(2,2),I_B_RW(2,1),I_B_RW(2,2),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
    I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,m_LW,m_RW,p_bcom(1),p_bcom(2),p_bcom(3),theta_eq);

nle = nle_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3), I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3), ...
    L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), 0,theta_eq,0,0);

dM_dtheta = dM_dtheta_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
    I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3),theta_eq);

dnle_dtheta = dnle_dtheta_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
    I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), ...
    0,theta_eq,0,0);

dnle_dqdot = dnle_dqdot_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
    I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
    I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), ...
    0,theta_eq,0,0);

A_ = calculate_fx(M, dM_dtheta, nle, dnle_dtheta, dnle_dqdot);

B_ = calculate_fu(M, B);

% 计算可控性矩阵
C = ctrb(A_, B_);  % ctrb 函数是 MATLAB 内置函数，用于计算可控性矩阵。

% 检查你的排名
rank_C = rank(C);  % 计算可控性矩阵的秩


% 采样时间（T）
Ts = 0.01;
dt = 0.0001; % simulation dt

% 离散系统转换
sys_c = ss(A_, B_, [], []);        % 创建连续系统（无 C、D）
sys_d = c2d(sys_c, Ts, 'zoh');    % 使用 ZOH 方法进行离散化
Ad = sys_d.A;                    % 离散 A 矩阵
Bd = sys_d.B;                    % 离散 B 矩阵

Q_ = diag([100 10 10000 10000]);  % 状态权重
R_ = diag([2e4 2e4]);            % 输入权重

% LQR增益计算
K_c = lqr(A_, B_, Q_, R_);
K_d = dlqr(Ad, Bd, Q_, R_)

% 闭环系统矩阵
Acl = Ad - Bd * K_d;

%闭环极点计算
closed_loop_poles = eig(Acl);

% 结果输出
% disp('Closed-loop poles:');
% disp(closed_loop_poles);

% 杆可视化
figure;
scatter(real(closed_loop_poles), imag(closed_loop_poles), 'filled');
xlabel('Real Part');
ylabel('Imaginary Part');
title('Closed-Loop Pole Locations (Discrete System)');
grid on;

% 单位圆可视化
hold on;
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), '--r', 'LineWidth', 1); %单位圆
axis equal;



% 初始设置
x_eq = [theta_eq; 0; 0; 0];
x_next = x_eq + [0; 0; 0; 0];
del_x = zeros(4, 1);

x_d = [theta_eq; 0; 1; 1];

xi = zeros(3,1) + [x_next(1);0;0];

u = [0; 0];
x_log = zeros(4, 1);
u_log = zeros(2, 1);
xi_log = zeros(3,1);

% 模拟
idx = 1;
for t = 0:dt:3
    x = x_next;
    del_x = x - x_eq;
    del_x_d = x_d - x_eq;


    % 系统矩阵计算
    M = M_f(I_B_B(1,1),I_B_B(2,1),I_B_B(2,2),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
        I_B_LW(2,1),I_B_LW(2,2),I_B_RW(2,1),I_B_RW(2,2),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
        I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,m_LW,m_RW,p_bcom(1),p_bcom(2),p_bcom(3),x(1));

    nle = nle_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
        I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3), I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3), ...
        L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), x(4),x(1),x(2),x(3));

    % Input updates based on sampling time
    if mod(idx, floor(Ts / dt)) == 0
        % u = -K_d * (del_x - del_x_d) ;
        u = -K_c * (del_x - del_x_d) ;

        u_min = -0.18;
        u_max = 0.18;

        % 应用饱和度
        % u = max(min(u, u_max), u_min);

        % noise_std = 0.01;
        % noise_mean = 0;
        %% 在输入中添加噪声
        % noise = noise_std * randn(size(u)) + noise_mean;
        % u = u + noise;
    end

    % 计算下一个状态
    x_next = step(x, M, nle, B, u, dt);
    xi = xi + x_next(2:4)*dt;


    % 数据存储
    x_log(:, idx) = x;
    u_log(:, idx) = u;
    xi_log(:, idx) = xi;
    idx = idx + 1;
end
% 转换角度单位 (rad → deg)
x_log(1:2, :) = x_log(1:2, :) * 180 / pi;
x_log(4, :) = x_log(4, :) * 180 / pi;
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
yline(x_d(1), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('\theta (deg)');
grid on;
title('State Variable: \theta');
legend('x', 'x_{d}', 'Location', 'best');
% ylim([-20, 20]); % Adjust as needed

% Plotting state variables (\dot{\theta})
subplot(2,3,2);
plot(time, x_log(2, :), 'b', 'LineWidth', 1.5); hold on;
yline(x_d(2), 'r--', 'LineWidth', 1.5);
plot(time, xi_log(1,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('d\theta (deg/s)');
grid on;
title('State Variable: d\theta');
legend('x', 'x_{d}', 'x_i', 'Location', 'best');
% ylim([-100, 100]); % Adjust as needed

% Plotting state variables (v)
subplot(2,3,4);
plot(time, x_log(3, :), 'b', 'LineWidth', 1.5); hold on;
yline(x_d(3), 'r--', 'LineWidth', 1.5);
plot(time, xi_log(2,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('v (m/s)');
grid on;
title('State Variable: v');
legend('x', 'x_{d}', 'x_i', 'Location', 'best');
% ylim([-3, 3]); % Adjust as needed

% Plotting state variables (\dot{\psi})
subplot(2,3,5);
plot(time, x_log(4, :), 'b', 'LineWidth', 1.5); hold on;
yline(x_d(4), 'r--', 'LineWidth', 1.5);
plot(time, xi_log(3,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('d\psi (deg/s)');
grid on;
title('State Variable: d\psi');
legend('x', 'x_{d}', 'x_i', 'Location', 'best');
% ylim([-100, 100]); % Adjust as needed

% 绘制输入变量 (u_1)
subplot(2,3,3);
plot(time, u_log(1, :), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('u_1');
grid on;
title('Control Input: u_1');
legend('u', 'Location', 'best');
ylim([-0.5, 0.5]); % Adjust as needed

% 绘制输入变量 (u_2)
subplot(2,3,6);
plot(time, u_log(2, :), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('u_2');
grid on;
title('Control Input: u_2');
legend('u', 'Location', 'best');
ylim([-0.5, 0.5]); % Adjust as needed



