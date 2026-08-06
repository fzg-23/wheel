clc;clear;close all;
addpath("lib\");
format long;
load('dynamic_properties.mat');
load('dynamics_functions.mat');

phi = 0;
% 定义状态权重 (Q) 和输入权重 (R)
% Q_ = diag([50 10 10 5]);  % 状态权重
% R_=diag([150 150]);     % 输入重量
Q_ = diag([100 0 20 5]);  % 状态权重
R_ = diag([150 150]);     % 输入权重

% 采样时间（T）
Ts = 0.008;

Ks = [];
for h = 0.07:0.01:0.2

    model = Pol(properties, dynamic_functions);
    model.setState(zeros(4,1), zeros(2,1),h);
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

    % 离散系统转换
    sys_c = ss(A_, B_, [], []);        % 创建连续系统（无 C、D）
    sys_d = c2d(sys_c, Ts, 'zoh');    % 使用 ZOH 方法进行离散化
    Ad = sys_d.A;                    % 离散 A 矩阵
    Bd = sys_d.B;                    % 离散 B 矩阵

    % LQR增益计算
    K_d = dlqr(Ad, Bd, Q_, R_);
    Ks = [Ks; K_d];
end

% 检查数据的大小（矩阵的行和列）
[numRows, numCols] = size(Ks);

% 以 C++ 代码格式输出
for i = 1:numRows/2
    fprintf("mat << ");
    fprintf('   % .8ff,  % .8ff,  % .8ff,  % .8ff,\n', Ks(2*i-1,1), Ks(2*i-1,2), Ks(2*i-1,3), Ks(2*i-1,4));
    fprintf('   % .8ff,  % .8ff,  % .8ff,  % .8ff;\n', Ks(2*i,1), Ks(2*i,2), Ks(2*i,3), Ks(2*i,4));
    fprintf('Ks.push_back(mat);');
    fprintf('\n\n');
end