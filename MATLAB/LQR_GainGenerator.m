clc;clear;close all;
addpath("lib\");
format long;
load('dynamic_properties.mat');
load('dynamics_functions.mat');

phi = 0;
% 상태 가중치(Q)와 입력 가중치(R) 정의
% Q_ = diag([50 10  10 5]);  % 상태 가중치
% R_ = diag([150 150]);     % 입력 가중치
Q_ = diag([100 0 20 5]);  % 상태 가중치
R_ = diag([150 150]);     % 입력 가중치

% 샘플링 시간 (T)
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

    % 이산 시스템 변환
    sys_c = ss(A_, B_, [], []);        % 연속 시스템 생성 (C, D 없음)
    sys_d = c2d(sys_c, Ts, 'zoh');    % ZOH 방식으로 이산화
    Ad = sys_d.A;                    % 이산화된 A 행렬
    Bd = sys_d.B;                    % 이산화된 B 행렬

    % LQR Gain 계산
    K_d = dlqr(Ad, Bd, Q_, R_);
    Ks = [Ks; K_d];
end

% 데이터의 크기 확인 (행렬의 행과 열)
[numRows, numCols] = size(Ks);

% C++ 코드 형식으로 출력
for i = 1:numRows/2
    fprintf("mat << ");
    fprintf('   % .8ff,  % .8ff,  % .8ff,  % .8ff,\n', Ks(2*i-1,1), Ks(2*i-1,2), Ks(2*i-1,3), Ks(2*i-1,4));
    fprintf('   % .8ff,  % .8ff,  % .8ff,  % .8ff;\n', Ks(2*i,1), Ks(2*i,2), Ks(2*i,3), Ks(2*i,4));
    fprintf('Ks.push_back(mat);');
    fprintf('\n\n');
end