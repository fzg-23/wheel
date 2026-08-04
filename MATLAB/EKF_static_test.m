clc;clear;close all;
addpath("lib\");
format long;
load('dynamic_properties.mat');
load('dynamics_functions.mat');

h = 0.2; % (m)
phi = 0;

[mass_total, r_total, I_total] = calculate_body_total_property(h, phi, properties);
m_B = mass_total;
p_bcom = r_total;
I_B_B = I_total;
L = properties.L;
R = properties.R;
m_RW = mass_of_Wheel_Right * 1e-3;
m_LW = mass_of_Wheel * 1e-3;

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

% disp('Minv_B : ');
% disp('tau_RW                tau_LW');
% disp(M\B);
% disp('-Minv_nle : ');
% disp(-M\nle);

epsilon = 1e-9;

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

% 컨트롤 가능성 행렬 계산
C = ctrb(A_, B_);  % ctrb 함수는 MATLAB 내장 함수로, 컨트롤 가능성 행렬을 계산

% 랭크 확인
rank_C = rank(C);  % 컨트롤 가능성 행렬의 랭크를 계산


% 샘플링 시간 (T)
Ts = 0.05;
dt = 0.001; % simulation dt

% 이산 시스템 변환
sys_c = ss(A_, B_, [], []);        % 연속 시스템 생성 (C, D 없음)
sys_d = c2d(sys_c, Ts, 'zoh');    % ZOH 방식으로 이산화
Ad = sys_d.A;                    % 이산화된 A 행렬
Bd = sys_d.B;                    % 이산화된 B 행렬

% 상태 가중치(Q)와 입력 가중치(R) 정의
Q_ = diag([100 10 100 10]);  % 상태 가중치
R_ = diag([0.1 0.1]);            % 입력 가중치

% LQR Gain 계산
K_d = dlqr(Ad, Bd, Q_, R_)

% 폐루프 시스템 행렬
Acl = Ad - Bd * K_d;

% 폐루프 극점 계산
closed_loop_poles = eig(Acl);

% 결과 출력
% disp('Closed-loop poles:');
% disp(closed_loop_poles);

% 극점 시각화
figure;
scatter(real(closed_loop_poles), imag(closed_loop_poles), 'filled');
xlabel('Real Part');
ylabel('Imaginary Part');
title('Closed-Loop Pole Locations (Discrete System)');
grid on;

% 단위원 시각화
hold on;
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), '--r', 'LineWidth', 1); % 단위원
axis equal;


% EKF covariance matrix
P = eye(4); % 초기 추정 오차 공분산 행렬
R_cov = diag([10, 10, 10, 10, 10, 10, 1e-6, 1e-6]); % 측정 오차 공분산 행렬
Q_cov = diag([0.01, 0.1, 0.1, 1]); % 프로세스 노이즈 공분산 행렬

% 초기 설정
x_eq = [theta_eq; 0; 0; 0];
x = x_eq + [1 * pi/180; 0; 0; 0];
x_hat = x + [0 * pi/180; 0; 0; 0];
del_x = zeros(4, 1);

x_d = [theta_eq; 0; 0; 0*pi/180];

xi = zeros(3,1) + [x(1);0;0];

u = [0; 0];
x_log = zeros(4, 1);
x_hat_log = zeros(4,1);
u_log = zeros(2, 1);
xi_log = zeros(3,1);

% 시뮬레이션
idx = 1;
for t = 0:dt:3
    del_x = x - x_eq;
    del_x_d = x_d - x_eq;

    % 시스템 행렬 계산
    M = M_f(I_B_B(1,1),I_B_B(2,1),I_B_B(2,2),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
        I_B_LW(2,1),I_B_LW(2,2),I_B_RW(2,1),I_B_RW(2,2),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
        I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,m_LW,m_RW,p_bcom(1),p_bcom(2),p_bcom(3),x(1));

    nle = nle_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
        I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3), I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3), ...
        L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), x(4),x(1),x(2),x(3));


    % 샘플링 시간에 따라 입력 업데이트
    if mod(idx, floor(Ts / dt)) == 0
        M_hat = M_f(I_B_B(1,1),I_B_B(2,1),I_B_B(2,2),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
            I_B_LW(2,1),I_B_LW(2,2),I_B_RW(2,1),I_B_RW(2,2),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
            I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,m_LW,m_RW,p_bcom(1),p_bcom(2),p_bcom(3),x_hat(1));

        dM_dtheta_hat = dM_dtheta_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
            I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
            I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3),x_hat(1));

        nle_hat = nle_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
            I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3), I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3), ...
            L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), x_hat(4),x_hat(1),x_hat(2),x_hat(3));

        dnle_dtheta_hat = dnle_dtheta_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
            I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
            I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,g,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), ...
            x_hat(4),x_hat(1),x_hat(2),x_hat(3));

        dnle_dqdot_hat = dnle_dqdot_f(I_B_B(1,1),I_B_B(2,1),I_B_B(3,1),I_B_B(3,2),I_B_B(3,3),I_B_LW(1,1),I_B_RW(1,1), ...
            I_B_LW(2,1),I_B_RW(2,1),I_B_LW(3,1),I_B_LW(3,2),I_B_LW(3,3),...
            I_B_RW(3,1),I_B_RW(3,2),I_B_RW(3,3),L,R,h,m_B,p_bcom(1),p_bcom(2),p_bcom(3), ...
            x_hat(4),x_hat(1),x_hat(2),x_hat(3));
        
        % 측정 벡터 (예시)
        % true 값을 통해 sensor 값 추출
        [~, f, ~] = predict_state(x_prev, u, M, dM_dtheta, nle, dnle_dtheta, dnle_dqdot, B,  dt);
        [z, ~] = predict_measurement(f, x_prev, g, h, L, R);
        
        acc_noise_std = 0.0175;
        gyro_noise_std = 3.90267*1e-4;
        % 입력에 노이즈 추가
        acc_noise = acc_noise_std * randn(3,1);
        gyro_noise = gyro_noise_std * randn(3,1);
        
        % z(1:3) = z(1:3) + acc_noise;
        % z(4:6) = z(4:6) + gyro_noise;

        

        % z(1:3) = 

        % 예측 단계 (Prediction)
        [x_pred, dx, F] = predict_state(x_hat, u, M_hat, dM_dtheta_hat, nle_hat, dnle_dtheta_hat, dnle_dqdot_hat, B,  Ts);
        P_pred = F * P * F' + Q_cov; % 예측된 오차 공분산 행렬
        % dx = zeros(4,1);
        % 업데이트 단계 (Update)
        [h_obs, H] = predict_measurement(dx, x_pred, g, h, L, R);
        K = P_pred * H' / (H * P_pred * H' + R_cov); % 칼만 이득
        x_hat = x_pred + K * (z - h_obs); % 상태 업데이트
        P = (eye(length(x_prev)) - K * H) * P_pred; % 오차 공분산 업데이트
        
        del_x_hat = x_hat - x_eq;
        
        % u = -K_d * (del_x - del_x_d);
        u = -K_d * (del_x_hat - del_x_d);


    end
    noise_std = 0.01;
    noise_mean = 0.00;
    % 입력에 노이즈 추가
    noise = noise_std * randn(size(u)) + noise_mean;
    u = u + noise;

    % 다음 상태 계산
    x_prev = x;
    % x = step(x, M, nle, B, u, dt);
    x = x;
    xi = xi + x(2:4)*dt;


    % 데이터 저장
    x_log(:, idx) = x_prev;
    x_hat_log(:, idx) = x_hat;
    u_log(:, idx) = u;
    xi_log(:, idx) = xi;
    idx = idx + 1;
end
% 각도 단위 변환 (rad → deg)
x_log(1:2, :) = x_log(1:2, :) * 180 / pi;
x_log(4, :) = x_log(4, :) * 180 / pi;
x_hat_log(1:2, :) = x_hat_log(1:2, :) * 180 / pi;
x_hat_log(4, :) = x_hat_log(4, :) * 180 / pi;
x_d(1:2) = x_d(1:2) * 180 / pi;
x_d(4) = x_d(4) * 180 / pi;
xi_log(1,:) = xi_log(1,:) * 180 / pi;
xi_log(3,:) = xi_log(3,:) * 180 / pi;



% 시간 벡터 생성
time = 0:dt:(size(x_log, 2) - 1) * dt;

% 상태 변수 플로팅 (\theta)
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

% 상태 변수 플로팅 (\dot{\theta})
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

% 상태 변수 플로팅 (v)
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

% 상태 변수 플로팅 (\dot{\psi})
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
ylim([-100, 100]); % Adjust as needed

% 입력 변수 플로팅 (u_1)
subplot(2,3,3);
plot(time, u_log(1, :), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('u_1');
grid on;
title('Control Input: u_1');
legend('u', 'Location', 'best');
ylim([-2, 2]); % Adjust as needed

% 입력 변수 플로팅 (u_2)
subplot(2,3,6);
plot(time, u_log(2, :), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('u_2');
grid on;
title('Control Input: u_2');
legend('u', 'Location', 'best');
ylim([-2, 2]); % Adjust as needed

