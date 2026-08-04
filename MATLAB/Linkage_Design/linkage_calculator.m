clc; clear all; close all;  % 콘솔, 변수, 그리고 모든 창을 초기화

% 링크의 길이 정의
l1 = 106;  % 첫 번째 링크의 길이 (mm)
l2 = 77;   % 두 번째 링크의 길이 (mm)
l3 = 50;   % 세 번째 링크의 길이 (mm)
l4 = 137;  % 네 번째 링크의 길이 (mm)
l5 = -8;   % 네 번째 링크의 수직방향 offset (mm)

theta_root = 30;  % 루트 각도 (deg)
len_ab = 75;      % 링크 AB의 길이 (mm)

W = 2;  % 질량 (kg)

y_min = -200;  % 최소 y 값 (mm)
y_max = -70;   % 최대 y 값 (mm)

% 계산을 위한 각도 범위 정의
theta = -pi/2 : 0.001 : pi/1;  % theta 범위 (-90도에서 180도까지)

W = 9.81 * W;    % 질량을 무게(N)로 변환

% (a, b) 점의 좌표 정의
a = len_ab * cos(theta_root * pi / 180);  % a 점의 x 좌표
b = len_ab * sin(theta_root * pi / 180);  % b 점의 y 좌표

% 각도 beta와 DE 길이 계산
beta = atan(l5 / l4);            % 각도 beta 계산
len_de = sqrt(l4^2 + l5^2);      % DE의 길이 계산

% L1 계산
L1 = sqrt((a - l1 * cos(theta)).^2 + (b - l1 * sin(theta)).^2);

% L2 계산
L2 = 1/(2 * l2) * (l3^2 - l1^2 - l2^2 - a^2 - b^2 + 2 * l1 * (a * cos(theta) + b * sin(theta)));

% 각도 alpha 계산 (atan2로 사분면 처리 개선)
alpha = atan2(b - l1 * sin(theta), a - l1 * cos(theta));

% L2와 L1의 비율을 사용해 phi 각도를 계산
ratio = L2 ./ L1;

% phi와 phi_2 배열 미리 할당
phi = zeros(size(theta));   % phi 배열
phi2 = zeros(size(theta));  % phi_2 배열

% 각 theta 값에 대해 phi와 phi_2 계산
for i = 1:length(theta)
    % 비율이 acos 함수 범위를 벗어나면 phi를 NaN으로 설정
    if (ratio(i) > 1) || (ratio(i) < -1)
        phi(i) = NaN;
        phi2(i) = NaN;
    else
        % acos를 사용해 phi와 phi_2 계산
        phi(i) = alpha(i) - acos(ratio(i));  % 첫 번째 가능한 phi 값
        if( phi(i) > pi )
            phi(i) = phi(i) - 2*pi;
        elseif( phi(i) < -pi)
            phi(i) = phi(i) + 2*pi;
        end
        
        % 두 번째 가능한 phi 값 계산
        phi2(i) = alpha(i) + acos(ratio(i));
        if( phi2(i) > pi )
            phi2(i) = phi2(i) - 2*pi;
        elseif( phi2(i) < -pi)
            phi2(i) = phi2(i) + 2*pi;
        end
    end
end

% 선 DC의 x축으로부터의 각도 계산
theta_k = atan2((b+l2*sin(phi)-l1*sin(theta)),(a+l2*cos(phi)-l1*cos(theta)));

% 점 E의 x, y 좌표 계산
E_x = l1 * cos(theta) - l4 * cos(theta_k) + l5 * sin(theta_k);
E_y = l1 * sin(theta) - l4 * sin(theta_k) - l5 * cos(theta_k);

% E_y의 최소 및 최대 값 및 인덱스
[E_y_min, min_idx] = min(E_y);
[E_y_max, max_idx] = max(E_y);

% y_min 및 y_max 범위에서 E_y가 만족하는 인덱스 찾기
idx_gt_low = find(E_y(min_idx:max_idx) > y_min, 1) + min_idx - 1;
idx_gt_up = find(E_y(min_idx:max_idx) > y_max, 1) + min_idx - 1;

% 잘라낸 값들 저장 (E_x, E_y, theta, phi)
E_x_cut = E_x(idx_gt_low:idx_gt_up);
E_y_cut = E_y(idx_gt_low:idx_gt_up);
theta_cut = theta(idx_gt_low:idx_gt_up);
theta_k_cut = theta_k(idx_gt_low:idx_gt_up);
phi_cut = phi(idx_gt_low:idx_gt_up);

% A점이 동력원일 때의 토크 계산
FDx = (len_de * cos(theta_k_cut + beta) * W) ./ ((-tan(phi_cut)+tan(theta_k_cut))*l3.*cos(theta_k_cut)*2);
FDy = -W/2 + tan(phi_cut).*FDx;
tau_A = l1 * (FDy.*cos(theta_cut) - FDx.*sin(theta_cut));
tau_A = tau_A * 1e-3;  % 단위 Nmm에서 Nm로 변환

% B점이 동력원일 때의 토크 계산
FCx = -W ./ (2*(-tan(theta_cut)+tan(theta_k_cut))) .* (1 + len_de / l3 * cos(theta_k_cut + beta) ./ cos(theta_k_cut));
FCy = -W/2 + tan(theta_cut) .* FCx;
tau_B = l2 * (FCy.*cos(phi_cut) - FCx.*sin(phi_cut));
tau_B = tau_B * 1e-3;  % 단위 Nmm에서 Nm로 변환

% 결과를 시각화하는 서브플롯 생성
figure('units','normalized','outerposition',[0 0 1 1]);

% theta와 tau_A의 관계 플롯
subplot(2,4,1);
plot(rad2deg(theta_cut), tau_A, 'r' ,'LineWidth',2);
title("theta vs tau_A");
xlabel('\theta (deg)');
ylabel('torque (Nm)');
grid on;
legend('\tau_A');

% phi와 tau_B의 관계 플롯
subplot(2,4,2);
plot(rad2deg(phi_cut), tau_B, 'b','LineWidth',2);
title("phi vs tau_B");
xlabel('\phi (deg)');
ylabel('torque (Nm)');
grid on;
legend('\tau_B');

% E_y와 토크의 관계 플롯
subplot(2,4,3);
plot(E_y_cut, tau_A, 'r', 'LineWidth',2);
hold on;
plot(E_y_cut, tau_B, 'b', 'LineWidth',2);
title("E_y vs torque");
xlabel('E_y (mm)');
ylabel('torque (Nm)');
grid on;
legend('\tau_A', '\tau_B');

% theta와 phi의 관계 플롯
subplot(2,4,4);
plot(rad2deg(theta), rad2deg(phi),'r','LineWidth',2);
hold on;
plot(rad2deg(theta), rad2deg(phi2),'r','LineWidth',2)
title("theta vs phi");
xlabel('\theta (deg)');
ylabel('\phi (deg)');
axis equal;
grid on;

% E 점의 궤적 플롯
subplot(2,4,5);
plot(E_x_cut,E_y_cut,'r','LineWidth',2);
title("Trajectory of the point E");
xlabel('x (mm)');
ylabel('y (mm)');
axis equal;
grid on;
grid minor;
% legend('E');
xlim([-120, 120]);
ylim([-250, 0]);

% theta와 E_y의 관계 플롯
subplot(2,4,6);
plot(rad2deg(theta_cut), E_y_cut, 'r' ,'LineWidth',2);
title("theta vs E_y");
xlabel('\theta (deg)');
ylabel('y (mm)');
grid on;

% phi와 E_y의 관계 플롯
subplot(2,4,7);
plot(rad2deg(phi_cut), E_y_cut, 'b' ,'LineWidth',2);
title("phi vs E_y");
xlabel('\phi(deg)');
ylabel('y (mm)');
grid on;

figure();
% E 점의 궤적 플롯
plot(E_x_cut,E_y_cut,'r','LineWidth',3);
title("Trajectory of the point E");
xlabel('x (mm)');
ylabel('y (mm)');
axis equal;
grid on;
grid minor;
% legend('E');
xlim([-120, 120]);
ylim([-250, 0]);
