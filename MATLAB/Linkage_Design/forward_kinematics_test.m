clc; clear all; close all;  % 콘솔, 변수, 그리고 모든 창을 초기화

% 링크의 길이 정의
l1 = 106;  % 첫 번째 링크의 길이 (mm)
l2 = 77;   % 두 번째 링크의 길이 (mm)
l3 = 50;   % 세 번째 링크의 길이 (mm)
l4 = 137;  % 네 번째 링크의 길이 (mm)
l5 = -8;   % 네 번째 링크의 수직방향 offset (mm)

theta_root = 30;  % 루트 각도 (deg)
len_ab = 75;      % 링크 AB의 길이 (mm)

% 계산을 위한 각도 범위 정의
theta_hipR = -29.68*pi/180 : 0.001 : 69.97*pi/180;
theta_BR = -theta_hipR;  % theta 범위 (-90도에서 180도까지)

% (a, b) 점의 좌표 정의
a = len_ab * cos(theta_root * pi / 180);  % a 점의 x 좌표
b = len_ab * sin(theta_root * pi / 180);  % b 점의 y 좌표

% 각도 beta와 DE 길이 계산
beta = atan(l5 / l4);            % 각도 beta 계산
len_de = sqrt(l4^2 + l5^2);      % DE의 길이 계산

% L1 계산
L1 = sqrt((a + l2 * cos(theta_BR)).^2 + (b + l2 * sin(theta_BR)).^2);

% L2 계산
L2 = (a^2 + b^2 + l1^2 + l2^2 - l3^2 + 2*l2*(a*cos(theta_BR) +b*sin(theta_BR)))/(2*l1);

% alpha 계산
alpha = atan2(b+l2*sin(theta_BR), a+l2*cos(theta_BR));
ratio = L2 ./ L1;

theta_AR = zeros(size(theta_BR));   % phi 배열
theta_AR2 = zeros(size(theta_BR));  % phi_2 배열

% 각 theta 값에 대해 phi와 phi_2 계산
for i = 1:length(theta_BR)
    % 비율이 acos 함수 범위를 벗어나면 phi를 NaN으로 설정
    if (ratio(i) > 1) || (ratio(i) < -1)
        theta_AR(i) = NaN;
        theta_AR2(i) = NaN;
    else
        % 0~pi의 acos에 대한 결과
        theta_AR(i) = acos(ratio(i)) + alpha(i);  % 첫 번째 가능한 phi 값
        if( theta_AR(i) > pi )
            theta_AR(i) = theta_AR(i) - 2*pi;
        elseif( theta_AR(i) < -pi)
            theta_AR(i) = theta_AR(i) + 2*pi;
        end
        
        % -pi~0의 acos에 대한 결과
        theta_AR2(i) = -acos(ratio(i)) + alpha(i);
        if( theta_AR2(i) > pi )
            theta_AR2(i) = theta_AR2(i) - 2*pi;
            fprintf("pi보다 큼");
        elseif( theta_AR2(i) < -pi)
            theta_AR2(i) = theta_AR2(i) + 2*pi;
            fprintf("-pi보다 작음");
        end
    end
end

% 각 경우에 대한 theta_k
theta_kR = atan2((b+l2*sin(theta_BR)-l1*sin(theta_AR)),(a+l2*cos(theta_BR)-l1*cos(theta_AR)));
theta_kR2 = atan2((b+l2*sin(theta_BR)-l1*sin(theta_AR2)),(a+l2*cos(theta_BR)-l1*cos(theta_AR2)));


% 결과를 시각화하는 서브플롯 생성
figure('units','normalized','outerposition',[0 0.25 1 0.5]);

% 첫 번째 서브플롯
subplot(1,3,1);
plot(rad2deg(theta_BR), rad2deg(theta_AR),'r','LineWidth',2);
hold on;
plot(rad2deg(theta_BR), rad2deg(theta_AR2),'b','LineWidth',2)
title("\theta_{BR} vs \theta_{AR}");
xlabel('\theta_{BR} (deg)');
ylabel('\theta_{AR} (deg)');
axis equal;
grid on;
legend('\theta_{AR}', '\theta_{AR2}'); % Legend 추가

% 두 번째 서브플롯
subplot(1,3,2);
plot(rad2deg(theta_BR), rad2deg(theta_kR),'r','LineWidth',2);
hold on;
plot(rad2deg(theta_BR), rad2deg(theta_kR2),'b','LineWidth',2)
title("\theta_{BR} vs \theta_{kR}");
xlabel('\theta_{BR} (deg)');
ylabel('\theta_{kR} (deg)');
axis equal;
grid on;
legend('\theta_{kR}', '\theta_{kR2}'); % Legend 추가

% 세 번째 서브플롯
subplot(1,3,3);
plot(rad2deg(theta_BR), rad2deg(alpha),'r','LineWidth',2);
hold on;
plot(rad2deg(theta_BR), rad2deg(acos(ratio)),'b','LineWidth',2);
title("\alpha, acos");
xlabel('\theta_{BR} (deg)');
ylabel('(deg)');
axis equal;
grid on;
legend('\alpha', 'acos(L2/L1)'); % Legend 추가
