function [theta_As, theta_Bs, theta_ks, c_poses, e_poses] = solve_forward_kinematics(theta_hips, properties)
% theta_hips = [theta_hipR; theta_hipL]

% Extract link length
a = properties.a;
b = properties.b;
l1 = properties.l1;
l2 = properties.l2;
l3 = properties.l3;
l4 = properties.l4;
l5 = properties.l5;

% 계산을 위해 두 hip 각도를 사용
theta_Bs = [-theta_hips(1); theta_hips(2)]; % BR, BL 각도

% L1, L2 계산
L1 = sqrt((a + l2 * cos(theta_Bs)).^2 + (b + l2 * sin(theta_Bs)).^2); % 거리 계산
L2 = (a^2 + b^2 + l1^2 + l2^2 - l3^2 + 2*l2.*(a*cos(theta_Bs) + b*sin(theta_Bs))) / (2 * l1);

% Ratio 확인 및 예외 처리
ratio = L2 ./ L1;
if any(abs(ratio) > 1)
    error('Ratio exceeds valid range. Check input parameters or theta_hips.');
end

% 각도 계산
alpha = atan2(b + l2 * sin(theta_Bs), a + l2 * cos(theta_Bs)); % atan2로 정확한 사분면 계산
theta_As = -acos(ratio) + alpha; % 각도 A 계산

% 무릎 각도 계산
theta_ks = atan2((b + l2 * sin(theta_Bs) - l1 * sin(theta_As)), ...
    (a + l2 * cos(theta_Bs) - l1 * cos(theta_As))); % 무릎 각도

% C 위치 계산
c_poses = [-a - l2 * cos(theta_Bs), b + l2 * sin(theta_Bs)]'; % 좌표 계산

e_poses = [-l1 * cos(theta_As) + l4 * cos(theta_ks) + l5 * sin(theta_ks), ...
    l1 * sin(theta_As) - l4 * sin(theta_ks) + l5 * cos(theta_ks)]';

end