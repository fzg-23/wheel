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

% 使用两个髋部角度进行计算
theta_Bs = [-theta_hips(1); theta_hips(2)]; % BR、BL角

% L1、L2计算
L1 = sqrt((a + l2 * cos(theta_Bs)).^2 + (b + l2 * sin(theta_Bs)).^2); % 距离计算
L2 = (a^2 + b^2 + l1^2 + l2^2 - l3^2 + 2*l2.*(a*cos(theta_Bs) + b*sin(theta_Bs))) / (2 * l1);

% 比率检查和异常处理
ratio = L2 ./ L1;
if any(abs(ratio) > 1)
    error('Ratio exceeds valid range. Check input parameters or theta_hips.');
end

% 角度计算
alpha = atan2(b + l2 * sin(theta_Bs), a + l2 * cos(theta_Bs)); % 使用atan2进行精确的象限计算
theta_As = -acos(ratio) + alpha; % 计算角度 A

% 膝盖角度计算
theta_ks = atan2((b + l2 * sin(theta_Bs) - l1 * sin(theta_As)), ...
    (a + l2 * cos(theta_Bs) - l1 * cos(theta_As))); % 膝盖角度

% C位计算
c_poses = [-a - l2 * cos(theta_Bs), b + l2 * sin(theta_Bs)]'; % 坐标计算

e_poses = [-l1 * cos(theta_As) + l4 * cos(theta_ks) + l5 * sin(theta_ks), ...
    l1 * sin(theta_As) - l4 * sin(theta_ks) + l5 * cos(theta_ks)]';

end