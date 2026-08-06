function [h_obs, H] = predict_measurement(f, x_pred, g, h, L, R)
% 预先计算的 sin 和 cos
theta_ddot = f(2);
v_dot = f(3);
psi_ddot = f(4);
% theta_ddot = 0;
% v_dot = 0;
% psi_ddot = 0;



theta = x_pred(1);
theta_dot = x_pred(2);
v = x_pred(3);
psi_dot = x_pred(4);


cos_theta = cos(theta);
sin_theta = sin(theta);

% h_obs计算
h_obs = zeros(8, 1); % 用于存储结果的向量
h_obs(1) = h * theta_ddot + v_dot * cos_theta - g * sin_theta - h * psi_dot^2 * cos_theta * sin_theta;
h_obs(2) = psi_dot * v + h * psi_ddot * sin_theta + h * psi_dot * theta_dot * cos_theta * 2.0;
h_obs(3) = -h * theta_dot^2 + g * cos_theta + v_dot * sin_theta - psi_dot^2 * (h - h * cos_theta^2);
h_obs(4) = -psi_dot * sin_theta;
h_obs(5) = theta_dot;
h_obs(6) = psi_dot * cos_theta;
h_obs(7) = theta_dot - v / R - (L * psi_dot) / R;
h_obs(8) = -theta_dot + v / R - (L * psi_dot) / R;

% H矩阵计算
H = zeros(8, 4); % 8x4矩阵
H(1, 1) = psi_dot^2 * (h * sin_theta^2 - h * cos_theta^2) - g * cos_theta - v_dot * sin_theta;
H(2, 1) = h * psi_ddot * cos_theta - h * psi_dot * theta_dot * sin_theta * 2.0;
H(3, 1) = v_dot * cos_theta - g * sin_theta - h * psi_dot^2 * cos_theta * sin_theta * 2.0;
H(4, 1) = -psi_dot * cos_theta;
H(5, 1) = 0.0;
H(6, 1) = -psi_dot * sin_theta;
H(7, 1) = 0.0;
H(8, 1) = 0.0;

H(1, 2) = 0.0;
H(2, 2) = h * psi_dot * cos_theta * 2.0;
H(3, 2) = h * theta_dot * -2.0;
H(4, 2) = 0.0;
H(5, 2) = 1.0;
H(6, 2) = 0.0;
H(7, 2) = 1.0;
H(8, 2) = -1.0;

H(1, 3) = 0.0;
H(2, 3) = psi_dot;
H(3, 3) = 0.0;
H(4, 3) = 0.0;
H(5, 3) = 0.0;
H(6, 3) = 0.0;
H(7, 3) = -1.0 / R;
H(8, 3) = 1.0 / R;

H(1, 4) = h * psi_dot * cos_theta * sin_theta * -2.0;
H(2, 4) = v + h * theta_dot * cos_theta * 2.0;
H(3, 4) = psi_dot * (h - h * cos_theta^2) * -2.0;
H(4, 4) = -sin_theta;
H(5, 4) = 0.0;
H(6, 4) = cos_theta;
H(7, 4) = -L / R;
H(8, 4) = -L / R;
end