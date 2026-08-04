function [x_pred, f, F] = predict_state(x, u, M, dM_dtheta, nle, dnle_dtheta, dnle_dqdot, B,  dt)
f = [x(2); M \ (-nle + B * u)];
x_pred = x + f * dt;

M_inv = M \ eye(3,3);
F = zeros(4,4);
F(1,2) = 1;
% F(2:4,1) = -M_inv * (dM_dtheta * M_inv * (-nle + B * u) + dnle_dtheta);
F(2:4,1) = M_inv * (dM_dtheta * M_inv * (nle - B * u) - dnle_dtheta);
F(2:4, 2:4) = -M_inv*dnle_dqdot;
end