function [x_pred, F] = statePrediction(x, u, dt)
% 비선형 상태 방정식 (예시)
f = @(x, u) [x(2) + dt * x(3);
    -sin(x(1)) + u(1);  % 예시 동역학
    cos(x(1)) * u(2);   % 예시 동역학
    x(4)];

% 상태 예측
x_pred = f(x, u);

% Jacobian 행렬 계산 (선형화)
F = jacobianState(x, u, dt); % Jacobian 행렬을 계산하는 함수
end