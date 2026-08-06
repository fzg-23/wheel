function [x_pred, F] = statePrediction(x, u, dt)
% 非线性状态方程（示例）
f = @(x, u) [x(2) + dt * x(3);
    -sin(x(1)) + u(1);  %动态示例
    cos(x(1)) * u(2);   %动态示例
    x(4)];

% 状态预测
x_pred = f(x, u);

% 雅可比矩阵计算（线性化）
F = jacobianState(x, u, dt); % 计算雅可比矩阵的函数
end