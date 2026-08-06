function F = jacobianState(x, u, dt)
% 状态方程的雅可比计算
% 非线性状态方程的雅可比计算（示例）
F = [1, dt, 0, 0;     % Partial differentiation of state variables
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
end