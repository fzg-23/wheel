function H = jacobianMeasurement(x_pred)
% 非线性测量方程的雅可比计算（示例）
H = [1, 0, 0, 0;     % Partial differentiation of state variables
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
end