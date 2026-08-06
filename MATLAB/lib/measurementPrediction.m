function [H, h] = measurementPrediction(x_pred)
% 非线性测量方程（示例）
h = @(x_pred) [x_pred(1);  % 测量值作为示例
    x_pred(2);  % 测量值作为示例
    x_pred(3);  % 测量值作为示例
    x_pred(4);  % 测量值作为示例
    0;          % 测量
    0;          % 测量
    0;          % 测量
    0];         % 测量

% 测量预测值
h = h(x_pred);

% 雅可比矩阵计算（测量方程的线性化）
H = jacobianMeasurement(x_pred); %计算测量方程的雅可比行列式的函数
end