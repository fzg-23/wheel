function R = rotation_matrix_y(theta)
% 旋转角度 theta 的 y 轴旋转矩阵
% Theta 以弧度输入。

R = [cos(theta), 0, sin(theta);
    0, 1, 0;
    -sin(theta), 0, cos(theta)];
end