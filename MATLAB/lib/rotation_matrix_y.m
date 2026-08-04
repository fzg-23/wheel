function R = rotation_matrix_y(theta)
% 회전 각도 theta에 대한 y-axis 회전 행렬
% theta는 라디안 단위로 입력

R = [cos(theta), 0, sin(theta);
    0, 1, 0;
    -sin(theta), 0, cos(theta)];
end