function H = jacobianMeasurement(x_pred)
% 비선형 측정 방정식의 Jacobian 계산 (예시)
H = [1, 0, 0, 0;     % 상태 변수에 대한 편미분
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 0];
end