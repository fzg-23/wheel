function F = jacobianState(x, u, dt)
% 상태 방정식에 대한 Jacobian 계산
% 비선형 상태 방정식의 Jacobian 계산 (예시)
F = [1, dt, 0, 0;     % 상태 변수에 대한 편미분
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
end