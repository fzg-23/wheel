function [H, h] = measurementPrediction(x_pred)
% 비선형 측정 방정식 (예시)
h = @(x_pred) [x_pred(1);  % 예시로 측정하는 값
    x_pred(2);  % 예시로 측정하는 값
    x_pred(3);  % 예시로 측정하는 값
    x_pred(4);  % 예시로 측정하는 값
    0;          % 측정값
    0;          % 측정값
    0;          % 측정값
    0];         % 측정값

% 측정 예측값
h = h(x_pred);

% Jacobian 행렬 계산 (측정 방정식 선형화)
H = jacobianMeasurement(x_pred); % 측정 방정식에 대한 Jacobian을 계산하는 함수
end