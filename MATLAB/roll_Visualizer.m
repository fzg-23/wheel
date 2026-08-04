clc; clear; close;

syms L h phi theta R rho real
syms theta_dot_LW theta_dot_RW real

hL = h + L * tan(phi);
hR = h - L * tan(phi);
r = (R - rho) / sqrt(1 + cos(theta)^2 * tan(phi)^2);
u_hat = [0; L; -L * tan(phi)] / (L * sec(phi));
R_u_theta = angleAxisToRotationMatrix(-theta, u_hat);

R_cB = angleAxisToRotationMatrix(theta, [0;1;0]) ...
    * angleAxisToRotationMatrix(phi, [1;0;0]);

proj_xy = [R_cB(1:2,2);0];
theta_c = atan2(R_cB(3,2), norm(proj_xy));
z_c = -(R-rho)*cos(theta_c)-rho;
xy_c = (R-rho)*sin(theta_c)*proj_xy/norm(proj_xy);

wheel_offset_c = simplify(xy_c + [0;0;z_c]);
wheel_offset = simplify(R_cB'*wheel_offset_c);

% wheel_offset = u_hat * r * cos(theta) * tan(phi) ...
%     + R_u_theta * (r + rho) * [0; -L * tan(phi); -L] / (L * sec(phi));
p_LW = [0; L; -hL] + wheel_offset;
p_RW = [0; -L; -hR] + wheel_offset;

p_LW = simplify(p_LW);
p_RW = simplify(p_RW);


f_r = matlabFunction(r);
f_LW = matlabFunction(p_LW)
f_RW = matlabFunction(p_RW)

R_phi = angleAxisToRotationMatrix(phi, [1;0;0]);
R_theta = angleAxisToRotationMatrix(theta, [0;1;0]);

R_body = R_theta * R_phi;
angle = atan2(R_body(2,2),R_body(1,2)) - pi/2;
v_hat = simplify([cos(angle);sin(angle);0]);


v_B_LW = cross([0;theta_dot_LW;0], wheel_offset);
v_c_LW = R_theta*R_phi*v_B_LW;
f_v_B_LW = matlabFunction(v_B_LW)
f_v_c_LW = matlabFunction(v_c_LW)

v_B_RW = cross([0;-theta_dot_RW;0], wheel_offset);
v_c_RW = R_theta*R_phi*v_B_RW;
f_v_B_RW = matlabFunction(v_B_RW);
f_v_c_RW = matlabFunction(v_c_RW);

f_vhat = matlabFunction(v_hat);

L = 123;
R = 75;
rho = 15;
h = 130;
theta_dot_LW = -1;
theta_dot_RW = 1;
% phi_range = linspace(-pi/12, pi/12, 10); % phi의 범위를 10개 구간으로 나눔
phi_range = linspace(pi/4, pi/4, 10); % phi의 범위를 10개 구간으로 나눔

% theta = linspace(-pi/2, pi/2, 50); % theta의 범위
theta = linspace(-pi/2, pi/2, 50); % theta의 범위


% 축 범위
x_lim = [-200, 200];
y_lim = [-300, 300];
z_lim = [-300, 300];

% initialize
hL = h + L * tan(phi_range(1));
hR = h - L * tan(phi_range(1));
body_center = [0; 0; 0];
LW_center = [0; L; -hL];
RW_center = [0; -L; -hR];
W_center = [0; 0; -h];


% 초기 플롯
figure;
ax1 = subplot(1, 2, 1); % 기존 뷰
ax2 = subplot(1, 2, 2); % c-frame 뷰

% 기존 뷰 플롯 설정
LW_line = plot3(ax1, NaN, NaN, NaN, 'r-', 'LineWidth', 3); hold(ax1, "on");
RW_line = plot3(ax1, NaN, NaN, NaN, 'r-', 'LineWidth', 3);
body_line = plot3(ax1, NaN, NaN, NaN, 'k-', 'LineWidth', 4);
W_line = plot3(ax1, NaN, NaN, NaN, 'k-', 'LineWidth', 4);

body_center_point = plot3(ax1, NaN, NaN, NaN, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
LW_center_point = plot3(ax1, NaN, NaN, NaN,  'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
RW_center_point = plot3(ax1, NaN, NaN, NaN,  'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
W_center_point = plot3(ax1, NaN, NaN, NaN,  'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
LW = draw_wheel(ax1, LW_center, [0; 1; 0], R - rho, rho, 1e2, 0.3);
RW = draw_wheel(ax1, RW_center, [0; 1; 0], R - rho, rho, 1e2, 0.3);
v_dLW = quiver3(ax1, NaN, NaN, NaN, NaN, NaN, NaN, 'b', 'LineWidth', 4);
v_dRW = quiver3(ax1, NaN, NaN, NaN, NaN, NaN, NaN, 'b', 'LineWidth', 4);


% x, y, z 축 강조를 위한 벡터 그리기
quiver3(ax1, 0, 0, 0, x_lim(2), 0, 0, 0, 'r', 'LineWidth', 2); % x축 강조 (빨간색)
quiver3(ax1, 0, 0, 0, 0, y_lim(2), 0, 0, 'g', 'LineWidth', 2); % y축 강조 (초록색)
quiver3(ax1, 0, 0, 0, 0, 0, z_lim(2), 0, 'b', 'LineWidth', 2); % z축 강조 (파란색)


% 축 설정 (기존 뷰)
axis(ax1, 'vis3d');
grid(ax1, 'on');
xlabel(ax1, 'X'); ylabel(ax1, 'Y'); zlabel(ax1, 'Z');
title(ax1, 'Original Frame');
axis(ax1, 'equal'); % Add this line
view(ax1, [1,1,1]);
xlim(ax1, x_lim); ylim(ax1, y_lim); zlim(ax1, z_lim);

% c-frame 뷰 플롯 설정
LW_line_c = plot3(ax2, NaN, NaN, NaN, 'r-', 'LineWidth', 3); hold(ax2, "on");
RW_line_c = plot3(ax2, NaN, NaN, NaN, 'r-', 'LineWidth', 3);
body_line_c = plot3(ax2, NaN, NaN, NaN, 'k-', 'LineWidth', 4);
W_line_c = plot3(ax2, NaN, NaN, NaN, 'k-', 'LineWidth', 4);
body_center_trajectory_c = plot3(ax2, NaN, NaN, NaN, 'k:', 'LineWidth', 3);

body_center_point_c = plot3(ax2, NaN, NaN, NaN,  'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
LW_center_point_c = plot3(ax2, NaN, NaN, NaN,  'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
RW_center_point_c = plot3(ax2, NaN, NaN, NaN,  'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
LW_c = draw_wheel(ax2, LW_center, [0; 1; 0], R - rho, rho, 1e2, 0.3);
RW_c = draw_wheel(ax2, RW_center, [0; 1; 0], R - rho, rho, 1e2, 0.3);
W_center_point_c = plot3(ax2, NaN, NaN, NaN,  'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
v_hat_c = quiver3(ax2, 0, 0, 0, 100, 0, 0, 'r', 'LineWidth', 2);
v_dLW_c = quiver3(ax2, NaN, NaN, NaN, NaN, NaN, NaN, 'b', 'LineWidth', 4);
v_dRW_c = quiver3(ax2, NaN, NaN, NaN, NaN, NaN, NaN, 'b', 'LineWidth', 4);


% x, y, z 축 강조 (굵은 선으로 표시)
% x, y, z 축 강조를 위한 벡터 그리기
quiver3(ax2, 0, 0, 0, x_lim(2), 0, 0, 0, 'r', 'LineWidth', 2); % x축 강조 (빨간색)
quiver3(ax2, 0, 0, 0, 0, y_lim(2), 0, 0, 'g', 'LineWidth', 2); % y축 강조 (초록색)
quiver3(ax2, 0, 0, 0, 0, 0, z_lim(2)+h, 0, 'b', 'LineWidth', 2); % z축 강조 (파란색) -> z축 h만큼 이동


% 축 설정 (c-frame 뷰)
axis(ax2, 'vis3d');
grid(ax2, 'on');
xlabel(ax2, 'X'); ylabel(ax2, 'Y'); zlabel(ax2, 'Z');
title(ax2, 'C-Frame (Shifted and Rotated)');
axis(ax2, 'equal'); % Add this line
view(ax2, [1,1,1]);
xlim(ax2, x_lim); ylim(ax2, y_lim); zlim(ax2, z_lim + h);

% 무한히 반복하도록 설정
while true
    for i = 1:length(phi_range)
        % 기존 뷰 계산
        hL = h + L * tan(phi_range(i));
        hR = h - L * tan(phi_range(i));
        LW_center = [0; L; -hL];
        RW_center = [0; -L; -hR];

        LW_trajectory = f_LW(L, R, h, phi_range(i), rho, theta);
        RW_trajectory = f_RW(L, R, h, phi_range(i), rho, theta);
        body_line_trajectory = [body_center, W_center];
        W_line_trajectory = [LW_center, RW_center];
        update_wheel(LW, LW_center, [0; 1; 0], R - rho, rho, 20);
        update_wheel(RW, RW_center, [0; 1; 0], R - rho, rho, 20);

        set(LW_line, 'XData', LW_trajectory(1, :), ...
            'YData', LW_trajectory(2, :), ...
            'ZData', LW_trajectory(3, :));
        set(RW_line, 'XData', RW_trajectory(1, :), ...
            'YData', RW_trajectory(2, :), ...
            'ZData', RW_trajectory(3, :));
        set(body_line, 'XData', body_line_trajectory(1, :), ...
            'YData', body_line_trajectory(2, :), ...
            'ZData', body_line_trajectory(3, :));
        set(W_line, 'XData', W_line_trajectory(1, :), ...
            'YData', W_line_trajectory(2, :), ...
            'ZData', W_line_trajectory(3, :));
        
        set(body_center_point, 'XData', body_center(1), ...
            'YData', body_center(2), ...
            'ZData', body_center(3));
        set(LW_center_point, 'XData', LW_center(1), ...
            'YData', LW_center(2), ...
            'ZData', LW_center(3));
        set(RW_center_point, 'XData', RW_center(1), ...
            'YData', RW_center(2), ...
            'ZData', RW_center(3));
        set(W_center_point, 'XData', W_center(1), ...
            'YData', W_center(2), ...
            'ZData', W_center(3));

        % c-frame 계산
        % theta의 각도 수와 총 시간을 계산
        theta_steps = length(theta); % theta의 분할 개수
        total_time = 5; % 루프가 1초 걸리게 설정
        time_per_step = total_time / theta_steps; % 각 단계의 목표 시간

        % body_center_trajectory 초기화
        c_body_center_trajectory = NaN(3, theta_steps); % theta에 대한 body_center 경로

        for j = 1:theta_steps
            tic; % 현재 시간을 측정

            p_B_dLW = LW_trajectory(:,j);
            p_B_dRW = RW_trajectory(:,j);

            check_torus(p_B_dLW, LW_center, R-rho, rho, 1e-9)
            check_torus(p_B_dRW, RW_center, R-rho, rho, 1e-9)
            

            v_B_dLW = f_v_B_LW(R, phi_range(i), rho, theta(j), theta_dot_LW);
            v_B_dRW = f_v_B_RW(R, phi_range(i), rho, theta(j), theta_dot_RW);



            % rotate roll and translate +h
            c_body_center = rotate_roll_and_translate(body_center, phi_range(i), h);
            c_LW_center = rotate_roll_and_translate(LW_center, phi_range(i), h);
            c_RW_center = rotate_roll_and_translate(RW_center, phi_range(i), h);
            c_W_center = rotate_roll_and_translate(W_center, phi_range(i), h);
            c_LW_trajectory = rotate_roll_and_translate(LW_trajectory, phi_range(i), h);
            c_RW_trajectory = rotate_roll_and_translate(RW_trajectory, phi_range(i), h);

            % rotate pitch
            c_body_center = rotate_pitch_and_translate(c_body_center, theta(j), 0);
            c_LW_center = rotate_pitch_and_translate(c_LW_center , theta(j), 0);
            c_RW_center = rotate_pitch_and_translate(c_RW_center, theta(j), 0);
            c_W_center = rotate_roll_and_translate(c_W_center, theta(j), 0);
            c_LW_trajectory = rotate_pitch_and_translate(c_LW_trajectory, theta(j), 0);
            c_RW_trajectory = rotate_pitch_and_translate(c_RW_trajectory, theta(j), 0);

            % elevate wheel h
            wheel_h = f_r(R, phi_range(i), rho, theta(j)) + rho;
            c_body_center = c_body_center + [0;0;wheel_h];
            c_LW_center = c_LW_center + [0;0;wheel_h];
            c_RW_center = c_RW_center + [0;0;wheel_h];
            c_W_center = c_W_center + [0;0;wheel_h];
            c_LW_trajectory = c_LW_trajectory + [0;0;wheel_h];
            c_RW_trajectory = c_RW_trajectory + [0;0;wheel_h];

            c_p_c_dLW = c_LW_trajectory(:,j);
            c_p_c_dRW = c_RW_trajectory(:,j);

            % c_v_c_dLW = f_v_c_LW(L,R,phi_range(i),rho, theta(j), theta_dot_LW);
            % c_v_c_dRW = f_v_c_RW(L,R,phi_range(i),rho, theta(j), theta_dot_RW);
            c_v_c_dLW = rotate_roll_and_translate(v_B_dLW, phi_range(i), 0);
            c_v_c_dLW = rotate_pitch_and_translate(c_v_c_dLW, theta(j), 0)
            c_v_c_dRW = rotate_roll_and_translate(v_B_dRW, phi_range(i), 0);
            c_v_c_dRW = rotate_pitch_and_translate(c_v_c_dRW, theta(j), 0)



            % 저장된 c_body_center 위치를 c-body_center_trajectory에 기록
            c_body_center_trajectory(:, j) = c_body_center;

            c_wheel_axis = angleAxisToRotationMatrix(theta(j), [0;1;0])...
                * angleAxisToRotationMatrix(phi_range(i), [1;0;0]) * [0;1;0];
            update_wheel(LW_c, c_LW_center, c_wheel_axis, R - rho, rho, 20);
            update_wheel(RW_c, c_RW_center, c_wheel_axis, R - rho, rho, 20);

            c_body_line_trajectory = [c_body_center, c_W_center];
            c_W_line_trajectory = [c_LW_center, c_RW_center];

            c_v_hat = f_vhat(phi_range(i), theta(j));

            set(LW_line_c, 'XData', c_LW_trajectory(1, :), ...
                'YData', c_LW_trajectory(2, :), ...
                'ZData', c_LW_trajectory(3, :));
            set(RW_line_c, 'XData', c_RW_trajectory(1, :), ...
                'YData', c_RW_trajectory(2, :), ...
                'ZData', c_RW_trajectory(3, :));
            set(body_line_c, 'XData', c_body_line_trajectory(1, :), ...
                'YData', c_body_line_trajectory(2, :), ...
                'ZData', c_body_line_trajectory(3, :));
            set(W_line_c, 'XData', c_W_line_trajectory(1, :), ...
                'YData', c_W_line_trajectory(2, :), ...
                'ZData', c_W_line_trajectory(3, :));
            set(body_center_trajectory_c, 'XData', c_body_center_trajectory(1, :), ...
                'YData', c_body_center_trajectory(2, :), ...
                'ZData', c_body_center_trajectory(3, :));
            


            set(body_center_point_c, 'XData', c_body_center(1), ...
                'YData', c_body_center(2), ...
                'ZData', c_body_center(3));
            set(LW_center_point_c, 'XData', c_LW_center(1), ...
                'YData', c_LW_center(2), ...
                'ZData', c_LW_center(3));
            set(RW_center_point_c, 'XData', c_RW_center(1), ...
                'YData', c_RW_center(2), ...
                'ZData', c_RW_center(3));
            set(W_center_point_c, 'XData', c_W_center(1), ...
                'YData', c_W_center(2), ...
                'ZData', c_W_center(3));

            set(v_hat_c, 'Udata', 100 * c_v_hat(1), ...
                'Vdata', 100 * c_v_hat(2),...
                'WData', 100 * c_v_hat(3));

            set(v_dLW, 'XData', p_B_dLW(1), ...
                'YData', p_B_dLW(2), ...
                'ZData', p_B_dLW(3), ...
                'Udata', v_B_dLW(1), ...
                'Vdata', v_B_dLW(2),...
                'WData', v_B_dLW(3));
            set(v_dRW, 'XData', p_B_dRW(1), ...
                'YData', p_B_dRW(2), ...
                'ZData', p_B_dRW(3), ...
                'Udata', v_B_dRW(1), ...
                'Vdata', v_B_dRW(2),...
                'WData', v_B_dRW(3));
            set(v_dLW_c, 'XData', c_p_c_dLW(1), ...
                'YData', c_p_c_dLW(2), ...
                'ZData', c_p_c_dLW(3), ...
                'Udata', c_v_c_dLW(1), ...
                'Vdata', c_v_c_dLW(2),...
                'WData', c_v_c_dLW(3));
            set(v_dRW_c, 'XData', c_p_c_dRW(1), ...
                'YData', c_p_c_dRW(2), ...
                'ZData', c_p_c_dRW(3), ...
                'Udata', c_v_c_dRW(1), ...
                'Vdata', c_v_c_dRW(2),...
                'WData', c_v_c_dRW(3));

            drawnow limitrate;

            % 남은 시간을 계산하여 pause 실행
            elapsed_time = toc; % 루프 실행 시간 측정
            remaining_time = max(0, time_per_step - elapsed_time); % 남은 시간 계산
            pause(remaining_time); % 지연 시간 적용
        end

    end
    phi_range = flip(phi_range);
end

function rotated_trajectory = rotate_roll_and_translate(trajectory, phi, h)
    R = angleAxisToRotationMatrix(phi, [1; 0; 0]);
    rotated_trajectory = R * (trajectory + [0; 0; h]);
end

function rotated_trajectory = rotate_pitch_and_translate(trajectory, theta, h)
    R = angleAxisToRotationMatrix(theta, [0; 1; 0]);
    rotated_trajectory = R * (trajectory + [0; 0; h]);
end

function R = angleAxisToRotationMatrix(angle, axis)
% angleAxisToRotationMatrix - Convert Angle-Axis to Rotation Matrix
%
% Inputs:
%   angle - Rotation angle in radians
%   axis  - 3D unit vector representing the rotation axis [x, y, z]
%
% Output:
%   R - 3x3 Rotation Matrix

% Normalize the axis to ensure it's a unit vector
axis = axis / norm(axis);
x = axis(1);
y = axis(2);
z = axis(3);

% Compute trigonometric values
c = cos(angle);
s = sin(angle);
one_c = 1 - c;

% Compute the rotation matrix using the Angle-Axis formula
R = [c + x^2 * one_c, x * y * one_c - z * s, x * z * one_c + y * s;
    y * x * one_c + z * s, c + y^2 * one_c, y * z * one_c - x * s;
    z * x * one_c - y * s, z * y * one_c + x * s, c + z^2 * one_c];
end

function surf_obj = draw_wheel(ax, center, axis, R, r, num_points, face_alpha)
% ax: axes 객체
% center: [x, y, z] 형태의 중심점
% axis: [vx, vy, vz] 형태의 축 벡터
% R: 큰 반지름
% r: 작은 반지름
% num_points: 각도를 나눌 샘플링 수 (해상도)
% face_alpha: 투명도 (0 ~ 1)

% 기본 각도 생성
theta = linspace(0, 2*pi, num_points); % 큰 원의 각도
phi = linspace(0, 2*pi, num_points);   % 작은 원의 각도
[Theta, Phi] = meshgrid(theta, phi);

% 기본 토러스 좌표 (정렬되지 않은 상태)
Xc = (R + r * cos(Phi)) .* cos(Theta);
Yc = (R + r * cos(Phi)) .* sin(Theta);
Zc = r * sin(Phi);

% 축 벡터를 정규화
axis = axis / norm(axis);

% 축 벡터의 방향을 기준으로 회전 행렬 계산
% 기본 축은 [0, 0, 1]이라고 가정
z_axis = [0, 0, 1];
if ~all(axis == z_axis) % 축이 기본 축과 다를 경우만 회전 수행
    v = cross(z_axis, axis); % 축 사이의 외적
    s = norm(v);             % 외적의 크기
    c = dot(z_axis, axis);   % 두 벡터의 내적

    % 회전 행렬 생성 (Rodrigues' rotation formula)
    Vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
    R_matrix = eye(3) + Vx + Vx^2 * ((1 - c) / s^2);
else
    R_matrix = eye(3); % 축이 동일하면 회전 필요 없음
end

% 회전 행렬 적용
coords = R_matrix * [Xc(:)'; Yc(:)'; Zc(:)'];
X_rot = reshape(coords(1, :), size(Xc));
Y_rot = reshape(coords(2, :), size(Yc));
Z_rot = reshape(coords(3, :), size(Zc));

% 중심점 이동
X = X_rot + center(1);
Y = Y_rot + center(2);
Z = Z_rot + center(3);

% 토러스 그리기
surf_obj = surf(ax, X, Y, Z, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', face_alpha); % 투명한 표면

% 조명 추가
light('Position', [1, 1, 1], 'Style', 'infinite', 'Parent', ax);
end


function update_wheel(surf_obj, center, axis, R, r, num_points)
    % Update the position and orientation of the wheel
    % surf_obj: Surface object representing the wheel
    % center: New center [x, y, z]
    % axis: Rotation axis [vx, vy, vz]
    % R: Large radius of the wheel
    % r: Small radius of the wheel
    % num_points: Resolution of the toroidal surface

    % Recompute the wheel surface
    theta = linspace(0, 2*pi, num_points); % Large circle
    phi = linspace(0, 2*pi, num_points);   % Small circle
    [Theta, Phi] = meshgrid(theta, phi);

    % Generate torus coordinates
    Xc = (R + r * cos(Phi)) .* cos(Theta);
    Yc = (R + r * cos(Phi)) .* sin(Theta);
    Zc = r * sin(Phi);

    % Normalize the rotation axis
    axis = axis / norm(axis);

    % Calculate rotation matrix (Rodrigues' formula)
    z_axis = [0, 0, 1]; % Default axis of the torus
    if ~all(axis == z_axis)
        v = cross(z_axis, axis); % Cross product
        s = norm(v);             % Magnitude of cross product
        c = dot(z_axis, axis);   % Dot product

        % Construct rotation matrix
        Vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
        R_matrix = eye(3) + Vx + Vx^2 * ((1 - c) / s^2);
    else
        R_matrix = eye(3);
    end

    % Apply rotation
    coords = R_matrix * [Xc(:)'; Yc(:)'; Zc(:)'];
    X_rot = reshape(coords(1, :), size(Xc));
    Y_rot = reshape(coords(2, :), size(Yc));
    Z_rot = reshape(coords(3, :), size(Zc));

    % Translate to the new center
    X = X_rot + center(1);
    Y = Y_rot + center(2);
    Z = Z_rot + center(3);

    % Update the surface object
    set(surf_obj, 'XData', X, 'YData', Y, 'ZData', Z);
end


function is_on_torus = check_torus(point, torus_center, R, rho, epsilon)
    % check_torus_equation: Checks if a point satisfies the torus equation.
    % 
    % Inputs:
    %   point        - 3x1 vector representing the point to check [x; y; z].
    %   torus_center - 3x1 vector representing the center of the torus [x_c; y_c; z_c].
    %   R            - Major radius of the torus.
    %   rho          - Minor radius of the torus.
    %   epsilon      - Small tolerance to account for floating-point errors.
    %
    % Output:
    %   is_on_torus  - Logical value (true if the point lies on the torus surface, false otherwise).

    % Transform point to torus-centered coordinates
    rel_point = point - torus_center;
    x = rel_point(1);
    y = rel_point(2);
    z = rel_point(3);
 
    % Compute torus equation value
    left_side = (sqrt(x^2 + z^2) - R)^2 + y^2
    right_side = rho^2

    % Check if the equation is satisfied within tolerance
    is_on_torus = abs(left_side - right_side) <= epsilon;
end
