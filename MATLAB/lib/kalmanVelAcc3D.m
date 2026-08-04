function [velocity, acceleration] = kalmanVelAcc3D(position, time)
% KALMANVELACC3D Estimate velocity and acceleration from 3D position data using Kalman Filter
%
% Inputs:
%   position - Measured 3D position data (Nx3 matrix: [x, y, z])
%   time     - Time stamps corresponding to position data (Nx1 vector)
%
% Outputs:
%   velocity     - Estimated 3D velocity (Nx3 matrix: [vx, vy, vz])
%   acceleration - Estimated 3D acceleration (Nx3 matrix: [ax, ay, az])

% Validate input
if size(position, 1) ~= 3
    error('Position data must have three columns (x, y, z).');
end
if size(position, 2) ~= length(time)
    error('Position and time must have the same number of rows.');
end

% Time step calculation
dt = diff(time);
if any(dt <= 0)
    error('Time stamps must be monotonically increasing.');
end
dt = mean(dt);

% Initial state vector: [pos_x, vel_x, acc_x, pos_y, vel_y, acc_y, pos_z, vel_z, acc_z]
n = size(position, 2);
x = [position(:, 1); zeros(6, 1)]; % [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]

% State transition matrix
F = [1, 0, 0, dt, 0, 0, 0.5*dt^2, 0, 0; ...
     0, 1, 0, 0, dt, 0, 0, 0.5*dt^2, 0; ...
     0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt^2; ...
     0, 0, 0, 1, 0, 0, dt, 0, 0;...
     0, 0, 0, 0, 1, 0, 0, dt, 0;...
     0, 0, 0, 0, 0, 1, 0, 0, dt;...
     0, 0, 0, 0, 0, 0, 1, 0, 0;...
     0, 0, 0, 0, 0, 0, 0, 1, 0;...
     0, 0, 0, 0, 0, 0, 0, 0, 1];

% Observation matrix (only position is measured)
H = [1, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 1, 0, 0, 0, 0, 0, 0, 0;...
     0, 0, 1, 0, 0, 0, 0, 0, 0];

% Process noise covariance
q_pos = 1e-4;
q_vel = 1e-3;
q_acc = 1e-2;
Q = blkdiag([q_pos 0 0; 0 q_pos 0; 0 0 q_pos], ...
            [q_vel 0 0; 0 q_vel 0; 0 0 q_vel], ...
            [q_acc 0 0; 0 q_acc 0; 0 0 q_acc]);

% Measurement noise covariance
r = 1e-6;
R = blkdiag(r, r, r);

% Initial estimation covariance
P = eye(9);

% Allocate space for results
velocity = zeros(3, n);
acceleration = zeros(3, n);

% Kalman Filter loop
for i = 1:n
    % Prediction step
    if i > 1
        x = F * x;
        P = F * P * F' + Q;
    end
    
    % Measurement update step
    z = position(:, i); % Measured position [x, y, z]'
    y = z - H * x; % Innovation
    S = H * P * H' + R; % Innovation covariance
    K = P * H' / S; % Kalman gain
    x = x + K * y; % State update
    P = (eye(9) - K * H) * P; % Covariance update
    
    % Store results
    velocity(:, i) = x(4:6);       % Extract velocity [vx, vy, vz]
    acceleration(:, i) = x(7:9);   % Extract acceleration [ax, ay, az]
end
end
