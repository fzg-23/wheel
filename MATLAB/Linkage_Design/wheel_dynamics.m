clc; clear all; close all;

% Define Parameters

mw = 0.1; % mass of wheel (kg)
mb = 1.9; % mass of body (kg)

R = 0.10; % radius of Wheel (m)
l = 0.25; % length from CoM to wheel (m)

Iw = mw*R^2; % moment of Inertia of wheel (kgm^2)
Ib = 1/12 * mb *(0.1^2 + 0.2^2); % moment of Inertia of body (kgm^2)

theta_p_var = -pi/4:0.01:pi/4; % pitch angle of body (rad)
theta_p_dot = 0; % pitch angular velocity of body (rad/s)
theta_p_ddot = 0; % pitch angular acceleration of body (rad/s^2)

g = 9.81; % gravity acceleration (m/s^2)

x_acc = zeros(length(theta_p_var),1); 
tau = zeros(length(theta_p_var),1); % desired torque

i = 1;
for theta_p = theta_p_var
    % Mass Matrix (theta_w version)
    M = [Iw/R + mw*R + mb*R,         mb*l*cos(theta_p);
        (Iw/R+mw*R)*l*cos(theta_p), -mb*l^2*sin(theta_p)-Ib];

    % Mass Matrix (theta_m version)
    % M = [Iw/R + mw*R + mb*R,         Iw/R + mw*R + mb*R+ mb*l*cos(theta_p);
    %     (Iw/R+mw*R)*l*cos(theta_p), (Iw/R+mw*R)*l*cos(theta_p)- mb*l^2*sin(theta_p)^2-Ib];

    % torque from gravity
    G = [0;
        mb*g*l*sin(theta_p)];

    % Centrifugal & Coriolis
    C = [-mb*l*sin(theta_p)*theta_p_dot^2;
        -mb*l^2*sin(theta_p)*cos(theta_p)*theta_p_dot^2];

    % Nonlinear Effect
    h = G + C;

    % Input Matrix
    B = [1/R ; 1 + l*cos(theta_p)/R];

    A = [M(:,1), -B];

    % Calculate Ainv(-h - M_c2 * theta_p_ddot)
    result = A \ (-h - M(:,2) * theta_p_ddot);
    x_acc(i) = R * result(1);
    tau(i) = result(2);
    tau_g(i) = G(2);
    i = i+1;
end

% Plot

figure('units','normalized','outerposition',[0 0.5 0.5 0.5]);

% Plot tau vs theta_p_var
subplot(1,2,1);
plot(rad2deg(theta_p_var), tau, 'r', 'LineWidth', 2);
title('\theta_p vs \tau');
xlabel('\theta_p (deg)');
ylabel('\tau (Nm)');
grid on;

% Find max and min values for tau
[max_tau, idx_max_tau] = max(tau);
[min_tau, idx_min_tau] = min(tau);

% Annotate max and min values on the plot
hold on;
plot(rad2deg(theta_p_var(idx_max_tau)), max_tau, 'bo', 'MarkerSize', 8);
text(rad2deg(theta_p_var(idx_max_tau)), max_tau, ['\leftarrow Max: ', num2str(max_tau)], 'Color', 'blue');
plot(rad2deg(theta_p_var(idx_min_tau)), min_tau, 'bo', 'MarkerSize', 8);
text(rad2deg(theta_p_var(idx_min_tau)), min_tau, ['\leftarrow Min: ', num2str(min_tau)], 'Color', 'blue');
hold off;

% Plot x_acc vs theta_p_var
subplot(1,2,2);
plot(rad2deg(theta_p_var), x_acc, 'r', 'LineWidth', 2);
title('\theta_p vs a_{xw}');
xlabel('\theta_p (deg)');
ylabel('a_{xw} (m/s^2)');
grid on;

% Find max and min values for x_acc
[max_x_acc, idx_max_x_acc] = max(x_acc);
[min_x_acc, idx_min_x_acc] = min(x_acc);

% Annotate max and min values on the plot
hold on;
plot(rad2deg(theta_p_var(idx_max_x_acc)), max_x_acc, 'bo', 'MarkerSize', 8);
text(rad2deg(theta_p_var(idx_max_x_acc)), max_x_acc, ['\leftarrow Max: ', num2str(max_x_acc)], 'Color', 'blue');
plot(rad2deg(theta_p_var(idx_min_x_acc)), min_x_acc, 'bo', 'MarkerSize', 8);
text(rad2deg(theta_p_var(idx_min_x_acc)), min_x_acc, ['\leftarrow Min: ', num2str(min_x_acc)], 'Color', 'blue');
hold off;
%%

clc; clear all; close all;

% Define Parameters
mw = 0.1; % mass of wheel (kg)
mb = 1.9; % mass of body (kg)
R = 0.08; % radius of Wheel (m)
g = 9.81; % gravity acceleration (m/s^2)

theta_p_var = -pi/2:0.01:pi/2; % pitch angle of body (rad)
theta_p_dot = 0; % pitch angular velocity of body (rad/s)
theta_p_ddot = 0; % pitch angular acceleration of body (rad/s^2)

% Moment of Inertia of wheel
Iw = mw * R^2;

% Define different lengths (l values)
l_values = 0.15:0.05:0.25; % Different lengths from CoM to wheel (m)
colors = lines(length(l_values)); % Generate colors for each l value

% Initialize storage for tau values
tau_results = zeros(length(theta_p_var), length(l_values));

% Iterate over different l values
figure('units', 'normalized', 'outerposition', [0 0.5 0.5 0.5]);
hold on;

for j = 1:length(l_values)
    l = l_values(j);
    
    % Moment of Inertia of body
    Ib = 1/12 * mb * (0.1^2 + 0.2^2); % moment of Inertia of body (kgm^2)

    % Pre-allocate results
    tau = zeros(length(theta_p_var), 1);

    i = 1;
    for theta_p = theta_p_var
        % Mass Matrix (theta_w version)
        M = [Iw/R + mw*R + mb*R,         mb*l*cos(theta_p);
            (Iw/R+mw*R)*l*cos(theta_p), -mb*l^2*sin(theta_p)-Ib];

        % torque from gravity
        G = [0;
            mb*g*l*sin(theta_p)];

        % Centrifugal & Coriolis
        C = [-mb*l*sin(theta_p)*theta_p_dot^2;
            -mb*l^2*sin(theta_p)*cos(theta_p)*theta_p_dot^2];

        % Nonlinear Effect
        h = G + C;

        % Input Matrix
        B = [1/R ; 1 + l*cos(theta_p)/R];

        A = [M(:,1), -B];

        % Calculate Ainv(-h - M_c2 * theta_p_ddot)
        result = A \ (-h - M(:,2) * theta_p_ddot);
        tau(i) = result(2);
        i = i+1;
    end
    
    % Store tau values
    tau_results(:, j) = tau;

    % Plot for the current l value
    plot(rad2deg(theta_p_var), tau, 'Color', colors(j,:), 'LineWidth', 2);
end

% Add title and labels
title('\theta_p vs \tau for different l values');
xlabel('\theta_p (deg)');
ylabel('\tau (Nm)');

% Create dynamic legend based on l values
legend_labels = arrayfun(@(l) sprintf('l = %.2f m', l), l_values, 'UniformOutput', false);
legend(legend_labels);

grid on;
hold off;
