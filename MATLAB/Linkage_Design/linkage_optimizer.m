clc; clear all; close all;

% Define lengths of links
l1 = 106;
l2 = 77;
l3 = 50;
l4 = 137;
l5 = -8;
theta_root = 30; % (degree)
len_ab = 75;

% Define theta range for calculations (from -pi/2 to 0)
theta = -pi/2:0.001:pi/4;

% Define coordinates for point (a, b)
a = len_ab * cos(theta_root * pi / 180); % x-coordinate of point
b = len_ab * sin(theta_root * pi / 180); % y-coordinate of point
beta = atan(l5/l4);
len_de = sqrt(l4^2 + l5^2);

% Calculate distance L1 from point (a, b) to the end of link l1
L1 = sqrt((a - l1 * cos(theta)).^2 + (b - l1 * sin(theta)).^2);
% Calculate distance L2, based on system geometry and given formula
L2 = 1/(2 * l2) * (l3^2 - l1^2 - l2^2 - a^2 - b^2 + 2 * l1 * (a * cos(theta) + b * sin(theta)));
% Calculate the angle alpha using atan2 for better handling of quadrants
alpha = atan2(b - l1 * sin(theta), a - l1 * cos(theta));
% Calculate the ratio L2/L1 for determining the angle phi
ratio = L2 ./ L1;
% Preallocate arrays for phi and phi_2
phi_1 = zeros(size(theta));
phi_2 = zeros(size(theta));
% Loop through each theta value to compute phi and phi_2
for i = 1:length(theta)
    % If the ratio is out of bounds for acos (i.e., >1 or <-1), set phi to NaN
    if (ratio(i) > 1) || (ratio(i) < -1)
        phi_1(i) = NaN;
        phi_2(i) = NaN;
    else
        % Calculate phi and phi_2 using acos, ensuring the angle wraps properly
        phi_1(i) = alpha(i) + acos(ratio(i)) - 2 * pi; % Shift phi to proper range
        phi_2(i) = alpha(i) - acos(ratio(i));        % Second possible solution for phi
    end
end

% angle of line DC from x axis
theta_k1 = atan2((b+l2*sin(phi_1)-l1*sin(theta)),(a+l2*cos(phi_1)-l1*cos(theta)));
theta_k2 = atan2((b+l2*sin(phi_2)-l1*sin(theta)),(a+l2*cos(phi_2)-l1*cos(theta)));

% Calculate the position of point E (x and y coordinates) for both phi solutions
E_x1 = l1 * cos(theta) - l4 * cos(theta_k1) + l5 * sin(theta_k1);
E_y1 = l1 * sin(theta) - l4 * sin(theta_k1) - l5 * cos(theta_k1);

E_x2 = l1 * cos(theta) - l4 * cos(theta_k2) + l5 * sin(theta_k2);
E_y2 = l1 * sin(theta) - l4 * sin(theta_k2) - l5 * cos(theta_k2);

% Create a figure with subplots to visualize the results
figure('units','normalized','outerposition',[0 0 1 1]);

% Plot phi and phi_2 as a function of theta
subplot(2, 2, 1);
plot(theta, phi_1, 'b'); % Plot phi in blue
hold on;
plot(theta, phi_2, 'r'); % Plot phi_2 in red
title('Phi vs Theta');
xlabel('Theta');
ylabel('Phi');
grid on;
legend('Phi', 'Phi_2');
axis equal;

% Plot the ratio (L2/L1) as a function of theta
subplot(2, 2, 2);
plot(theta, ratio, 'g'); % Plot ratio in green
title('Ratio (L2/L1) vs Theta');
xlabel('Theta');
ylabel('Ratio');
grid on;

% Plot the x-coordinate of point E vs theta for both solutions
subplot(2, 2, 3);
plot(theta, E_x1, 'b'); % Plot E_x1 in blue
hold on;
plot(theta, E_x2, 'r'); % Plot E_x2 in red
title('E_x vs Theta');
xlabel('Theta');
ylabel('x');
grid on;
legend('E_x1', 'E_x2');

% Plot the y-coordinate of point E vs theta for both solutions
subplot(2, 2, 4);
plot(theta, E_y1, 'b'); % Plot E_y1 in blue
hold on;
plot(theta, E_y2, 'r'); % Plot E_y2 in red
title('E_y vs Theta');
xlabel('Theta');
ylabel('y');
grid on;
legend('E_y1', 'E_y2');

% Create another figure for the x and y components of point E over theta
figure('units','normalized','outerposition',[0 0.5 0.5 0.5]);

% Calculate the limits for x and y based on the data you want to plot
x_min = min([E_x1(:); E_x2(:)]);  % Find minimum x value from all plotted data
x_max = max([E_x1(:); E_x2(:)]);  % Find maximum x value from all plotted data

y_min = min([E_y1(:); E_y2(:)]);  % Find minimum y value from all plotted data
y_max = max([E_y1(:); E_y2(:)]);  % Find maximum y value from all plotted data

% Plot the trajectory of point E for both phi and phi_2
plot(E_x1, E_y1, 'b', 'LineWidth', 2); % Plot the first trajectory in blue with thicker line
hold on;
plot(E_x2, E_y2, 'r', 'LineWidth', 2); % Plot the second trajectory in red with thicker line


% Plot the point at (0, 0) as a black circle and label it as 'A'
plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % Black filled circle at (0,0)
text(0, 0, '  A', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'FontWeight', 'bold'); % Label A

% Plot the point at (a, b) as a green circle and label it as 'B'
plot(a, b, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Green filled circle at (a,b)
text(a, b, '  B', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 12, 'FontWeight', 'bold'); % Label B

% Add title and labels
title('Trajectory of Point E');
xlabel('x');
ylabel('y');
axis equal;

% Set the x and y limits dynamically based on the data range
xlim([x_min-0.01, x_max+0.01]);
ylim([y_min-0.01, y_max+0.01]);

grid on;

% Add a legend to label the trajectories and the points
legend('Trajectory 1', 'Trajectory 2', 'Origin (0,0)', 'Point (a,b)');




%% Trajectory Change all view

clc; clear all; close all;

% Define lengths of links
% l1 = 114;
% l2 = 99;
% l3 = 67;
% l4 = 109;
% l5 = -19;


l1_min_Max = [106 107];
l2_min_Max = [77 78];
l3_min_Max = [49 50];
l4_min_Max = [137 138];
l5_min_Max = [-10 -8];


theta_root = 30; % (degree)
len_ab = 75;

W = 2; % mass (kg)
W = W * 9.81; % N

y_min = -200;
y_max = -60;

% Define theta range for calculations (from -pi/2 to 0)
theta = -pi/2:0.001:pi/4;

% Define coordinates for point (a, b)
a = len_ab * cos(theta_root * pi / 180); % x-coordinate of point
b = len_ab * sin(theta_root * pi / 180); % y-coordinate of point

% Range of l1, l2, l3, l4, l5 values to evaluate
gap = 1;
l1_values = l1_min_Max(1) : gap : l1_min_Max(2);
l2_values = l2_min_Max(1) : gap : l2_min_Max(2);
l3_values = l3_min_Max(1) : gap : l3_min_Max(2);
l4_values = l4_min_Max(1) : gap : l4_min_Max(2);
l5_values = l5_min_Max(1) : gap : l5_min_Max(2);

l1_len = length(l1_values);
l2_len = length(l2_values);
l3_len = length(l3_values);
l4_len = length(l4_values);
l5_len = length(l5_values);

num_loop = l1_len * l2_len * l3_len * l4_len * l5_len;
fprintf("the number of loops : %d", num_loop);

plot_idx = 1;
num_plot = 5; % 在一张图中最多设置 5 个绘图

current_loop = 0; % 进度循环计数

% Loop through each l1 value
for l1 = l1_values
    for l2 = l2_values
        for l3 = l3_values
            progress = (current_loop / num_loop) * 100;
            fprintf("Progress : %.2f%% \n", progress);
            for l4 = l4_values
                for l5 = l5_values
                    current_loop = current_loop + 1; % 添加循环进度计数

                    beta = atan(l5/l4);
                    len_de = sqrt(l4^2 + l5^2);
                    % Calculate distance L1 from point (a, b) to the end of link l1
                    L1 = sqrt((a - l1 * cos(theta)).^2 + (b - l1 * sin(theta)).^2);
                    % Calculate distance L2 based on system geometry and formula
                    L2 = 1/(2 * l2) * (l3^2 - l1^2 - l2^2 - a^2 - b^2 + 2 * l1 * (a * cos(theta) + b * sin(theta)));
                    % Calculate the angle alpha using atan2 for better handling of quadrants
                    alpha = atan2(b - l1 * sin(theta), a - l1 * cos(theta));
                    % Calculate the ratio L2/L1 for determining the angle phi
                    ratio = L2 ./ L1;
                    % Preallocate arrays for phi and phi_2
                    phi = zeros(size(theta));

                    % Loop through each theta value to compute phi and phi_2
                    for i = 1:length(theta)
                        % If the ratio is out of bounds for acos (i.e., >1 or <-1), set phi to NaN
                        if (ratio(i) > 1) || (ratio(i) < -1)
                            phi(i) = NaN;
                        else
                            % Calculate phi and phi_2 using acos, ensuring the angle wraps properly
                            phi(i) = alpha(i) - acos(ratio(i));        % Second possible solution for phi
                        end
                    end

                    theta_k = atan2((b+l2*sin(phi)-l1*sin(theta)),(a+l2*cos(phi)-l1*cos(theta)));
                    E_x = l1 * cos(theta) - l4 * cos(theta_k) + l5 * sin(theta_k);
                    E_y = l1 * sin(theta) - l4 * sin(theta_k) - l5 * cos(theta_k);

                    [E_y_min, min_idx] = min(E_y);
                    [E_y_max, max_idx] = max(E_y);
                    
                    % 检查 E_y 是否具有所需的变化
                    if(E_y_max < y_max || E_y_min > y_min)
                        continue;
                    end
                    idx_gt_low = find(E_y(min_idx:max_idx) > y_min, 1) + min_idx - 1;
                    idx_gt_up = find(E_y(min_idx:max_idx) > y_max, 1) + min_idx - 1;

                    % cutting
                    E_x_cut = E_x(idx_gt_low:idx_gt_up);
                    E_y_cut = E_y(idx_gt_low:idx_gt_up);
                    theta_cut = theta(idx_gt_low:idx_gt_up);
                    theta_k_cut = theta_k(idx_gt_low:idx_gt_up);
                    phi_cut = phi(idx_gt_low:idx_gt_up);

                    % 检查E_x的x变化是否在10mm以内
                    if isempty(E_x_cut)
                        continue;
                    end
                    
                    x_var = max(E_x_cut)-min(E_x_cut);
                    if(x_var > 4 || max(E_x_cut) > 40 || min(E_x_cut) < 0)
                        continue;
                    end
                    
                    % A点为动力源时的扭矩计算
                    FDx = (len_de * cos(theta_k_cut + beta) * W) ./ ((-tan(phi_cut)+tan(theta_k_cut))*l3.*cos(theta_k_cut)*2);
                    FDy = -W/2+tan(phi_cut).*FDx;
                    tau_A = l1 * (FDy.*cos(theta_cut) - FDx.*sin(theta_cut));
                    tau_A = tau_A * 1e-3; % convert unit Nmm to Nm

                    % B点为动力源时的扭矩计算
                    FCx = -W ./ (2*(-tan(theta_cut)+tan(theta_k_cut))) .* (1 + len_de / l3 * cos(theta_k_cut + beta) ./ cos(theta_k_cut));
                    FCy = -W/2 + tan(theta_cut) .* FCx;
                    tau_B = l2 * (FCy.*cos(phi_cut) - FCx.*sin(phi_cut));
                    tau_B = tau_B * 1e-3; % convert unit Nmm to Nm

                    % Maximum and minimum torques for A and B
                    tau_A_max = max(tau_A);
                    tau_A_min = min(tau_A);
                    tau_B_max = max(tau_B);
                    tau_B_min = min(tau_B);

                    if(tau_A_min < -1 && tau_B_min < -1)
                        continue;
                    end
                    if(tau_A_max < -0.1 && tau_B_max < -0.1)
                        continue;
                    end


                    % Plot the trajectory of point E for both phi and phi_2
                    if(mod(plot_idx,num_plot) == 1)
                        plot_idx = 1;
                        figure('units','normalized','outerposition',[0 0 1 1]);
                        sgtitle('Trajectory of Point E');
                    end
                    subplot(2,num_plot,plot_idx);
                    plot(E_x_cut, E_y_cut, 'LineWidth', 2);
                    axis equal;
                    xlabel('x');
                    ylabel('y');
                    grid on;
                    title(sprintf('l1=%.1f, l2=%.1f, l3=%.1f, l4=%.1f, l5=%.1f\n\\tau_A=[%.2f, %.2f]Nm, \\tau_B=[%.2f, %.2f]Nm \n x_var=%.2f',...
                        l1, l2, l3, l4, l5, tau_A_max, tau_A_min, tau_B_max, tau_B_min, x_var));

                    subplot(2,num_plot,num_plot + plot_idx);
                    plot(E_y_cut, tau_A, 'r', 'LineWidth',2);
                    hold on;
                    plot(E_y_cut, tau_B, 'b', 'LineWidth',2);
                    title("E_y vs torque");
                    xlabel('E_y (mm)');
                    ylabel('torque (Nm)');
                    grid on;
                    legend('\tau_A', '\tau_B');

                    plot_idx = plot_idx + 1;
                end
            end
            drawnow;
        end
    end
end


