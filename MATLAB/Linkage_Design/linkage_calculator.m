clc; clear all; close all;  % 初始化控制台、变量和所有窗口

% 定义链接的长度
l1 = 106;  % 第一连杆长度（mm）
l2 = 77;   % 第二连杆长度（mm）
l3 = 50;   % 第三连杆长度（mm）
l4 = 137;  % 第四连杆长度（mm）
l5 = -8;   % 第四连杆垂直偏移（mm）

theta_root = 30;  % 根角（度）
len_ab = 75;      % 连杆 AB 长度 (mm)

W = 2;  %质量（公斤）

y_min = -200;  % 最小 y 值（毫米）
y_max = -70;   % 最大 y 值（毫米）

%定义计算的角度范围
theta = -pi/2 : 0.001 : pi/1;  % theta 范围（-90 度至 180 度）

W = 9.81 * W;    % 将质量转换为重量 (N)

% (a,b)点坐标的定义
a = len_ab * cos(theta_root * pi / 180);  % a点的x坐标
b = len_ab * sin(theta_root * pi / 180);  % b点的y坐标

% 计算角度 beta 和 DE 长度
beta = atan(l5 / l4);            %计算角度β
len_de = sqrt(l4^2 + l5^2);      % 计算DE的长度

% L1计算
L1 = sqrt((a - l1 * cos(theta)).^2 + (b - l1 * sin(theta)).^2);

% L2计算
L2 = 1/(2 * l2) * (l3^2 - l1^2 - l2^2 - a^2 - b^2 + 2 * l1 * (a * cos(theta) + b * sin(theta)));

% 计算角度 alpha（使用 atan2 改进象限处理）
alpha = atan2(b - l1 * sin(theta), a - l1 * cos(theta));

% 使用 L2 和 L1 的比率计算 phi 角
ratio = L2 ./ L1;

%预分配 phi 和 phi_2 数组
phi = zeros(size(theta));   % phi 数组
phi2 = zeros(size(theta));  % phi_2 数组

% 计算每个 theta 值的 phi 和 phi_2
for i = 1:length(theta)
    % 如果比率超出 acos 函数的范围，则将 phi 设置为 NaN
    if (ratio(i) > 1) || (ratio(i) < -1)
        phi(i) = NaN;
        phi2(i) = NaN;
    else
        % 使用 acos 计算 phi 和 phi_2
        phi(i) = alpha(i) - acos(ratio(i));  % 第一个可能的 phi 值
        if( phi(i) > pi )
            phi(i) = phi(i) - 2*pi;
        elseif( phi(i) < -pi)
            phi(i) = phi(i) + 2*pi;
        end
        
        % 计算第二个可能的 phi 值
        phi2(i) = alpha(i) + acos(ratio(i));
        if( phi2(i) > pi )
            phi2(i) = phi2(i) - 2*pi;
        elseif( phi2(i) < -pi)
            phi2(i) = phi2(i) + 2*pi;
        end
    end
end

% 计算线 DC 与 x 轴的角度
theta_k = atan2((b+l2*sin(phi)-l1*sin(theta)),(a+l2*cos(phi)-l1*cos(theta)));

% 计算E点的x、y坐标
E_x = l1 * cos(theta) - l4 * cos(theta_k) + l5 * sin(theta_k);
E_y = l1 * sin(theta) - l4 * sin(theta_k) - l5 * cos(theta_k);

% E_y 的最小值和最大值以及索引
[E_y_min, min_idx] = min(E_y);
[E_y_max, max_idx] = max(E_y);

% 在 y_min 和 y_max 范围内找到 E_y 满足的索引
idx_gt_low = find(E_y(min_idx:max_idx) > y_min, 1) + min_idx - 1;
idx_gt_up = find(E_y(min_idx:max_idx) > y_max, 1) + min_idx - 1;

% 保存剪切值（E_x、E_y、theta、phi）
E_x_cut = E_x(idx_gt_low:idx_gt_up);
E_y_cut = E_y(idx_gt_low:idx_gt_up);
theta_cut = theta(idx_gt_low:idx_gt_up);
theta_k_cut = theta_k(idx_gt_low:idx_gt_up);
phi_cut = phi(idx_gt_low:idx_gt_up);

% A点为动力源时的扭矩计算
FDx = (len_de * cos(theta_k_cut + beta) * W) ./ ((-tan(phi_cut)+tan(theta_k_cut))*l3.*cos(theta_k_cut)*2);
FDy = -W/2 + tan(phi_cut).*FDx;
tau_A = l1 * (FDy.*cos(theta_cut) - FDx.*sin(theta_cut));
tau_A = tau_A * 1e-3;  % 将单位 Nmm 转换为 Nm

% B点为动力源时的扭矩计算
FCx = -W ./ (2*(-tan(theta_cut)+tan(theta_k_cut))) .* (1 + len_de / l3 * cos(theta_k_cut + beta) ./ cos(theta_k_cut));
FCy = -W/2 + tan(theta_cut) .* FCx;
tau_B = l2 * (FCy.*cos(phi_cut) - FCx.*sin(phi_cut));
tau_B = tau_B * 1e-3;  % 将单位 Nmm 转换为 Nm

% 创建子图以可视化结果
figure('units','normalized','outerposition',[0 0 1 1]);

%theta 和 tau_A 之间的关系图
subplot(2,4,1);
plot(rad2deg(theta_cut), tau_A, 'r' ,'LineWidth',2);
title("theta vs tau_A");
xlabel('\theta (deg)');
ylabel('torque (Nm)');
grid on;
legend('\tau_A');

% phi 和 tau_B 之间的关系图
subplot(2,4,2);
plot(rad2deg(phi_cut), tau_B, 'b','LineWidth',2);
title("phi vs tau_B");
xlabel('\phi (deg)');
ylabel('torque (Nm)');
grid on;
legend('\tau_B');

% E_y与扭矩关系图
subplot(2,4,3);
plot(E_y_cut, tau_A, 'r', 'LineWidth',2);
hold on;
plot(E_y_cut, tau_B, 'b', 'LineWidth',2);
title("E_y vs torque");
xlabel('E_y (mm)');
ylabel('torque (Nm)');
grid on;
legend('\tau_A', '\tau_B');

% theta 和 phi 之间的关系图
subplot(2,4,4);
plot(rad2deg(theta), rad2deg(phi),'r','LineWidth',2);
hold on;
plot(rad2deg(theta), rad2deg(phi2),'r','LineWidth',2)
title("theta vs phi");
xlabel('\theta (deg)');
ylabel('\phi (deg)');
axis equal;
grid on;

% E点轨迹图
subplot(2,4,5);
plot(E_x_cut,E_y_cut,'r','LineWidth',2);
title("Trajectory of the point E");
xlabel('x (mm)');
ylabel('y (mm)');
axis equal;
grid on;
grid minor;
% legend('E');
xlim([-120, 120]);
ylim([-250, 0]);

% theta 和 E_y 之间的关系图
subplot(2,4,6);
plot(rad2deg(theta_cut), E_y_cut, 'r' ,'LineWidth',2);
title("theta vs E_y");
xlabel('\theta (deg)');
ylabel('y (mm)');
grid on;

% phi 和 E_y 之间的关系图
subplot(2,4,7);
plot(rad2deg(phi_cut), E_y_cut, 'b' ,'LineWidth',2);
title("phi vs E_y");
xlabel('\phi(deg)');
ylabel('y (mm)');
grid on;

figure();
% E点轨迹图
plot(E_x_cut,E_y_cut,'r','LineWidth',3);
title("Trajectory of the point E");
xlabel('x (mm)');
ylabel('y (mm)');
axis equal;
grid on;
grid minor;
% legend('E');
xlim([-120, 120]);
ylim([-250, 0]);
