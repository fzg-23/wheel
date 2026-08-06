clc; clear all; close all;  % 初始化控制台、变量和所有窗口

% 定义链接的长度
l1 = 106;  % 第一连杆长度（mm）
l2 = 77;   % 第二连杆长度（mm）
l3 = 50;   % 第三连杆长度（mm）
l4 = 137;  % 第四连杆长度（mm）
l5 = -8;   % 第四连杆垂直偏移（mm）

theta_root = 30;  % 根角（度）
len_ab = 75;      % 连杆 AB 长度 (mm)

%定义计算的角度范围
theta_hipR = -29.68*pi/180 : 0.001 : 69.97*pi/180;
theta_BR = -theta_hipR;  % theta 范围（-90 度至 180 度）

% (a,b)点坐标的定义
a = len_ab * cos(theta_root * pi / 180);  % a点的x坐标
b = len_ab * sin(theta_root * pi / 180);  % b点的y坐标

% 计算角度 beta 和 DE 长度
beta = atan(l5 / l4);            %计算角度β
len_de = sqrt(l4^2 + l5^2);      % 计算DE的长度

% L1计算
L1 = sqrt((a + l2 * cos(theta_BR)).^2 + (b + l2 * sin(theta_BR)).^2);

% L2计算
L2 = (a^2 + b^2 + l1^2 + l2^2 - l3^2 + 2*l2*(a*cos(theta_BR) +b*sin(theta_BR)))/(2*l1);

% 阿尔法计算
alpha = atan2(b+l2*sin(theta_BR), a+l2*cos(theta_BR));
ratio = L2 ./ L1;

theta_AR = zeros(size(theta_BR));   % phi 数组
theta_AR2 = zeros(size(theta_BR));  % phi_2 数组

% 计算每个 theta 值的 phi 和 phi_2
for i = 1:length(theta_BR)
    % 如果比率超出 acos 函数的范围，则将 phi 设置为 NaN
    if (ratio(i) > 1) || (ratio(i) < -1)
        theta_AR(i) = NaN;
        theta_AR2(i) = NaN;
    else
        % acos 从 0~pi 的结果
        theta_AR(i) = acos(ratio(i)) + alpha(i);  % 第一个可能的 phi 值
        if( theta_AR(i) > pi )
            theta_AR(i) = theta_AR(i) - 2*pi;
        elseif( theta_AR(i) < -pi)
            theta_AR(i) = theta_AR(i) + 2*pi;
        end
        
        % -pi~0 的 acos 结果
        theta_AR2(i) = -acos(ratio(i)) + alpha(i);
        if( theta_AR2(i) > pi )
            theta_AR2(i) = theta_AR2(i) - 2*pi;
            fprintf("大于 pi");
        elseif( theta_AR2(i) < -pi)
            theta_AR2(i) = theta_AR2(i) + 2*pi;
            fprintf("小于 -pi");
        end
    end
end

% 每种情况的 theta_k
theta_kR = atan2((b+l2*sin(theta_BR)-l1*sin(theta_AR)),(a+l2*cos(theta_BR)-l1*cos(theta_AR)));
theta_kR2 = atan2((b+l2*sin(theta_BR)-l1*sin(theta_AR2)),(a+l2*cos(theta_BR)-l1*cos(theta_AR2)));


% 创建子图以可视化结果
figure('units','normalized','outerposition',[0 0.25 1 0.5]);

% 第一个子图
subplot(1,3,1);
plot(rad2deg(theta_BR), rad2deg(theta_AR),'r','LineWidth',2);
hold on;
plot(rad2deg(theta_BR), rad2deg(theta_AR2),'b','LineWidth',2)
title("\theta_{BR} vs \theta_{AR}");
xlabel('\theta_{BR} (deg)');
ylabel('\theta_{AR} (deg)');
axis equal;
grid on;
legend('\theta_{AR}', '\theta_{AR2}'); %添加图例

% 第二个次要情节
subplot(1,3,2);
plot(rad2deg(theta_BR), rad2deg(theta_kR),'r','LineWidth',2);
hold on;
plot(rad2deg(theta_BR), rad2deg(theta_kR2),'b','LineWidth',2)
title("\theta_{BR} vs \theta_{kR}");
xlabel('\theta_{BR} (deg)');
ylabel('\theta_{kR} (deg)');
axis equal;
grid on;
legend('\theta_{kR}', '\theta_{kR2}'); %添加图例

% 第三个次要情节
subplot(1,3,3);
plot(rad2deg(theta_BR), rad2deg(alpha),'r','LineWidth',2);
hold on;
plot(rad2deg(theta_BR), rad2deg(acos(ratio)),'b','LineWidth',2);
title("\alpha, acos");
xlabel('\theta_{BR} (deg)');
ylabel('(deg)');
axis equal;
grid on;
legend('\alpha', 'acos(L2/L1)'); %添加图例
