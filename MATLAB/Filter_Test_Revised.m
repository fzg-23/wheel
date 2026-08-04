clc; clear;
% delete(findall(0, 'Type', 'figure')); % 이렇게 해야 uifigure가 제거됨
format long;

addpath("lib\");
addpath("log_plot\");
load('dynamic_properties.mat');
load('dynamics_functions.mat');

% CSV 파일 읽기
filename = '20250112_logdata_EKF_LowGain_withaccLPF_VICON_005.csv'; % CSV 파일 이름

data = readtable(filename);

Ts = 0.008;
Case_num = 1;

x_lim = [15.038, 41.038];

psi_dot_hat = data.psi_dot_hat;
theta_hat = data.theta_hat;
theta_dot_hat = data.theta_dot_hat;
v_hat = data.v_hat;


% Estimator
x_hat_init = [0; 0; 0; 0];
x_hat = x_hat_init;

model = Pol(properties, dynamic_functions);
obs_model = Observe_model(properties);
Estimator = EKF_c(x_hat_init, P_init, Q_cov, R_cov, model, Ts);

% extract data
timeStamp = data.TimeStamp / 10^3;

h_d = data.h_d;
theta_d = data.theta_d;
v_d = data.v_d;
psi_dot_d = data.psi_dot_d;

acc_x_imu = data.acc_x; % m/s^2
acc_y_imu = data.acc_y; % m/s^2
acc_z_imu = data.acc_z; % m/s^2
gyr_x_imu = data.gyr_x; % rad/s
gyr_y_imu = data.gyr_y; % rad/s
gyr_z_imu = data.gyr_z; % rad/s
theta_dot_LW = data.theta_dot_LW;
theta_dot_RW = data.theta_dot_RW;
z_m =  [acc_x_imu'; acc_y_imu'; acc_z_imu';...
    gyr_x_imu'; gyr_y_imu'; gyr_z_imu'; ...
    theta_dot_RW'; theta_dot_LW'];

z = [];
if Case_num == 1
    z = [acc_x_imu'; acc_z_imu'; gyr_y_imu'; theta_dot_RW'; theta_dot_LW'];
elseif Case_num == 2
    z = [acc_x_imu'; acc_z_imu'; gyr_y_imu'; theta_dot_RW'; theta_dot_LW'];
elseif Case_num == 3
    z = [atan2(-acc_x_imu', acc_z_imu'); gyr_y_imu'; theta_dot_RW'; theta_dot_LW'];
end



tau_LW = data.tau_LW;
tau_RW = data.tau_RW;
u = [tau_RW'; tau_LW'];

x_hat_cal = zeros(4,length(timeStamp));
x_pred = x_hat_cal;
P_pred_log = x_hat_cal;
P_log = x_hat_cal;
h_obs = zeros(8,length(timeStamp));
for i = 1:length(timeStamp)
    if i <= 2
        u_prevprev = [0;0];
    else
        u_prevprev = u(:,i-2);
        % u_prevprev = [0;0];
    end
    h = h_d(i);

    Estimator.predict(u_prevprev, h);
    x_pred(:,i) = Estimator.x;
    P_pred_log(:,i) = sqrt(abs([Estimator.P(1,1); Estimator.P(2,2); Estimator.P(3,3); Estimator.P(4,4)]));

    Estimator.update_test(z(:,i), obs_model, Case_num);
    x_hat_cal(:,i) = Estimator.x;
    P_log(:,i) = sqrt(abs([Estimator.P(1,1); Estimator.P(2,2); Estimator.P(3,3); Estimator.P(4,4)]));
    
    if Case_num == 1
        h_obs(:,i) = [Estimator.h_obs(1); 0; Estimator.h_obs(2); 0; Estimator.h_obs(3); 0; ...
            Estimator.h_obs(4); Estimator.h_obs(5)];
        h_obs(2,i) = 2*h*cos(x_pred(1,i))*x_pred(4,i)^2+x_pred(3,i)*x_pred(4,i);
    elseif Case_num == 2
        h_obs(:,i) = [Estimator.h_obs(1); 0; Estimator.h_obs(2); 0; Estimator.h_obs(3); 0; ...
            Estimator.h_obs(4); Estimator.h_obs(5)];
        h_obs(2,i) = 2*h*cos(x_pred(1,i))*x_pred(4,i)^2+x_pred(3,i)*x_pred(4,i);
    elseif Case_num == 3
        h_obs(:,i) = [-model.g*sin(Estimator.h_obs(1)); 0; model.g*cos(Estimator.h_obs(1)); 0; Estimator.h_obs(2); 0; ...
            Estimator.h_obs(3); Estimator.h_obs(4)];
    end
end

% error = h_obs-z; 
% for i = 1 : 8
%     var(error(i,:))
% end

% ========================================================================
% ==================== Plot Start ========================================
% ========================================================================
screenSize = get(0, 'ScreenSize'); % 화면 크기 가져오기

% uifigure 생성 (전체 화면 크기로 설정)
fig = uifigure('Name', filename + "/ Filter_Test_Revised", 'Position', [0, 0, screenSize(3), screenSize(4)*0.97]);
tabGroup = uitabgroup(fig, 'Position', [0, 0, fig.Position(3), fig.Position(4)]);

% ========================== Figure 1 ============================
figure_title = "Estimated states";
Line_width = 1.5;

tab1 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab1, 2, 2, "TileSpacing", "compact");
title(layout,figure_title, 'FontSize', 14, 'FontWeight', 'bold')

% ==== Fig.1 - 1/4 ====
ax = nexttile(layout, 1);
upper_bound = x_hat_cal(1,:) + 1.96 * P_log(1,:);
lower_bound = x_hat_cal(1,:) - 1.96 * P_log(1,:);
% upper_bound = x_pred(1,:) + 1.96 * P_pred_log(1,:);
% lower_bound = x_pred(1,:) - 1.96 * P_pred_log(1,:);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(theta_hat), 'r', 'DisplayName', '\theta_{ hat}', 'LineWidth', Line_width);
plot(ax, timeStamp, rad2deg(x_hat_cal(1,:)), 'b--', 'DisplayName', '\theta_{ hat(cal)}', 'LineWidth', Line_width); 
% plot(ax, timeStamp, rad2deg(x_pred(1,:)), 'g.', 'DisplayName', '\theta_{ pred(cal)}'); 
plot(ax, timeStamp, rad2deg(theta_d), 'k', 'DisplayName', '\theta_{ d}');
fill(ax, [timeStamp', fliplr(timeStamp')], ...
     [rad2deg(upper_bound), fliplr(rad2deg(lower_bound))], ...
     'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% Confidence Interval');
title(ax, '\theta_{ hat}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
xlim(ax, x_lim);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.1 - 2/4 ====
ax = nexttile(layout, 2);
upper_bound = x_hat_cal(2,:) + 1.96 * P_log(2,:);
lower_bound = x_hat_cal(2,:) - 1.96 * P_log(2,:);
% upper_bound = x_pred(2,:) + 1.96 * P_pred_log(2,:);
% lower_bound = x_pred(2,:) - 1.96 * P_pred_log(2,:);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(theta_dot_hat), 'r', 'DisplayName', '\theta^{ dot}_{ hat}', 'LineWidth', Line_width); 
plot(ax, timeStamp, rad2deg(x_hat_cal(2,:)), 'b--', 'DisplayName', '\theta^{ dot}_{ hat(cal)}', 'LineWidth', Line_width); 
% plot(ax, timeStamp, rad2deg(x_pred(2,:)), 'g.', 'DisplayName', '\theta^{ dot}_{ pred(cal)}'); 
plot(ax, timeStamp, zeros(length(timeStamp),1), 'k', 'DisplayName', '\theta^{ dot}_{ d}');
fill(ax, [timeStamp', fliplr(timeStamp')], ...
     [rad2deg(upper_bound), fliplr(rad2deg(lower_bound))], ...
     'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% Confidence Interval');
title(ax, '\theta^{ dot}_{ hat}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.1 - 3/4 ====
ax = nexttile(layout, 3);
upper_bound = x_hat_cal(3,:) + 1.96 * P_log(3,:);
lower_bound = x_hat_cal(3,:) - 1.96 * P_log(3,:);
% upper_bound = x_pred(3,:) + 1.96 * P_pred_log(3,:);
% lower_bound = x_pred(3,:) - 1.96 * P_pred_log(3,:);
hold(ax, 'on');
plot(ax, timeStamp, v_hat, 'r', 'DisplayName', 'v_{ hat}', 'LineWidth', Line_width); 
plot(ax, timeStamp, x_hat_cal(3,:), 'b--', 'DisplayName', 'v_{ hat(cal)}', 'LineWidth', Line_width); 
% plot(ax, timeStamp, x_pred(3,:), 'g.', 'DisplayName', 'v_{ pred(cal)}'); 
plot(ax, timeStamp, v_d, 'k', 'DisplayName', 'v_{ d}');
fill(ax, [timeStamp', fliplr(timeStamp')], ...
     [upper_bound, fliplr(lower_bound)], ...
     'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% Confidence Interval');
title(ax, 'v_{ hat}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, x_lim);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.1 - 4/4 ====
ax = nexttile(layout, 4);
upper_bound = x_hat_cal(4,:) + 1.96 * P_log(4,:);
lower_bound = x_hat_cal(4,:) - 1.96 * P_log(4,:);
% upper_bound = x_pred(4,:) + 1.96 * P_pred_log(4,:);
% lower_bound = x_pred(4,:) - 1.96 * P_pred_log(4,:);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(psi_dot_hat), 'r', 'DisplayName', '\psi^{ dot}_{ hat}', 'LineWidth', Line_width); 
plot(ax, timeStamp, rad2deg(x_hat_cal(4,:)), 'b--', 'DisplayName', '\psi^{ dot}_{ hat(cal)}', 'LineWidth', Line_width);
% plot(ax, timeStamp, rad2deg(x_pred(4,:)), 'g.', 'DisplayName', '\psi^{ dot}_{ pred(cal)}'); 
plot(ax, timeStamp, rad2deg(psi_dot_d), 'k', 'DisplayName', '\psi^{ dot}_{ d}');
fill(ax, [timeStamp', fliplr(timeStamp')], ...
     [rad2deg(upper_bound), fliplr(rad2deg(lower_bound))], ...
     'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', '95% Confidence Interval');
title(ax, '\psi^{ dot}_{ hat}');
xlabel(ax, 'TimeStamp');
ylabel(ax, 'deg/s');
xlim(ax, x_lim);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");

% ========================== Figure 2 ============================
figure_title = "Measurements";
Line_width = 1.5;

tab2 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab2, 2, 4, "TileSpacing", "compact");
title(layout,figure_title, 'FontSize', 14, 'FontWeight', 'bold')

% ==== Fig.2 - 1/8 ====
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp, z_m(1,:), 'r', 'DisplayName', 'a_{ x}'); 
plot(ax, timeStamp, h_obs(1,:), 'g', 'DisplayName', 'a_{ obs,x}'); 
title(ax, 'a_{ IMU,x}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Acceleration (m/s^2)');
xlim(ax, x_lim);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 2/8 ====
ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp, z_m(2,:), 'r', 'DisplayName', 'a_{ y}');
plot(ax, timeStamp, h_obs(2,:), 'g', 'DisplayName', 'a_{ obs,y}'); 
title(ax, 'a_{ IMU,y}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Acceleration (m/s^2)');
xlim(ax, x_lim);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 3/8 ====
ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, z_m(3,:), 'r', 'DisplayName', 'a_{ z}');
plot(ax, timeStamp, h_obs(3,:), 'g', 'DisplayName', 'a_{ obs,z}'); 
title(ax, 'a_{ IMU,z}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Acceleration (m/s^2)');
xlim(ax, x_lim);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 4/8 ====
ax = nexttile(layout, 5);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(z_m(4,:)), 'r', 'DisplayName', 'w_{ x}'); 
plot(ax, timeStamp, rad2deg(h_obs(4,:)), 'g', 'DisplayName', 'w_{ obs,x}'); 
title(ax, 'w_{ IMU,x}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim);
legend(ax, 'show');  
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 5/8 ====
ax = nexttile(layout, 6);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(z_m(5,:)), 'r', 'DisplayName', 'w_{ y}');
plot(ax, timeStamp, rad2deg(h_obs(5,:)), 'g', 'DisplayName', 'w_{ obs,y}'); 
title(ax, 'w_{ IMU,y}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 6/8 ====
ax = nexttile(layout, 7);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(z_m(6,:)), 'r', 'DisplayName', 'w_{ z}');
plot(ax, timeStamp, rad2deg(h_obs(6,:)), 'g', 'DisplayName', 'w_{ obs,z}'); 
title(ax, 'w_{ IMU,z}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 7/8 ====
ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(z_m(7,:)), 'r', 'DisplayName', '\theta^{ dot}_{ RW}');
plot(ax, timeStamp, rad2deg(h_obs(7,:)), 'g', 'DisplayName', '\theta^{ dot}_{ obs,RW}'); 
title(ax, 'Right Wheel speed');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.2 - 8/8 ====
ax = nexttile(layout, 8);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(z_m(8,:)), 'r', 'DisplayName', '\theta^{ dot}_{ LW}');
plot(ax, timeStamp, rad2deg(h_obs(8,:)), 'g', 'DisplayName', '\theta^{ dot}_{ obs,LW}'); 
title(ax, 'Left Wheel speed');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");

% ========================== Figure 3 ============================
figure_title = "Estimated states_Only Calculated";
Line_width = 1.5;

tab3 = uitab(tabGroup, 'Title', figure_title);
layout = tiledlayout(tab3, 2, 2, "TileSpacing", "compact");
title(layout,figure_title, 'FontSize', 14, 'FontWeight', 'bold')

% ==== Fig.3 - 1/4 ====
ax = nexttile(layout, 1);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(x_hat_cal(1,:)), 'b', 'DisplayName', '\theta_{ hat(cal)}', 'LineWidth', Line_width); 
plot(ax, timeStamp, rad2deg(theta_d), 'k', 'DisplayName', '\theta_{ d}');
title(ax, '\theta_{ hat}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
xlim(ax, x_lim);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.3 - 2/4 ====
ax = nexttile(layout, 2);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(x_hat_cal(2,:)), 'b', 'DisplayName', '\theta^{ dot}_{ hat(cal)}', 'LineWidth', Line_width); 
plot(ax, timeStamp, zeros(length(timeStamp),1), 'k', 'DisplayName', '\theta^{ dot}_{ d}');
title(ax, '\theta^{ dot}_{ hat}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angular rate (deg/s)');
xlim(ax, x_lim);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.3 - 3/4 ====
ax = nexttile(layout, 3);
hold(ax, 'on');
plot(ax, timeStamp, x_hat_cal(3,:), 'b', 'DisplayName', 'v_{ hat(cal)}', 'LineWidth', Line_width); 
plot(ax, timeStamp, v_d, 'k', 'DisplayName', 'v_{ d}');
title(ax, 'v_{ hat}');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Velocity (m/s)');
xlim(ax, x_lim);
legend(ax, 'show');
grid(ax, "on");
grid(ax, "minor");

% ==== Fig.3 - 4/4 ====
ax = nexttile(layout, 4);
hold(ax, 'on');
plot(ax, timeStamp, rad2deg(x_hat_cal(4,:)), 'b', 'DisplayName', '\psi^{ dot}_{ hat(cal)}', 'LineWidth', Line_width);
plot(ax, timeStamp, rad2deg(psi_dot_d), 'k', 'DisplayName', '\psi^{ dot}_{ d}');
title(ax, '\psi^{ dot}_{ hat}');
xlabel(ax, 'TimeStamp');
ylabel(ax, 'deg/s');
xlim(ax, x_lim);
legend(ax, 'show'); 
grid(ax, "on");
grid(ax, "minor");