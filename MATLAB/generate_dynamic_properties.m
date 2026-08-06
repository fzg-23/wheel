clc;clear;close all;
addpath("lib\");
format long;

% 原参数：
% mass_of_Mainbody = 1524.76209213;
% center_of_mass_MainBody = [13.71923256; -0.22808627; 34.91864017];
% inertia_tensor_MainBody = [4274811.10362144, 21823.60087554, 202865.50474913;
%                            21823.60087554, 7103674.50655196, 7275.19023018;
%                            202865.50474913, 7275.19023018, 8108785.33349067];
mass_of_Mainbody = 948.127;  % 质量（克）
center_of_mass_MainBody = [15.484; -0.147; 34.389];  % 重心位置（mm）
inertia_tensor_MainBody = [2636000, 4369.971, 70032.962;
                           4369.971, 4630000, 3016.742;
                           70032.962, 3016.742, 5330000];  % 相对于重心的转动惯量 (g mm^2)


% 原参数：
% mass_of_ThighLink_Active_Left = 42.41994494;
% center_of_mass_ThighLink_Active_Left = [-48.49051712; 4.61247327; 2.04421822];
% inertia_tensor_ThighLink_Active_Left = [5008.67928627, 6441.33404437, -848.78989403;
%                                         6441.33404437, 48822.93805935, 404.75116121;
%                                         -848.78989403, 404.75116121, 49903.74714712];
mass_of_ThighLink_Active_Left = 19.091;
center_of_mass_ThighLink_Active_Left = [-40.856192; 3.532000; 3.316360];
inertia_tensor_ThighLink_Active_Left = [2861.914, 3470.281, -126.016;
                                        3470.281, 20707.064, 223.834;
                                        -126.016, 223.834, 21158.111];

% 原参数：
% mass_of_ThighLink_Active_Right = 42.41994494;
% center_of_mass_ThighLink_Active_Right = [-48.49050768; -4.61247326; 2.04421032];
% inertia_tensor_ThighLink_Active_Right = [5008.68060146, -6441.33591254, -848.79062342;
%                                          -6441.33591254, 48822.95563592, -404.74966887;
%                                          -848.79062342, -404.74966887, 49903.76341241];
mass_of_ThighLink_Active_Right = 19.091;
center_of_mass_ThighLink_Active_Right = [-40.856192; -3.532000; 3.316360];
inertia_tensor_ThighLink_Active_Right = [2861.914, -3470.281, -126.018;
                                         -3470.281, 20707.060, -223.833;
                                         -126.018, -223.833, 21158.106];

% 原参数：
% mass_of_ThighLink_Passive_Left = 38.26139565;
% center_of_mass_ThighLink_Passive_Left = [-77.93299656; 10.41097168; -3.75891919];
% inertia_tensor_ThighLink_Passive_Left = [5119.23827939, 5810.74900473, 2235.15477477;
%                                          5810.74900473, 58048.93720347, -778.21403282;
%                                          2235.15477477, -778.21403282, 58325.14231362];
mass_of_ThighLink_Passive_Left = 19.314;
center_of_mass_ThighLink_Passive_Left = [-63.299815; 9.192000; -5.740369];
inertia_tensor_ThighLink_Passive_Left = [3042.430, 3456.981, 90.386;
                                         3456.981, 32115.604, -461.757;
                                         90.386, -461.757, 32052.001];

% 原参数：
% mass_of_ThighLink_Passive_Right = 38.26139565;
% center_of_mass_ThighLink_Passive_Right = [-77.93299656; -10.41097168; -3.75891919];
% inertia_tensor_ThighLink_Passive_Right = [5119.23827329, -5810.74900488, 2235.15477431;
%                                           -5810.74900488, 58048.93720431, 778.21403278;
%                                           2235.15477431, 778.21403278, 58325.14232056];
mass_of_ThighLink_Passive_Right = 19.314;
center_of_mass_ThighLink_Passive_Right = [-63.299815; -9.192000; -5.740369];
inertia_tensor_ThighLink_Passive_Right = [3042.430, -3456.981, 90.386;
                                          -3456.981, 32115.604, 461.757;
                                          90.386, 461.757, 32052.001];

% 原参数：
% mass_of_Calf_Link_Left = 319.23782393;
% center_of_mass_Calf_Link_Left = [172.54736867; -3.72877629; 7.27850779];
% inertia_tensor_Calf_Link_Left = [101346.64715298, -5353.69484487, -27779.40548575;
%                                  -5353.69484487, 703160.42663078, -258.16655555;
%                                  -27779.40548575, -258.16655555, 676774.26748659];
mass_of_Calf_Link_Left = 341.503;
center_of_mass_Calf_Link_Left = [176.363988; -2.977000; 6.498837];
inertia_tensor_Calf_Link_Left = [88801.584, -5916.959, -17279.432;
                                 -5916.959, 516300.000, -256.839;
                                 -17279.432, -256.839, 491998.416];

% 原参数：
% mass_of_Calf_Link_Right = 319.23782393;
% center_of_mass_Calf_Link_Right = [172.54753946; 3.72875356; 7.27718211];
% inertia_tensor_Calf_Link_Right = [101347.04479182, 5354.04268934, -27779.76191434;
%                                   5354.04268934, 703157.66050555, 252.37915614;
%                                   -27779.76191434, 252.37915614, 676771.26272286];
mass_of_Calf_Link_Right = 341.503;
center_of_mass_Calf_Link_Right = [168.662856; 3.578000; 6.154992];
inertia_tensor_Calf_Link_Right = [106143.753, 8301.241, -32322.310;
                                  8301.241, 895700.000, 350.165;
                                  -32322.310, 350.165, 864256.247];

% 原参数：
% mass_of_Wheel = 237.11770281;
% center_of_mass_Wheel = [-0.00000687; 0.43740164; -0.00000028];
% inertia_tensor_Wheel = [352917.56444663, -0.00684532, -0.00133537;
%                         -0.00684532, 676120.35437132, 0.00006188;
%                         -0.00133537, 0.00006188, 352917.58100268];
mass_of_Wheel = 126.059;
center_of_mass_Wheel = [0; 0.537; 0];
inertia_tensor_Wheel = [177500, 0, 0; 0, 339000, 0; 0, 0, 177500];

% 原参数：
% mass_of_Wheel_Right = 214.11770281;
% center_of_mass_Wheel_Right = [-0.00000761; 0.48438625; -0.00000031];
% inertia_tensor_Wheel_Right = [312911.58508430, -0.00692278, -0.00051598;
%                               -0.00692278, 598903.10955900, 0.00005987;
%                               -0.00051598, 0.00005987, 312911.64073952];
mass_of_Wheel_Right = 126.059;
center_of_mass_Wheel_Right = [0; 0.537; 0];
inertia_tensor_Wheel_Right = [177500, 0, 0; 0, 339000, 0; 0, 0, 177500];


% link_parameter
properties.a = 75 * cos(pi/6) * 1e-3;   % (m)
properties.b = 75 * sin(pi/6) * 1e-3;   % (m)
properties.l1 = 106 * 1e-3;             % (m)
properties.l2 = 77 * 1e-3;              % (m)
properties.l3 = 50 * 1e-3;              % (m)
properties.l4 = 137 * 1e-3;             % (m)
properties.l5 = 8 * 1e-3;               % (m)


properties.L = 0.123;              % Distance between center and wheel (m)
properties.R = 0.0725;             % Wheel Radius (m)


% 每个链接的 CoM 偏移量（顺序：[Body, TAR, TAL, TPR, TPL, CR, CL]）
properties.c_vectors = [
    center_of_mass_MainBody, ...
    center_of_mass_ThighLink_Active_Right, ...
    center_of_mass_ThighLink_Active_Left, ...
    center_of_mass_ThighLink_Passive_Right, ...
    center_of_mass_ThighLink_Passive_Left, ...
    center_of_mass_Calf_Link_Right, ...
    center_of_mass_Calf_Link_Left
    ];
properties.c_vectors = properties.c_vectors * 1e-3; % 单位换算（毫米到米）

% 每个链接的质量（顺序：[Body、TAR、TAL、TPR、TPL、CR、CL]）
properties.masses = [
    mass_of_Mainbody, ...
    mass_of_ThighLink_Active_Right, ...
    mass_of_ThighLink_Active_Left, ...
    mass_of_ThighLink_Passive_Right, ...
    mass_of_ThighLink_Passive_Left, ...
    mass_of_Calf_Link_Right, ...
    mass_of_Calf_Link_Left
    ];
properties.masses = properties.masses * 1e-3; % 单位换算（克换算为公斤）

% 每个链接的基于 CoM 的转动惯量张量（顺序：[Body, TAR, TAL, TPR, TPL, CR, CL]）
properties.IG_matrices(:,:,1) = inertia_tensor_MainBody * 1e-9;
properties.IG_matrices(:,:,2) = inertia_tensor_ThighLink_Active_Right * 1e-9;
properties.IG_matrices(:,:,3) = inertia_tensor_ThighLink_Active_Left * 1e-9;
properties.IG_matrices(:,:,4) = inertia_tensor_ThighLink_Passive_Right * 1e-9;
properties.IG_matrices(:,:,5) = inertia_tensor_ThighLink_Passive_Left * 1e-9;
properties.IG_matrices(:,:,6) = inertia_tensor_Calf_Link_Right * 1e-9;
properties.IG_matrices(:,:,7) = inertia_tensor_Calf_Link_Left * 1e-9;

properties.m_LW = mass_of_Wheel * 1e-3;        % 单位换算（克换算为公斤）
properties.m_RW = mass_of_Wheel_Right * 1e-3;  % 单位换算（克换算为公斤）

properties.I_B_RW = inertia_tensor_Wheel_Right * 1e-9; % 单位换算（g mm^2 至 kg m^2）
properties.I_B_LW = inertia_tensor_Wheel * 1e-9;       % 单位换算（g mm^2 至 kg m^2）

properties.g = 9.80665;    % gravity acceleration (m/s^2)

save('dynamic_properties.mat', "properties");
