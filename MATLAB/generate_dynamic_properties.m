clc;clear;close all;
addpath("lib\");
format long;

mass_of_Mainbody = 1524.76209213;  % 질량 (g)
center_of_mass_MainBody = [13.71923256; -0.22808627; 34.91864017];  % 무게 중심 위치 (mm)
inertia_tensor_MainBody = [4274811.10362144, 21823.60087554, 202865.50474913;
                           21823.60087554, 7103674.50655196, 7275.19023018;
                           202865.50474913, 7275.19023018, 8108785.33349067];  % 무게 중심 기준 관성 모멘트 (g mm^2)


mass_of_ThighLink_Active_Left = 42.41994494;  % 질량 (g)
center_of_mass_ThighLink_Active_Left = [-48.49051712; 4.61247327; 2.04421822];  % 무게 중심 위치 (mm)
inertia_tensor_ThighLink_Active_Left = [5008.67928627, 6441.33404437, -848.78989403;
                                        6441.33404437, 48822.93805935, 404.75116121;
                                        -848.78989403, 404.75116121, 49903.74714712];  % 무게 중심 기준 관성 모멘트 (g mm^2)

mass_of_ThighLink_Active_Right = 42.41994494;  % 질량 (g)
center_of_mass_ThighLink_Active_Right = [-48.49050768; -4.61247326; 2.04421032];  % 무게 중심 위치 (mm)
inertia_tensor_ThighLink_Active_Right = [5008.68060146, -6441.33591254, -848.79062342;
                                         -6441.33591254, 48822.95563592, -404.74966887;
                                         -848.79062342, -404.74966887, 49903.76341241];  % 무게 중심 기준 관성 모멘트 (g mm^2)

mass_of_ThighLink_Passive_Left = 38.26139565;  % 질량 (g)
center_of_mass_ThighLink_Passive_Left = [-77.93299656; 10.41097168; -3.75891919];  % 무게 중심 위치 (mm)
inertia_tensor_ThighLink_Passive_Left = [5119.23827939, 5810.74900473, 2235.15477477;
                                          5810.74900473, 58048.93720347, -778.21403282;
                                          2235.15477477, -778.21403282, 58325.14231362];  % 무게 중심 기준 관성 모멘트 (g mm^2)

mass_of_ThighLink_Passive_Right = 38.26139565;  % 질량 (g)
center_of_mass_ThighLink_Passive_Right = [-77.93299656; -10.41097168; -3.75891919];  % 무게 중심 위치 (mm)
inertia_tensor_ThighLink_Passive_Right = [5119.23827329, -5810.74900488, 2235.15477431;
                                           -5810.74900488, 58048.93720431, 778.21403278;
                                           2235.15477431, 778.21403278, 58325.14232056];  % 무게 중심 기준 관성 모멘트 (g mm^2)

mass_of_Calf_Link_Left = 319.23782393;  % 질량 (g)
center_of_mass_Calf_Link_Left = [172.54736867; -3.72877629; 7.27850779];  % 무게 중심 위치 (mm)
inertia_tensor_Calf_Link_Left = [101346.64715298, -5353.69484487, -27779.40548575;
                                  -5353.69484487, 703160.42663078, -258.16665555;
                                  -27779.40548575, -258.16665555, 676774.26748659];  % 무게 중심 기준 관성 모멘트 (g mm^2)

mass_of_Calf_Link_Right = 319.23782393;  % 질량 (g)
center_of_mass_Calf_Link_Right = [172.54753946; 3.72875356; 7.27718211];  % 무게 중심 위치 (mm)
inertia_tensor_Calf_Link_Right = [101347.04479182, 5354.04268934, -27779.76191434;
                                   5354.04268934, 703157.66050555, 252.37915614;
                                   -27779.76191434, 252.37915614, 676771.26272286];  % 무게 중심 기준 관성 모멘트 (g mm^2)

mass_of_Wheel = 237.11770281;  % 질량 (g)
center_of_mass_Wheel = [-0.00000687; 0.43740164; -0.00000028];  % 무게 중심 위치 (mm)
inertia_tensor_Wheel = [352917.56444663, -0.00684532, -0.00133537;
                                  -0.00684532, 676120.35437132, 0.00006188;
                                  -0.00133537, 0.00006188, 352917.58100268];  % 무게 중심 기준 관성 모멘트 (g mm^2)

mass_of_Wheel_Right = 214.11770281;  % 질량 (g)
center_of_mass_Wheel_Right = [-0.00000761; 0.48438625; -0.00000031];  % 무게 중심 위치 (mm)
inertia_tensor_Wheel_Right = [312911.58508430, -0.00692278, -0.00051598;
                                       -0.00692278, 598903.10955900, 0.00005987;
                                       -0.00051598, 0.00005987, 312911.64073952];  % 무게 중심 기준 관성 모멘트 (g mm^2)


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


% 각 link들의 CoM Offset (순서: [Body, TAR, TAL, TPR, TPL, CR, CL])
properties.c_vectors = [
    center_of_mass_MainBody, ...
    center_of_mass_ThighLink_Active_Right, ...
    center_of_mass_ThighLink_Active_Left, ...
    center_of_mass_ThighLink_Passive_Right, ...
    center_of_mass_ThighLink_Passive_Left, ...
    center_of_mass_Calf_Link_Right, ...
    center_of_mass_Calf_Link_Left
    ];
properties.c_vectors = properties.c_vectors * 1e-3; % 단위 변환 (mm to m)

% 각 link들의 질량 (순서: [Body, TAR, TAL, TPR, TPL, CR, CL])
properties.masses = [
    mass_of_Mainbody, ...
    mass_of_ThighLink_Active_Right, ...
    mass_of_ThighLink_Active_Left, ...
    mass_of_ThighLink_Passive_Right, ...
    mass_of_ThighLink_Passive_Left, ...
    mass_of_Calf_Link_Right, ...
    mass_of_Calf_Link_Left
    ];
properties.masses = properties.masses * 1e-3; % 단위 변환 (g to kg)

% 각 link들의 CoM 기준 관성 모멘트 텐서 (순서: [Body, TAR, TAL, TPR, TPL, CR, CL])
properties.IG_matrices(:,:,1) = inertia_tensor_MainBody * 1e-9;
properties.IG_matrices(:,:,2) = inertia_tensor_ThighLink_Active_Right * 1e-9;
properties.IG_matrices(:,:,3) = inertia_tensor_ThighLink_Active_Left * 1e-9;
properties.IG_matrices(:,:,4) = inertia_tensor_ThighLink_Passive_Right * 1e-9;
properties.IG_matrices(:,:,5) = inertia_tensor_ThighLink_Passive_Left * 1e-9;
properties.IG_matrices(:,:,6) = inertia_tensor_Calf_Link_Right * 1e-9;
properties.IG_matrices(:,:,7) = inertia_tensor_Calf_Link_Left * 1e-9;

properties.m_LW = mass_of_Wheel * 1e-3;        % 단위 변환 (g to kg)
properties.m_RW = mass_of_Wheel_Right * 1e-3;  % 단위 변환 (g to kg)

properties.I_B_RW = inertia_tensor_Wheel_Right * 1e-9; % 단위 변환 (g mm^2 to kg m^2)
properties.I_B_LW = inertia_tensor_Wheel * 1e-9;       % 단위 변환 (g mm^2 to kg m^2)

properties.g = 9.80665;    % gravity acceleration (m/s^2)

save('dynamic_properties.mat', "properties");