function theta_hips = solve_inverse_kinematics(h, phi, properties)
% Extract link length
a = properties.a;
b = properties.b;
l1 = properties.l1;
l2 = properties.l2;
l3 = properties.l3;
l4 = properties.l4;
l5 = properties.l5;

L = properties.L;

% h_饱和度设置
phi_max = min(atan((h - 60*1e-3) / L), atan((200*1e-3 - h) / L));
phi_min = -phi_max;

if phi > phi_max
    phi = phi_max;
elseif phi < phi_min
    phi = phi_min;
end
hR = h - L*tan(phi);
hL = h + L*tan(phi);

hs = [hR; hL];


AB = sqrt(a^2 + b^2);
angle_ADE = acos((l1^2 + l4^2 + l5^2 - hs.^2) / (2 * l1 * sqrt(l4^2 + l5^2)));
angle_EDF = atan(l5 / l4);
angle_ADC = pi - (angle_ADE + angle_EDF);
AC = sqrt(l1^2 + l3^2 - 2 * l1 * l3 * cos(angle_ADC));

S = (AB^2 + l2^2 - AC.^2) / (2 * AB * l2);

% 按元素检查条件
invalid_indices = abs(S) > 1 & abs(AC - sqrt(a^2 + b^2) - l2) < 0.1;
for i = 1:length(S)
    if invalid_indices(i)
        AC(i) = sqrt(a^2 + b^2) + l2;
        S(i) = 1;
    end
end
angle_ABC = real(acos((AB^2 + l2^2 - AC.^2) / (2 * AB * l2)));
theta_hips = (5 * pi / 6) - angle_ABC;

theta_hips(2) = -theta_hips(2);
end