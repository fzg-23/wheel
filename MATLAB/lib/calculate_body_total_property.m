function [mass_total, r_total, I_total] = calculate_body_total_property(h, phi, properties)
% 주어진 height과 roll angle에 대해서, system의 Inertia tensor를 계산한다. 이때 frame과 중심은 IMU를 기준으로 한다.
theta_hips = solve_inverse_kinematics(h, phi, properties);

% Extract properties
c_vectors = properties.c_vectors;
masses = properties.masses;
IG_matrices = properties.IG_matrices;

mass_total = sum(masses);

p_vectors = calculate_p_vectors(theta_hips, properties);
R_matrices = calculate_R_matrices(theta_hips, properties);
r_total = zeros(3,1);
I_total = zeros(3,3);

% calculate Body_CoM_offset
for i = 1:7
    r_Bi = p_vectors(:,i) + R_matrices(:,:,i) * c_vectors(:,i);
    r_total = r_total + masses(i) * r_Bi;
end
r_total = r_total / mass_total;

% calculate Body_CoM_Inertia_Tensor
for i = 1:7
    p = p_vectors(:,i);
    R = R_matrices(:,:,i);
    c = c_vectors(:,i);
    m = masses(i);

    IG_ii = IG_matrices(:,:,i);
    r_Bi = p + R * c;
    r_CoMi = r_Bi - r_total;

    IG_Bi =  R * IG_ii * R' + m*(norm(r_CoMi)^2*eye(3,3) - r_CoMi * r_CoMi');
    I_total = I_total + IG_Bi;
end
end