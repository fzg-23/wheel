function R_matrices = calculate_R_matrices(theta_hips, properties)
R_matrices = zeros(3,3,7);
[theta_As, theta_Bs, theta_ks, ~] = solve_forward_kinematics(theta_hips, properties);
% Body
R_matrices(:,:,1) = eye(3,3);
% TAR
R_matrices(:,:,2) = rotation_matrix_y(theta_Bs(1));
% TAL
R_matrices(:,:,3) = rotation_matrix_y(theta_Bs(2));
% TPR
R_matrices(:,:,4) = rotation_matrix_y(theta_As(1));
% TPL
R_matrices(:,:,5) = rotation_matrix_y(theta_As(2));
% CR
R_matrices(:,:,6) = rotation_matrix_y(theta_ks(1));
% CL
R_matrices(:,:,7) = rotation_matrix_y(theta_ks(2));
end