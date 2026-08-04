function p_vectors = calculate_p_vectors(theta_hips, properties)
    p_vectors = zeros(3,7);
    [~, ~, ~, c_poses] = solve_forward_kinematics(theta_hips, properties);
    c_posR = c_poses(:,1);
    c_posL = c_poses(:,2);
    
    % Body
    p_vectors(:,1) = [0; 0; 0];
    % TAR
    p_vectors(:,2) = [-64.951905284*1e-3 ; -86*1e-3; 37.5*1e-3];
    % TAL
    p_vectors(:,3) = [-64.951905284*1e-3 ; 86*1e-3; 37.5*1e-3];
    % TPR
    p_vectors(:,4) = [0; -81*1e-3; 0];
    % TPL
    p_vectors(:,5) = [0; 81*1e-3; 0];
    % CR
    p_vectors(:,6) = [c_posR(1); -102*1e-3; c_posR(2)];
    % CL
    p_vectors(:,7) = [c_posL(1); 102*1e-3; c_posL(2)];

end