function fx = calculate_fx(M, dM_dtheta, nle, dnle_dtheta, dnle_dqdot)
    fx = zeros(4,4);
    fx(1,2) = 1;
    M_inv = M \ eye(3,3);
    fx(2:4,1) = M_inv * (dM_dtheta * M_inv * nle - dnle_dtheta);
    fx(2:4,2:4) = - M_inv * dnle_dqdot;
    % function from TWIP
end