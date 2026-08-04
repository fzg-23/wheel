function R = eulerZYXtoRotationMatrix(roll, pitch, yaw)
    % roll (phi): rotation about X-axis
    % pitch (theta): rotation about Y-axis
    % yaw (psi): rotation about Z-axis
    
    % Compute trigonometric values
    c1 = cos(yaw);  % cos(psi)
    s1 = sin(yaw);  % sin(psi)
    c2 = cos(pitch); % cos(theta)
    s2 = sin(pitch); % sin(theta)
    c3 = cos(roll);  % cos(phi)
    s3 = sin(roll);  % sin(phi)
    
    % ZYX Rotation Matrix
    R = [c1*c2, c1*s2*s3 - s1*c3, c1*s2*c3 + s1*s3;
         s1*c2, s1*s2*s3 + c1*c3, s1*s2*c3 - c1*s3;
         -s2,   c2*s3,            c2*c3];
end
