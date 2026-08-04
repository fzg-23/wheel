function R = angleAxisToRotationMatrix(angle, axis)
    
    % Normalize the axis to ensure it's a unit vector
    axis = axis / norm(axis);
    x = axis(1);
    y = axis(2);
    z = axis(3);
    
    % Compute trigonometric values
    c = cos(angle);
    s = sin(angle);
    one_c = 1 - c;
    
    % Compute the rotation matrix using the Angle-Axis formula
    R = [c + x^2 * one_c, x*y*one_c - z*s, x*z*one_c + y*s;
         y*x*one_c + z*s, c + y^2 * one_c, y*z*one_c - x*s;
         z*x*one_c - y*s, z*y*one_c + x*s, c + z^2 * one_c];
end