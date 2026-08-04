function fu = calculate_fu(M, B)
    fu = zeros(4,2);
    fu(2:4,:) = M \ B;
end