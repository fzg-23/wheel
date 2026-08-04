function M = extract_inertia_matrix(EOM, ddq)
    h = subs(EOM, ddq, zeros(3,1));
    M = EOM-h;
    M1 = subs(M, ddq, [1; 0; 0]);
    M2 = subs(M, ddq, [0; 1; 0]);
    M3 = subs(M, ddq, [0; 0; 1]);
    M = [M1, M2, M3];
    M = simplify(M);
end