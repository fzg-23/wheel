function h = extract_nonlinear_effect(EOM, ddq)
    h = subs(EOM, ddq, zeros(3,1));
    h = simplify(h);
end