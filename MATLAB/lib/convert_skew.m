function v_x = convert_skew(v)
    v_x = [0, -v(3), v(2);
            v(3), 0, -v(1);
            -v(2), v(1), 0];
end