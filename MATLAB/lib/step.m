function x_pred = step(x, M, nle, B, u, dt)
x_pred = zeros(size(x));
dq = x(2:4);
% disp(dq');

dq_next = dq + M \ (-nle + B * u) * dt;
% disp(dq_next');
x_pred(1) = x(1) + dq_next(1)*dt;
x_pred(2:4) = dq_next;
end