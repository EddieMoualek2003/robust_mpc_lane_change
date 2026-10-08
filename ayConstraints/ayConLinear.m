function [E, F, G] = ayConLinear(s_fun, x, u, aymax)
    % Soft |a_y| <= aymax, a_y = c_vy*vy + c_w*omega + c_d*delta, coefficients at (x, u)
    c = s_fun.ayCoeffF(x, u);                 % [c_vy, c_w, c_d]
    E = [0,  c(3);  0, -c(3)];                % columns: u = [ax, delta]
    F = [0 0 0 0  c(1)  c(2);
         0 0 0 0 -c(1) -c(2)];                % columns: y = [X Y psi vx vy omega]
    G = [aymax; aymax];
end