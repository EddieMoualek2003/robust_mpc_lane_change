function [f_sym, g_sym, x_sym, u_sym] = dynamic_bicycle(params)
    % Define symbolic states
    syms pos_x pos_y psi_angle vx vy omega real
    % Define symbolic inputs
    syms ax delta_angle real

    % Pack symbolic vectors
    x_sym = [pos_x; pos_y; psi_angle; vx; vy; omega];
    u_sym = [ax; delta_angle];

    % Physical constants
    m  = params.m;
    Iz = params.Iz;
    lf = params.lf;
    lr = params.lr;
    Cf = params.Cf;
    Cr = params.Cr;

    % Linear tire slip angles (small-angle approximation)
    % vx remains symbolic; evaluated later at nominal trim (e.g., vx0 = 15 m/s)
    alpha_f = delta_angle - (vy + lf * omega) / vx;
    alpha_r = - (vy - lr * omega) / vx;

    % Lateral tire forces
    Fyf = Cf * alpha_f;
    Fyr = Cr * alpha_r;

    % Symbolic state derivative vector f(x, u)
    f_sym = sym(zeros(6, 1));
    f_sym(1) = vx * cos(psi_angle) - vy * sin(psi_angle);            % d(px)/dt
    f_sym(2) = vx * sin(psi_angle) + vy * cos(psi_angle);            % d(py)/dt
    f_sym(3) = omega;                                   % d(psi)/dt
    f_sym(4) = ax + vy * omega;                         % d(vx)/dt
    f_sym(5) = (Fyf + Fyr) / m - vx * omega;            % d(vy)/dt
    f_sym(6) = (lf * Fyf - lr * Fyr) / Iz;              % d(omega)/dt

    % Symbolic state derivative vector g(x, u)
    g_sym = sym(zeros(6, 1));      % was zeros(4, 1)
    g_sym(1) = pos_x;              % Note 1: needed for observability
    g_sym(2) = pos_y;
    g_sym(3) = psi_angle;
    g_sym(4) = vx;
    g_sym(5) = vy;                 % new: lateral velocity (for the a_y constraint)
    g_sym(6) = omega;              % new: yaw rate (for the a_y constraint)
end