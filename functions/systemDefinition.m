function [s_sym, s_fun] = systemDefinition(params)
% Symbolic states and inputs
    syms pos_x pos_y psi_angle vx vy omega real
    syms ax delta_angle real
    x_sym = [pos_x; pos_y; psi_angle; vx; vy; omega];
    u_sym = [ax; delta_angle];
% Physical constants
    m  = params.m;   Iz = params.Iz;
    lf = params.lf;  lr = params.lr;
    Cf = params.Cf;  Cr = params.Cr;
% Linear tyre slip angles (vx is a state; linearised controllers substitute their operating point)
    alpha_f = delta_angle - (vy + lf * omega) / vx;
    alpha_r = - (vy - lr * omega) / vx;
% Lateral tyre forces
    Fyf = Cf * alpha_f;
    Fyr = Cr * alpha_r;
% Lateral acceleration felt by the passenger
    ay_sym = (Fyf + Fyr) / m;
% State derivative f(x, u)
    f_sym = sym(zeros(6, 1));
    f_sym(1) = vx * cos(psi_angle) - vy * sin(psi_angle);   % d(px)/dt
    f_sym(2) = vx * sin(psi_angle) + vy * cos(psi_angle);   % d(py)/dt
    f_sym(3) = omega;                                       % d(psi)/dt
    f_sym(4) = ax + vy * omega;                             % d(vx)/dt
    f_sym(5) = ay_sym - vx * omega;                         % d(vy)/dt = a_y - vx*omega
    f_sym(6) = (lf * Fyf - lr * Fyr) / Iz;                  % d(omega)/dt
% Output map y = g(x, u): full state
    g_sym = x_sym;

% Jacobians (continuous-time state-space matrices)
    Ac_sym = jacobian(f_sym, x_sym);
    Bc_sym = jacobian(f_sym, u_sym);
    Cc_sym = jacobian(g_sym, x_sym);
    Dc_sym = jacobian(g_sym, u_sym);

% Symbolic struct
    s_sym.x  = x_sym;   s_sym.u  = u_sym;
    s_sym.f  = f_sym;   s_sym.g  = g_sym;   s_sym.ay = ay_sym;
    s_sym.Ac = Ac_sym;  s_sym.Bc = Bc_sym;
    s_sym.Cc = Cc_sym;  s_sym.Dc = Dc_sym;

% Numeric function handles, all with signature (x, u)
    v = {x_sym, u_sym};
    s_fun.fF   = matlabFunction(f_sym,  'Vars', v);
    s_fun.gF   = matlabFunction(g_sym,  'Vars', v);
    s_fun.ayF  = matlabFunction(ay_sym, 'Vars', v);
    s_fun.AcF  = matlabFunction(Ac_sym, 'Vars', v);
    s_fun.BcF  = matlabFunction(Bc_sym, 'Vars', v);
    s_fun.CcF  = matlabFunction(Cc_sym, 'Vars', v);
    s_fun.DcF  = matlabFunction(Dc_sym, 'Vars', v);
    s_fun.jacF = matlabFunction(Ac_sym, Bc_sym, 'Vars', v);   % [A, B] pair for nlmpc
    s_fun.ayCoeffF = matlabFunction(jacobian(ay_sym, [vy omega delta_angle]), 'Vars', v);   % NEW: [c_vy, c_w, c_d]
end