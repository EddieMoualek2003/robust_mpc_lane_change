function [linearisedSystem,plant_d, params] = lineariseSystem(params, f_sym, g_sym, x_sym, u_sym, x0, u0)
    
    %% Linearise the system and evaluate at the operating point
    Ac_sym = jacobian(f_sym, x_sym);
    Bc_sym = jacobian(f_sym, u_sym);
    Cc_sym = jacobian(g_sym, x_sym);
    Dc_sym = jacobian(g_sym, u_sym);

    % Substitute variables and convert symbolic objects to numeric doubles
    Ac = double(subs(Ac_sym, [x_sym; u_sym], [x0; u0]));
    Bc = double(subs(Bc_sym, [x_sym; u_sym], [x0; u0]));
    Cc = double(subs(Cc_sym, [x_sym; u_sym], [x0; u0]));
    Dc = double(subs(Dc_sym, [x_sym; u_sym], [x0; u0]));

    %% Form the state-space model, along with the discrete variant
    plant_c = ss(Ac, Bc, Cc, Dc);
    plant_d = c2d(plant_c, params.Ts, 'zoh');

    % Set Input Names and Units (Manipulated Variables)
    plant_d.InputName = {'Longitudinal Acceleration'; 'Steering Angle'};
    plant_d.InputUnit = {'m/s^2'; 'rad'};

    % Set Output Names and Units (Controlled / Measured Variables)
    plant_d.OutputName = {'Longitudinal Position'; 'Lateral Position'; 'Yaw Angle'; ...
                        'Longitudinal Velocity'; 'Lateral Velocity'; 'Yaw Rate'};
    plant_d.OutputUnit = {'m'; 'm'; 'rad'; 'm/s'; 'm/s'; 'rad/s'};

    % Set State Names and Units
    plant_d.StateName = {'X Position'; 'Y Position'; 'Yaw Angle'; ...
                        'Longitudinal Velocity'; 'Lateral Velocity'; 'Yaw Rate'};
    plant_d.StateUnit = {'m'; 'm'; 'rad'; 'm/s'; 'm/s'; 'rad/s'};
    [linearisedSystem.Ad,linearisedSystem.Bd,linearisedSystem.Cd,linearisedSystem.Dd] = ssdata(plant_d);            % must be the same Ts as mpcobj
    
    params.fNum = matlabFunction(f_sym, 'Vars',{x_sym, u_sym});
    [params.xNomFun, ~] = computeNominal(params.fNum, params.x0, params.u0, params.t);

    params.gNum = matlabFunction(g_sym, 'Vars',{x_sym, u_sym});
    params.yNomFun = @(t) params.gNum(params.xNomFun(t), params.u0);
end