function [Ad,Bd,Cd,Dd,plant_d] = plantConstruction(params, f_sym, g_sym, x_sym, u_sym, x0, u0)
    %% Linearise the system and evaluate at the operating point
    Ac_sym = jacobian(f_sym, x_sym);
    Bc_sym = jacobian(f_sym, u_sym);
    Cc_sym = jacobian(g_sym, x_sym);
    Dc_sym = jacobian(g_sym, u_sym);

    % Operating point definition
    x0 = [0; 0; 0; params.vx0; 0; 0]; % Assume initially cruising at 15m/s.
    u0 = [0; 0];

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
    [Ad,Bd,Cd,Dd] = ssdata(plant_d);            % must be the same Ts as mpcobj
end