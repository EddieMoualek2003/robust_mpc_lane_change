function params = paramDefs(o)
    arguments
        o.vxSim (1,1) double {mustBePositive} = 20   % m/s, plant's actual speed
    end
% Vehicle inertia and geometry
    params.m   = 1500;       % kg
    params.Iz  = 2250;       % kg*m^2
    params.lf  = 1.2;        % m
    params.lr  = 1.4;        % m
% Tyre cornering stiffness (axle total)
    params.Cf  = 80000;      % N/rad
    params.Cr  = 85000;      % N/rad
    params.mu  = 0.9;        % friction coefficient
% Operating conditions
    params.vxDesign = 15;                               % m/s, LTI controller's operating point
    params.vxSim    = o.vxSim;                              % m/s, plant's actual speed
    params.xLin = [0; 0; 0; params.vxDesign; 0; 0];     % LTI operating point (state)
    params.uLin = [0; 0];                               % LTI operating point (input)
    params.x0   = [0; 0; 0; params.vxSim;    0; 0];     % plant initial state
    params.u0   = [0; 0];                               % input applied before the first step

% Timing and horizons
    params.Ts    = 0.05;     % s
    params.Np    = 120;
    params.Nc    = 20;
    params.Tstop = 50;       % s
    params.numSamples = round(params.Tstop/params.Ts);
    params.samples    = (1:params.numSamples);
    params.t          = (0:params.numSamples-1)' * params.Ts;
% Design constraints
    params.Jlong        = 2.0;    % m/s^3, longitudinal jerk limit
    params.ay_max       = 0.98;   % m/s^2, lateral acceleration comfort limit
    params.T            = 8;      % s, lane-change duration
    params.Jlat_comfort = 1.0;    % m/s^3, fixed comfort limit: plots and reporting only
    params.Jlat_design  = 0.6;    % m/s^3, steering-rate proxy: tuning knob for dd only

    % Weights — [X Y psi vx vy omega], [ax delta]
    params.W.y   = [0, 10, 2, 2, 0, 0];
    params.W.u   = [0, 0];
    params.W.du  = [2, 100];
    params.W.ecr = 1e5;

    % MV limits — define after Ts, m, Cf, Jlong, Jlat_design
    params.mvMin  = [-0.8; -0.5];
    params.mvMax  = [ 0.8;  0.5];
    params.mvRate = [params.Jlong * params.Ts;
        params.Jlat_design * params.Ts * params.m / params.Cf];   % dd
end