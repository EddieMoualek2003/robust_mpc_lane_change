function params = paramDefs()

    % Vehicle inertial and geometry
    params.m   = 1500;       % kg
    params.Iz  = 2250;       % kg*m^2
    params.lf  = 1.2;        % m
    params.lr  = 1.4;        % m
    
    % Tire cornering stiffness (axle total)
    params.Cf  = 80000;      % N/rad
    params.Cr  = 85000;      % N/rad
    params.mu  = 0.9;        % friction coefficient
    
    % Operating condition
    params.vx0 = 15.0;       % m/s
    params.Ts  = 0.05;       % s (sampling period)

    % Lateral acceleration
    params.c_vy = -(params.Cf + params.Cr) / (params.m * params.vx0);
    params.c_w  =  (params.lr*params.Cr - params.lf*params.Cf) / (params.m * params.vx0);
    params.c_d  =  params.Cf / params.m;

    % MPC Parameters
    params.Np = 120;
    params.Nc = 20;
    params.Tstop = 50;
    params.numSamples = round(params.Tstop/params.Ts);
    params.samples = (1:params.numSamples);
    params.t = (0:params.numSamples-1)' * params.Ts;

    % System Design Constraints
    params.Jlong = 2.0;
    params.ay_max = 0.98;
    params.T = 8;
    params.Jlat_comfort = 1.0;   % m/s^3, fixed comfort limit: plots and reporting only
    params.Jlat_design  = 0.6;   % m/s^3, steering-rate proxy: tuning knob for dd only
end