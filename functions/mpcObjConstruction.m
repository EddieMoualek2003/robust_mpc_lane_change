function [xmpc, mpcobj] = mpcObjConstruction(params, plant_d)
    %Comfort parameters
    Jlong  = params.Jlong;      % m/s^3, longitudinal jerk limit
    Jlat   = params.Jlat_design;      % m/s^3, lateral jerk limit (design choice)
    ay_max = params.ay_max;     % m/s^2, lateral "So-so"; use 0.64 for "Good"

    % Make vy and omega available to the constraint (after building plant_d)
    plant_d = setmpcsignals(plant_d, 'MO', 1:4, 'UO', 5:6);

    % Create the MPC object
    mpcobj = mpc(plant_d, params.Ts, params.Np, params.Nc);

    % Set the weights
    mpcobj.Weights.OutputVariables          = [0, 10, 2, 2, 0, 0];   % [X, Y, psi, vx, vy, omega]
    mpcobj.Weights.ManipulatedVariables     = [0, 0];                % [ax, delta]
    mpcobj.Weights.ManipulatedVariablesRate = [2, 100];              % [d(ax), d(delta)] per step

    % Longitudinal: acceleration and jerk
    mpcobj.MV(1).Min     = -0.8;
    mpcobj.MV(1).Max     =  0.8;
    mpcobj.MV(1).RateMin = -Jlong * params.Ts;
    mpcobj.MV(1).RateMax =  Jlong * params.Ts;

    % Steering: physical angle limit and comfort rate limit
    dd = Jlat * params.Ts * params.m / params.Cf;   % max change in delta per step (rad)
    mpcobj.MV(2).Min     = -0.5;                    % actuator limit, not comfort
    mpcobj.MV(2).Max     =  0.5;
    mpcobj.MV(2).RateMin = -dd;
    mpcobj.MV(2).RateMax =  dd;

    % Lateral acceleration constraint: a_y = c_vy*vy + c_w*omega + c_d*delta
    c_vy = params.c_vy;
    c_w  = params.c_w;
    c_d  = params.c_d;

    E = [0,  c_d; 0, -c_d];                      % columns: u = [ax, delta]
    F = [0 0 0 0  c_vy  c_w;
        0 0 0 0 -c_vy -c_w];           % columns: y = [X Y psi vx vy omega]
    G = [ay_max; ay_max];

    setconstraint(mpcobj, E, F, G, [1; 1]);   % [1;1] makes both rows soft
    setoutdist(mpcobj, 'model', tf(zeros(6,1)));   % no disturbance integrators on the 4 MOs
    setEstimator(mpcobj, 'custom');                % mpcmove uses xmpc.Plant as given
    xmpc = mpcstate(mpcobj);            % get handle to controller state

end