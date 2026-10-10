function [Ylog, Xlog, Ulog, AYlog, QPflag, run] = ltiControllerNLPlant(params)
%% System
fprintf("Test:\t vxDesign: %f \n\t vxSim: %f \n", params.vxDesign, params.vxSim);
[~, s_fun] = systemDefinition(params);

%% Linearise once at the operating point (vxDesign)
xLin = params.xLin;   uLin = params.uLin;
nx = numel(xLin);     nu = numel(uLin);
Ac = s_fun.AcF(xLin, uLin);   Bc = s_fun.BcF(xLin, uLin);
Cc = s_fun.CcF(xLin, uLin);   Dc = s_fun.DcF(xLin, uLin);
f0 = s_fun.fF(xLin, uLin);
plant_d = c2d(ss(Ac, Bc, Cc, Dc), params.Ts, 'zoh');
plant_d.InputName  = {'Longitudinal Acceleration'; 'Steering Angle'};
plant_d.InputUnit  = {'m/s^2'; 'rad'};
plant_d.OutputName = {'Longitudinal Position'; 'Lateral Position'; 'Yaw Angle'; ...
    'Longitudinal Velocity'; 'Lateral Velocity'; 'Yaw Rate'};
plant_d.OutputUnit = {'m'; 'm'; 'rad'; 'm/s'; 'm/s'; 'rad/s'};
plant_d.StateName  = {'X Position'; 'Y Position'; 'Yaw Angle'; ...
    'Longitudinal Velocity'; 'Lateral Velocity'; 'Yaw Rate'};
plant_d.StateUnit  = {'m'; 'm'; 'rad'; 'm/s'; 'm/s'; 'rad/s'};
% Nominal at the same point: DX is the one-step drift from f0
M    = expm([Ac Bc f0; zeros(nu+1, nx+nu+1)] * params.Ts);
nom0 = struct('X', xLin, 'U', uLin, 'Y', s_fun.gF(xLin, uLin), 'DX', M(1:nx, end));

%% MPC object
plant_d = setmpcsignals(plant_d, 'MO', 1:4, 'UO', 5:6);
mpcobj  = mpc(plant_d, params.Ts, params.Np, params.Nc);
mpcobj.Model.Nominal = nom0;
mpcobj.Weights.OutputVariables          = [0, 10, 2, 2, 0, 0];   % [X, Y, psi, vx, vy, omega]
mpcobj.Weights.ManipulatedVariables     = [0, 0];                % [ax, delta]
mpcobj.Weights.ManipulatedVariablesRate = [2, 100];
% Longitudinal: acceleration and jerk
mpcobj.MV(1).Min     = -0.8;
mpcobj.MV(1).Max     =  0.8;
mpcobj.MV(1).RateMin = -params.Jlong * params.Ts;
mpcobj.MV(1).RateMax =  params.Jlong * params.Ts;
% Steering: actuator limit and comfort rate limit
dd = params.Jlat_design * params.Ts * params.m / params.Cf;
mpcobj.MV(2).Min     = -0.5;
mpcobj.MV(2).Max     =  0.5;
mpcobj.MV(2).RateMin = -dd;
mpcobj.MV(2).RateMax =  dd;
% Lateral acceleration (soft): a_y = c_vy*vy + c_w*omega + c_d*delta, frozen at xLin
c = s_fun.ayCoeffF(xLin, uLin);           % [c_vy, c_w, c_d]
E = [0,  c(3); 0, -c(3)];
F = [0 0 0 0  c(1)  c(2);
    0 0 0 0 -c(1) -c(2)];
G = [params.ay_max; params.ay_max];
setconstraint(mpcobj, E, F, G, [1; 1]);
setoutdist(mpcobj, 'model', tf(zeros(6,1)));
setEstimator(mpcobj, 'custom');
xmpc = mpcstate(mpcobj);                  % after Nominal is set

%% Reference (absolute)
ref = refPath(params, s_fun);

%% Closed loop
N    = params.numSamples;
xAbs = params.x0;
uAbs = params.u0;                         % holds u_{k-1} at the top of each iteration
Ylog  = zeros(N, 6);  Xlog   = zeros(N, 6);  Ulog = zeros(N, 2);
AYlog = zeros(N, 1);  QPflag = zeros(N, 1);
SLlog = zeros(N, 1);  Jlog   = zeros(N, 1);
for k = 1:N
    yAbs = s_fun.gF(xAbs, uAbs);          % y_k = g(x_k, u_{k-1})
    xmpc.Plant = xAbs;
    yRef = ref(k+1:k+params.Np, :);

    [uAbs, info] = mpcmove(mpcobj, xmpc, yAbs(1:4), yRef);

    % Plant step (same model, ode45, mv held over Ts)
    [xNext, ay] = nonlinearPlantStep(xAbs, uAbs, s_fun, params);

    Ylog(k,:) = yAbs.';  Xlog(k,:) = xAbs.';  Ulog(k,:) = uAbs.';
    AYlog(k)  = ay;      QPflag(k) = info.Iterations;
    SLlog(k)  = info.Slack;  Jlog(k) = info.Cost;
    xAbs = xNext;
end

key = matlab.lang.makeValidName(sprintf('d%g_s%g', params.vxDesign, params.vxSim));
run = struct('key', key, 't', (0:N-1).' * params.Ts, ...
    'Y', Ylog, 'X', Xlog, 'U', Ulog, 'AY', AYlog, ...
    'QP', QPflag, 'Slack', SLlog, 'Cost', Jlog, ...
    'Ref', ref(1:N, :), 'params', params);
end