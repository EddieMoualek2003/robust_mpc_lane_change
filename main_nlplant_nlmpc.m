clc; clear; close all;
fprintf("Lane Change Model Predictive Control \n")
fprintf("\t - Non-linear plant\n")
fprintf("\t - Non-linear controller\n")
addpath("controller_models/", "functions/", "plant_models/", "simulation_frameworks/", "ayConstraints/");

%% Parameters and system
params = paramDefs();
[s_sym, s_fun] = systemDefinition(params);   % single source of f, g, a_y and Jacobians

nx = numel(params.x0);
nu = numel(params.u0);
ny = numel(s_fun.gF(params.x0, params.u0));

%% Controller object
nlobj = nlmpc(nx, ny, nu);                   % outputs before inputs
nlobj.Ts                = params.Ts;
nlobj.PredictionHorizon = params.Np;
nlobj.ControlHorizon    = params.Nc;

% Model and Jacobians (continuous; nlmpc discretises internally)
nlobj.Model.StateFcn         = s_fun.fF;
nlobj.Model.IsContinuousTime = true;
nlobj.Jacobian.StateFcn      = s_fun.jacF;
nlobj.Model.OutputFcn        = s_fun.gF;     % set output fcn and its Jacobian as a pair
nlobj.Jacobian.OutputFcn     = s_fun.CcF;

%% Weights (shared with LTI/LTV via params)
nlobj.Weights.OutputVariables          = params.W.y;
nlobj.Weights.ManipulatedVariables     = params.W.u;
nlobj.Weights.ManipulatedVariablesRate = params.W.du;
nlobj.Weights.ECR                      = params.W.ecr;
% NOTE: if ScaleFactors are set here, set the same ones on mpcobj,
% otherwise identical weights no longer mean identical costs.

%% MV constraints (shared via params)
for i = 1:nu
    nlobj.MV(i).Min     = params.mvMin(i);
    nlobj.MV(i).Max     = params.mvMax(i);
    nlobj.MV(i).RateMin = -params.mvRate(i);
    nlobj.MV(i).RateMax =  params.mvRate(i);
end

%% Lateral acceleration comfort constraint (soft, nonlinear)
nlobj.Optimization.CustomIneqConFcn = ...
    @(X,U,e,data) ayConNonlinear(X, U, e, data, s_fun.ayF, params.ay_max);

%% Check everything
validateFcns(nlobj, params.x0, params.u0);

%% Reference (absolute: cruise baseline + lane-change offset)
ref    = refPath(params, s_fun);
nSteps = params.numSamples;
assert(size(ref,1) >= nSteps + params.Np + 1, ...
    'ref must cover numSamples + Np + 1 samples for the final horizon slice');

%% Closed-loop simulation
x      = params.x0;
lastmv = params.u0;
opt    = nlmpcmoveopt;

xLog     = zeros(nSteps, nx);    % row k = x at t = (k-1)*Ts, aligned with ref row k
uLog     = zeros(nSteps, nu);    % row k = u applied over [t_k, t_k + Ts]
ayLog    = zeros(nSteps, 1);     % row k = a_y at (x_k, u_k)
exitFlag = zeros(nSteps, 1);
tSolve   = zeros(nSteps, 1);

for k = 1:nSteps
    % Controller: horizon reference y(k+1) ... y(k+Np)
    refH = ref(k+1 : k+params.Np, :);
    tic;
    [mv, opt, info] = nlmpcmove(nlobj, x, lastmv, refH, [], opt);   % opt in/out = warm start
    tSolve(k) = toc;

    % Solver check: >0 converged, 0 iteration limit, <0 failed
    exitFlag(k) = info.ExitFlag;
    if info.ExitFlag < 0
        warning('NLP failed at step %d', k);
    end

    % Plant step (same model, ode45, mv held over Ts)
    [xNext, ay] = nonlinearPlantStep(x, mv, s_fun, params);

    xLog(k,:) = x';
    uLog(k,:) = mv';
    ayLog(k)  = ay;

    x      = xNext;
    lastmv = mv;
    fprintf('Completed step %d \n', k);
end

fprintf("Exit flags: %d converged, %d at iteration limit, %d failed\n", ...
    sum(exitFlag > 0), sum(exitFlag == 0), sum(exitFlag < 0));
fprintf("Solve time: mean %.3f s, max %.3f s\n", mean(tSolve), max(tSolve));