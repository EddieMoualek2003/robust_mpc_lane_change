%% House keeping
% clc; clear; close all;

fprintf("Lange Change Model Predictive Control \n")
fprintf("\t - Non-Linear Plant\n")
fprintf("\t - LTI Controller\n")

addpath("controller_models\", "functions\", "plant_models\", "simulation_frameworks\");

%% Parameter Definition
params = paramDefs();
fprintf("Test:\t vxDesign: %f \n\t vxSim: %f \n", params.vxDesign, params.vxSim);

%% Generate the system functions
[f_sym, g_sym, x_sym, u_sym] = dynamic_bicycle(params);
[linearisedSystem, plant_d, params] = lineariseSystem(params, f_sym, g_sym, x_sym, u_sym, params.xLin, params.u0);

% Construct the MPC object based on the linearised plant
[xmpc, mpcobj] = mpcObjConstruction(params, plant_d);
N    = params.numSamples;

xAbs = params.x0; % Start with the initial absolute x
uAbs = params.u0;
ref = refPath(params);
Ylog = zeros(params.numSamples, 6);
Xlog = zeros(N, 6);  Ulog = zeros(N, 2);  AYlog = zeros(N, 1);  QPflag = zeros(N, 1);


for k = 1:params.numSamples
    yRef = ref(k+1:k+params.Np, :);
    
    [uAbs, info, ymDev] = ltiControllerStep(params, xAbs, uAbs, yRef, k, mpcobj, xmpc); 
    [xNext, ay] = nonlinearPlantStep(xAbs, uAbs, k, params);
    Ylog(k,:) = ymDev';
    Xlog(k,:) = xAbs.';  Ulog(k,:) = uAbs.';  AYlog(k) = ay;  QPflag(k) = info.Iterations;
    xAbs = xNext;
end

key = sprintf('d%d_s%d', params.vxDesign, params.vxSim);
runs.(key) = struct('Y', Xlog(:,2), 'U', Ulog, 'AY', AYlog, 'QP', QPflag);