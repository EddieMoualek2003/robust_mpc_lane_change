% clc; clear; close all;

fprintf("Lange Change Model Predictive Control \n")
fprintf("\t - Non-Linear Plant\n")
fprintf("\t - LTI Controller\n")

addpath("controller_models\", "functions\", "plant_models\", "simulation_frameworks\");

% Parameter Definition
params = paramDefs();
vxDesignArray = [10, 15, 20, 25];
vxSimArray = [10, 15, 20, 25];

% Define the system
params = paramDefParameterised(vxDesignArray(3), vxSimArray(3));
[f_sym, g_sym, x_sym, u_sym] = dynamic_bicycle(params);

%Define the linearised system as a function of symbolic functions
AcF = matlabFunction(jacobian(f_sym, x_sym), 'Vars', {x_sym, u_sym});
BcF = matlabFunction(jacobian(f_sym, u_sym), 'Vars', {x_sym, u_sym});
CcF = matlabFunction(jacobian(g_sym, x_sym), 'Vars', {x_sym, u_sym});
DcF = matlabFunction(jacobian(g_sym, u_sym), 'Vars', {x_sym, u_sym});
[linearisedSystem, plant_d, params] = lineariseSystem(params, f_sym, g_sym, x_sym, u_sym);

% Get the key sizes for the affine term
nx = size(x_sym, 1); nu = size(u_sym, 1); N = params.numSamples;

[xmpc, mpcobj] = mpcObjConstruction(params, plant_d);

xAbs = params.x0; uPrev = params.u0;
ref = refPath(params);
Xlog = zeros(N, 6); Ylog = zeros(params.numSamples, 6); Ulog = zeros(N, 2);
AYlog = zeros(N, 1);  QPflag = zeros(N, 1);

for k = 1:params.numSamples
    yRef = ref(k+1:k+params.Np, :);
    yAbsFull  = params.gNum(xAbs, uPrev);
    yAbs = yAbsFull(1:4,1);
    xmpc.Plant = xAbs;

    % Linearise at (x_k, u_{k-1})
    Ac = AcF(xAbs, uPrev); Bc = BcF(xAbs, uPrev);

    % This is for the affine term
    f0 = params.fNum(xAbs, uPrev);
    M  = expm([Ac Bc f0; zeros(nu+1, nx+nu+1)] * params.Ts);

    % Construct the new plant based on the affine linearisation
    plant_k = ss(   M(1:nx,1:nx), ...
                    M(1:nx,nx+1:nx+nu), ...
                    CcF(xAbs, uPrev), ...
                    DcF(xAbs, uPrev), params.Ts);

    nom_k = struct('X', xAbs, 'U', uPrev, 'Y', yAbsFull, 'DX', M(1:nx,end)); % Note: Still have to pass through all outputs, whether measured or unmeasured

    [uAbs, info] = mpcmoveAdaptive(mpcobj, xmpc, plant_k, nom_k, yAbs, yRef);
    [xAbs, ay]   = nonlinearPlantStep(xAbs, uAbs, k, params);
    uPrev = uAbs;

    Ylog(k,:) = yAbsFull';
end

hold on;
plot(params.t, Ylog(:, 2), params.t, ref(1:params.numSamples, 2))