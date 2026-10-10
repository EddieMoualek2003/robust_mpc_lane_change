%% System
if ~exist('firstRun')
    addpath("controller_models\", "functions\", "plant_models\", "simulation_frameworks\");
    clc; clear; close all;
    firstRun = true;
end

%% Run
vxSimArray = linspace(5, 30, 6);

for i = 1:size(vxSimArray, 2)
    params = paramDefs("vxSim",vxSimArray(i));

    % Begin with the LTV case
    [~, ~, ~, ~, ~, r] = ltvControllerNLPlant(params);
    runs_ltv.(r.key) = r;

    [~, ~, ~, ~, ~, r] = ltiControllerNLPlant(params);
    runs_lti.(r.key) = r;
end