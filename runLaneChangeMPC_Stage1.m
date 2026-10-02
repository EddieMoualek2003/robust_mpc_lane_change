%% Housekeeping
clc; clear; close all;
addpath('functions\', 'simulation_frameworks\');

%% Parameter Definition
params = paramDefs();

%% Generate te system functions
[f_sym, g_sym, x_sym, u_sym] = dynamic_bicycle(params);

%% Construct the plant
% Operating point definition
x0 = [0; 0; 0; params.vx0; 0; 0]; % Assume initially cruising at 15m/s.
u0 = [0; 0];

% Get the discretised system matrices
[Ad,Bd,Cd,Dd,plant_d] = plantConstruction(params, f_sym, g_sym, x_sym, u_sym, x0, u0);

discrete_ss.Ad = Ad;
discrete_ss.Bd = Bd;
discrete_ss.Cd = Cd;
discrete_ss.Dd = Dd;
discrete_ss.plant_d = plant_d;

%% Construct the MPC Object
[xmpc, mpcobj] = mpcObjConstruction(params, plant_d);

%% Define the reference path
ref = refPath(params);

%% Run the Simulation Frameworks
[x_mpcmove, y_mpcmove, u_mpcmove, QPlog_mpcmove, Slacklog_mpcmove, Itlog_mpcmove, aylog_mpcmove, runTime_mpcmove] = mpcmoveMPCSimulation(xmpc, mpcobj, params, Ad,Bd,Cd,Dd,plant_d, ref);
[y_sim, t_sim, u_sim, xp_sim, xc_sim, runTime_sim] = simMPCSimulation(ref, params, mpcobj);

%% Prepare the data for analysis
ref = ref(1:params.numSamples, :);
y.mpcmove = y_mpcmove; y.simdata = y_sim;