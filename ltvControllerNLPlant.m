function ltvControllerNLPlant(params)
%% Define the system functions and 
fprintf("Test:\t vxDesign: %f \n\t vxSim: %f \n", params.vxDesign, params.vxSim);
[s_sym, s_fun] = systemDefinition(params);

end