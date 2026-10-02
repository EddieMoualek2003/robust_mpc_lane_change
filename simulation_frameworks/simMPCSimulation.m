function [y_sim, t_sim, u_sim, xp_sim, xc_sim, runTime] = simMPCSimulation(ref, params, mpcobj)

    tLoop = tic;                              % NEW: total closed-loop run time
    SimOpt = mpcsimopt(mpcobj);
    SimOpt.RefLookAhead = 'on';

    [y_sim, t_sim, u_sim, xp_sim, xc_sim] = ...
        sim(mpcobj, params.numSamples, ref(1:params.numSamples, :), SimOpt);
    runTime = toc(tLoop);                     % NEW

end