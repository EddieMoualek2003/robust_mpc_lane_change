function [y_sim, t_sim, u_sim, xp_sim, xc_sim] = simMPCSimulation(ref, params, mpcobj)

    SimOpt = mpcsimopt(mpcobj);
    SimOpt.RefLookAhead = 'on';

    [y_sim, t_sim, u_sim, xp_sim, xc_sim] = ...
        sim(mpcobj, params.numSamples, ref(1:params.numSamples, :), SimOpt);

end