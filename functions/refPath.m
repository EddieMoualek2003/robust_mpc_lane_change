function ref = refPath(params)
    Yf = 3.5; T = params.T;
    N  = params.numSamples + params.Np + 1;        % full preview length, no padding needed
    t  = (0:N-1)' * params.Ts;                     % same convention as params.t: row j at (j-1)*Ts
    tau = min(t/T, 1);

    Yref   = Yf*(10*tau.^3 - 15*tau.^4 + 6*tau.^5);
    dYref  = (Yf/T)*(30*tau.^2 - 60*tau.^3 + 30*tau.^4);
    psiref = dYref/params.vxSim;                     % small-angle

    yNom = cell2mat(arrayfun(@(tt) params.yNomFun(tt).', t, 'UniformOutput', false));
    ref  = yNom + [zeros(N,1), Yref, psiref, zeros(N,3)];   % absolute
end