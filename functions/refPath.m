function ref = refPath(params)
    Yf = 3.5; T = params.T;
    t  = (1:params.numSamples)' * params.Ts;   % column vector, sample k is at time k*Ts
    tau = min(t/T, 1);
    Yref   = Yf*(10*tau.^3 - 15*tau.^4 + 6*tau.^5);
    dYref  = (Yf/T)*(30*tau.^2 - 60*tau.^3 + 30*tau.^4).*(t < T);
    psiref = dYref/params.vx0;                 % small-angle
    ref = [zeros(size(t)), Yref, psiref, zeros(size(t)), zeros(size(t)), zeros(size(t))];  % deviation coords: vx_ref - vx0 = 0

    % Pad the reference by holding the final row, so the preview window never runs past the end
    nPad = params.numSamples + params.Np + 1 - size(ref,1);
    if nPad > 0
        ref = [ref; repmat(ref(end,:), nPad, 1)];
    end

end