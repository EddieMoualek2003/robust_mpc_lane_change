function [refPath, xBaselineTraj, yBaselineTraj, refDeviation] = refPath(params, s_fun)
    N = params.numSamples + params.Np + 1;         % full preview length
    t = (0:N-1)' * params.Ts;                      % row j at (j-1)*Ts

    %% 1. State reference trajectory
    opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
    [~, X] = ode45(@(~, x) s_fun.fF(x, params.u0), t, params.x0, opts);   % N-by-nx
    xBaselineTraj = X;
    ny = numel(s_fun.gF(X(1,:).', params.u0));
    yBaselineTraj = zeros(N, ny);
    for j = 1:N
        yBaselineTraj(j,:) = s_fun.gF(X(j,:).', params.u0).';
    end

    %% 2. Lane-change offset
    Yf = 3.5; T = params.T;
    tau    = min(t/T, 1);
    Yref   = Yf*(10*tau.^3 - 15*tau.^4 + 6*tau.^5);
    dYref  = (Yf/T)*(30*tau.^2 - 60*tau.^3 + 30*tau.^4);
    psiref = dYref ./ xBaselineTraj(:,4);          % baseline's own vx
    refDeviation = [zeros(N,1), Yref, psiref, zeros(N,3)];

    %% 3. Absolute reference
    refPath = yBaselineTraj + refDeviation;
end