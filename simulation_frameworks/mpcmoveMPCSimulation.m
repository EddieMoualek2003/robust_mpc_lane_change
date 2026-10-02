function [Xlog, Ylog, Ulog, QPlog, Slacklog, Itlog, aylog, runTime] = mpcmoveMPCSimulation(xmpc, mpcobj, params, Ad,Bd,Cd,Dd,plant_d, ref)

    tLoop = tic;                              % NEW: total closed-loop run time
    c_vy = params.c_vy;
    c_w  = params.c_w;
    c_d  = params.c_d;

    x = zeros(size(Ad,1),1); % true plant state (deviation coords)
    u = zeros(size(Bd,2),1);
    Xlog = zeros(params.numSamples+1, numel(x));
    Ylog = zeros(params.numSamples, size(Cd,1));
    Ulog = zeros(params.numSamples, size(Bd,2));
    QPlog = strings(params.numSamples,1);
    Slacklog = zeros(params.numSamples,1);
    Itlog = zeros(params.numSamples,1);
    aylog = zeros(params.numSamples, 1);
    Xlog(1,:) = x';
    Ulog(1,:) = u';

    for k = 1:params.numSamples
        % Extract the reference path for this step
        ref_path = ref(k:k+params.Np-1, :);

        % First, measure the system
        y_full = Cd*x;
        y = y_full(1:4);
        
        % Move the system
        [u, info] = mpcmove(mpcobj, xmpc, y, ref_path);
        if k == 1, Yopt0 = info.Yopt; end
        
        % Log the signals
        Ulog(k, :) = u';
        Ylog(k,:) = y_full';
        QPlog(k) = string(info.QPCode);
        Slacklog(k) = info.Slack;
        Itlog(k) = info.Iterations;
        aylog(k) = c_vy*x(5) + c_w*x(6) + c_d*u(2);   % exact for the linear plant
        
        % Stop immediately if the QP failed, so a fallback move can't hide a failure
        if ~strcmp(info.QPCode, 'feasible')
            error('QP not feasible at step %d: %s', k, info.QPCode);
        end
        
        % Advance the plant
        x = Ad*x + Bd*u;
        Xlog(k+1,:) = x';
    end

    fprintf('QP failures: %d, max slack: %.3g, max iterations: %d\n', ...
        nnz(QPlog ~= "feasible"), max(Slacklog), max(Itlog));
    runTime = toc(tLoop);                     % NEW
end