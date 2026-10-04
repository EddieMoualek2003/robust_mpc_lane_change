function [uAbs, info, ymDev] = ltiControllerStep(params, xAbs, uPrev, yRef, k, mpcobj, xmpc)
    tk = params.t(k);

    % Plant side -> deviation coordinates
    xDev  = xAbs - params.xNomFun(tk);
    yAbs  = params.gNum(xAbs, uPrev);               % sensor reading, absolute
    yNomK = params.yNomFun(tk);
    ymDev = yAbs - yNomK;                

    % Reference preview -> deviation, row i at tk + i*Ts
    p        = size(yRef, 1);
    tPrev    = tk + (1:p).' * params.Ts; % It should be noted this is for preview, not previous.
    yNomPrev = cell2mat(arrayfun(@(t) params.yNomFun(t).', tPrev, 'UniformOutput', false));
    yRefDev  = yRef - yNomPrev;

    % Full-state feedback: QP starts from the true state
    xmpc.Plant = xDev;
    [uDev, info] = mpcmove(mpcobj, xmpc, ymDev(1:4), yRefDev); % measured outputs only (MO = 1:4)

    uAbs = uDev + params.u0;
end