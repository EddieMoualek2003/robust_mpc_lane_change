function [xNomFun, xNomTraj] = computeNominal(f, x0, u0, t)
% Integrate once, with input held at u0
opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
[~, X] = ode45(@(~, x) f(x, u0), t, x0, opts);   % numel(t)-by-nx

xNomTraj = X.';

% Query at any time, e.g. t_k or the whole horizon t_{k+1..k+p}
xNomFun = @(tq) interp1(t, X, tq(:), 'linear', 'extrap').';

end