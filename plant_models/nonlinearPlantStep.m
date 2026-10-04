function [xNext, ay] = nonlinearPlantStep(x, u, k, params)
    %% Compute the next state
    opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
    odeFun = @(t, xs) params.fNum(xs, u); % Standard form to solve /dot{xs} = f(x, u)
    [~, xTraj] = ode45(odeFun, [0 params.Ts], x, opts); % Integrate using ODE45, over one timestep, with initial condition x
    xNext = xTraj(end, :)'; % Take the final row, since the whole trajectory is returned for all the time steps the integrator solved for.

    %% Compute the current ay
    dx = params.fNum(x, u);
    ay = dx(5) + x(4)*x(6); 

end