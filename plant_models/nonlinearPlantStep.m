function [xNext, ay] = nonlinearPlantStep(x, u, s_fun, params)
    %% Compute the next state
    opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
    odeFun = @(t, xs) s_fun.fF(xs, u); % Standard form to solve \dot{xs} = f(xs, u), u held over Ts (ZOH)
    [~, xTraj] = ode45(odeFun, [0 params.Ts], x, opts); % Integrate over one timestep from initial condition x
    xNext = xTraj(end, :)'; % Final row: the state at t = Ts

    %% Compute the current ay
    % a_y at (x_k, u_k): the instant the new input is applied, matching the constraint at i = 1
    ay = s_fun.ayF(x, u);
end