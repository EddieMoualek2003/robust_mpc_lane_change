function c = ayConNonlinear(X, U, e, data, ayF, aymax)
    % Soft |a_y| <= aymax at every point of the prediction.
    % X: (p+1)-by-nx, row 1 = current state x(k), row p+1 = x(k+p)
    % U: (p+1)-by-nu, row i = u(k+i-1); last row repeats row p
    n  = data.PredictionHorizon + 1;
    ay = zeros(n, 1);
    for i = 1:n
        ay(i) = ayF(X(i,:)', U(i,:)');
    end
    c = [ay - aymax; -ay - aymax] - e;
end