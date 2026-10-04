dd     = params.Jlat_design*params.Ts*params.m/params.Cf;
yr     = ref(1:params.numSamples, 2);
t95ref = params.t(find(yr >= 0.95*3.5, 1));
keys   = ["d15_s15", "d15_s20", "d20_s20"];

fprintf('%-8s %6s %6s %7s %6s %7s %7s %7s %6s\n', 'run','RMSe','lag95','final','over','atLim%','max|ay|','maxJerk','QPfail');
for f = keys
    r = runs.(f);  e = r.Y - yr;
    fprintf('%-8s %6.3f %6.2f %7.3f %6.3f %7.1f %7.2f %7.2f %6d\n', f, sqrt(mean(e.^2)), ...
        params.t(find(r.Y >= 0.95*3.5, 1)) - t95ref, r.Y(end) - 3.5, max(r.Y) - 3.5, ...
        100*mean(abs(abs(diff(r.U(:,2))) - dd) < 1e-6), max(abs(r.AY)), ...
        max(abs(diff(r.AY)))/params.Ts, sum(r.QP < 0));
end

figure; subplot(2,1,1); hold on; grid on
plot(params.t, yr, 'k--');  for f = keys, plot(params.t, runs.(f).Y); end
ylabel('Y (m)'); legend(["ref" keys], 'Interpreter', 'none');
subplot(2,1,2); hold on; grid on
for f = keys, plot(params.t, runs.(f).AY); end
yline([-1 1]*params.ay_max, 'k:'); ylabel('a_y (m/s^2)'); xlabel('t (s)'); legend(keys, 'Interpreter', 'none');