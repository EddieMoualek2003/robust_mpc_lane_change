function [T_lti, T_ltv, T_cmp] = analyseRuns(runs_lti, runs_ltv)
% analyseRuns  Lane-change tracking metrics for LTI and LTV runs.
%   1. Parameter variation: each controller across its (vxDesign, vxSim) runs
%   2. Controller variation: LTV vs LTI at each matched (vxDesign, vxSim)

T_lti = metricsTable(runs_lti);
T_ltv = metricsTable(runs_ltv);

%% 1. Parameter variation within each controller
fprintf('\n=== 1. Parameter variation (within controller) ===\n');
fprintf('\n-- LTI --\n');  disp(T_lti);
fprintf('\n-- LTV --\n');  disp(T_ltv);
plotFamily(runs_lti, T_lti, 'LTI');
plotFamily(runs_ltv, T_ltv, 'LTV');
plotTrends(T_lti, T_ltv);

%% 2. Controller variation at each operating point
keys = intersect(T_lti.Properties.RowNames, T_ltv.Properties.RowNames, 'stable');
miss = setxor(T_lti.Properties.RowNames, T_ltv.Properties.RowNames);
if ~isempty(miss)
    warning('analyseRuns:unmatched', 'Runs without a counterpart: %s', strjoin(miss, ', '));
end
cl = toCost(T_lti(keys, :));
cv = toCost(T_ltv(keys, :));
vars  = cl.Properties.VariableNames;
mvars = setdiff(vars, {'vxDesign', 'vxSim', 'mismatch'}, 'stable');
T_cmp = cv;
T_cmp{:, mvars} = cv{:, mvars} - cl{:, mvars};
fprintf('\n=== 2. Controller variation (LTV - LTI, cost form: negative = LTV better) ===\n');
disp(T_cmp);
plotCompare(runs_lti, runs_ltv, keys);
end

%% ---------- Metrics ----------
function T = metricsTable(runs)
    keys = string(fieldnames(runs));
    for i = numel(keys):-1:1
        m(i) = runMetrics(runs.(keys(i)));
    end
    T = struct2table(m, 'RowNames', cellstr(keys));
    T = sortrows(T, {'vxDesign', 'vxSim'});
end

function m = runMetrics(r)
    p  = r.params;  t = r.t;
    y  = r.Y(:, 2); yr = r.Ref(:, 2);
    w  = yr(end);                                   % lane width from the reference
    e  = y - yr;
    dd = p.Jlat_design * p.Ts * p.m / p.Cf;
    nSS = round(2 / p.Ts);                          % last 2 s for steady state

    m.vxDesign = p.vxDesign;
    m.vxSim    = p.vxSim;
    m.mismatch = p.vxSim - p.vxDesign;
    m.RMSe     = sqrt(mean(e.^2));
    m.maxErr   = max(abs(e));
    m.lag95    = firstCross(t, y, 0.95*w) - firstCross(t, yr, 0.95*w);
    m.settle2  = settleTime(t, y, w, 0.02) - settleTime(t, yr, w, 0.02);
    m.ssErr    = mean(e(end-nSS+1:end));
    m.over     = max(y) - w;
    m.atLim    = 100 * mean(abs(abs(diff(r.U(:, 2))) - dd) < 1e-6);
    m.maxAy    = max(abs(r.AY));
    m.ayViol   = 100 * mean(abs(r.AY) > p.ay_max);
    m.maxJerk  = max(abs(diff(r.AY))) / p.Ts;
    m.maxSlack = max(r.Slack);
    m.QPfail   = sum(r.QP < 0);
end

function tc = firstCross(t, y, lvl)
    i = find(y >= lvl, 1);
    if isempty(i), tc = NaN; else, tc = t(i); end
end

function ts = settleTime(t, y, w, tol)
    i = find(abs(y - w) > tol*w, 1, 'last');
    if isempty(i),        ts = t(1);
    elseif i == numel(y), ts = NaN;                 % never settles
    else,                 ts = t(i+1);
    end
end

function T = toCost(T)
% Every metric non-negative, lower = better, so deltas have one meaning
    T.lag95   = abs(T.lag95);
    T.settle2 = abs(T.settle2);
    T.ssErr   = abs(T.ssErr);
    T.over    = max(T.over, 0);
end

%% ---------- Plots ----------
function plotFamily(runs, T, name)
    keys = T.Properties.RowNames;
    figure('Name', [name ' - parameter variation']);
    tiledlayout(3, 1, 'TileSpacing', 'compact');
    r0 = runs.(keys{1});
    ax1 = nexttile; hold on; grid on
    plot(r0.t, r0.Ref(:, 2), 'k--', 'DisplayName', 'ref');
    ax2 = nexttile; hold on; grid on
    ax3 = nexttile; hold on; grid on
    for i = 1:numel(keys)
        r = runs.(keys{i});
        plot(ax1, r.t, r.Y(:, 2),              'DisplayName', keys{i});
        plot(ax2, r.t, r.Y(:, 2) - r.Ref(:, 2), 'DisplayName', keys{i});
        plot(ax3, r.t, r.AY,                   'DisplayName', keys{i});
    end
    yline(ax3, [-1 1] * r0.params.ay_max, 'k:', 'HandleVisibility', 'off');
    ylabel(ax1, 'Y (m)');  ylabel(ax2, 'e_Y (m)');  ylabel(ax3, 'a_y (m/s^2)');
    xlabel(ax3, 't (s)');  title(ax1, [name ': parameter variation']);
    legend(ax1, 'Interpreter', 'none', 'Location', 'best');
    linkaxes([ax1 ax2 ax3], 'x');
end

function plotTrends(T_lti, T_ltv)
    mets = {'RMSe', 'maxErr', 'lag95', 'maxAy'};
    labs = {'RMS e_Y (m)', 'max |e_Y| (m)', 'lag_{95} (s)', 'max |a_y| (m/s^2)'};
    ds   = unique([T_lti.vxDesign; T_ltv.vxDesign]);
    cols = lines(numel(ds));
    figure('Name', 'Metric trends vs vxSim');
    tiledlayout(2, 2, 'TileSpacing', 'compact');
    for j = 1:numel(mets)
        nexttile; hold on; grid on
        for c = 1:2
            if c == 1, T = T_lti; sty = '-o'; nm = 'LTI';
            else,      T = T_ltv; sty = '--s'; nm = 'LTV'; end
            for d = 1:numel(ds)
                rows = T.vxDesign == ds(d);
                if ~any(rows), continue; end
                plot(T.vxSim(rows), T.(mets{j})(rows), sty, 'Color', cols(d, :), ...
                    'DisplayName', sprintf('%s, v_{design}=%g', nm, ds(d)));
            end
        end
        xlabel('v_{x,sim} (m/s)'); ylabel(labs{j});
    end
    legend('Location', 'bestoutside');
end

function plotCompare(runs_lti, runs_ltv, keys)
    figure('Name', 'LTI vs LTV');
    tiledlayout('flow', 'TileSpacing', 'compact');
    for i = 1:numel(keys)
        a = runs_lti.(keys{i});  b = runs_ltv.(keys{i});
        nexttile; hold on; grid on
        plot(a.t, a.Y(:, 2) - a.Ref(:, 2), 'DisplayName', 'LTI');
        plot(b.t, b.Y(:, 2) - b.Ref(:, 2), 'DisplayName', 'LTV');
        title(keys{i}, 'Interpreter', 'none'); ylabel('e_Y (m)'); xlabel('t (s)');
        legend('Location', 'best');
    end
end