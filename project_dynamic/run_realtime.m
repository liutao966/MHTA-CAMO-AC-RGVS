% =========================================================================
%  Real-time Performance Experiment -- Step-wise Decision Timing
% -------------------------------------------------------------------------
%  Algorithms: ACOSRAR(RGVS, ours) / AVSS_RRT / A_APF / VO / RVO / DWA
%  Procedure:
%    For each algo x obstacle count(2/6/11) x seed(20 runs), run dynamic
%    obstacle avoidance simulation, record per-step "decision time"
%    (excluding move_obstacles), aggregate mean / max / 95th percentile,
%    compare against UAV control cycles 50Hz(20ms) / 100Hz(10ms).
%
%  Output:
%    Fig 1: CDF of per-step decision time (1x3 subplots by obstacle count)
%    Fig 2: Mean + 95th-percentile upper bound vs obstacle count
%    Table: realtime_results.csv
% =========================================================================
clear; clc; close all;
root = fileparts(mfilename('fullpath'));
addpath(root);   % 依赖的 build_env/plan_*/gen_obstacles/move_obstacles 等已与本脚本同目录

counts      = [2 6 11];
nseed       = 20;
algos       = {'This algorithm', 'AVSS_RRT', 'A_APF', 'VO', 'RVO', 'DWA'};
nAlgo       = numel(algos);
nCount      = numel(counts);

env   = build_env();
start = env.start; goal = env.goal;

fprintf('======== Real-time Performance Experiment ========\n');
fprintf('UAV control cycle reference: 50Hz = 20ms, 100Hz = 10ms\n\n');

% ---------- Static baseline path ----------
% This algorithm / VO / RVO / DWA 共用 ACOSRAR 全局参考路径（与主对比一致）；
% A*+APF 与 AVSS-RRT 用各自规划的路径。plan_this 较慢，只算一次并复用。
path_this = plan_this(env);
path_avss    = plan_avss(env);
path_aapf    = plan_aapf(env);
static_path  = {path_this, path_avss, path_aapf, path_this, path_this, path_this};
for a = 1:nAlgo
    fprintf('[%s] Static path ready\n', algos{a});
end

% ---------- Run simulations and collect timing ----------
all_times = cell(nAlgo, nCount, nseed);
all_status = zeros(nAlgo, nCount, nseed);   % 0=超时 1=到达 2=碰撞

for c = 1:nCount
    cnt = counts(c);
    for a = 1:nAlgo
        for s = 1:nseed
            dyn = gen_obstacles(cnt, s, static_path{a}, env);
            switch a
                case 1
                    [times, st] = sim_rgvs_timed(static_path{a}, dyn, start, goal, env);
                case 2
                    [times, st] = sim_avss_timed(static_path{a}, dyn, start, goal, env);
                case 3
                    [times, st] = sim_aapf_timed(static_path{a}, dyn, start, goal, env);
                case 4
                    [times, st] = sim_vo_timed(static_path{a}, dyn, start, goal, env);
                case 5
                    [times, st] = sim_rvo_timed(static_path{a}, dyn, start, goal, env);
                case 6
                    [times, st] = sim_dwa_timed(static_path{a}, dyn, start, goal, env);
            end
            all_times{a, c, s} = times;
            all_status(a, c, s) = st;
        end
        fprintf('[%s] obstacles=%2d done (%d runs)\n', algos{a}, cnt, nseed);
        if a == 1   % 离群尖峰诊断：打印 This algorithm 每个 seed 的最大单步耗时 + 步数
            mx_per_seed = zeros(nseed, 1);
            ix_per_seed = zeros(nseed, 1);
            ns_per_seed = zeros(nseed, 1);
            for s = 1:nseed
                t = all_times{a, c, s}(:) * 1000;   % 秒 -> ms
                [mx_per_seed(s), ix_per_seed(s)] = max(t);
                ns_per_seed(s) = numel(t);
            end
            [worst, worst_s] = max(mx_per_seed);
            fprintf('    [diag] per-seed max (ms): %s\n', sprintf('%.1f ', mx_per_seed));
            fprintf('    [diag] per-seed nsteps : %s\n', sprintf('%d ', ns_per_seed));
            fprintf('    [diag] worst seed=%d (%.2f ms @ step %d / %d)\n', ...
                worst_s, worst, ix_per_seed(worst_s), ns_per_seed(worst_s));
        end
    end
end

% ---------- Aggregate statistics ----------
T_50Hz  = 20;
T_100Hz = 10;

agg = cell(nAlgo, nCount);
stats = zeros(nAlgo, nCount, 6);

fprintf('\n================ Real-time Performance Table ================\n');
fprintf('Algorithm        | Obstacles | Mean(ms) | Max(ms) | P95(ms) | <=50Hz(20ms) | <=100Hz(10ms)\n');
fprintf('%s\n', repmat('-', 1, 105));

for a = 1:nAlgo
    for c = 1:nCount
        agg{a, c} = [];          % 供 Fig1 CDF：所有 step 混在一起（分布形状）
        seed_mean = zeros(nseed, 1);
        seed_p95  = zeros(nseed, 1);
        for s = 1:nseed
            t = all_times{a, c, s}(:);
            t = t(isfinite(t));
            agg{a, c} = [agg{a, c}; t]; %#ok<AGROW>
            seed_mean(s) = mean(t) * 1000;
            seed_p95(s)  = local_prctile(t, 95) * 1000;
        end

        mu   = mean(seed_mean);       % 等权 mean over seeds（隔离长绕行 seed 的样本稀释）
        mx   = max(agg{a, c}) * 1000; % 全局 max（所有 step 的最大单步耗时）
        p95  = mean(seed_p95);        % 等权 mean of per-seed P95
        sd   = std(seed_mean);        % 跨 seed 的 mean 标准差
        r50  = mu / T_50Hz * 100;
        r100 = mu / T_100Hz * 100;
        stats(a, c, 1) = mu;
        stats(a, c, 2) = mx;
        stats(a, c, 3) = p95;
        stats(a, c, 4) = sd;
        stats(a, c, 5) = r50;
        stats(a, c, 6) = r100;

        fprintf('%-16s | %9d | %8.2f | %8.2f | %7.2f | %12.1f%% | %13.1f%%\n', ...
            algos{a}, counts(c), mu, mx, p95, r50, r100);
    end
    fprintf('%s\n', repmat('-', 1, 105));
end

% ---------- CSV output ----------
oc = zeros(nAlgo * nCount, 1);
al = cell(nAlgo * nCount, 1);
mn = zeros(nAlgo * nCount, 1);
mx = zeros(nAlgo * nCount, 1);
p9 = zeros(nAlgo * nCount, 1);
r5 = zeros(nAlgo * nCount, 1);
r1 = zeros(nAlgo * nCount, 1);
k = 0;
for a = 1:nAlgo
    for c = 1:nCount
        k = k + 1;
        oc(k) = counts(c);
        al{k} = algos{a};
        mn(k) = stats(a, c, 1);
        mx(k) = stats(a, c, 2);
        p9(k) = stats(a, c, 3);
        r5(k) = stats(a, c, 5);
        r1(k) = stats(a, c, 6);
    end
end
T_out = table(oc, string(al), mn, mx, p9, r5, r1, ...
    'VariableNames', {'obstacle_count', 'algorithm', 'mean_ms', 'max_ms', ...
                      'p95_ms', 'ratio_50hz_pct', 'ratio_100hz_pct'});
writetable(T_out, fullfile(root, 'realtime_results.csv'));
fprintf('\nResults saved to %s\n', fullfile(root, 'realtime_results.csv'));

% ---------- 避障成功率（配套输出：2/6/11 障碍下的到达率） ----------
succ = zeros(nAlgo, nCount);
for a = 1:nAlgo
    for c = 1:nCount
        succ(a, c) = mean(all_status(a, c, :) == 1) * 100;
    end
end

fprintf('\n================ Avoidance Success Rate (2/6/11 obstacles) ================\n');
fprintf('%-16s | %12s | %12s | %12s\n', 'Algorithm', 'Obstacles=2', 'Obstacles=6', 'Obstacles=11');
fprintf('%s\n', repmat('-', 1, 66));
for a = 1:nAlgo
    fprintf('%-16s | %11.1f%% | %11.1f%% | %12.1f%%\n', algos{a}, succ(a, 1), succ(a, 2), succ(a, 3));
end
fprintf('%s\n', repmat('-', 1, 66));

% 成功率 csv
T_succ = array2table(succ, 'VariableNames', {'obstacles_2', 'obstacles_6', 'obstacles_11'}, ...
    'RowNames', algos);
writetable(T_succ, fullfile(root, 'realtime_success.csv'), 'WriteRowNames', true);
fprintf('成功率已保存至 %s\n', fullfile(root, 'realtime_success.csv'));

% ---------- Fig 1: CDF (1x3 subplots by obstacle count) ----------
fig1 = figure('Position', [100 100 1300 450], 'Color', 'white');
colors  = [0.85 0.10 0.10; 0.10 0.45 0.80; 0.10 0.60 0.20; ...
           0.60 0.20 0.80; 0.90 0.55 0.10; 0.00 0.60 0.70];
linew   = [2.4 2.0 2.0 2.0 2.0 2.0];

for ci = 1:nCount
    subplot(1, nCount, ci);
    hold on; grid on; box on;

    for a = 1:nAlgo
        pooled_ms = agg{a, ci}(:) * 1000;
        pooled_ms = sort(pooled_ms);
        F = (1:length(pooled_ms))' / length(pooled_ms);
        plot(pooled_ms, F, '-', 'Color', colors(a, :), 'LineWidth', linew(a), ...
            'DisplayName', algos{a});
    end

    xline(T_50Hz,  'k--', 'LineWidth', 1.5, 'DisplayName', '50Hz (20ms)');
    xline(T_100Hz, 'k-.', 'LineWidth', 1.5, 'DisplayName', '100Hz (10ms)');

    xlabel('Step-wise decision time (ms)', 'FontSize', 12);
    ylabel('Cumulative probability F(t)', 'FontSize', 12);
    title(sprintf('Obstacles = %d', counts(ci)), 'FontSize', 13, 'FontWeight', 'bold');
    xlim([0 50]);
    ylim([0 1.02]);
    set(gca, 'FontSize', 11);
    if ci == 1
        legend('Location', 'southeast', 'FontSize', 9, 'Interpreter', 'none');
    end
    hold off;
end
sgtitle('CDF of Step-wise Decision Time', 'FontSize', 14, 'FontWeight', 'bold');

% ---------- Fig 2: Mean (solid) + P95 (dashed) + Max (dash-dot) ----------
fig2 = figure('Position', [150 150 950 580], 'Color', 'white');
mu_mat  = squeeze(stats(:, :, 1));
p95_mat = squeeze(stats(:, :, 3));
mx_mat  = squeeze(stats(:, :, 2));

hold on; grid on; box on;

x = counts;
for a = 1:nAlgo
    plot(x, mu_mat(a, :), '-', 'Color', colors(a, :), ...
        'LineWidth', 2.2, 'Marker', 'o', 'MarkerSize', 7, 'MarkerFaceColor', 'w', ...
        'DisplayName', [algos{a}, ' (Mean)']);
    plot(x, p95_mat(a, :), '--', 'Color', colors(a, :), ...
        'LineWidth', 1.5, 'Marker', 's', 'MarkerSize', 5, ...
        'DisplayName', [algos{a}, ' (P95)']);
    plot(x, mx_mat(a, :), '-.', 'Color', colors(a, :), ...
        'LineWidth', 1.2, 'Marker', '^', 'MarkerSize', 4, ...
        'DisplayName', [algos{a}, ' (Max)']);
end

yline(T_50Hz,  'k:', 'LineWidth', 1.8, 'DisplayName', '50Hz (20ms)');
yline(T_100Hz, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');

xlabel('Number of dynamic obstacles', 'FontSize', 12);
ylabel('Step-wise decision time (ms)', 'FontSize', 12);
title('Mean / P95 / Max Decision Time vs Obstacle Count', ...
    'FontSize', 14, 'FontWeight', 'bold');
set(gca, 'XTick', x, 'FontSize', 11);
legend('Location', 'northwest', 'FontSize', 8, 'Interpreter', 'none');
hold off;

fprintf('================ Experiment Complete ================\n');

% =========================================================================
%  Timed simulation functions (decision core tic/toc, move_obstacles excluded)
% =========================================================================

function [step_times, status] = sim_rgvs_timed(path, dyn, start, goal, env)
uav_speed  = 2.0;
look_ahead = 28;
max_steps  = 4000;
r_safe = dyn.r + env.safe;

glob = densify_path(path, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);

pos = start;
vel = (glob(2, :) - glob(1, :)) / max(norm(glob(2, :) - glob(1, :)), 1e-9) * uav_speed;
s_glob = 0;
s_prog = 0;
detour = false;
detour_steps = 0;
min_detour_steps = 12;

step_times = [];
status = 0;   % 0=超时 1=到达 2=碰撞

for step = 1:max_steps
    dyn = move_obstacles(dyn);

    tic;
    la = 12;
    if ~detour
        la = look_ahead;
    end
    glob_blocked = lookahead_hit(glob, glob_cum, s_glob, dyn, r_safe + 1.5, la, uav_speed);

    if ~glob_blocked && (~detour || detour_steps >= min_detour_steps)
        if detour
            [glob, glob_cum, glob_len, s_glob] = replan_after_detour(pos, glob, env);
            s_prog = 0;
            if size(glob, 1) >= 2 && norm(glob(2, :) - glob(1, :)) > 1e-9
                vel = (glob(2, :) - glob(1, :)) / norm(glob(2, :) - glob(1, :)) * uav_speed;
            end
            detour = false;
            detour_steps = 0;
        end
        s_glob = min(glob_len, s_glob + uav_speed);
        target = interp_arc(glob, glob_cum, s_glob);
        pos = move_towards(pos, target, uav_speed);
        if norm(target - pos) > 1e-9
            vel = (target - pos) / norm(target - pos) * uav_speed;
        end
    else
        detour = true;
        detour_steps = detour_steps + 1;
        s_glob = min(glob_len, s_glob + uav_speed);
        ref = interp_arc(glob, glob_cum, min(glob_len, s_glob + look_ahead));
        dw = struct('vmax', uav_speed, 'vmin', 0.3, 'dvmax', 0.6, 'dthmax', 1.5, ...
                    'dt', 1.0, 'pred', 8, 'w_head', 1.0, 'w_clear', 6.0, ...
                    'w_speed', 0.4, 'safem', env.safe);
        [pos, vel] = dwa_local_plan(pos, vel, ref, goal, env, dyn, r_safe, dw);
    end

    dd = sqrt(sum((glob - pos) .^ 2, 2));
    [~, ix_near] = min(dd);
    s_now = glob_cum(ix_near);
    if ~detour && s_now < s_prog - 0.05
        pos = interp_arc(glob, glob_cum, s_prog);
    else
        s_prog = max(s_prog, s_now);
    end

    step_times(end + 1, 1) = toc; %#ok<AGROW>

    if dyn_min_dist(pos, dyn) < env.safe || env_dist(pos, env) < env.safe
        status = 2;
        return;
    end
    if norm(pos - goal) <= uav_speed
        status = 1;
        return;
    end
end
end

function [step_times, status] = sim_avss_timed(path, dyn, start, goal, env)
uav_speed  = 2.0;
look_ahead = 12;
max_steps  = 3000;

glob = densify_path(path, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);

pos = start;
s_glob = 0;
detour = false;

step_times = [];
status = 0;   % 0=超时 1=到达 2=碰撞

for step = 1:max_steps
    dyn = move_obstacles(dyn);
    r_safe = dyn.r + env.safe;

    tic;
    blocked = lookahead_hit_avss(glob, glob_cum, s_glob, dyn, r_safe, look_ahead, uav_speed);

    if ~blocked
        if detour
            s_glob = regain_arc(glob, glob_cum, pos, s_glob, dyn.curr, r_safe, look_ahead);
            detour = false;
        end
        s_glob = min(glob_len, s_glob + uav_speed);
        target = interp_arc(glob, glob_cum, s_glob);
        pos = move_towards(pos, target, uav_speed);
    else
        detour = true;
        aim = interp_arc(glob, glob_cum, min(glob_len, s_glob + look_ahead));
        wp = pick_detour(pos, aim, goal, env, dyn.curr, r_safe, uav_speed);
        if ~isempty(wp)
            pos = move_towards(pos, wp, uav_speed);
        end
    end
    step_times(end + 1, 1) = toc; %#ok<AGROW>

    if dyn_min_dist(pos, dyn) < env.safe || env_dist(pos, env) < env.safe
        status = 2;
        return;
    end
    if norm(pos - goal) <= uav_speed || (s_glob >= glob_len - 1e-6 && norm(pos - goal) <= 8)
        status = 1;
        return;
    end
end
end

function [step_times, status] = sim_aapf_timed(path, dyn, start, goal, env)
uav_speed = 2.0;
max_steps = 4000;

k_att   = 2.0;
k_rep   = 1.5;
rho_rep = 8.0;

glob = densify_path(path, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);
r_safe = dyn.r + env.safe;

pos = start;
s_glob = 0;

step_times = [];
status = 0;   % 0=超时 1=到达 2=碰撞

for step = 1:max_steps
    dyn = move_obstacles(dyn);

    tic;
    aim = interp_arc(glob, glob_cum, min(glob_len, s_glob + 5));

    Fatt = aim - pos;
    if norm(Fatt) < 1e-6
        Fatt = goal - pos;
    end
    Fatt = k_att * Fatt / max(norm(Fatt), 1e-9);

    fwd = (aim - pos);
    if norm(fwd) < 1e-6
        fwd = goal - pos;
    end
    if norm(fwd) < 1e-6
        fwd = [1 0 0];
    end
    fwd = fwd / norm(fwd);

    Frep = [0 0 0];
    for pred_step = 0:6
        if pred_step > 0
            dpred = dyn_after(dyn, pred_step);
        else
            dpred = dyn;
        end
        pred_decay = 0.55 ^ pred_step;
        for i = 1:dpred.num
            to = pos - dpred.curr(i, :);
            d = norm(to) - dpred.r;
            if d < rho_rep
                lat = to - dot(to, fwd) * fwd;
                if norm(lat) < 1e-6
                    lat = cross(fwd, [0 0 1]);
                    if norm(lat) < 1e-6
                        lat = cross(fwd, [0 1 0]);
                    end
                end
                lat = lat / norm(lat);
                Frep = Frep + k_rep * (rho_rep - d) * lat * pred_decay;
            end
        end
    end

    F = Fatt + Frep;
    if norm(F) < 1e-6
        F = goal - pos;
    end
    if norm(F) < 1e-6
        F = [1 0 0];
    end
    base_dir = F / norm(F);

    up0 = [0 0 1];
    right2 = cross(base_dir, up0);
    if norm(right2) < 1e-6
        right2 = [1 0 0];
    end
    right2 = right2 / norm(right2);
    ang_cands = linspace(-90, 90, 37);
    best_score = -inf;
    best_dir = base_dir;
    for ai = 1:length(ang_cands)
        dir_cand = rot3d(base_dir, up0, ang_cands(ai) * pi/180);
        npos_cand = pos + dir_cand * uav_speed;
        score = dot(dir_cand, base_dir);
        safe = true;
        Lcand = norm(npos_cand - pos);
        ns_pts = max(3, ceil(Lcand / 0.3));
        for ti = 1:ns_pts
            tt = ti / ns_pts;
            pc = pos + tt * (npos_cand - pos);
            if env_dist(pc, env) < env.safe
                safe = false; break;
            end
            for pr = 0:3
                if pr == 0
                    dc = dyn;
                else
                    dc = dyn_after(dyn, pr);
                end
                for ki = 1:dc.num
                    if norm(pc - dc.curr(ki, :)) <= r_safe
                        safe = false; break;
                    end
                end
                if ~safe, break; end
            end
            if ~safe, break; end
        end
        if safe
            score = score + 10;
        else
            score = score - 100;
        end
        if score > best_score
            best_score = score;
            best_dir = dir_cand;
        end
    end
    dirmv = best_dir;
    npos = pos + dirmv * uav_speed;

    if ~step_safe(pos, npos, env, dyn, r_safe)
        npos = safe_detour(pos, aim, goal, env, dyn, r_safe, uav_speed);
        if isempty(npos)
            step_times(end + 1, 1) = toc; %#ok<AGROW>
            status = 2;
            return;
        end
    end

    pos = npos;
    dd = sqrt(sum((glob - pos) .^ 2, 2));
    [~, ix_near] = min(dd);
    s_glob = glob_cum(ix_near);

    step_times(end + 1, 1) = toc; %#ok<AGROW>

    if dyn_min_dist(pos, dyn) < env.safe || env_dist(pos, env) < env.safe
        status = 2;
        return;
    end
    if norm(pos - goal) <= uav_speed
        status = 1;
        return;
    end
end
end

% =========================================================================
%  Timed simulation functions -- VO / RVO / DWA (decision core tic/toc)
% =========================================================================

function [step_times, status] = sim_vo_timed(path, dyn, start, goal, env)
% 与 sim_vo.m 一致的速度障碍法决策核心，仅记录每步决策耗时(排除 move_obstacles)。
uav_speed  = 2.0;
look_ahead = 28;    % 前视距离(m)，与 sim_rgvs 一致
tau        = 15;    % VO 碰撞时间窗（步）
near_radius = 40;   % 近场障碍过滤半径(m)
max_steps  = 4000;

r_safe = dyn.r + env.safe;

glob = densify_path(path, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);

pos   = start;
vel   = (glob(2, :) - glob(1, :)) / max(norm(glob(2, :) - glob(1, :)), 1e-9) * uav_speed;
s_glob = 0;
s_prog = 0;
detour = false;

step_times = [];
status = 0;   % 0=超时 1=到达 2=碰撞

for step = 1:max_steps
    dyn = move_obstacles(dyn);

    tic;
    glob_blocked = lookahead_hit(glob, glob_cum, s_glob, dyn, r_safe + 1.5, look_ahead, uav_speed);

    if ~glob_blocked
        detour = false;
        s_glob = min(glob_len, s_glob + uav_speed);
        target = interp_arc(glob, glob_cum, s_glob);
        pos = move_towards(pos, target, uav_speed);
        if norm(target - pos) > 1e-9
            vel = (target - pos) / norm(target - pos) * uav_speed;
        end
    else
        detour = true;
        ref = interp_arc(glob, glob_cum, min(glob_len, s_glob + look_ahead));
        v_pref = (ref - pos) / max(norm(ref - pos), 1e-9) * uav_speed;

        obs_pos = zeros(0, 3);
        obs_vel = zeros(0, 3);
        for i = 1:dyn.num
            if norm(dyn.curr(i, :) - pos) <= near_radius
                obs_pos(end + 1, :) = dyn.curr(i, :); %#ok<AGROW>
                obs_vel(end + 1, :) = obs_velocity(dyn, i); %#ok<AGROW>
            end
        end

        vel = vo_select(pos, v_pref, obs_pos, obs_vel, r_safe, tau, env);
        pos = pos + vel;
    end

    dd = sqrt(sum((glob - pos) .^ 2, 2));
    [~, ix_near] = min(dd);
    s_now = glob_cum(ix_near);
    if ~detour && s_now < s_prog - 0.05
        pos = interp_arc(glob, glob_cum, s_prog);
    else
        s_prog = max(s_prog, s_now);
    end
    if detour
        s_glob = min(glob_len, max(s_glob, min(s_now, s_glob + uav_speed)));
    end

    step_times(end + 1, 1) = toc; %#ok<AGROW>

    if dyn_min_dist(pos, dyn) < env.safe || env_dist(pos, env) < env.safe
        status = 2;
        return;
    end
    if norm(pos - goal) <= uav_speed
        status = 1;
        return;
    end
end
end

function [step_times, status] = sim_rvo_timed(path, dyn, start, goal, env)
% 与 sim_rvo.m 一致的互惠速度障碍法决策核心，仅记录每步决策耗时。
uav_speed  = 2.0;
look_ahead = 28;
tau        = 15;
near_radius = 40;
max_steps  = 4000;

r_safe = dyn.r + env.safe;

glob = densify_path(path, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);

pos   = start;
vel   = (glob(2, :) - glob(1, :)) / max(norm(glob(2, :) - glob(1, :)), 1e-9) * uav_speed;
s_glob = 0;
s_prog = 0;
detour = false;

step_times = [];
status = 0;   % 0=超时 1=到达 2=碰撞

for step = 1:max_steps
    dyn = move_obstacles(dyn);

    tic;
    glob_blocked = lookahead_hit(glob, glob_cum, s_glob, dyn, r_safe + 1.5, look_ahead, uav_speed);

    if ~glob_blocked
        detour = false;
        s_glob = min(glob_len, s_glob + uav_speed);
        target = interp_arc(glob, glob_cum, s_glob);
        pos = move_towards(pos, target, uav_speed);
        if norm(target - pos) > 1e-9
            vel = (target - pos) / norm(target - pos) * uav_speed;
        end
    else
        detour = true;
        ref = interp_arc(glob, glob_cum, min(glob_len, s_glob + look_ahead));
        v_pref = (ref - pos) / max(norm(ref - pos), 1e-9) * uav_speed;

        obs_pos = zeros(0, 3);
        obs_vel = zeros(0, 3);
        for i = 1:dyn.num
            if norm(dyn.curr(i, :) - pos) <= near_radius
                obs_pos(end + 1, :) = dyn.curr(i, :); %#ok<AGROW>
                obs_vel(end + 1, :) = obs_velocity(dyn, i); %#ok<AGROW>
            end
        end

        vel = rvo_select(pos, vel, v_pref, obs_pos, obs_vel, r_safe, tau, env);
        pos = pos + vel;
    end

    dd = sqrt(sum((glob - pos) .^ 2, 2));
    [~, ix_near] = min(dd);
    s_now = glob_cum(ix_near);
    if ~detour && s_now < s_prog - 0.05
        pos = interp_arc(glob, glob_cum, s_prog);
    else
        s_prog = max(s_prog, s_now);
    end
    if detour
        s_glob = min(glob_len, max(s_glob, min(s_now, s_glob + uav_speed)));
    end

    step_times(end + 1, 1) = toc; %#ok<AGROW>

    if dyn_min_dist(pos, dyn) < env.safe || env_dist(pos, env) < env.safe
        status = 2;
        return;
    end
    if norm(pos - goal) <= uav_speed
        status = 1;
        return;
    end
end
end

function [step_times, status] = sim_dwa_timed(path, dyn, start, goal, env)
% 与 sim_dwa.m 一致的纯 DWA 决策核心，仅记录每步决策耗时。
uav_speed  = 2.0;
look_ahead = 28;
max_steps  = 4000;

r_safe = dyn.r + env.safe;

p.vmax    = uav_speed;
p.vmin    = 0.3;
p.dvmax   = 0.6;
p.dthmax  = 1.5;
p.dt      = 1.0;
p.pred    = 8;
p.w_head  = 1.0;
p.w_clear = 6.0;
p.w_speed = 0.4;
p.safem   = env.safe;

glob = densify_path(path, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);

pos   = start;
vel   = (glob(2, :) - glob(1, :)) / max(norm(glob(2, :) - glob(1, :)), 1e-9) * uav_speed;
s_glob = 0;
s_prog = 0;
detour = false;

step_times = [];
status = 0;   % 0=超时 1=到达 2=碰撞

for step = 1:max_steps
    dyn = move_obstacles(dyn);

    tic;
    glob_blocked = lookahead_hit(glob, glob_cum, s_glob, dyn, r_safe + 1.5, look_ahead, uav_speed);

    if ~glob_blocked
        detour = false;
        s_glob = min(glob_len, s_glob + uav_speed);
        target = interp_arc(glob, glob_cum, s_glob);
        pos = move_towards(pos, target, uav_speed);
        if norm(target - pos) > 1e-9
            vel = (target - pos) / norm(target - pos) * uav_speed;
        end
    else
        detour = true;
        ref = interp_arc(glob, glob_cum, min(glob_len, s_glob + look_ahead));
        [pos, vel] = dwa_step(pos, vel, ref, goal, env, dyn, r_safe, p);
    end

    dd = sqrt(sum((glob - pos) .^ 2, 2));
    [~, ix_near] = min(dd);
    s_now = glob_cum(ix_near);
    if ~detour && s_now < s_prog - 0.05
        pos = interp_arc(glob, glob_cum, s_prog);
    else
        s_prog = max(s_prog, s_now);
    end
    if detour
        s_glob = min(glob_len, max(s_glob, min(s_now, s_glob + uav_speed)));
    end

    step_times(end + 1, 1) = toc; %#ok<AGROW>

    if dyn_min_dist(pos, dyn) < env.safe || env_dist(pos, env) < env.safe
        status = 2;
        return;
    end
    if norm(pos - goal) <= uav_speed
        status = 1;
        return;
    end
end
end

% =========================================================================
%  Shared utility functions
% =========================================================================

function q = densify_path(path, ds)
d = [0; cumsum(sqrt(sum(diff(path) .^ 2, 2)))];
if d(end) < 1e-6
    q = path; return;
end
[d, ia] = unique(d, 'stable'); path = path(ia, :);
tt = (0:ds:d(end))';
if tt(end) < d(end)
    tt(end + 1) = d(end); %#ok<AGROW>
end
q = [interp1(d, path(:, 1), tt), interp1(d, path(:, 2), tt), interp1(d, path(:, 3), tt)];
end

function p = interp_arc(glob, glob_cum, s)
s = min(max(s, 0), glob_cum(end));
p = [interp1(glob_cum, glob(:, 1), s), ...
     interp1(glob_cum, glob(:, 2), s), ...
     interp1(glob_cum, glob(:, 3), s)];
end

function pos = move_towards(pos, target, spd)
v = target - pos; d = norm(v);
if d <= spd || d < 1e-9
    pos = target;
else
    pos = pos + (v / d) * spd;
end
end

function hit = lookahead_hit(glob, glob_cum, s0, dyn, r_safe, look_ahead, uav_speed)
hit = false;
if isempty(dyn) || ~isfield(dyn, 'num') || dyn.num == 0
    return;
end
s_end = min(glob_cum(end), s0 + look_ahead);
ns = max(2, ceil((s_end - s0) / 1.5));
for s = linspace(s0, s_end, ns)
    q = interp_arc(glob, glob_cum, s);
    ts = round((s - s0) / uav_speed);
    if ts <= 0
        dpred = dyn;
    else
        dpred = dyn_after(dyn, ts);
    end
    for k = 1:size(dpred.curr, 1)
        if norm(q - dpred.curr(k, :)) <= r_safe
            hit = true;
            return;
        end
    end
end
end

function hit = lookahead_hit_avss(glob, glob_cum, s0, dyn, r_safe, look_ahead, uav_speed)
hit = false;
if isempty(dyn) || ~isfield(dyn, 'num') || dyn.num == 0
    return;
end
s_end = min(glob_cum(end), s0 + look_ahead);
ns = max(2, ceil((s_end - s0) / 0.5));
for s = linspace(s0, s_end, ns)
    q = interp_arc(glob, glob_cum, s);
    ts = round((s - s0) / uav_speed);
    if ts <= 0
        dpred = dyn;
    else
        dpred = dyn_after(dyn, min(ts, 5));
    end
    for k = 1:size(dpred.curr, 1)
        if norm(q - dpred.curr(k, :)) <= r_safe
            hit = true;
            return;
        end
    end
end
end

function dn = dyn_after(dyn, n)
dn = dyn;
for i = 1:n
    dn = move_obstacles(dn);
end
end

function [glob, glob_cum, glob_len, s_new] = replan_after_detour(pos, glob, env)
dd = sqrt(sum((glob - pos) .^ 2, 2));
[~, ix] = min(dd);
join = ix;
for k = (ix + 5):5:size(glob, 1)
    if k <= ix
        continue;
    end
    if seg_free_static(pos, glob(k, :), env, 1.0)
        join = k;
    else
        break;
    end
end
refpts = [pos; glob(join:end, :)];
if size(refpts, 1) < 4
    newpath = refpts;
else
    newpath = smooth_spline(refpts);
end
newpath(1, :) = pos;
newpath = enforce_clearance(newpath, env);
glob = densify_path(newpath, 1.0);
if ~allpoints_ok(glob, env)
    glob = densify_path(enforce_clearance(refpts, env), 1.0);
end
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);
s_new = 0;
end

function ok = allpoints_ok(path, env)
ok = true;
for i = 1:size(path, 1)
    if env_dist(path(i, :), env) < env.safe + 0.5
        ok = false;
        return;
    end
end
end

function [npos, nvel] = dwa_local_plan(pos, vel, ref, goal, env, dyn, r_safe, p)
if isfield(dyn, 'num') && dyn.num > 0
    d2 = sqrt(sum((dyn.curr - pos) .^ 2, 2));
    keep = d2 <= 35;
    if any(keep)
        dyn.num    = sum(keep);
        dyn.params = dyn.params(keep, :);
        dyn.curr   = dyn.curr(keep, :);
    else
        dyn.num    = 0;
        dyn.params = zeros(0, 7);
        dyn.curr   = zeros(0, 3);
    end
end

v_mag = norm(vel);
if v_mag < 1e-9
    v_mag = p.vmin;
end
refdir = (ref - pos) / max(norm(ref - pos), 1e-9);

up0 = [0 0 1];
right = cross(refdir, up0);
if norm(right) < 1e-6
    right = [1 0 0];
end
right = right / norm(right);
up = cross(right, refdir); up = up / max(norm(up), 1e-9);

v_lo = max(p.vmin, v_mag - p.dvmax);
v_hi = min(p.vmax, v_mag + p.dvmax);
vcand = linspace(v_lo, v_hi, 5);
angcand = linspace(-p.dthmax, p.dthmax, 21);
pchcand = [-0.45 0 0.45];

obs_pred = cell(1, p.pred);
for st = 1:p.pred
    obs_pred{st} = dyn_after(dyn, st).curr;
end

best_score = -inf;
npos = pos; nvel = vel;
found = false;
for v = vcand
    for da = angcand
        for dpc = pchcand
            newdir = rot3d(refdir, up, da);
            newdir = rot3d(newdir, right, dpc);
            newdir = newdir / max(norm(newdir), 1e-9);
            if dot(newdir, refdir) < -0.3
                continue;
            end
            newvel = newdir * v;
            p0 = pos; clear_min = inf; ok = true;
            for st = 1:p.pred
                p0 = p0 + newvel * p.dt;
                obs_st = obs_pred{st};
                if ~dwa_feasible(p0, env, obs_st, r_safe)
                    ok = false;
                    break;
                end
                if dyn.num > 0
                    d = min(sqrt(sum((p0 - obs_st) .^ 2, 2))) - dyn.r;
                    clear_min = min(clear_min, max(d, 0));
                end
            end
            if ~ok
                continue;
            end
            hd = dot(newdir, refdir);
            cl = min(clear_min, 30) / 30;
            score = p.w_head * hd + p.w_clear * cl + p.w_speed * (v / p.vmax) - 0.05 * abs(da);
            if score > best_score
                best_score = score;
                nvel = newvel;
                npos = pos + newvel * p.dt;
                found = true;
            end
        end
    end
end
if ~found
    next_obs = obs_pred{1};
    d1 = (right + up);  d1 = d1 / max(norm(d1), 1e-9);
    d2 = (right - up);  d2 = d2 / max(norm(d2), 1e-9);
    d3 = (-right + up); d3 = d3 / max(norm(d3), 1e-9);
    d4 = (-right - up); d4 = d4 / max(norm(d4), 1e-9);
    dirs = [right; -right; up; -up; d1; d2; d3; d4];
    amps = [6 12 18];
    for a = amps
        for k = 1:size(dirs, 1)
            cd = pos + dirs(k, :) * a;
            if dwa_feasible(cd, env, next_obs, r_safe)
                nvel = dirs(k, :) * p.vmin;
                npos = pos + nvel * p.dt;
                return;
            end
        end
    end
    return;
end
end

function ok = dwa_feasible(p0, env, obs, r_safe)
if any(p0 < 0) || any(p0 > 200)
    ok = false;
    return;
end
if env_dist(p0, env) < env.safe + 0.5
    ok = false;
    return;
end
for k = 1:size(obs, 1)
    if norm(p0 - obs(k, :)) <= r_safe + 1.0
        ok = false;
        return;
    end
end
ok = true;
end

function r = rot3d(v, axis, ang)
c = cos(ang); s = sin(ang);
a = axis / max(norm(axis), 1e-9);
r = v * c + cross(a, v) * s + a * dot(a, v) * (1 - c);
end

function s_new = regain_arc(glob, glob_cum, pos, s_cur, dyn_pos, r_safe, look_ahead)
best_s = s_cur; best_d = inf;
s_from = s_cur;
s_to = min(glob_cum(end), s_cur + look_ahead + 20);
for s = linspace(s_from, s_to, 40)
    p = interp_arc(glob, glob_cum, s);
    if norm(p - pos) < best_d && ~lookahead_hit_avss(glob, glob_cum, s, dyn_pos, r_safe, look_ahead)
        best_d = norm(p - pos);
        best_s = s;
    end
end
s_new = best_s;
end

function wp = pick_detour(pos, aim, goal, env, dyn_pos, r_safe, spd)
wp = [];
fwd = aim - pos;
if norm(fwd) < 1e-6
    fwd = goal - pos;
end
if norm(fwd) < 1e-6
    return;
end
fwd = fwd / norm(fwd);
up0 = [0 0 1];
right = cross(fwd, up0);
if norm(right) < 1e-6
    right = [1 0 0];
end
right = right / norm(right);
up = cross(right, fwd); up = up / norm(up);
amp = [5 8 11 14 17 20];
dirs_base = [right; -right; up; -up; ...
             right + up; right - up; -right + up; -right - up; ...
             right * cos(pi/8) + up * sin(pi/8); ...
             right * cos(pi/8) - up * sin(pi/8); ...
             -right * cos(pi/8) + up * sin(pi/8); ...
             -right * cos(pi/8) - up * sin(pi/8); ...
             right * cos(pi/4); -right * cos(pi/4); ...
             up * cos(pi/4); -up * cos(pi/4)];
dirs_base = dirs_base ./ vecnorm(dirs_base, 2, 2);
dirs_rand = [];
for k = 1:30
    phi = 2 * pi * rand;
    rand_dir = right * cos(phi) + up * sin(phi);
    dirs_rand = [dirs_rand; rand_dir / norm(rand_dir)]; %#ok<AGROW>
end
dirs = [dirs_base; dirs_rand];
best = -inf;
for a = amp
    for j = 1:size(dirs, 1)
        cand = pos + fwd * spd + dirs(j, :) * a;
        if any(cand < env.map(1)) || any(cand > env.map(2))
            continue;
        end
        if ~seg_free_dyn(pos, cand, env, dyn_pos, r_safe)
            continue;
        end
        score = -norm(cand - goal) - 0.3 * a;
        if score > best
            best = score; wp = cand;
        end
    end
    if ~isempty(wp)
        break;
    end
end
end

function tf = seg_free_dyn(a, b, env, dyn_pos, r_safe)
L = norm(b - a);
n = max(2, ceil(L / 0.5));
t = linspace(0, 1, n)';
pts = a + t .* (b - a);
for i = 1:n
    if env_dist(pts(i, :), env) < env.safe
        tf = false; return;
    end
    for k = 1:size(dyn_pos, 1)
        if norm(pts(i, :) - dyn_pos(k, :)) <= r_safe
            tf = false; return;
        end
    end
end
tf = true;
end

function ok = step_safe(a, b, env, dyn, r_safe)
L = norm(b - a);
n = max(2, ceil(L / 1.0));
t = linspace(0, 1, n)';
pts = a + t .* (b - a);
for i = 1:n
    if env_dist(pts(i, :), env) < env.safe
        ok = false; return;
    end
    for k = 1:dyn.num
        if norm(pts(i, :) - dyn.curr(k, :)) <= r_safe
            ok = false; return;
        end
    end
end
ok = true;
end

function npos = safe_detour(pos, aim, goal, env, dyn, r_safe, spd)
fwd = aim - pos;
if norm(fwd) < 1e-6
    fwd = goal - pos;
end
if norm(fwd) < 1e-6
    fwd = [1 0 0];
end
fwd = fwd / norm(fwd);
right = cross(fwd, [0 0 1]);
if norm(right) < 1e-6
    right = [1 0 0];
end
right = right / norm(right);

amp = [5 8];
dirs = [right; -right];
dirs = dirs ./ vecnorm(dirs, 2, 2);
npos = [];
for a = amp
    for j = 1:size(dirs, 1)
        cand = pos + dirs(j, :) * a;
        if any(cand < 0) || any(cand > 200)
            continue;
        end
        if step_safe(pos, cand, env, dyn, r_safe)
            npos = pos + dirs(j, :) * 2.0;
            return;
        end
    end
end
end

function v = local_prctile(x, p)
x = sort(x(:));
if isempty(x)
    v = NaN; return;
end
n = length(x);
idx = p / 100 * (n - 1) + 1;
lo = floor(idx); hi = ceil(idx);
if lo == hi
    v = x(lo);
else
    w = idx - lo;
    v = x(lo) * (1 - w) + x(hi) * w;
end
end

% =========================================================================
%  VO / RVO / DWA 决策辅助函数（与 VO/sim_vo.m、RVO/sim_rvo.m、DWA/sim_dwa.m 一致）
% =========================================================================

function vel = vo_select(pos, v_pref, obs_pos, obs_vel, r_safe, tau, env)
uav_speed = norm(v_pref);
if uav_speed < 1e-9
    uav_speed = 2.0;
end
pref_dir = v_pref / max(norm(v_pref), 1e-9);

if ~any_vo_collision(pos, v_pref, obs_pos, obs_vel, r_safe, tau) ...
        && step_static_free(pos, v_pref, env, env.safe)
    vel = v_pref;
    return;
end

up0 = [0 0 1];
right = cross(pref_dir, up0);
if norm(right) < 1e-6
    right = [1 0 0];
end
right = right / norm(right);
up = cross(right, pref_dir); up = up / max(norm(up), 1e-9);

speeds = uav_speed * [1.0 0.7 0.5 0.3];
yaws   = linspace(-1.5, 1.5, 21);
pitchs = [-0.45 0 0.45];

best = [];
best_score = -inf;
for sp = speeds
    for ya = yaws
        for pc = pitchs
            dir = rot3d(pref_dir, up, ya);
            dir = rot3d(dir, right, pc);
            dir = dir / max(norm(dir), 1e-9);
            if dot(dir, pref_dir) < -0.2
                continue;
            end
            v = dir * sp;
            if any_vo_collision(pos, v, obs_pos, obs_vel, r_safe, tau)
                continue;
            end
            if ~step_static_free(pos, v, env, env.safe)
                continue;
            end
            score = dot(dir, pref_dir) + 0.4 * (sp / uav_speed) - 0.02 * abs(ya);
            if score > best_score
                best_score = score;
                best = v;
            end
        end
    end
end
if ~isempty(best)
    vel = best;
    return;
end

d1 = (right + up);  d1 = d1 / max(norm(d1), 1e-9);
d2 = (right - up);  d2 = d2 / max(norm(d2), 1e-9);
d3 = (-right + up); d3 = d3 / max(norm(d3), 1e-9);
d4 = (-right - up); d4 = d4 / max(norm(d4), 1e-9);
escape_dirs = [right; -right; up; -up; d1; d2; d3; d4];
for k = 1:size(escape_dirs, 1)
    v_test = escape_dirs(k, :) * uav_speed * 0.5;
    if ~any_vo_collision(pos, v_test, obs_pos, obs_vel, r_safe, tau) ...
            && step_static_free(pos, v_test, env, env.safe)
        vel = v_test;
        return;
    end
end
vel = [0 0 0];
end

function tf = any_vo_collision(pos, v, obs_pos, obs_vel, r_safe, tau)
tf = false;
for i = 1:size(obs_pos, 1)
    if in_vo(pos, v, obs_pos(i, :), obs_vel(i, :), r_safe, tau)
        tf = true;
        return;
    end
end
end

function tf = in_vo(pos, v, obs_pos, obs_vel, r_safe, tau)
r = obs_pos - pos;
dv = v - obs_vel;
d0 = norm(r);
if d0 <= r_safe
    tf = true;
    return;
end
ndv2 = sum(dv .^ 2);
if ndv2 < 1e-12
    tf = false;
    return;
end
t_c = dot(r, dv) / ndv2;
if t_c < 0 || t_c > tau
    tf = false;
    return;
end
d_min = norm(r - dv * t_c);
tf = d_min <= r_safe;
end

function v = obs_velocity(dyn, i)
% 障碍物 i 的瞬时速度向量（m/步），供 VO/RVO 碰撞锥外推使用。
if isfield(dyn, 'obs') && ~isempty(dyn.obs)
    o = dyn.obs{i};
    switch o.type
        case 1
            u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
            v = u * o.speed * o.dir;
        case 2
            v = o.R * o.omega * [-sin(o.theta), cos(o.theta), 0];
        case 3
            u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
            v = u * o.v0 * (1 + o.amp * sin(2 * pi * o.t / o.period)) * o.dir;
        case 4
            u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
            v = u * o.hspeed * o.dir + [0, 0, o.A * (2 * pi / o.Tz) * cos(2 * pi * o.t / o.Tz)];
        otherwise
            v = [0 0 0];
    end
else
    p1 = dyn.params(i, 1:3);
    p2 = dyn.params(i, 4:6);
    u = (p2 - p1) / max(norm(p2 - p1), 1e-9);
    v = u * dyn.speed * dyn.params(i, 7);
end
end

function tf = step_static_free(pos, vel, env, safe)
L = norm(vel);
if L < 1e-9
    tf = env_dist(pos, env) >= safe + 0.5;
    return;
end
n = max(2, ceil(L / 0.5));
t = linspace(0, 1, n)';
pts = pos + t .* vel;
for i = 1:n
    if env_dist(pts(i, :), env) < safe + 0.5
        tf = false;
        return;
    end
end
tf = true;
end

function vel = rvo_select(pos, v_cur, v_pref, obs_pos, obs_vel, r_safe, tau, env)
uav_speed = norm(v_pref);
if uav_speed < 1e-9
    uav_speed = 2.0;
end
pref_dir = v_pref / max(norm(v_pref), 1e-9);

if ~any_rvo_collision(pos, v_pref, v_cur, obs_pos, obs_vel, r_safe, tau) ...
        && step_static_free(pos, v_pref, env, env.safe)
    vel = v_pref;
    return;
end

up0 = [0 0 1];
right = cross(pref_dir, up0);
if norm(right) < 1e-6
    right = [1 0 0];
end
right = right / norm(right);
up = cross(right, pref_dir); up = up / max(norm(up), 1e-9);

speeds = uav_speed * [1.0 0.7 0.5 0.3];
yaws   = linspace(-1.5, 1.5, 21);
pitchs = [-0.45 0 0.45];

best = [];
best_score = -inf;
for sp = speeds
    for ya = yaws
        for pc = pitchs
            dir = rot3d(pref_dir, up, ya);
            dir = rot3d(dir, right, pc);
            dir = dir / max(norm(dir), 1e-9);
            if dot(dir, pref_dir) < -0.2
                continue;
            end
            v = dir * sp;
            if any_rvo_collision(pos, v, v_cur, obs_pos, obs_vel, r_safe, tau)
                continue;
            end
            if ~step_static_free(pos, v, env, env.safe)
                continue;
            end
            score = dot(dir, pref_dir) + 0.4 * (sp / uav_speed) - 0.02 * abs(ya);
            if score > best_score
                best_score = score;
                best = v;
            end
        end
    end
end
if ~isempty(best)
    vel = best;
    return;
end

d1 = (right + up);  d1 = d1 / max(norm(d1), 1e-9);
d2 = (right - up);  d2 = d2 / max(norm(d2), 1e-9);
d3 = (-right + up); d3 = d3 / max(norm(d3), 1e-9);
d4 = (-right - up); d4 = d4 / max(norm(d4), 1e-9);
escape_dirs = [right; -right; up; -up; d1; d2; d3; d4];
for k = 1:size(escape_dirs, 1)
    v_test = escape_dirs(k, :) * uav_speed * 0.5;
    if ~any_rvo_collision(pos, v_test, v_cur, obs_pos, obs_vel, r_safe, tau) ...
            && step_static_free(pos, v_test, env, env.safe)
        vel = v_test;
        return;
    end
end
vel = [0 0 0];
end

function tf = any_rvo_collision(pos, v, v_cur, obs_pos, obs_vel, r_safe, tau)
tf = false;
for i = 1:size(obs_pos, 1)
    if in_rvo(pos, v, v_cur, obs_pos(i, :), obs_vel(i, :), r_safe, tau)
        tf = true;
        return;
    end
end
end

function tf = in_rvo(pos, v_cand, v_cur, obs_pos, obs_vel, r_safe, tau)
tf = in_vo(pos, 2 * v_cand - v_cur, obs_pos, obs_vel, r_safe, tau);
end

function [npos, nvel] = dwa_step(pos, vel, ref, goal, env, dyn, r_safe, p)
% 纯 DWA 单步（与 DWA/sim_dwa.m 一致）：近场过滤 + 动态窗口采样 + 前向预测打分。
if isfield(dyn, 'num') && dyn.num > 0
    d2 = sqrt(sum((dyn.curr - pos) .^ 2, 2));
    keep = d2 <= 35;
    if any(keep)
        dyn.num  = sum(keep);
        dyn.curr = dyn.curr(keep, :);
        if isfield(dyn, 'obs') && ~isempty(dyn.obs)
            dyn.obs = dyn.obs(keep);
        else
            dyn.params = dyn.params(keep, :);
        end
    else
        dyn.num  = 0;
        dyn.curr = zeros(0, 3);
        if isfield(dyn, 'obs') && ~isempty(dyn.obs)
            dyn.obs = cell(0, 1);
        else
            dyn.params = zeros(0, 7);
        end
    end
end

v_mag = norm(vel);
if v_mag < 1e-9
    v_mag = p.vmin;
end
refdir = (ref - pos) / max(norm(ref - pos), 1e-9);

up0 = [0 0 1];
right = cross(refdir, up0);
if norm(right) < 1e-6
    right = [1 0 0];
end
right = right / norm(right);
up = cross(right, refdir); up = up / max(norm(up), 1e-9);

v_lo = max(p.vmin, v_mag - p.dvmax);
v_hi = min(p.vmax, v_mag + p.dvmax);
vcand   = linspace(v_lo, v_hi, 5);
angcand = linspace(-p.dthmax, p.dthmax, 21);
pchcand = [-0.45 0 0.45];

obs_pred = cell(1, p.pred);
for st = 1:p.pred
    obs_pred{st} = dyn_after(dyn, st).curr;
end

best_score = -inf;
npos = pos; nvel = vel;
found = false;
for v = vcand
    for da = angcand
        for dpc = pchcand
            newdir = rot3d(refdir, up, da);
            newdir = rot3d(newdir, right, dpc);
            newdir = newdir / max(norm(newdir), 1e-9);
            if dot(newdir, refdir) < -0.3
                continue;
            end
            newvel = newdir * v;
            p0 = pos; clear_min = inf; ok = true;
            for st = 1:p.pred
                p0 = p0 + newvel * p.dt;
                if ~dwa_feasible(p0, env, obs_pred{st}, r_safe)
                    ok = false;
                    break;
                end
                if dyn.num > 0
                    d = min(sqrt(sum((p0 - obs_pred{st}) .^ 2, 2))) - dyn.r;
                    clear_min = min(clear_min, max(d, 0));
                end
            end
            if ~ok
                continue;
            end
            hd = dot(newdir, refdir);
            cl = min(clear_min, 30) / 30;
            score = p.w_head * hd + p.w_clear * cl + p.w_speed * (v / p.vmax) - 0.05 * abs(da);
            if score > best_score
                best_score = score;
                nvel = newvel;
                npos = pos + newvel * p.dt;
                found = true;
            end
        end
    end
end

if ~found
    next_obs = obs_pred{1};
    d1 = (right + up);  d1 = d1 / max(norm(d1), 1e-9);
    d2 = (right - up);  d2 = d2 / max(norm(d2), 1e-9);
    d3 = (-right + up); d3 = d3 / max(norm(d3), 1e-9);
    d4 = (-right - up); d4 = d4 / max(norm(d4), 1e-9);
    dirs = [right; -right; up; -up; d1; d2; d3; d4];
    for k = 1:size(dirs, 1)
        cd = pos + dirs(k, :) * 6;
        if dwa_feasible(cd, env, next_obs, r_safe)
            nvel = dirs(k, :) * p.vmin;
            npos = pos + nvel * p.dt;
            return;
        end
    end
    nvel = [0 0 0];
    npos = pos;
end
end