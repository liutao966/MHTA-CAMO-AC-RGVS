% =========================================================================
%  逐 seed 诊断：本文算法(sim_rgvs) 30 个 seed 的成功/轨迹长/追踪误差
% -------------------------------------------------------------------------
%  目的：找出追踪误差异常大（"追圆弧障碍跑飞"）的 seed，便于单独可视化定位根因。
% =========================================================================
clear; clc; close all;

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'obstacle_gradient_exp'));
addpath(fullfile(root, 'VO'));
addpath(fullfile(root, 'RVO'));
addpath(fullfile(root, 'DWA'));

env = build_env();
start = env.start; goal = env.goal;

fprintf('生成全局参考路径 (plan_this) ...\n');
path_ours = plan_this(env);

nseed = 30;
res = zeros(nseed, 4);   % [seed, succ, trail_len, trackerr]

fprintf('\n=== 本文算法逐 seed 结果 ===\n');
fprintf('%-6s | %8s | %10s | %10s | %s\n', 'seed', 'succ', 'trail(m)', 'trackerr(m)', 'status');
for s = 1:nseed
    dyn = gen_obstacles_v2(s, path_ours, env);
    [ok, trail, st] = sim_rgvs(path_ours, dyn, start, goal, env);
    len = sum(sqrt(sum(diff(trail) .^ 2, 2)));
    te  = max_tracking_error(trail, path_ours);
    res(s, :) = [s, ok, len, te];
    if st == 1, sts = '到达'; elseif st == 2, sts = '碰撞'; else, sts = '超时'; end
    fprintf('%-6d | %8d | %10.2f | %10.2f | %s\n', s, ok, len, te, sts);
end

% 只看"成功到达"的 seed，按追踪误差降序
okidx = res(:, 2) == 1;
res_ok = res(okidx, :);
[~, ix] = sort(res_ok(:, 4), 'descend');
fprintf('\n=== 成功到达的 seed 中，追踪误差最大的 5 个 ===\n');
for k = 1:min(5, size(res_ok, 1))
    r = res_ok(ix(k), :);
    fprintf('seed %d: trail=%.2f m, trackerr=%.2f m\n', r(1), r(3), r(4));
end
