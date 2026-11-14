% =========================================================================
%  障碍物类型分组实验：4 类型 × 6 算法成功率
% -------------------------------------------------------------------------
%  目的：证明 VO/RVO/DWA 恰好死在非直线障碍（圆弧/变速/垂直），本文算法全稳。
%  做法：gen_obstacles_v2 的单类型模式，分别生成 10 个 纯直线/纯圆弧/纯变速/纯垂直，
%        各跑 6 算法 × 30 seeds，统计成功率。
% =========================================================================
clear; clc; close all;

root = fileparts(mfilename('fullpath'));
addpath(root);
addpath(fullfile(root, 'obstacle_gradient_exp'));
addpath(fullfile(root, 'VO'));
addpath(fullfile(root, 'RVO'));
addpath(fullfile(root, 'DWA'));

nseed = 30;
env   = build_env();
start = env.start; goal = env.goal;

fprintf('生成静态路径 ...\n');
path_ours = plan_this(env);
path_aapf = plan_aapf(env);
path_avss = plan_avss(env);

algos   = {'This algorithm', 'A*+APF', 'AVSS-RRT', 'VO', 'RVO', 'DWA'};
paths   = {path_ours, path_aapf, path_avss, path_ours, path_ours, path_ours};
simFcns = {@sim_rgvs, @sim_aapf, @sim_avss, @sim_vo, @sim_rvo, @sim_dwa};
nAlgo = numel(algos);

type_names = {'直线匀速', '匀速圆弧', '变速直线', '垂直运动'};
nType = 4;

succ = zeros(nType, nAlgo);   % 成功率(%)

fprintf('======== 障碍物类型分组实验 (4 类型 × 6 算法, %d seeds) ========\n', nseed);
for t = 1:nType
    for a = 1:nAlgo
        s = zeros(nseed, 1);
        for seed = 1:nseed
            dyn = gen_obstacles_v2(seed, paths{a}, env, t);   % 单类型 t
            [ok, ~, ~] = simFcns{a}(paths{a}, dyn, start, goal, env);
            s(seed) = ok;
        end
        succ(t, a) = mean(s) * 100;
    end
    fprintf('类型 %d (%s) 完成\n', t, type_names{t});
end

% ---------- 输出表 ----------
fprintf('\n%-10s | %-16s | %-10s | %-10s | %-9s | %-9s | %-9s\n', ...
    '障碍类型', algos{1}, algos{2}, algos{3}, algos{4}, algos{5}, algos{6});
fprintf('%s\n', repmat('-', 1, 92));
for t = 1:nType
    fprintf('%-12s | %14.1f%% | %8.1f%% | %8.1f%% | %8.1f%% | %8.1f%% | %8.1f%%\n', ...
        type_names{t}, succ(t, 1), succ(t, 2), succ(t, 3), succ(t, 4), succ(t, 5), succ(t, 6));
end
fprintf('%s\n', repmat('-', 1, 92));
fprintf('（成功率 = 30 seeds 中到达终点的比例，单位 %%）\n');

% ---------- 保存 xlsx ----------
outfile = fullfile(root, 'type_grouping_results.xlsx');
T = array2table(succ, 'VariableNames', algos, 'RowNames', type_names);
writetable(T, outfile, 'WriteRowNames', true);
fprintf('结果已保存至 %s\n', outfile);
fprintf('================ 完成 ================\n');
