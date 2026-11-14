% =========================================================================
%  四类动态障碍物基准对比 —— 6 算法（30 次均±std + 碰撞率 + 统计显著性）
% -------------------------------------------------------------------------
%  障碍物：gen_obstacles_v2 —— 14 个四类障碍（直线×3 圆弧×3 变速×4 垂直×4），
%          **按各自路径生成**：本文算法/VO/RVO/DWA 用 path_ours；A*+APF 用 path_aapf；
%          AVSS-RRT 用 path_avss。每个算法面对自己路径上的障碍。
%  算法：This algorithm(ACOSRAR+RGVS) / A*+APF / AVSS-RRT / VO / RVO / DWA。
%  输出（30 次，无"单次最优"，追踪误差已删除）：
%    主表：Succ(%) 均值±std / Coll(%) 均值 / LenIncRate(%) 中位数±std / EngIncRate(%) 中位数±std
%    显著性：本文算法 vs 各基线（配对 McNemar + 配对 Wilcoxon + Cliff's delta + Holm 校正）
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

static_len = zeros(1, nAlgo);
static_eng = zeros(1, nAlgo);
for a = 1:nAlgo
    [static_len(a), static_eng(a)] = path_metrics(paths{a});
end

succ = zeros(nAlgo, nseed);   % 1=到达
coll = zeros(nAlgo, nseed);   % 1=碰撞
plen = zeros(nAlgo, nseed);
peng = zeros(nAlgo, nseed);

fprintf('======== 四类动态障碍基准 (14 障碍, %d seeds, 按各自路径生成) ========\n', nseed);
for s = 1:nseed
    for a = 1:nAlgo
        dyn = gen_obstacles_v2(s, paths{a}, env);   % 每个算法按自己的路径生成障碍
        [ok, trail, st] = simFcns{a}(paths{a}, dyn, start, goal, env);
        [l, e] = path_metrics(trail);
        succ(a, s) = (st == 1);
        coll(a, s) = (st == 2);
        plen(a, s) = l;
        peng(a, s) = e;
    end
    if mod(s, 5) == 0 || s == nseed
        fprintf('seed %2d/%d done\n', s, nseed);
    end
end

% ---------- 主表（成功率/碰撞率均值±std；路径/能耗增长率中位数±std） ----------
fprintf('\n%-16s | %14s | %10s | %16s | %16s\n', ...
    'Algorithm', 'Succ(%)', 'Coll(%)', 'LenIncRate(%)', 'EngIncRate(%)');
fprintf('%s\n', repmat('-', 1, 76));

summ = cell(nAlgo, 8);
for a = 1:nAlgo
    sr_m = mean(succ(a, :)) * 100;
    sr_s = std(succ(a, :)) * 100;
    cl_m = mean(coll(a, :)) * 100;
    okidx = succ(a, :) > 0;
    if any(okidx)
        lir_m = median((plen(a, okidx) - static_len(a)) / static_len(a) * 100);
        lir_s = std((plen(a, okidx) - static_len(a)) / static_len(a) * 100);
        eir_m = median((peng(a, okidx) - static_eng(a)) / static_eng(a) * 100);
        eir_s = std((peng(a, okidx) - static_eng(a)) / static_eng(a) * 100);
    else
        lir_m = NaN; lir_s = NaN; eir_m = NaN; eir_s = NaN;
    end
    summ(a, :) = {algos{a}, sr_m, sr_s, cl_m, lir_m, lir_s, eir_m, eir_s};
    fprintf('%-16s | %6.1f±%.1f | %9.1f | %7.2f±%.2f | %7.2f±%.2f\n', ...
        algos{a}, sr_m, sr_s, cl_m, lir_m, lir_s, eir_m, eir_s);
end
fprintf('%s\n', repmat('-', 1, 76));
fprintf('（Succ/Coll 为 30 次均值±std；LenInc/EngInc 为中位数±std，仅统计成功运行）\n');

% ---------- 统计显著性（本文算法 vs 各基线，配对 30 seeds） ----------
a_ours = 1;
p_mcn = zeros(1, nAlgo - 1);
p_len = zeros(1, nAlgo - 1);
p_eng = zeros(1, nAlgo - 1);

fprintf('\n======== 统计显著性（本文算法 vs 各基线，配对 %d seeds） ========\n', nseed);
fprintf('%-12s | %10s | %9s | %11s | %11s | %11s\n', ...
    'Baseline', 'McNemar p', 'SuccDiff', 'Wilcox p(Len)', 'Wilcox p(Eng)', 'Cliff delta');
fprintf('%s\n', repmat('-', 1, 70));

for b = 2:nAlgo
    % 成功率：配对 McNemar（二值）
    b_cnt = sum(succ(a_ours, :) == 1 & succ(b, :) == 0);   % 本文成功、基线失败
    c_cnt = sum(succ(a_ours, :) == 0 & succ(b, :) == 1);   % 本文失败、基线成功
    if b_cnt + c_cnt > 0
        chi2 = (abs(b_cnt - c_cnt) - 1)^2 / (b_cnt + c_cnt);   % 连续性校正
        p_mcn(b - 1) = 1 - chi2cdf(chi2, 1);
    else
        p_mcn(b - 1) = NaN;
    end
    succ_diff = (mean(succ(a_ours, :)) - mean(succ(b, :))) * 100;   % 效应量：成功率差

    % 路径/能耗增长率：配对 Wilcoxon（连续，只在两者都成功的 seed 上比较）
    both = succ(a_ours, :) == 1 & succ(b, :) == 1;
    if sum(both) >= 3
        xl = (plen(a_ours, both) - static_len(a_ours)) / static_len(a_ours) * 100;
        yl = (plen(b, both) - static_len(b)) / static_len(b) * 100;
        p_len(b - 1) = signrank(xl, yl);

        xe = (peng(a_ours, both) - static_eng(a_ours)) / static_eng(a_ours) * 100;
        ye = (peng(b, both) - static_eng(b)) / static_eng(b) * 100;
        p_eng(b - 1) = signrank(xe, ye);

        % Cliff's delta（路径增长率，配对）：(更多 - 更少) / 配对对数
        n_pairs = sum(both);
        cd = (sum(xl > yl) - sum(xl < yl)) / n_pairs;
    else
        p_len(b - 1) = NaN; p_eng(b - 1) = NaN; cd = NaN;
    end

    fprintf('%-12s | %10.4f | %8.1f%% | %11.4f | %11.4f | %11.3f\n', ...
        algos{b}, p_mcn(b - 1), succ_diff, p_len(b - 1), p_eng(b - 1), cd);
end
fprintf('%s\n', repmat('-', 1, 70));

% Holm-Bonferroni 校正（对 5 个基线的 McNemar / Wilcoxon 各自校正）
fprintf('\nHolm-Bonferroni 校正后的 p 值（越小越显著）：\n');
fprintf('%-12s | %12s | %13s | %13s\n', 'Baseline', 'McNemar p_adj', 'Wilcox p_adj(Len)', 'Wilcox p_adj(Eng)');
p_mcn_adj = holm_bonferroni(p_mcn);
p_len_adj = holm_bonferroni(p_len);
p_eng_adj = holm_bonferroni(p_eng);
for b = 2:nAlgo
    fprintf('%-12s | %12.4f | %13.4f | %13.4f\n', ...
        algos{b}, p_mcn_adj(b - 1), p_len_adj(b - 1), p_eng_adj(b - 1));
end

% ---------- 保存到 xlsx ----------
outfile = fullfile(root, 'dynamic_benchmark_results.xlsx');

raw = cell(nAlgo * nseed, 6);
row = 0;
for a = 1:nAlgo
    for s = 1:nseed
        row = row + 1;
        raw(row, :) = {s, algos{a}, succ(a, s), coll(a, s), plen(a, s), peng(a, s)};
    end
end
T_raw = cell2table(raw, 'VariableNames', ...
    {'seed', 'algorithm', 'success', 'collision', 'path_length', 'energy'});

T_summ = cell2table(summ, 'VariableNames', ...
    {'algorithm', 'succ_mean', 'succ_std', 'coll_mean', ...
     'len_inc_median', 'len_inc_std', 'eng_inc_median', 'eng_inc_std'});

writetable(T_raw,  outfile, 'Sheet', 'RawData');
writetable(T_summ, outfile, 'Sheet', 'Summary');
fprintf('\n结果已保存至 %s\n', outfile);
fprintf('================ 完成 ================\n');
