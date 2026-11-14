% =========================================================================
%  障碍数量梯度实验 —— 动态避障成功率 / 路径代价随动态障碍数量的变化
% -------------------------------------------------------------------------
%  三个动态避障算法（共用同一套随机动态障碍，保证公平对比）：
%    1. ACOSRAR  (untitled.m)  —— 本方法（A*+ACOSRAR + DWA 局部规划）
%    2. AVSS_RRT (main_dynamic.m)
%    3. A_APF    (main.m)
%  动态障碍数量梯度：2 / 4 / 6 / 8 / 10，每个数量下跑 30 次独立运行统计。
%
%  输出：
%    图1：成功率(左) 与 路径增量(右) 随障碍数量的折线（三算法各一线）
%    表 ：障碍数量扩展表（数量 × 算法 × 成功率 / 路径增量 / 能耗增量）
%    文件：obstacle_gradient_results.csv
% =========================================================================
clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));

counts = [2 4 6 8 10];
nseed  = 30;

algos    = {'This algorithm', 'AVSS_RRT', 'A_APF'};
planFcns = {@plan_this, @plan_avss, @plan_aapf};
simFcns  = {@sim_rgvs, @sim_avss, @sim_aapf};
colors   = [0.85 0.10 0.10; 0.10 0.45 0.80; 0.10 0.60 0.20];
markers  = {'o', 's', '^'};
lineWidths = [2.4 2.0 2.0];   % 本方法线最粗

nAlgo  = numel(algos);
nCount = numel(counts);

env   = build_env();
start = env.start; goal = env.goal;

succ_rate  = zeros(nAlgo, nCount);   % 成功率(%)
path_inc   = zeros(nAlgo, nCount);   % 平均路径增量(m)
energy_inc = zeros(nAlgo, nCount);   % 平均能耗增量(Wh)

% ---------- 1. 各算法静态基线路径（一次，固定种子） ----------
fprintf('======== 障碍数量梯度实验 ========\n');
static_path   = cell(1, nAlgo);
static_len    = zeros(1, nAlgo);
static_energy = zeros(1, nAlgo);
for a = 1:nAlgo
    sp = planFcns{a}(env);
    static_path{a} = sp;
    [static_len(a), static_energy(a)] = path_metrics(sp);
    md = min(arrayfun(@(i) env_dist(sp(i, :), env), 1:size(sp, 1)));
    fprintf('[%s] 静态路径长度 = %.2f m, 静态能耗 = %.2f Wh, 最小静态间隙 = %.2f m\n', ...
        algos{a}, static_len(a), static_energy(a), md);
end

% ---------- 2. 梯度循环 ----------
for c = 1:nCount
    cnt = counts(c);
    for a = 1:nAlgo
        succ = false(nseed, 1);
        plen = zeros(nseed, 1);
        peng = zeros(nseed, 1);
        for s = 1:nseed
            dyn = gen_obstacles(cnt, s, static_path{a}, env);   % 每算法按各自路径横穿生成障碍
            [ok, trail] = simFcns{a}(static_path{a}, dyn, start, goal, env);
            [l, e] = path_metrics(trail);
            succ(s) = ok;
            plen(s) = l;
            peng(s) = e;
        end
        succ_rate(a, c) = mean(succ) * 100;
        % 路径/能耗增量只统计“成功避障”的运行：失败运行会中途碰撞被截断、长度无意义
        if any(succ)
            path_inc(a, c)   = mean(plen(succ)) - static_len(a);
            energy_inc(a, c) = mean(peng(succ)) - static_energy(a);
        else
            path_inc(a, c)   = NaN;
            energy_inc(a, c) = NaN;
        end
        fprintf('[%s] 障碍数=%2d  成功率=%5.1f%%  路径增量=%+6.2f m  能耗增量=%+6.2f Wh\n', ...
            algos{a}, cnt, succ_rate(a, c), path_inc(a, c), energy_inc(a, c));
    end
end

% ---------- 3. 可视化（图1：成功率 + 路径增量） ----------
fig = figure('Position', [100 100 1200 520], 'Color', 'white');
ax1 = subplot(1, 2, 1);
hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
for a = 1:nAlgo
    plot(ax1, counts, succ_rate(a, :), '-', 'Color', colors(a, :), ...
        'LineWidth', lineWidths(a), 'Marker', markers{a}, 'MarkerSize', 8, ...
        'DisplayName', algos{a});
end
xlabel(ax1, 'Number of Dynamic Obstacles'); ylabel(ax1, 'Success Rate (%)');
title(ax1, 'Success Rate vs. Obstacle Count');
xlim(ax1, [1 11]); ylim(ax1, [0 105]);
set(ax1, 'XTick', counts, 'FontSize', 11);
legend(ax1, 'Location', 'southwest', 'FontSize', 10, 'Interpreter', 'none');

ax2 = subplot(1, 2, 2);
hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
for a = 1:nAlgo
    plot(ax2, counts, path_inc(a, :), '-', 'Color', colors(a, :), ...
        'LineWidth', lineWidths(a), 'Marker', markers{a}, 'MarkerSize', 8, ...
        'DisplayName', algos{a});
end
xlabel(ax2, 'Number of Dynamic Obstacles'); ylabel(ax2, 'Path Increment (m)');
title(ax2, 'Path Increment vs. Obstacle Count');
xlim(ax2, [1 11]);
set(ax2, 'XTick', counts, 'FontSize', 11);
legend(ax2, 'Location', 'northwest', 'FontSize', 10, 'Interpreter', 'none');

sgtitle('Obstacle Count Gradient Experiment (Mean of 30 Independent Runs)', 'FontSize', 14, 'FontWeight', 'bold');

% ---------- 4. 表格输出（控制台 + CSV） ----------
fprintf('\n================ 障碍数量扩展表 ================\n');
fprintf('数量 | %-18s | %-12s | %-12s | %-12s\n', '算法', '成功率(%)', '路径增量(m)', '能耗增量(Wh)');
fprintf('%s\n', repmat('-', 1, 76));
for c = 1:nCount
    for a = 1:nAlgo
        fprintf('%4d | %-18s | %12.1f | %12.2f | %12.2f\n', ...
            counts(c), algos{a}, succ_rate(a, c), path_inc(a, c), energy_inc(a, c));
    end
    fprintf('%s\n', repmat('-', 1, 76));
end

% 保存 CSV（行序与上方控制台表格一致：数量 × 算法）
oc = zeros(nAlgo * nCount, 1);
al = cell(nAlgo * nCount, 1);
sr = zeros(nAlgo * nCount, 1);
pi = zeros(nAlgo * nCount, 1);
ei = zeros(nAlgo * nCount, 1);
k = 0;
for c = 1:nCount
    for a = 1:nAlgo
        k = k + 1;
        oc(k) = counts(c);
        al{k} = algos{a};
        sr(k) = succ_rate(a, c);
        pi(k) = path_inc(a, c);
        ei(k) = energy_inc(a, c);
    end
end
T = table(oc, string(al), sr, pi, ei, ...
    'VariableNames', {'obstacle_count', 'algorithm', 'success_rate', ...
                      'path_increment', 'energy_increment'});
writetable(T, 'obstacle_gradient_results.csv');
fprintf('\n结果已保存至 obstacle_gradient_results.csv\n');
fprintf('================ 实验完成 ================\n');