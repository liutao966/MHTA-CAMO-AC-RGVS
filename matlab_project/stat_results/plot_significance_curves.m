% =========================================================================
%  plot_significance_curves.m
%  绘制 8 个算法在障碍物地图上的 30 次显著性曲线（5 个指标），
%  全部拼接在一张图上（2x3 紧凑布局，只用 5 格，第 6 格留空）。
%  每张子图下方标编号 (a)(b)(c)(d)(e)，图例放在整张图底部（横排）。
%  横轴 = run (seed)，纵轴 = 指标值；8 条曲线不同颜色，本文算法用红色。
%  数据源: stat_results_obstacle.xlsx 的 RawData sheet（列索引访问）。
% =========================================================================
clear; clc; close all;

% ---------- 读取数据 ----------
T = readtable('stat_results_obstacle.xlsx', 'Sheet', 'RawData');

% RawData 列顺序：
%   1 Algorithm, 2 Run (seed), 3 Path length, 4 Min safe dist,
%   5 Energy, 6 Flight time, 7 Planning time, 8 A* time, 9 CAMO-AC time, 10 Smoothness
alg_col    = 1;
seed_col   = 2;
metric_cols = [3 4 5 6 10];   % Path, Safe, Energy, Flight, Smoothness

% ---------- 算法顺序（与论文主对比一致，本文算法放最后） ----------
algos = {'ACOSRAR', 'Chaos-SSA (ICSSA)', 'HSMA', 'Improved RRT*', ...
         'NMOPOS', 'Octree', 'RRT*', 'This algorithm'};
nAlgo = numel(algos);

% ---------- 5 个指标（纵轴标签） ----------
metric_names = {'Path length (m)', 'Min safe dist (m)', 'Energy (Wh)', ...
                'Flight time (s)', 'Smoothness'};
nMetric = numel(metric_names);

% ---------- 颜色：本文算法红色，其余用区分度高的颜色 ----------
colors = [
    0.20 0.60 0.30;   % ACOSRAR        绿
    0.25 0.35 0.80;   % Chaos-SSA      蓝
    0.95 0.50 0.15;   % HSMA           橙
    0.55 0.15 0.70;   % Improved RRT*  紫
    0.20 0.75 0.85;   % NMOPOS         青
    0.30 0.55 0.50;   % Octree         蓝绿
    0.60 0.40 0.15;   % RRT*           棕
    0.90 0.10 0.25;   % This algorithm 红
];

% ---------- 一张大图：2x3 紧凑布局，只用 5 格 ----------
figure('Position', [60 40 1280 800], 'Color', 'w');
t = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

letters = 'abcde';
h_first = gobjects(nAlgo, 1);   % 第 1 个子图的线句柄，用于底部图例

for mi = 1:nMetric
    nexttile;
    hold on; grid on; box on;

    for ai = 1:nAlgo
        idx = strcmp(T{:, alg_col}, algos{ai});
        x = T{idx, seed_col};            % seed 1..30
        y = T{idx, metric_cols(mi)};     % 指标值
        hh = plot(x, y, '-o', 'Color', colors(ai, :), 'LineWidth', 1.2, ...
            'MarkerSize', 3, 'MarkerFaceColor', 'w', 'DisplayName', algos{ai});
        if mi == 1
            h_first(ai) = hh;
        end
    end

    xlabel({'Run (seed)', sprintf('(%s)', letters(mi))}, 'FontSize', 18);
    ylabel(metric_names{mi}, 'FontSize', 18);
    set(gca, 'FontSize', 15);
    xlim([1 30]);
end

% ---------- 第 6 格：图例（竖排，用第 1 子图句柄，不画线） ----------
nexttile;
axis off;
legend(h_first, algos, 'Location', 'west', 'FontSize', 18, 'Interpreter', 'none');

% ---------- 保存 ----------
saveas(gcf, 'significance_curves.png');
fprintf('图已保存: %s\n', fullfile(pwd, 'significance_curves.png'));
