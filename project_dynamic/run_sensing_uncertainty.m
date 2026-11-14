% =========================================================================
%  Sensing Uncertainty Robustness -- 障碍物位置观测噪声下的避障性能
% -------------------------------------------------------------------------
%  目的：回应审稿人 R3-5 的 "sensing uncertainty"。给障碍物位置观测加高斯噪声
%        sigma ∈ {0, 0.5, 1.0, 1.5, 2.0} m（聚焦合理感知噪声，σ=3m 已超障碍半径舍去）。
%        规划决策用有噪声观测、碰撞检测用真实位置。
%  算法：6 算法 × 14 四类障碍；σ=0 用 30 seeds（对齐主对比 Table IX），σ>0 用 60 seeds。
%  指标（均只统计成功运行，增长率/平滑度用中位数）：
%        成功率、能耗增长率、平滑度指数(cosθ_avg，越大越平滑)、路径增长率。
%  输出：控制台表格 + 1 张 2×2 拼图 png + xlsx。
% =========================================================================
clear; clc; close all;
root = fileparts(mfilename('fullpath'));
addpath(root);
addpath(fullfile(root, 'obstacle_gradient_exp'));
addpath(fullfile(root, 'VO'));
addpath(fullfile(root, 'RVO'));
addpath(fullfile(root, 'DWA'));

sigmas  = [0 0.5 1.0 1.5 2.0];   % 聚焦合理噪声范围
nseed   = 60;                     % 加 seeds 平滑随机波动
algos   = {'This algorithm', 'A*+APF', 'AVSS-RRT', 'VO', 'RVO', 'DWA'};
simFcns = {@sim_rgvs, @sim_aapf, @sim_avss, @sim_vo, @sim_rvo, @sim_dwa};

env   = build_env();
start = env.start; goal = env.goal;

% 静态路径：本文/VO/RVO/DWA 共用 ACOSRAR；A*+APF、AVSS-RRT 用各自路径（与主对比一致）
path_ours = plan_this(env);
path_aapf = plan_aapf(env);
path_avss = plan_avss(env);
paths = {path_ours, path_aapf, path_avss, path_ours, path_ours, path_ours};

nSigma = numel(sigmas);
nAlgo  = numel(algos);

% 静态路径长度/能耗（基准，用于算增长率）
static_len = zeros(1, nAlgo);
static_eng = zeros(1, nAlgo);
for a = 1:nAlgo
    [static_len(a), static_eng(a)] = path_metrics(paths{a});
end

% 指标
succ = zeros(nSigma, nAlgo);   % 成功率(%)
engr = zeros(nSigma, nAlgo);   % 能耗增长率中位数(%)
smth = zeros(nSigma, nAlgo);   % 平滑度指数 cosθ_avg（越大越平滑）
lenr = zeros(nSigma, nAlgo);   % 路径增长率中位数(%)

fprintf('======== Sensing Uncertainty Robustness (位置观测噪声) ========\n');
fprintf('噪声 σ(m): %s  |  seeds=%d\n\n', sprintf('%.1f ', sigmas), nseed);

for si = 1:nSigma
    sigma = sigmas(si);
    % σ=0 用 30 seeds 对齐主对比（Table IX）；σ>0 用 60 seeds 看退化趋势
    if sigma == 0
        nseed_this = 30;
    else
        nseed_this = nseed;
    end
    for a = 1:nAlgo
        ok   = zeros(nseed_this, 1);
        einc = nan(nseed_this, 1);
        smth_vec = nan(nseed_this, 1);
        linc = nan(nseed_this, 1);
        for s = 1:nseed_this
            dyn = gen_obstacles_v2(s, paths{a}, env);   % 14 四类障碍，按各自路径生成
            [~, trail, st] = simFcns{a}(paths{a}, dyn, start, goal, env, sigma);
            ok(s) = (st == 1);
            if st == 1
                [L, E] = path_metrics(trail);
                linc(s) = (L - static_len(a)) / static_len(a) * 100;
                einc(s) = (E - static_eng(a)) / static_eng(a) * 100;
                smth_vec(s) = smoothness_index(trail);
            end
        end
        succ(si, a) = mean(ok) * 100;
        engr(si, a) = median(einc(ok == 1));
        smth(si, a) = median(smth_vec(ok == 1));
        lenr(si, a) = median(linc(ok == 1));
    end
    fprintf('σ=%.1f m done (%d seeds)\n', sigma, nseed_this);
end

% ---------- 控制台表格 ----------
print_block('Avoidance Success Rate (%)', succ, sigmas, algos);
print_block('Energy Increase Rate (%)', engr, sigmas, algos);
print_block('Smoothness Index (cos\theta_{avg}, 越大越平滑)', smth, sigmas, algos);
print_block('Path Length Increase Rate (%)', lenr, sigmas, algos);

% ---------- 4 个指标拼成一张 2×2 图并保存 ----------
colors = [0.85 0.10 0.10; 0.10 0.60 0.20; 0.10 0.45 0.80; ...
          0.60 0.20 0.80; 0.90 0.55 0.10; 0.00 0.60 0.70];
markers = {'-o', '-s', '-^', '-d', '-v', '-p'};

M_all   = {succ, engr, smth, lenr};
ylabels = {'Avoidance Success Rate (%)', 'Energy Increase Rate (%)', ...
           'Smoothness Index (cos\theta_{avg})', 'Path Length Increase Rate (%)'};
letters = {'a', 'b', 'c', 'd'};

fig = figure('Color', 'white', 'Position', [60 60 1200 900]);
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

for k = 1:4
    ax = nexttile;
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    for a = 1:numel(algos)
        plot(ax, sigmas, M_all{k}(:, a), markers{a}, 'LineWidth', 1.8, ...
            'Color', colors(a, :), 'MarkerFaceColor', 'w', 'MarkerSize', 7, ...
            'DisplayName', algos{a});
    end
    % 编号紧贴在 xlabel "Obstacle position observation noise ..." 正下方
    xlabel(ax, {'Obstacle position observation noise \sigma (m)', sprintf('(%s)', letters{k})}, 'FontSize', 12);
    ylabel(ax, ylabels{k}, 'FontSize', 12);
    set(ax, 'FontSize', 11, 'XTick', sigmas);
    if k == 1
        legend(ax, 'Location', 'best', 'FontSize', 9, 'Interpreter', 'none');
    end
    hold(ax, 'off');
end

outpng = fullfile(root, 'sensing_combined.png');
print(fig, outpng, '-dpng', '-r200');
fprintf('图已保存: %s\n', outpng);

% ---------- xlsx 输出 ----------
outfile = fullfile(root, 'sensing_uncertainty_results.xlsx');
write_mat(succ, sigmas, algos, outfile, 'success_pct');
write_mat(engr, sigmas, algos, outfile, 'energy_inc_pct');
write_mat(smth, sigmas, algos, outfile, 'smoothness_cos');
write_mat(lenr, sigmas, algos, outfile, 'path_inc_pct');
fprintf('\n结果已保存至 %s\n', outfile);
fprintf('================ 完成 ================\n');

% ================= 局部函数 =================
function print_block(name, M, sigmas, algos)
fprintf('\n===== %s =====\n', name);
fprintf('%-8s', '\sigma(m)');
for a = 1:numel(algos)
    fprintf(' | %-14s', algos{a});
end
fprintf('\n%s\n', repmat('-', 1, 10 + 16 * numel(algos)));
for si = 1:numel(sigmas)
    fprintf('%-8.1f', sigmas(si));
    for a = 1:numel(algos)
        v = M(si, a);
        if isnan(v)
            fprintf(' | %14s', '--');
        else
            fprintf(' | %14.2f', v);
        end
    end
    fprintf('\n');
end
end

function write_mat(M, sigmas, algos, file, sheet)
T = array2table(M, 'VariableNames', algos, ...
    'RowNames', cellstr(num2str(sigmas', 'sigma_%.1fm')));
writetable(T, file, 'Sheet', sheet, 'WriteRowNames', true);
end

function c = smoothness_index(path)
% 平滑度指数 cosθ_avg = mean(cosθ)，θ 为相邻路径向量夹角（越大越平滑，与论文一致）。
% 论文表格报的是 cosθ_avg ∈ (0,1]，越接近 1 越平滑。
c = 1;   % 单段/退化路径视为最平滑
if size(path, 1) < 3
    return;
end
seg = diff(path, 1, 1);
cosv = [];
for j = 1:size(seg, 1) - 1
    v1 = seg(j, :); v2 = seg(j + 1, :);
    n1 = norm(v1); n2 = norm(v2);
    if n1 < 1e-6 || n2 < 1e-6
        continue;   % 跳过零长度段（重复点）
    end
    cosv(end + 1, 1) = dot(v1, v2) / (n1 * n2); %#ok<AGROW>
end
if isempty(cosv)
    c = 1;
else
    c = mean(cosv);
end
end
