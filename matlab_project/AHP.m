%% AHP 权重敏感性分析（回应 R1-4 / R2-3）
% 对论文 5 目标权重 [0.40, 0.35, 0.10, 0.10, 0.05] 的每个权重单独 ±50%（再归一化到和=1），
% 固定随机种子重跑"本文算法"（solve_this_v2），观察各指标是否稳定（结论对权重扰动是否稳健）。
% 说明：solve_this_v2 内部指标顺序为 [L, E, unsmooth, tm, dmin]（5 目标，与统计实验一致，ref_safe=10）。
clear; clc; close all; warning off;

here = fileparts(mfilename('fullpath'));
% 指向本文算法静态版（solve_this_v2.m 所在目录），与统计实验共用同一份代码，
% 以后改算法（权重/能耗/安全/走廊等）改这一处即可同步到 AHP 敏感性。
addpath(fullfile(here, 'MHTA-CAMO-AC'));

%% 1. 基准权重（论文 Table IV 的 5 目标，映射到代码顺序 [长度L, 能耗E, 平滑, 时间tm, 安全dmin]）
w0 = [0.40, 0.10, 0.10, 0.05, 0.35];
wNames = {'长度L', '能耗E', '平滑', '时间tm', '安全dmin'};

%% 2. 扰动组合：每个权重 ±50%（单独扰动后归一化到和=1）
combos = w0;                 % 第 1 行 = baseline
comboLabel = {'baseline'};
for i = 1:numel(w0)
    for s = [0.5, 1.5]       % -50% / +50%
        w = w0; w(i) = w0(i) * s; w = w / sum(w);
        combos(end+1, :) = w; %#ok<SAGROW>
        comboLabel{end+1} = sprintf('%s %+d%%', wNames{i}, round((s-1)*100)); %#ok<SAGROW>
    end
end
nCombo = size(combos, 1);

%% 3. 逐组合跑一遍（每个组合用同一随机种子，消除随机性对比较的干扰）
res = zeros(nCombo, 5);   % [长度, 安全, 能耗, 平滑度, 时间]
fprintf('正在运行 %d 组权重（pop=30, iter=120, safe_dist=2.5）...\n', nCombo);
for c = 1:nCombo
    wfull = combos(c, :);   % 5 目标权重
    rng(0);
    [~, ~, m5] = solve_this_v2('obstacle', false, wfull);
    % m5 = [L, E, unsmooth, tm, dmin]
    res(c, :) = [m5(1), m5(5), m5(2), 1 - m5(3), m5(4)];
end

%% 4. 输出对比表 + 稳定性统计
fprintf('\n================ AHP 权重敏感性结果 ================\n');
fprintf('%-16s %10s %10s %10s %10s %10s\n', '权重组合', '长度(m)', '安全(m)', '能耗', '平滑度', '时间(s)');
for c = 1:nCombo
    fprintf('%-16s %10.2f %10.2f %10.2f %10.4f %10.2f\n', comboLabel{c}, res(c,:));
end

% 稳定性：各指标在扰动下的相对变化范围（%）
base = res(1, :);
rng_rel = (max(res(2:end, :), [], 1) - min(res(2:end, :), [], 1)) ./ abs(base) * 100;
fprintf('\n各指标在 ±50%% 权重扰动下的相对变化范围（%%）：\n');
fprintf('  长度 %.1f%% | 安全 %.1f%% | 能耗 %.1f%% | 平滑度 %.1f%% | 时间 %.1f%%\n', rng_rel);

fprintf('\n关键结论：\n');
fprintf('  安全距离 baseline = %.2f m，扰动范围 [%.2f, %.2f] m\n', base(2), min(res(2:end,2)), max(res(2:end,2)));
fprintf('  路径长度 baseline = %.2f m，扰动范围 [%.2f, %.2f] m\n', base(1), min(res(2:end,1)), max(res(2:end,1)));
fprintf('  能耗     baseline = %.2f  ，扰动范围 [%.2f, %.2f]\n', base(3), min(res(2:end,3)), max(res(2:end,3)));
