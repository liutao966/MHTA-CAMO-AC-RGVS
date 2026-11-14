function run_ablation(s_start, s_end, only_variant)
% RUN_ABLATION  消融实验：完整算法 + 5 个消融变体，各独立运行 N 次，输出 6 指标 xlsx
%   变体:
%     full        = 完整算法（A* + ACOSRAR(Lévy/自适应修复) + 后处理）
%     no_mhta    = 去掉 A* 全局层（锚点直接用起终点直线）
%     no_levy     = 去掉 L
% évy 飞行（换成高斯随机方向）
%     no_repair   = 去掉自适应碰撞修复
%     no_postproc = 去掉后处理（prune/repair/resample）
%     no_safety   = 去掉"安全距离最大化"目标（d_min 权重置零）
% 用法:
%     run_ablation()                     % 全部 30 次、全部变体
%     run_ablation(1, 5)                 % 冒烟测试 5 次
%     run_ablation(1, 30, 'no_safety')   % 只跑单个变体
% 输出: ablation_results/ablation_results.x
% lsx（全量）或 ablation_results_<variant>.xlsx（单变体）
    clc; close all;

    N = 30;                            % 总独立运行次数
    USE_PARFOR = true;                 % true=parfor 并行(结果不变)；false=顺序(看逐条输出)
    if nargin < 1 || isempty(s_start), s_start = 1; end
    if nargin < 2 || isempty(s_end),   s_end   = N; end
    if nargin < 3, only_variant = ''; end
    seeds = s_start:s_end;
    nSeeds = numel(seeds);

    variants      = {'full', 'no_mhta', 'no_levy', 'no_repair', 'no_postproc', 'no_safety'};
    variant_names = {'Full', 'w/o MHTA', 'w/o Levy', 'w/o Repair', 'w/o Postproc', 'w/o Safety'};
    nV = numel(variants);

    root   = fileparts(mfilename('fullpath'));
    outdir = fullfile(root, 'ablation_results');
    if ~exist(outdir, 'dir'), mkdir(outdir); end

    M = nan(nSeeds, 6, nV);   % 种子 × 指标 × 变体；指标顺序 [路径,安全,能耗,平滑,飞行,规划]

    for v = 1:nV
        if ~isempty(only_variant) && ~strcmp(variants{v}, only_variant)
            continue;                      % 只跑指定变体
        end
        fprintf('\n===== [%d/%d] %s (%s) =====\n', v, nV, variants{v}, variant_names{v});
        if USE_PARFOR && nSeeds > 1
            parfor si = 1:nSeeds
                addpath(root);                    % worker 补路径，确保 ablation_solver 可见
                rng(seeds(si));                   % 每次独立随机种子
                [~, met] = ablation_solver(variants{v}, false);
                M(si, :, v) = met;
            end
        else
            for si = 1:nSeeds
                s = seeds(si);
                rng(s);
                [~, met] = ablation_solver(variants{v}, false);
                M(si, :, v) = met;
                fprintf('  seed %2d: L=%6.2f  safe=%5.2f  E=%6.2f  smooth=%.3f  flight=%5.2f  plan=%6.1f\n', ...
                    s, met(1), met(2), met(3), met(4), met(5), met(6));
            end
        end
    end

    tag = '';
    if ~isempty(only_variant), tag = ['_', only_variant]; end
    print_ablation_summary(M, variants, variant_names);
    write_ablation_xlsx(M, variants, variant_names, seeds, outdir, tag);
    fprintf('\n>>> 结果已保存到 %s\n', outdir);
end

function print_ablation_summary(M, variants, variant_names)
    dims = {'Path (m)','Safe (m)','Energy (Wh)','Smooth','Flight (s)','Plan (s)'};
    fprintf('\n================ 消融结果 mean ± std ================\n');
    fprintf('%-16s', 'Variant');
    for v = 1:numel(variants), fprintf('%16s', variant_names{v}); end
    fprintf('\n');
    for d = 1:6
        fprintf('%-16s', dims{d});
        for v = 1:numel(variants)
            x = M(:, d, v); x = x(~isnan(x));
            fprintf('%9.2f±%5.2f', mean(x), std(x));
        end
        fprintf('\n');
    end
end

function write_ablation_xlsx(M, variants, variant_names, seeds, outdir, tag)
    if nargin < 6 || isempty(tag), tag = ''; end
    % 过滤掉全 NaN 的变体（只跑单个变体时其余变体为 NaN）
    keep = false(1, numel(variants));
    for v = 1:numel(variants)
        keep(v) = ~all(isnan(M(:, 1, v)));
    end
    variants = variants(keep);
    variant_names = variant_names(keep);
    M = M(:, :, keep);

    dimStr = {'Path length (m)','Min safe dist (m)','Energy (Wh)', ...
              'Smoothness','Flight time (s)','Planning time (s)'};

    % Sheet1 RawData
    header = {'Variant','Run (seed)','Path length (m)','Min safe dist (m)', ...
              'Energy (Wh)','Smoothness','Flight time (s)','Planning time (s)'};
    rawC = header;
    for v = 1:numel(variants)
        for si = 1:size(M, 1)
            rawC(end+1, :) = {variant_names{v}, seeds(si), M(si,1,v), M(si,2,v), ...
                              M(si,3,v), M(si,4,v), M(si,5,v), M(si,6,v)}; %#ok<AGROW>
        end
    end

    % Sheet2 Summary：mean ± std
    sumC = [{'Metric'}, variant_names];
    for d = 1:6
        row = {dimStr{d}};
        for v = 1:numel(variants)
            x = M(:, d, v); x = x(~isnan(x));
            row{end+1} = sprintf('%.2f +/- %.2f', mean(x), std(x)); %#ok<AGROW>
        end
        sumC(end+1, :) = row; %#ok<AGROW>
    end

    outfile = fullfile(outdir, ['ablation_results', tag, '.xlsx']);
    writecell(rawC, outfile, 'Sheet', 'RawData');
    writecell(sumC, outfile, 'Sheet', 'Summary');
    fprintf('已导出 xlsx: %s\n', outfile);
end
