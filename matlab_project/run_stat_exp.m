function run_stat_exp(s_start, s_end, only_alg)
% RUN_STAT_EXP  8 个算法 × 30 次独立运行的统一静态实验（统计显著性用）
% -------------------------------------------------------------------------
% 用途：重
% 
% 跑统计显著性实验。8 个算法在【同一张地图】上各独立运行 30 次，
%       统一计算 6 个指标，输出到 xlsx（RawData 原始 30 次 + Summary 均值±std）。
%
% 6 个指标口径（全部在本脚本内统一计算，不依赖各算法自带的评估函数，保证跨算法可比）：
%   ① 路径长度 L   = 折线总长
%   ② 最小安全距离 = 路径点到障碍(高斯山+球形禁飞区+圆柱)的最近距离
%   ③ 能耗         = L*0.1 + 高度变化*0.3 + 0.2*转向角 + 0.15*加速项
%   ④ 飞行时间     = L / V   (V = 10 m/s，与论文表 VI 一致)
%   ⑤ 规划时间     
% = tic/toc 计时
%   ⑥ 平滑度       = 相邻段夹角余弦均值（越接近 1 越平滑）
%
% 环境：与 map.m 完全一致（3 座高斯山 + 球形禁飞区 + 9 
% 根圆柱），确定性无噪声，
%       起点 [10,10,50] → 终点 [190,190,80]。
%       随机性只来自算法内部；循环内 rng(seed) 换种子（seed = 1..30），每次独立。
%
% 固定种子处理：本脚本不调用各算法 run_*.m 入口（那些入口里有 rng(2025) 固定种子），
%       而是直接调用核心求解函数，核心函数本身不重置 rng，故 30 次真正独立。
%
% 运行方式：cd 到 matlab_project 目录，直接运行 run_stat_exp。
% 输出：stat_results/stat_results.xlsx 与 stat_summary.mat
% -------------------------------------------------------------------------
    clc; close all;
    warning('off', 'MATLAB:rmpath:DirNotFound');
    

    N = 30;      % 总独立运行次数（= 随机种子数）；冒烟测试可改 1
    USE_PARFOR = true;   % true=单命令 parfor 并行(结果不变)；false=顺序(调试时看逐条输出)
    V = 10;      % 巡航速度 m/s
    % （已确认统一，与论文表 VI 一致）
    MAP_TYPE = 'city';   % 地图类型: 'obstacle'(基准/统计) / 'city'(城市峡谷) / 'hill'(丘陵缓坡)


    % 可选参数（用于多开 MATLAB 并行提速，结果逐次不变）：
    %   run_stat_exp(s_start, s_end, '算法名')
    %     s_start/s_end : 只跑这一段种子，例如 run_stat_exp(1,10)、run_stat_exp(11,20)
    %     only_alg      : 只跑某个算法，例如 'This algorithm'，留空跑全部
    if nargin < 1 || isempty(s_start), s_start = 1; end
    if nargin < 2 || isempty(s_end),   s_end   = N; end
    if nargin < 3, only_alg = ''; end
    seeds = s_start:s_end;
    nSeeds = numel(seeds);

    root   = fileparts(mfilename('fullpath'));
    outdir = fullfile(root, 'stat_results');
    if ~exist(outdir, 'dir'), mkdir(outdir); end

    model = build_env(MAP_TYPE);   % 统一环境（obstacle/city/hill，无噪声，与各算法一致）

    % ---- 算法注册表 {显示名, 所在文件夹, 求解器句柄} ----
    algs = {
        'ACOSRAR'          , fullfile(root, 'ACOSRAR_3维')                  , @solve_ACOSRAR3d
        'Chaos-SSA (ICSSA)', fullfile(root, 'Chaos-SSA')                    , @solve_ICSSA
        'HSMA'             , fullfile(root, 'HSMA')                         , @solve_HSMA
        'Improved RRT*'    , fullfile(root, 'Improved RRT')    , @solve_IRRT
        'NMOPOS'           , fullfile(root, 'NMOPOS')                       , @solve_NMOPOS 
        'Octree'           , fullfile(root, 'Octree')                       , @solve_Octree
        'RRT*'             , fullfile(root, 'RRT_star')                     , @solve_RRTstar
        'This algorithm'   , fullfile(root, 'MHTA-CAMO-AC'), @solve_This
    };
    nAlg = size(algs, 1);
    algpaths = cell(nAlg, 1);
    for k = 1:nAlg, algpaths{k} = genpath(algs{k, 2}); end   % 预计算各算法 genpath（parfor 清理用）

    res = struct('name', {}, 'metrics', {}, 'plantime', {});   % metrics: Nx5, plantime: Nx1

    for a = 1:nAlg
        name   = algs{a,1};
        folder = algs{a,2};
        solver = algs{a,3};

        if ~isempty(only_alg) && ~strcmp(name, only_alg)
            continue;                  % 只跑指定算法
        end

        p0 = path;                     % 保存当前搜索路径
        addpath(genpath(folder));      % 只加入当前算法目录，避免同名辅助函数冲突
        M = nan(nSeeds, 5);  T = nan(nSeeds, 1);
        At = nan(nSeeds, 1);  Ct = nan(nSeeds, 1);   % A* 耗时 / CAMO-AC 耗时（仅 This algorithm 有值）

        fprintf('\n===== [%d/%d] %s（种子 %d..%d）=====\n', a, nAlg, name, seeds(1), seeds(end));
        if USE_PARFOR && nSeeds > 1
            parfor si = 1:nSeeds
                addpath(algpaths{a}, '-begin');      % 只把本算法目录放到路径最前（-begin 保证优先）
                rng(seeds(si));                       % 每次独立随机种子
                tic;
                [p, astar_t, acosrar_t] = run_solver(name, model, seeds(si));
                T(si) = toc;
                At(si) = astar_t;  Ct(si) = acosrar_t;
                p = repair_buildings(p, model, 2.5);   % 统一修穿楼（city 专属，安全路径 no-op）
                if ~isempty(p) && size(p, 1) >= 2
                    M(si, :) = eval_metrics(p, model, V);
                end
            end
        else
            for si = 1:nSeeds
                s = seeds(si);
                rng(s);                    % 每次独立随机种子
                tic;
                [p, astar_t, acosrar_t] = run_solver(name, model, s);
                T(si) = toc;
                At(si) = astar_t;  Ct(si) = acosrar_t;
                p = repair_buildings(p, model, 2.5);   % 统一修穿楼（city 专属，安全路径 no-op）
                if isempty(p) || size(p, 1) < 2
                    fprintf('  seed %2d: 无有效路径 (耗时 %.1f s)\n', s, T(si));
                    continue;
                end
                M(si, :) = eval_metrics(p, model, V);
                fprintf('  seed %2d/%-2d: L=%6.2f m  safe=%5.2f m  E=%6.2f Wh  plan=%6.1f s\n', ...
                    s, s_end, M(si,1), M(si,2), M(si,3), T(si));
            end
        end
        path(p0);                      % 恢复搜索路径

        if all(isnan(M(:, 1)))
            fprintf('>>> %s 30 次均无有效路径，已跳过\n', name);
            continue;
        end
        res(end+1).name        = name;   %#ok<AGROW>
        res(end).metrics       = M;
        res(end).plantime      = T;
        res(end).astar_time    = At;      % 仅 This algorithm 有值，其余 NaN
        res(end).acosrar_time  = Ct;
        res(end).seeds         = seeds(:);  %#ok<AGROW> 记录实际种子号(分片跑时用于合并)
    end

    % 输出文件名带"地图/算法/种子范围"标签，多开并行、多地图互不覆盖
    tag = ['_', MAP_TYPE];
    if ~isempty(only_alg), tag = [tag, '_', strrep(only_alg, ' ', '')]; end
    if s_start ~= 1 || s_end ~= N, tag = [tag, sprintf('_s%d_%d', s_start, s_end)]; end

    print_summary(res);
    save(fullfile(outdir, ['stat_summary', tag, '.mat']), 'res', 'V');
    write_xlsx(res, fullfile(outdir, ['stat_results', tag, '.xlsx']));
    fprintf('\n>>> 结果已保存到 %s\n', outdir);
end

% ========================================================================
%% ------------------------- 统一环境（与 map.m 一致） --------------------
% ========================================================================
function model = build_env(map_type)
% 统一构建 obstacle/city/hill 环境（与各算法 build_environment 完全一致，无噪声）
    if nargin < 1 || isempty(map_type), map_type = 'obstacle'; end
    model.type = map_type;
    model.buildings = [];   % 非 city 时为空
    switch map_type
        case 'city'
            model.mountains = [];
            model.ball = [];
            model.cylinder.r = 0;   model.cylinder.pos = [];
            model.buildings = build_city_buildings();
            model.start = [10, 10, 60];
            model.goal  = [190, 190, 60];
            model.bound = [0, 200, 55, 65];
        case 'hill'
            model.mountains = [40,140,90,30; 140,40,90,30; 140,140,120,30; ...
                               90,90,120,30; 40,40,50,30; 180,90,100,30; 90,180,100,30];
            model.ball = [];
            model.cylinder.r = 0;   model.cylinder.pos = [];
            model.start = [10, 10, 50];
            model.goal  = [190, 190, 80];
            model.bound = [0, 200, 50, 200];
        otherwise   % 'obstacle' / 'legacy'
            model.mountains = [40, 140, 90, 30; 140, 40, 90, 30; 140, 140, 120, 30];
            model.ball      = [100, 100, 0, 50];
            model.cylinder.r   = 5;
            model.cylinder.pos = [40,70,100; 70,40,100; 50,50,100; 90,160,100; 160,80,100; ...
                                  80,180,100; 100,180,100; 180,60,100; 180,100,100];
            model.start = [10, 10, 50];
            model.goal  = [190, 190, 80];
            model.bound = [0, 200, 50, 200];
    end
end

function B = build_city_buildings()
% 城市高楼排布（rng(26) 固定，与各算法 build_city_buildings 完全一致），返回 Nx5 [x0 x1 y0 y1 h]
    nCol = 6; nRow = 6; margin = 14; gap = 6;
    rng('default'); rng(26);
    xc = linspace(margin, 200-margin, nCol+1);
    yc = linspace(margin, 200-margin, nRow+1);
    cellWx = xc(2)-xc(1); cellWy = yc(2)-yc(1);
    bldWx = cellWx-gap; bldWy = cellWy-gap;
    roadRow = 3; roadCol = 3;
    plaza = rand(nRow, nCol) < 0.15;
    B = [];
    for r = 1:nRow
        for c = 1:nCol
            if r == roadRow || c == roadCol || plaza(r,c), continue; end
            cx = (xc(c)+xc(c+1))/2; cy = (yc(r)+yc(r+1))/2;
            x0 = cx-bldWx/2; x1 = cx+bldWx/2;
            y0 = cy-bldWy/2; y1 = cy+bldWy/2;
            h = 30 + 70*rand;
            if rand < 0.12; h = 120 + 30*rand; end
            B(end+1,:) = [x0 x1 y0 y1 h]; %#ok<AGROW>
        end
    end
end

% ========================================================================
%% ------------------------- 算法求解器适配器 -----------------------------
% ========================================================================
function p = solve_ACOSRAR3d(model, seed) %#ok<INUSD>
% ACOSRAR 三维双向蚁群（run_acosrar.m 已改为可返回路径 + 关闭绘图）
% 传入 model.buildings 与 eval_metrics 同一套楼，避免 worker 里 build_environment 生成不一致
    p = run_acosrar(model.type, false, model.buildings);
end

function p = solve_ICSSA(model, seed)     %#ok<INUSD>
    pp.N = 40; pp.K = 8; pp.M = 150; pp.n_pts = 350; pp.safe_dist = 2.5;
    if strcmp(model.type, 'city')
        pp.init_wps = city_feasible_seed(pp.K, model.start, model.goal);
    end
    [p, ~, ~, ~] = icssa_optimize(model, pp);
end

function p = solve_HSMA(model, seed)      %#ok<INUSD>
    pp.N = 40; pp.tmax = 150; pp.K = 8; pp.n_pts = 350; pp.safe_dist = 2.5;
    if strcmp(model.type, 'city')
        pp.init_wps = city_feasible_seed(pp.K, model.start, model.goal);
    end
    [arc_paths, arc_f] = hsma_multiobjective(model, pp);
    free = arc_f(:, 6) < 1e4;              % 第 6 列为威胁代价
    if any(free)
        f_id = find(free);  Ff = arc_f(f_id, :);
        G  = Ff(:, [1 2 3]);               % [长度, 能耗, 不平滑度]
        Gn = (G - min(G)) ./ (max(G) - min(G) + 1e-9);
        [~, li] = min(sum(Gn, 2));
        bestIdx = f_id(li);
    else
        [~, bestIdx] = min(sum(arc_f, 2));
    end
    p = arc_paths{bestIdx};
end

function p = solve_IRRT(model, seed)      %#ok<INUSD>
    pp.delta = 8; pp.maxIter = 2000; pp.near_radius = 24; pp.goal_bias = 0.10;
    pp.safe_dist = 2.5; pp.step = 2;
    pp.gamma = 0.8; pp.d0 = 18; pp.eta1 = 1.0; pp.eta2 = 0.3; pp.dl = 1;
    [p, ~, ~, ~, ~] = irrtstar_plan(model, pp);
end

function p = solve_NMOPOS(model, seed)    %#ok<INUSD>
% 调用真实入口 run_nmopos（原 nmopos_plan_world 是旧版本，参数/目标数不一致，已弃用）
% 传入 model.buildings 与 eval_metrics 同一套楼，避免 worker 里 build_environment 生成不一致
    p = run_nmopos(model.type, false, model.buildings);
end

function p = solve_Octree(model, seed)    %#ok<INUSD>
    p = octree_plan_world(model.type, model.buildings);
end

function p = solve_RRTstar(model, seed)   %#ok<INUSD>
% 调用真实入口 run_rrt_star（其建筑碰撞用 AABB 盒外扩，比旧封装 dist_to_box 更保守、不穿楼）
% 传入 model.buildings 与 eval_metrics 同一套楼，避免 worker 里 build_world 生成不一致
    p = run_rrt_star(model.type, false, model.buildings);
end

function [p, astar_elapsed, acosrar_elapsed] = solve_This(model, seed)  %#ok<INUSD>
% 本文算法：调用 solve_this_v2（A*+ACOSRAR+后处理），并额外返回 A*/CAMO-AC 分项耗时
    [p, acosrar_elapsed, ~, astar_elapsed] = solve_this_v2(model.type, false, [], model.buildings);
end

function wps = city_feasible_seed(K, start, goal)
% 城市地图 K 个可行中间航点(有知初始化种子): 沿街道低空穿行，不翻越楼顶。
% 与 run_icssa.m / run_hsma.m 的同名函数完全一致——保证显著性实验与演示图同口径。
% 主干道(row3/col3 空档)中心 y=85.6667，L 形航线 start→(start.x,85.67)→(goal.x,85.67)→goal 全程无楼。
    AVENUE_Y = 85.6667;
    z = start(3);
    verts = [start;
             start(1), AVENUE_Y, z;
             goal(1),  AVENUE_Y, z;
             goal];
    seg = sqrt(sum(diff(verts, 1, 1).^2, 2));
    cum = [0; cumsum(seg)] / sum(seg);
    wps = zeros(K, 3);
    for i = 1:K
        u = i / (K + 1);
        j = find(u >= cum(1:end-1) & u <= cum(2:end), 1);
        if isempty(j), j = numel(seg); end
        f = (u - cum(j)) / (cum(j+1) - cum(j) + 1e-9);
        wps(i, :) = (1 - f) * verts(j, :) + f * verts(j+1, :);
    end
end

function p = repair_buildings(p, model, safe_dist)
% 统一几何修复：把贴楼/穿楼的点推出到净空 >= safe_dist（city 专属，其余地图 no-op）
if ~isfield(model, 'buildings') || isempty(model.buildings) || isempty(p) || size(p, 1) < 2
    return;
end
B = model.buildings;
if ~any_close_building(p, B, safe_dist), return; end   % 无贴楼点，不改动
d = [0; cumsum(sqrt(sum(diff(p, 1, 1).^2, 2)))];
if d(end) < 1e-6, return; end
[d, ia] = unique(d, 'stable'); p = p(ia, :);   % 去重，保证 interp1 断点严格递增
if numel(d) < 2, return; end
n = max(2, round(d(end) / 0.5) + 1);
tt = linspace(0, d(end), n)';
p = [interp1(d, p(:,1), tt, 'linear'), interp1(d, p(:,2), tt, 'linear'), interp1(d, p(:,3), tt, 'linear')];
for i = 1:size(p, 1)
    p(i, :) = project_to_safe(p(i, :), B, safe_dist);
end
end

function close = any_close_building(p, B, safe_dist)
close = false;
for i = 1:size(p, 1)
    if box_sd(p(i, :), B) < safe_dist
        close = true; return;
    end
end
end

function d = box_sd(p, B)
% 点到最近建筑的有符号距离（内部为负）
d = inf;
for b = 1:size(B, 1)
    bb = B(b, :); x0 = bb(1); x1 = bb(2); y0 = bb(3); y1 = bb(4); h = bb(5);
    x = p(1); y = p(2); z = p(3);
    if x > x0 && x < x1 && y > y0 && y < y1 && z > 0 && z < h
        d = min(d, -min([x - x0, x1 - x, y - y0, y1 - y, z, h - z]));
    else
        dx = max([x0 - x, x - x1, 0]); dy = max([y0 - y, y - y1, 0]);
        if z < 0, dz = -z; elseif z > h, dz = z - h; else, dz = 0; end
        d = min(d, sqrt(dx^2 + dy^2 + dz^2));
    end
end
end

function p = project_to_safe(p, B, safe_dist)
% 把点推到净空 >= safe_dist：内部→推到最近面+safe_dist；贴楼→沿主导轴推到 safe_dist
for iter = 1:8
    moved = false;
    for b = 1:size(B, 1)
        bb = B(b, :); x0 = bb(1); x1 = bb(2); y0 = bb(3); y1 = bb(4); h = bb(5);
        x = p(1); y = p(2); z = p(3);
        if x > x0 && x < x1 && y > y0 && y < y1 && z > 0 && z < h
            pen = [x - x0, x1 - x, y - y0, y1 - y, h - z]; [~, mi] = min(pen);
            switch mi
                case 1, p(1) = x0 - safe_dist;
                case 2, p(1) = x1 + safe_dist;
                case 3, p(2) = y0 - safe_dist;
                case 4, p(2) = y1 + safe_dist;
                case 5, p(3) = h + safe_dist;
            end
            moved = true; continue;
        end
        dx = max([x0 - x, x - x1, 0]); dy = max([y0 - y, y - y1, 0]);
        if z < 0, dz = -z; elseif z > h, dz = z - h; else, dz = 0; end
        d = sqrt(dx^2 + dy^2 + dz^2);
        if d >= safe_dist || d < 1e-9, continue; end
        if dx >= dy && dx >= dz
            if x <= x0, p(1) = x0 - safe_dist; else, p(1) = x1 + safe_dist; end
        elseif dy >= dx && dy >= dz
            if y <= y0, p(2) = y0 - safe_dist; else, p(2) = y1 + safe_dist; end
        else
            if z <= 0, p(3) = safe_dist; else, p(3) = h + safe_dist; end
        end
        moved = true;
    end
    if ~moved, break; end
end
end

function [Z1, Z2, Z3, ball, cylinder] = build_proposed_env()
    map_x = 0:0.1:200;  map_y = 0:0.1:200;
    [Xg, Yg] = meshgrid(map_x, map_y);
    Z1 = 90*exp(-((Xg-40).^2+(Yg-140).^2)/(2*30^2));   Z1(Z1<0)=0;
    Z2 = 90*exp(-((Xg-140).^2+(Yg-40).^2)/(2*30^2));   Z2(Z2<0)=0;
    Z3 = 120*exp(-((Xg-140).^2+(Yg-140).^2)/(2*30^2)); Z3(Z3<0)=0;
    ball.cx = 100; ball.cy = 100; ball.cz = 0; ball.r = 50; ball.n_ball = 50;
    cylinder.r = 5; cylinder.n_cylinder = 50;
    cylinder.positions = [40,70,100; 70,40,100; 50,50,100; 90,160,100; 160,80,100; ...
                          80,180,100; 100,180,100; 180,60,100; 180,100,100];
end

% ========================================================================
%% ------------------------- 统一指标计算 ---------------------------------
% ========================================================================
% 统一指标计算已抽到独立文件 eval_metrics.m（与各基准/多环境实验共用同一口径）。
%   调用：m = eval_metrics(path, model, V)
%   返回：m = [路径长度, 最小安全距离, 能耗, 平滑度, 飞行时间]
%   model 结构与下方 build_env() 一致（mountains / ball / cylinder）。

% ========================================================================
%% ------------------------- 输出 ----------------------------------------
% ========================================================================
function print_summary(res)
    names = {res.name};
    fprintf('\n================ 30 次 mean ± std ================\n');
    fprintf('%-18s %8s %8s %9s %7s %9s %6s\n', 'Algorithm', 'Path', 'Safe', 'Energy', 'Flight', 'PlanTime', 'Smooth');
    for a = 1:numel(res)
        M = res(a).metrics; T = res(a).plantime;
        fprintf('%-18s %8.2f %8.2f %9.2f %7.2f %9.2f %6.2f\n', ...
            names{a}, nmean(M(:,1)), nmean(M(:,2)), nmean(M(:,3)), ...
            nmean(M(:,5)), nmean(T), nmean(M(:,4)));
    end
end

function write_xlsx(res, outfile)
    names    = {res.name};
    metrics  = {res.metrics};
    plantime = {res.plantime};
    astar_t  = {res.astar_time};
    acosrar_t = {res.acosrar_time};
    seedlist = {res.seeds};
    nAlg = numel(names);

    % Sheet1 RawData：每个算法 nSeeds 行，列序与论文一致
    header = {'Algorithm','Run (seed)','Path length (m)','Min safe dist (m)', ...
              'Energy (Wh)','Flight time (s)','Planning time (s)','A* time (s)','CAMO-AC time (s)','Smoothness'};
    rawC = header;
    for a = 1:nAlg
        M = metrics{a}; T = plantime{a}; At = astar_t{a}; Ct = acosrar_t{a}; sd = seedlist{a}(:);
        for s = 1:size(M, 1)
            rawC(end+1, :) = {names{a}, sd(s), M(s,1), M(s,2), M(s,3), M(s,5), T(s), At(s), Ct(s), M(s,4)}; %#ok<AGROW>
        end
    end

    % Sheet2 Summary：各指标 mean ± std
    dimStr = {'Path length (m)','Min safe dist (m)','Energy (Wh)', ...
              'Flight time (s)','Planning time (s)','Smoothness'};
    sumC = [{'Metric'}, names];
    for m = 1:6
        row = {dimStr{m}};
        for a = 1:nAlg
            v = pick(metrics{a}, plantime{a}, m);
            v = v(~isnan(v));
            if isempty(v)
                row{end+1} = 'NaN'; %#ok<AGROW>
            else
                row{end+1} = sprintf('%.2f +/- %.2f', mean(v), std(v)); %#ok<AGROW>
            end
        end
        sumC(end+1, :) = row; %#ok<AGROW>
    end

    writecell(rawC, outfile, 'Sheet', 'RawData');
    writecell(sumC, outfile, 'Sheet', 'Summary');
    fprintf('已导出 xlsx: %s\n', outfile);
end

function v = pick(metrics, plantime, m)
% 6 指标顺序 [长度, 安全距, 能耗, 飞行时间, 规划时间, 平滑度]
% metrics 内部顺序 [长度, 安全距, 能耗, 平滑度, 飞行时间]
    switch m
        case 1, v = metrics(:, 1);
        case 2, v = metrics(:, 2);
        case 3, v = metrics(:, 3);
        case 4, v = metrics(:, 5);
        case 5, v = plantime(:);
        case 6, v = metrics(:, 4);
    end
end

function m = nmean(x)
    x = x(~isnan(x));
    if isempty(x), m = NaN; else, m = mean(x); end
end

function [p, astar_t, acosrar_t] = run_solver(name, model, s)
% parfor 内按算法名分派求解器（避免在并行循环里用函数句柄触发透明度问题）
% astar_t/acosrar_t 仅 This algorithm 有值（MHTA 全局层 / CAMO-AC 局部层耗时），其余算法为 NaN
    astar_t = NaN; acosrar_t = NaN;
    switch name
        case 'ACOSRAR',            p = solve_ACOSRAR3d(model, s);
        case 'Chaos-SSA (ICSSA)',  p = solve_ICSSA(model, s);
        case 'HSMA',               p = solve_HSMA(model, s);
        case 'Improved RRT*',      p = solve_IRRT(model, s);
        case 'NMOPOS',             p = solve_NMOPOS(model, s);
        case 'Octree',             p = solve_Octree(model, s);
        case 'RRT*',               p = solve_RRTstar(model, s);
        case 'This algorithm',     [p, astar_t, acosrar_t] = solve_This(model, s);
        otherwise,                 p = [];
    end
end
