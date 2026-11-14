function plot_8_algorithms()
% PLOT_8_ALGORITHMS  8 个算法 × 3 张地图的路径对比图（合并为 2 张图）
% -------------------------------------------------------------------------
% 用途：把 8 个路径规划算法画到 3 张三维地图上，每张地图一种地形，
%       每个算法一条不同颜色的路径。与论文保持一致的关键：
%       直接调用每个算法自己的入口函数(即其 main.m 真正调用的 run_*.m)，
%       因此保留各算法自己的固定随机种子(rng(2025) 等)和参数，
%       不再用 run_stat_exp.m 里那套换种子的适配器。
%
%   3 张地图：city(城市高楼) / hill(丘陵 7 座高斯山) / obstacle(3山+禁飞球+9圆柱)
%   8 个算法：ACOSRAR、Chaos-SSA(ICSSA)、HSMA、Improved RRT*、NMOPOS、
%             Octree、RRT*、本文算法(This algorithm)。
%
% 说明：
%   - 对自带 rng(2025) 的入口(ICSSA/HSMA/Improved RRT*)，路径与单独跑其 main.m 完全一致；
%   - 对不自带种子的
% 入口(ACOSRAR/NMOPOS/RRT*/本文算法)，每个算法前先 rng('default')，
%     并按 main.m 的顺序 city→hill→obstacle 依次调用，模拟"刚打开 MATLAB 就跑该 main.m"。
%
% 运行方式：cd 到 matlab_project 目录，直接运行 plot_8_algorithms。
% 输出：2 个 figure 窗口 + 2 张 PNG(保存到 matlab_project 目录下)。
% -------------------------------------------------------------------------
    clc; close all;
    warning('off', 'MATLAB:rmpath:DirNotFound');

    root = fileparts(mfilename('fullpath'));

    % 算法注册表 {显示名, 文件夹, 入口句柄(返回 Nx3 路径), 颜色}
    entries = {
        'ACOSRAR'          , 'ACOSRAR_3维'                    , @(m) run_acosrar(m, false)  , [0.00 0.45 0.74]
        'Chaos-SSA (ICSSA)', 'Chaos-SSA'                      , @icssa_path                  , [0.85 0.33 0.10]
        'HSMA'             , 'HSMA'                           , @(m) run_hsma(m, false)      , [0.47 0.67 0.19]
        'Improved RRT*'    , 'Improved RRT'                   , @(m) run_irrtstar(m, false)  , [0.49 0.18 0.56]
        'NMOPOS'           , 'NMOPOS'                         , @(m) run_nmopos(m, false)    , [0.30 0.75 0.93]
        'Octree'           , 'Octree'                         , @(m) run_octree(m, false)    , [0.64 0.08 0.18]
        'RRT*'             , 'RRT_star'                       , @(m) rrtstar_with_seed(m, 1) , [0.93 0.69 0.13]
        'This algorithm'   , 'MHTA-CAMO-AC', @(m) solve_this_v2(m, false) , [1.00 0.00 0.00]
    };
    nAlg = size(entries, 1);
    maps = {'city', 'hill', 'obstacle'};

    %% ------------------------- 阶段 1：跑真实入口、收集路径 -------------------------
    paths = cell(nAlg, numel(maps))
    ;
    for a = 1:nAlg
        name   = entries{a, 1};
        folder = fullfile(root, entries{a, 2});
        entry  = entries{a, 3};

        rng('default');                 % 模拟 fresh MATLAB（自带 rng(2025) 的入口会自行覆盖）
        p0 = path;                      % 保存当前搜索路径
        addpath(genpath(folder));       % 只加入当前算法目录，避免同名辅助函数冲突

        for mi = 1:numel(maps)
            mp = maps{mi};
            try
                p = entry(mp);
            catch ME
                p = [];
                fprintf('[%s] %-18s 运行出错: %s\n', map_name_cn(mp), name, ME.message);
            end
            if isempty(p) || size(p, 1) < 2
                fprintf('[%s] %-18s 无有效路径\n', map_name_cn(mp), name);
            else
                fprintf('[%s] %-18s 路径点 %3d  长度 %.1f m\n', ...
                    map_name_cn(mp), name, size(p, 1), path_length(p));
            end
            paths{a, mi} = p;
        end
        path(p0);                       % 恢复搜索路径
    end

    %% ------------------------- 阶段 2：合并为 2 张图（子图并排，编号 (a)(b)(c)(d)） -------------------------
    % 8 个算法分成 2 组（每组 4 个）；本文算法 (This algorithm, 第 8 个) 在组 2，颜色为红色。
    groups = {[1 2 3 4], [5 6 7 8]};

    % ---- 图 1：障碍物地图，2 组并排，编号 (a)(b) ----
    figure('Name', '路径对比 - 障碍物地图', 'Position', [80 80 1400 700], 'Color', 'white');
    tiledlayout(1, 2, 'Padding', 'loose', 'TileSpacing', 'loose');
    letters1 = {'a', 'b'};
    for gi = 1:2
        nexttile;
        draw_tile(build_env('obstacle'), 3, paths, entries, groups{gi}, letters1{gi});
    end
    png1 = fullfile(root, 'PathComparison_obstacle_combined.png');
    print(gcf, png1, '-dpng', '-r200');
    fprintf('已保存: %s\n\n', png1);

    % ---- 图 2：城市 + 丘陵地图，1×4 单行布局（city 组1/组2 + hill 组1/组2），编号 (a)(b)(c)(d) ----
    figure('Name', '路径对比 - 城市与丘陵地图', 'Position', [20 80 2800 700], 'Color', 'white');
    tiledlayout(1, 4, 'Padding', 'loose', 'TileSpacing', 'loose');
    tiles2 = {{'city', 1, 'a'}, {'city', 2, 'b'}, {'hill', 1, 'c'}, {'hill', 2, 'd'}};
    for i = 1:numel(tiles2)
        nexttile;
        mi = find(strcmp(maps, tiles2{i}{1}));
        draw_tile(build_env(tiles2{i}{1}), mi, paths, entries, groups{tiles2{i}{2}}, tiles2{i}{3});
    end
    png2 = fullfile(root, 'PathComparison_city_hill_combined.png');
    print(gcf, png2, '-dpng', '-r200');
    fprintf('已保存: %s\n\n', png2);
end

%% ====================== 入口适配 ======================
function p = icssa_path(map_type)
% ICSSA 入口返回 result 结构体，这里取出其中的最优路径
    r = run_icssa(map_type, false);
    p = r.best_path;
end

function L = path_length(p)
    L = sum(sqrt(sum(diff(p, 1, 1).^2, 2)));
end

%% ====================== 统一环境(与 map.m 一致，无噪声) ======================
function model = build_env(map_type)
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
        case 'hill'
            model.mountains = [40,140,90,30; 140,40,90,30; 140,140,120,30; ...
                               90,90,120,30; 40,40,50,30; 180,90,100,30; 90,180,100,30];
            model.ball = [];
            model.cylinder.r = 0;   model.cylinder.pos = [];
            model.start = [10, 10, 50];
            model.goal  = [190, 190, 80];
        otherwise   % 'obstacle'
            model.mountains = [40, 140, 90, 30; 140, 40, 90, 30; 140, 140, 120, 30];
            model.ball      = [100, 100, 0, 50];
            model.cylinder.r   = 5;
            model.cylinder.pos = [40,70,100; 70,40,100; 50,50,100; 90,160,100; 160,80,100; ...
                                  80,180,100; 100,180,100; 180,60,100; 180,100,100];
            model.start = [10, 10, 50];
            model.goal  = [190, 190, 80];
    end
end

function B = build_city_buildings()
% 城市高楼排布(rng(26) 固定，与各算法 build_city_buildings 完全一致)，返回 Nx5 [x0 x1 y0 y1 h]
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

%% ====================== 地形绘制 ======================
function draw_terrain(model)
    tick_font  = 20;
    label_font = 20;

    hold on; grid on; axis equal; view(3);

    if strcmp(model.type, 'city')
        patch([0 200 200 0], [0 0 200 200], [1e-3 1e-3 1e-3 1e-3], ...
              [0.82 0.82 0.78], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
        for i = 1:size(model.buildings, 1)
            draw_building_box(model.buildings(i, :));
        end
    else
        [X, Y] = meshgrid(0:0.5:200, 0:0.5:200);
        for k = 1:size(model.mountains, 1)
            zk = model.mountains(k,3) * exp(-((X - model.mountains(k,1)).^2 + ...
                 (Y - model.mountains(k,2)).^2) / (2 * model.mountains(k,4)^2));
            zk(zk < 0) = 0;
            surf(X, Y, zk, 'FaceColor', 'interp', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
        end
        if numel(model.ball) == 4 && model.ball(4) > 0
            [Xb, Yb, Zb] = sphere(50);
            bx = model.ball(1) + Xb * model.ball(4);
            by = model.ball(2) + Yb * model.ball(4);
            bz = model.ball(3) + Zb * model.ball(4);
            bz(bz < 0) = 0;
            surf(bx, by, bz, 'FaceColor', [1 0 1], 'EdgeColor', 'none', 'FaceAlpha', 0.8);
        end
        if isfield(model, 'cylinder') && ~isempty(model.cylinder.pos) && model.cylinder.r > 0
            [Xc, Yc, Zc] = cylinder(model.cylinder.r, 50);
            theta = linspace(0, 2*pi, 50);
            for i = 1:size(model.cylinder.pos, 1)
                p = model.cylinder.pos(i, :);
                surf(p(1)+Xc, p(2)+Yc, p(3)*Zc, 'FaceColor', [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
                fill3(p(1)+model.cylinder.r*cos(theta), p(2)+model.cylinder.r*sin(theta), ...
                      repmat(p(3), size(theta)), [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
                fill3(p(1)+model.cylinder.r*cos(theta), p(2)+model.cylinder.r*sin(theta), ...
                      zeros(size(theta)), [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
            end
        end
    end

    scatter3(model.start(1), model.start(2), model.start(3), 200, 'g', 'filled', 'HandleVisibility', 'off');
    scatter3(model.goal(1), model.goal(2), model.goal(3), 200, 'b', 'filled', 'HandleVisibility', 'off');

    xlim([0 200]); ylim([0 200]); zlim([0 200]); axis manual;
    xlabel('X/m', 'FontSize', label_font);
    ylabel('Y/m', 'FontSize', label_font);
    zlabel('Z/m', 'FontSize', label_font);
    set(gca, 'FontSize', tick_font, 'Color', 'white');
    set(gca, 'XTick', 0:20:200, 'YTick', 0:20:200, 'ZTick', 0:20:200);
    daspect([1 1 1]);
end

function draw_tile(model, mi, paths, entries, alg_idx, letter)
% 在单个 tile 内绘制地形 + 4 条路径 + 图例，并在子图正下方标注编号
    draw_terrain(model);

    hLines = gobjects(numel(alg_idx), 1);
    ok     = false(numel(alg_idx), 1);
    for k = 1:numel(alg_idx)
        a = alg_idx(k);
        p = paths{a, mi};
        if isempty(p) || size(p, 1) < 2
            continue;
        end
        hLines(k) = plot3(p(:,1), p(:,2), p(:,3), '-', ...
            'Color', entries{a, 4}, 'LineWidth', 2.5, 'DisplayName', entries{a, 1});
        ok(k) = true;
    end

    legend(hLines(ok), entries(alg_idx(ok), 1), 'Location', 'best', 'FontSize', 12);

    % 编号紧贴子图正下方（用 annotation 定位到子图底边，而非 X/m 下方）
    add_tile_label(gca, letter);
end

%% ====================== 子图编号标注 ======================
function add_tile_label(ax, letter)
% 在子图正下方居中紧贴标注 (a)/(b)/(c)/(d)
    pos = get(ax, 'Position');        % 绘图框 figure 归一化 [left bottom width height]（3D 轴也可靠）
    xc = pos(1) + pos(3)/2;
    w = 0.07; h = 0.05;
    x0 = max(0, min(1 - w, xc - w/2));
    y0 = max(0, min(1 - h, pos(2) - 0.09));   % 绘图框底边下方，并夹在 [0,1] 内
    annotation(gcf, 'textbox', [x0, y0, w, h], ...
        'String', sprintf('(%s)', letter), ...
        'EdgeColor', 'none', 'LineStyle', 'none', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontSize', 20, 'FontWeight', 'bold');
end

function draw_building_box(b)
    x0 = b(1); x1 = b(2); y0 = b(3); y1 = b(4); h = b(5);
    t = min(max(h/150, 0), 1);
    col = [0.4 - 0.15*t, 0.48 - 0.10*t, 0.72 + 0.10*t];
    verts_x = [x0, x1, x1, x0, x0, x1, x1, x0];
    verts_y = [y0, y0, y1, y1, y0, y0, y1, y1];
    verts_z = [0, 0, 0, 0, h, h, h, h];
    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices', [verts_x(:), verts_y(:), verts_z(:)], 'Faces', faces, ...
          'FaceColor', col, 'EdgeColor', 'k', 'LineWidth', 0.5, 'FaceAlpha', 1.0);
    patch([x0 x1 x1 x0], [y0 y0 y1 y1], [h h h h], col .* 1.2, ...
          'EdgeColor', 'none', 'FaceAlpha', 1.0);
end

function str = map_name_cn(map_type)
    switch map_type
        case 'city',     str = '城市地图';
        case 'hill',     str = '丘陵地图';
        case 'obstacle', str = '障碍物地图';
        otherwise,       str = '山体障碍地图';
    end
end

function p = rrtstar_with_seed(m, s)
% RRT* 在城市地图对随机种子敏感，固定种子 s（对齐 run_stat_exp 的 seed）保证能找到路径
    rng(s);
    p = run_rrt_star(m, false);
end
