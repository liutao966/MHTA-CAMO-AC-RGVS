function plot_6_algorithms_dynamic()
% PLOT_6_ALGORITHMS_DYNAMIC  6 个动态避障算法在统一地图上的对比图（2×3）
% -------------------------------------------------------------------------
% 布局（2 行 × 3 列）：
%   第 1 行：This algorithm / AVSS-RRT / A*+APF
%   第 2 行：VO / RVO / DWA
% 每个子图绘制：
%   - 静态环境（三座高斯山 + 品红禁飞球 + 黄色圆柱）
%   - 起点（绿球）、终点（蓝球）
%   - 全局规划路径（棕色实线）、实际飞行轨迹（红色实线）
%   - 14 个四类动态障碍物（按类型着色：紫=直线、蓝=圆弧、绿=变速、橙=垂直），
%     球体为初始位置，虚线/点线为其运动轨迹示意。
% 图例只画在第 1 个子图（This algorithm）。
% 6 个算法面对同一组动态障碍（沿本文算法全局路径生成，seed=1），保证公平对比。
% -------------------------------------------------------------------------
    clc; close all;
    root = fileparts(mfilename('fullpath'));
    addpath(root);
    addpath(fullfile(root, 'obstacle_gradient_exp'));
    addpath(fullfile(root, 'VO'));
    addpath(fullfile(root, 'RVO'));
    addpath(fullfile(root, 'DWA'));

    env   = build_env();
    start = env.start; goal = env.goal;

    fprintf('生成静态全局路径 ...\n');
    path_ours = plan_this(env);
    path_aapf = plan_aapf(env);
    path_avss = plan_avss(env);

    % 布局顺序：第1行 本文/AVSS-RRT/A*+APF，第2行 VO/RVO/DWA
    algos   = {'This algorithm', 'AVSS-RRT', 'A*+APF', 'VO', 'RVO', 'DWA'};
    paths   = {path_ours, path_avss, path_aapf, path_ours, path_ours, path_ours};
    simFcns = {@sim_rgvs, @sim_avss, @sim_aapf, @sim_vo, @sim_rvo, @sim_dwa};

    seed = 1;
    dyn  = gen_obstacles_v2(seed, path_ours, env);   % 6 算法共用同一组障碍

    fprintf('运行 6 个算法动态仿真 (seed=%d) ...\n', seed);
    trails = cell(6, 1);
    stats  = zeros(6, 1);
    for a = 1:6
        [ok, trail, st] = simFcns{a}(paths{a}, dyn, start, goal, env);
        trails{a} = trail;
        stats(a)  = st;
        fprintf('  %-16s 成功=%d 状态=%d 轨迹点=%d\n', algos{a}, ok, st, size(trail, 1));
    end

    % 颜色：飞行轨迹=红，规划路径=棕，障碍四类=紫/蓝/绿/橙
    col_plan  = [0.55 0.27 0.07];   % 棕色：全局规划路径
    col_trail = [0.85 0.00 0.00];   % 正红：实际飞行轨迹
    type_color = [0.60 0.20 0.80;   % 1 直线：紫（避开红色飞行轨迹）
                  0.10 0.45 0.90;   % 2 圆弧：蓝
                  0.10 0.60 0.20;   % 3 变速：绿
                  0.95 0.55 0.10];  % 4 垂直：橙

    figure('Position', [40 40 1920 1440], 'Color', 'white');
    tiledlayout(2, 3, 'Padding', 'loose', 'TileSpacing', 'loose');

    letters = {'a','b','c','d','e','f'};
    for a = 1:6
        ax = nexttile;
        draw_world(env);

        draw_ball(start, 4, [0 0.8 0]);   % 起点（绿球）
        draw_ball(goal,  4, [0 0 1]);     % 终点（蓝球）

        % 全局规划路径（棕色）与实际飞行轨迹（红色）
        plot3(paths{a}(:,1), paths{a}(:,2), paths{a}(:,3), '-', ...
            'Color', col_plan, 'LineWidth', 2.0);
        tr = trails{a};
        plot3(tr(:,1), tr(:,2), tr(:,3), '-', 'Color', col_trail, 'LineWidth', 2.6);

        draw_dynamic_obstacles(dyn, type_color);

        if a == 1
            lh = [
                plot3(nan, nan, nan, '-', 'Color', col_trail, 'LineWidth', 2.6);
                plot3(nan, nan, nan, '-', 'Color', col_plan,  'LineWidth', 2.0);
                plot3(nan, nan, nan, 'o', 'MarkerFaceColor', type_color(1,:), 'MarkerEdgeColor', 'none', 'MarkerSize', 9);
                plot3(nan, nan, nan, 'o', 'MarkerFaceColor', type_color(2,:), 'MarkerEdgeColor', 'none', 'MarkerSize', 9);
                plot3(nan, nan, nan, 'o', 'MarkerFaceColor', type_color(3,:), 'MarkerEdgeColor', 'none', 'MarkerSize', 9);
                plot3(nan, nan, nan, 'o', 'MarkerFaceColor', type_color(4,:), 'MarkerEdgeColor', 'none', 'MarkerSize', 9);
                plot3(nan, nan, nan, 'o', 'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'none', 'MarkerSize', 9);
                plot3(nan, nan, nan, 'o', 'MarkerFaceColor', [0 0 1], 'MarkerEdgeColor', 'none', 'MarkerSize', 9)];
            legend(lh, {'Flight trajectory','Planned path','Linear obstacle','Arc obstacle','Variable-speed obstacle','Vertical obstacle','Start point','Goal point'}, ...
                'Location', 'northwest', 'FontSize', 12);
        end

        add_tile_label(ax, letters{a});
    end

    png = fullfile(root, 'dynamic_6_algorithms.png');
    print(gcf, png, '-dpng', '-r200');
    fprintf('已保存: %s\n', png);
end

%% ====================== 静态环境绘制 ======================
function draw_world(env)
    hold on; grid on; box on;

    [X, Y] = meshgrid(0:1:200, 0:1:200);
    for k = 1:size(env.mountains, 1)
        m = env.mountains(k, :);
        Z = m(3) * exp(-((X - m(1)).^2 + (Y - m(2)).^2) / (2*m(4)^2));
        Z(Z < 0) = 0;
        surf(X, Y, Z, 'FaceColor', 'interp', 'EdgeColor', 'none', 'FaceAlpha', 0.85);
        hold on;
    end

    % 球形禁飞区（品红）
    s = env.ball;
    [Xs, Ys, Zs] = sphere(30);
    bx = s(1) + Xs*s(4); by = s(2) + Ys*s(4); bz = s(3) + Zs*s(4);
    bz(bz < 0) = 0;
    surf(bx, by, bz, 'FaceColor', [1 0 1], 'EdgeColor', 'none', 'FaceAlpha', 0.9);

    % 圆柱障碍（黄色，带封顶）
    [Xc, Yc, Zc] = cylinder(env.cyl_r, 30);
    theta = linspace(0, 2*pi, 30);
    for i = 1:size(env.cylinders, 1)
        c = env.cylinders(i, :);
        surf(c(1)+Xc, c(2)+Yc, c(3)*Zc, 'FaceColor', [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
        tx = c(1) + env.cyl_r*cos(theta);
        ty = c(2) + env.cyl_r*sin(theta);
        tz = repmat(c(3), size(tx));
        fill3(tx, ty, tz, [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    end

    tick_font  = 20;
    label_font = 20;

    xlabel('X/m', 'FontSize', label_font);
    ylabel('Y/m', 'FontSize', label_font);
    zlabel('Z/m', 'FontSize', label_font);
    xlim([0 200]); ylim([0 200]); zlim([0 200]);
    set(gca, 'FontSize', tick_font, 'Color', 'white');
    set(gca, 'XTick', 0:20:200, 'YTick', 0:20:200, 'ZTick', 0:20:200);
    daspect([1 1 1]);
    view(45, 28);
end

%% ====================== 球体（起点/终点） ======================
function draw_ball(center, r, color)
    [Xs, Ys, Zs] = sphere(20);
    surf(center(1)+Xs*r, center(2)+Ys*r, center(3)+Zs*r, ...
        'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
end

%% ====================== 动态障碍物（按类型着色 + 运动轨迹） ======================
function draw_dynamic_obstacles(dyn, type_color)
    [Xd, Yd, Zd] = sphere(20);
    Xd = Xd * dyn.r; Yd = Yd * dyn.r; Zd = Zd * dyn.r;
    for i = 1:dyn.num
        o = dyn.obs{i};
        c = type_color(o.type, :);
        p = dyn.curr(i, :);

        % 障碍球（初始位置）
        surf(p(1)+Xd, p(2)+Yd, p(3)+Zd, 'FaceColor', c, 'EdgeColor', 'none', ...
            'FaceAlpha', 0.85, 'HandleVisibility', 'off');

        % 运动轨迹示意
        switch o.type
            case 1   % 直线 S<->E
                plot3([o.S(1) o.E(1)], [o.S(2) o.E(2)], [o.S(3) o.E(3)], '-', ...
                    'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
            case 2   % 圆弧轨道
                th = linspace(0, 2*pi, 80);
                plot3(o.center(1)+o.R*cos(th), o.center(2)+o.R*sin(th), o.center(3)+0*th, ...
                    ':', 'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
            case 3   % 变速直线 S<->E
                plot3([o.S(1) o.E(1)], [o.S(2) o.E(2)], [o.S(3) o.E(3)], '--', ...
                    'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
            case 4   % 垂直：水平 S<->E + 垂直范围
                plot3([o.S(1) o.E(1)], [o.S(2) o.E(2)], [o.S(3) o.E(3)], '-.', ...
                    'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
                xm = (o.S(1) + o.E(1)) / 2;  ym = (o.S(2) + o.E(2)) / 2;
                plot3([xm xm], [ym ym], [o.z0 - o.A, o.z0 + o.A], '-', ...
                    'Color', c, 'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end
end

%% ====================== 子图编号标注 ======================
function add_tile_label(ax, letter)
% 在子图正下方、xlabel 之下居中标注 (a)/(b)/(c)/(d)/(e)/(f)
% 用 TightInset 计算坐标轴（含刻度与坐标标签）的真实底边，避免与图形/坐标标签重叠
    pos   = get(ax, 'Position');        % 数据区 [left bottom width height]
    inset = get(ax, 'TightInset');      % 含刻度/坐标标签的边距 [left bottom right top]
    xc = pos(1) + pos(3)/2;
    w  = 0.07; h = 0.04;
    x0 = max(0, min(1 - w, xc - w/2));
    y0 = pos(2) - inset(2) - h - 0.02;  % 放在 xlabel 之下，再留 0.02 间隙
    y0 = max(0, min(1 - h, y0));
    annotation(gcf, 'textbox', [x0, y0, w, h], ...
        'String', sprintf('(%s)', letter), ...
        'EdgeColor', 'none', 'LineStyle', 'none', ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontSize', 20, 'FontWeight', 'bold');
end
