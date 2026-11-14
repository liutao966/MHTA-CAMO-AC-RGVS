% =========================================================================
%  可视化：VO / RVO / DWA 三个速度空间算法的动态避障飞行轨迹（各一张图）
% -------------------------------------------------------------------------
%  地图（三座高斯山 + 品红禁飞球 + 黄色圆柱带封顶）、四类动态障碍物（球体）、
%  起点/终点（绿球/蓝球）、全局规划路径（棕色实线）、实际飞行轨迹（红色实线）
%  的绘制方式与颜色完全对齐 AVSS_RRT/AVSS_RRT_dynamic.m。
%  每个算法单独输出一张 3D 图（共 3 张 png）。
% =========================================================================
clear; clc; close all;

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'obstacle_gradient_exp'));
addpath(fullfile(root, 'VO'));
addpath(fullfile(root, 'RVO'));
addpath(fullfile(root, 'DWA'));

env   = build_env();
start = env.start; goal = env.goal;

fprintf('生成本文算法全局参考路径 ...\n');
path_ours = plan_this(env);

seed = 1;
dyn  = gen_obstacles_v2(seed, path_ours, env);

% 三个算法各跑一次（同一份障碍初始状态）
[ok_vo,  trail_vo]  = sim_vo(path_ours, dyn, start, goal, env);
[ok_rvo, trail_rvo] = sim_rvo(path_ours, dyn, start, goal, env);
[ok_dwa, trail_dwa] = sim_dwa(path_ours, dyn, start, goal, env);

algos  = {'VO', 'RVO', 'DWA'};
trails = {trail_vo, trail_rvo, trail_dwa};
oks    = [ok_vo, ok_rvo, ok_dwa];

% 颜色（与 AVSS_RRT_dynamic.m 一致）
col_plan  = [0.55 0.27 0.07];   % 棕色：全局规划路径
col_trail = [0.85 0.0  0.0 ];   % 正红：实际飞行轨迹
type_color = [0.85 0.10 0.10;   % 1 直线：红
              0.10 0.40 0.90;   % 2 圆弧：蓝
              0.10 0.60 0.20;   % 3 变速：绿
              0.90 0.50 0.10];  % 4 垂直：橙

for k = 1:3
    figure('Position', [100 100 900 800], 'Color', 'white');
    draw_world_vo(env);
    hold on;

    % 起点（绿球）/ 终点（蓝球）
    draw_ball_vo(start, 4, [0 0.8 0]);
    draw_ball_vo(goal,  4, [0 0 1]);

    % 全局规划路径（棕色实线）
    plot3(path_ours(:,1), path_ours(:,2), path_ours(:,3), ...
        '-', 'Color', col_plan, 'LineWidth', 2.2);

    % 该算法实际飞行轨迹（红色实线）
    tr = trails{k};
    plot3(tr(:,1), tr(:,2), tr(:,3), '-', 'Color', col_trail, 'LineWidth', 3);

    % 动态障碍物（球体，按类型着色）
    [Xd, Yd, Zd] = sphere(20);
    Xd = Xd*dyn.r; Yd = Yd*dyn.r; Zd = Zd*dyn.r;
    for i = 1:dyn.num
        c = dyn.curr(i, :);
        surf(c(1)+Xd, c(2)+Yd, c(3)+Zd, ...
            'FaceColor', type_color(dyn.obs{i}.type, :), ...
            'EdgeColor', 'none', 'FaceAlpha', 1.0);
    end

    legend_h = [ plot3(nan,nan,nan,'-','Color',col_trail,'LineWidth',3), ...
                 plot3(nan,nan,nan,'-','Color',col_plan,'LineWidth',2.2), ...
                 plot3(nan,nan,nan,'o','MarkerFaceColor',type_color(1,:),'MarkerEdgeColor','none','MarkerSize',10), ...
                 plot3(nan,nan,nan,'o','MarkerFaceColor',type_color(2,:),'MarkerEdgeColor','none','MarkerSize',10), ...
                 plot3(nan,nan,nan,'o','MarkerFaceColor',type_color(3,:),'MarkerEdgeColor','none','MarkerSize',10), ...
                 plot3(nan,nan,nan,'o','MarkerFaceColor',type_color(4,:),'MarkerEdgeColor','none','MarkerSize',10), ...
                 plot3(nan,nan,nan,'o','MarkerFaceColor',[0 0.8 0],'MarkerEdgeColor','none','MarkerSize',10), ...
                 plot3(nan,nan,nan,'o','MarkerFaceColor',[0 0 1],'MarkerEdgeColor','none','MarkerSize',10)];
    legend(legend_h, {'飞行轨迹','规划路径','直线障碍','圆弧障碍','变速障碍','垂直障碍','起点','终点'}, ...
        'Location', 'northeast', 'FontSize', 11);

    title(sprintf('%s 动态障碍物路径规划与避障', algos{k}), 'FontSize', 16);
    view(45, 30);

    saveas(gcf, sprintf('%s.png', algos{k}));
    fprintf('已保存: %s.png\n', algos{k});
end

% ================= 地图（与 AVSS_RRT_dynamic.m 的 draw_world 一致） =================
function draw_world_vo(env)
    tick_font_size  = 20;
    label_font_size = 20;

    map_x = 0:0.5:200;
    map_y = 0:0.5:200;
    [X, Y] = meshgrid(map_x, map_y);

    % 三座高斯山体
    for i = 1:size(env.mountains, 1)
        m = env.mountains(i, :);
        Z = m(3) * exp(-((X - m(1)).^2 + (Y - m(2)).^2) / (2*m(4)^2));
        Z(Z < 0) = 0;
        surf(X, Y, Z, 'FaceColor', 'interp', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
        hold on;
    end

    % 球形禁飞区（品红，不透明）
    s = env.ball;
    [Xs, Ys, Zs] = sphere(50);
    bx = s(1) + Xs*s(4);  by = s(2) + Ys*s(4);  bz = s(3) + Zs*s(4);
    bz(bz < 0) = 0;
    surf(bx, by, bz, 'FaceColor', [1 0 1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);

    % 圆柱障碍（黄色，带封顶）
    [Xc, Yc, Zc] = cylinder(env.cyl_r, 50);
    theta = linspace(0, 2*pi, 50);
    for i = 1:size(env.cylinders, 1)
        c = env.cylinders(i, :);
        surf(c(1)+Xc, c(2)+Yc, c(3)*Zc, 'FaceColor', [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
        tx = c(1) + env.cyl_r*cos(theta);
        ty = c(2) + env.cyl_r*sin(theta);
        tz = repmat(c(3), size(tx));
        fill3(tx, ty, tz, [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    end

    % 坐标轴
    xlabel('X/m', 'FontSize', label_font_size);
    ylabel('Y/m', 'FontSize', label_font_size);
    zlabel('Z/m', 'FontSize', label_font_size);
    xlim([0 200]); ylim([0 200]); zlim([0 200]);
    xticks(0:20:200); yticks(0:20:200); zticks(0:20:200);
    daspect([1 1 1]);
    set(gca, 'FontSize', tick_font_size, 'Color', 'white');
    grid on;
end

% ================= 球体（与 AVSS_RRT_dynamic.m 的 draw_ball 一致） =================
function draw_ball_vo(center, r, color)
    [Xs, Ys, Zs] = sphere(20);
    surf(center(1)+Xs*r, center(2)+Ys*r, center(3)+Zs*r, ...
        'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
end
