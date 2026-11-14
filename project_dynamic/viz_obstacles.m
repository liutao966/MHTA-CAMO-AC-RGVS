% =========================================================================
%  可视化：四类动态障碍物 + 两条参考路径 + AVSS-RRT 动态轨迹
% -------------------------------------------------------------------------
%  用途：单 seed 直观查看 gen_obstacles_v2 生成的 14 个障碍长什么样、在哪、
%        与本文算法 / AVSS-RRT 路径的相对位置，以及 AVSS-RRT 的实际飞行轨迹。
%  类型着色：红=直线匀速  蓝=匀速圆弧  绿=变速直线  橙=垂直运动
% =========================================================================
clear; clc; close all;

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'obstacle_gradient_exp'));

env = build_env();
fprintf('生成本文算法与 AVSS-RRT 静态路径 ...\n');
path_ours = plan_this(env);
path_avss = plan_avss(env);

seed = 1;
dyn = gen_obstacles_v2(seed, path_ours, env);

% 跑 AVSS-RRT 单次，拿它的动态轨迹（障碍物仍放在 path_ours 沿线）
[ok, trail] = sim_avss(path_avss, dyn, env.start, env.goal, env);
fprintf('AVSS-RRT seed %d 成功=%d  轨迹点数=%d\n', seed, ok, size(trail, 1));

% ---------- 画图 ----------
figure('Position', [60 60 1080 800], 'Color', 'w');
hold on; grid on; box on;

% 地形（三座高斯山）
[gx, gy] = meshgrid(0:4:200, 0:4:200);
gz = zeros(size(gx));
for k = 1:size(env.mountains, 1)
    m = env.mountains(k, :);
    gz = max(gz, m(3) * exp(-((gx - m(1)).^2 + (gy - m(2)).^2) / (2 * m(4)^2)));
end
surf(gx, gy, gz, 'FaceAlpha', 0.22, 'EdgeColor', 'none', 'FaceColor', [0.55 0.48 0.30], 'HandleVisibility', 'off');

% 球形禁飞区（半透明）
[bsx, bsy, bsz] = sphere(20);
surf(env.ball(1) + env.ball(4)*bsx, env.ball(2) + env.ball(4)*bsy, env.ball(3) + env.ball(4)*bsz, ...
     'FaceAlpha', 0.15, 'EdgeColor', 'none', 'FaceColor', [0.45 0.45 0.45], 'HandleVisibility', 'off');

% 圆柱（竖杆示意）
for k = 1:size(env.cylinders, 1)
    c = env.cylinders(k, :);
    plot3([c(1) c(1)], [c(2) c(2)], [0 c(3)], '-', 'Color', [0.35 0.35 0.35], ...
        'LineWidth', 4, 'HandleVisibility', 'off');
end

% 起终点
plot3(env.start(1), env.start(2), env.start(3), 'kp', 'MarkerSize', 16, 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');
plot3(env.goal(1),  env.goal(2),  env.goal(3),  'kp', 'MarkerSize', 16, 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');

% 两条静态路径 + AVSS-RRT 动态轨迹
plot3(path_ours(:,1), path_ours(:,2), path_ours(:,3), 'r-', 'LineWidth', 2.2, 'DisplayName', 'This algorithm path (障碍放置基准)');
plot3(path_avss(:,1), path_avss(:,2), path_avss(:,3), 'b-', 'LineWidth', 1.6, 'DisplayName', 'AVSS-RRT static path');
plot3(trail(:,1),    trail(:,2),    trail(:,3),    'g-', 'LineWidth', 1.4, 'DisplayName', 'AVSS-RRT dynamic trail');

% 四类障碍物
colors = [0.85 0.10 0.10;   % 1 直线：红
          0.10 0.40 0.90;   % 2 圆弧：蓝
          0.10 0.60 0.20;   % 3 变速：绿
          0.90 0.50 0.10];  % 4 垂直：橙
type_names = {'直线匀速', '匀速圆弧', '变速直线', '垂直运动'};
leg_h = nan(4, 1);

for i = 1:dyn.num
    o = dyn.obs{i};
    c = colors(o.type, :);
    p = dyn.curr(i, :);

    % 障碍球（初始位置，大圆点示意）
    scatter3(p(1), p(2), p(3), 80, c, 'filled', 'MarkerFaceAlpha', 0.75, 'HandleVisibility', 'off');

    % 运动轨迹
    switch o.type
        case 1   % 直线 S<->E
            plot3([o.S(1) o.E(1)], [o.S(2) o.E(2)], [o.S(3) o.E(3)], '-', 'Color', c, 'LineWidth', 1.6, 'HandleVisibility', 'off');
        case 2   % 圆弧轨道
            th = linspace(0, 2*pi, 100);
            plot3(o.center(1) + o.R*cos(th), o.center(2) + o.R*sin(th), o.center(3) + 0*th, ...
                ':', 'Color', c, 'LineWidth', 1.6, 'HandleVisibility', 'off');
            plot3(o.center(1), o.center(2), o.center(3), '+', 'Color', c, 'MarkerSize', 8, 'HandleVisibility', 'off');
        case 3   % 变速直线 S<->E
            plot3([o.S(1) o.E(1)], [o.S(2) o.E(2)], [o.S(3) o.E(3)], '--', 'Color', c, 'LineWidth', 1.6, 'HandleVisibility', 'off');
        case 4   % 垂直：水平 S<->E + 垂直范围
            plot3([o.S(1) o.E(1)], [o.S(2) o.E(2)], [o.S(3) o.E(3)], '-.', 'Color', c, 'LineWidth', 1.6, 'HandleVisibility', 'off');
            xm = (o.S(1) + o.E(1)) / 2;  ym = (o.S(2) + o.E(2)) / 2;
            plot3([xm xm], [ym ym], [o.z0 - o.A, o.z0 + o.A], '-', 'Color', c, 'LineWidth', 1.6, 'HandleVisibility', 'off');
    end

    % 每种类型留一个图例手柄
    if isnan(leg_h(o.type))
        leg_h(o.type) = plot3(NaN, NaN, NaN, '-', 'Color', c, 'LineWidth', 2);
    end
end

legend([leg_h(1); leg_h(2); leg_h(3); leg_h(4)], type_names, 'Location', 'northwest');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(sprintf('四类动态障碍物布局 (seed %d, %d 个障碍)', seed, dyn.num), 'FontWeight', 'bold');
xlim([0 200]); ylim([0 200]); zlim([0 200]);
axis equal; view(135, 28);
