function result = run_icssa(map_type, do_plot)
% RUN_ICSSA  改进混沌麻雀搜索算法(ICSSA)三维路径规划主程序
% -------------------------------------------------------------------------
% 功能:
%   1) 按 map_type 构建三维环境(city 城市 / hill 丘陵 / legacy 山体+球+圆柱);
%   2) 用 ICSSA 单目标搜索, 得出一条不穿障碍的最优路径;
%   3) 绘制: 三维地图+红色路径+起点(绿球)+终点(蓝球), 以及收敛曲线;
%   4) 返回 result 结构体, 含全部评估指标供主程序汇总。
% -------------------------------------------------------------------------
if nargin < 1 || isempty(map_type), map_type = 'legacy'; end
if nargin < 2 || isempty(do_plot), do_plot = true; end
rng('default');   % 复位随机数生成器
rng(2025);        % 固定随机种子, 便于结果复现

%% 1. 环境模型
model = build_environment(map_type);
fprintf('地图类型: %s   起点: [%g %g %g]   终点: [%g %g %g]\n', ...
        map_type, model.start, model.goal);

%% 2. ICSSA 算法参数
params.N         = 40;   % 种群规模
params.K         = 8;    % 中间航点数
params.M         = 150;  % 最大迭代次数
params.n_pts     = 350;  % 单条路径采样点数
params.safe_dist = 2.5;    % 威胁安全阈值(m)

if strcmp(map_type, 'city')
    params.init_wps = city_feasible_seed(params.K, model.start, model.goal);
end

%% 3. 运行 ICSSA
fprintf('\n===== 开始 ICSSA 路径规划 =====\n');
[best_path, best_cost, stats, curve] = icssa_optimize(model, params);

fprintf('===== 规划完成 =====\n');
fprintf('综合成本       = %.3f\n', best_cost);
fprintf('路径长度       = %.2f m\n', stats(1));
fprintf('威胁代价       = %.2f (0 表示全程未贴近障碍)\n', stats(2));
fprintf('能耗           = %.2f Wh\n', stats(3));
fprintf('不平滑度       = %.4f (越小越平滑)\n', stats(4));
fprintf('最小安全距离   = %.2f m\n', stats(5));
fprintf('飞行时间       = %.2f s (V=10 m/s)\n', stats(6));

%% 4. 可视化
if do_plot
    draw_map_and_path(model, best_path, map_type);
    draw_curve(curve, map_type);
end

%% 5. 组装返回结果
result.map_type      = map_type;
result.map_name      = map_name_of(map_type);
result.best_path     = best_path;
result.best_cost     = best_cost;
result.length        = stats(1);
result.threat        = stats(2);
result.energy        = stats(3);
result.roughness     = stats(4);
result.min_clearance = stats(5);
result.flight_time   = stats(6);
result.model         = model;
result.curve         = curve;
end

% =========================================================================
function draw_map_and_path(model, best_path, map_type)
tick_font_size  = 20; label_font_size = 20; title_font_size = 16;

figure('Name', sprintf('环境与路径-%s', map_type), 'Position', [100, 100, 850, 800], 'Color', 'white');
ax = gca;
hold(ax, 'on');

if isfield(model, 'type') && strcmp(model.type, 'city')
    % ---- 城市: 平底地面 + 矩形高楼 ----
    patch(ax, [0 200 200 0], [0 0 200 200], [1e-3 1e-3 1e-3 1e-3], ...
          [0.82 0.82 0.78], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    for i = 1:size(model.buildings, 1)
        draw_building_box(ax, model.buildings(i, :));
    end
else
    % ---- 高斯山体 ----
    map_x = 0:0.5:200; map_y = 0:0.5:200;
    [X, Y] = meshgrid(map_x, map_y);
    for k = 1:size(model.mountains, 1)
        cx = model.mountains(k, 1); cy = model.mountains(k, 2);
        A  = model.mountains(k, 3); s = model.mountains(k, 4);
        z = A * exp(-((X - cx).^2 + (Y - cy).^2) / (2 * s^2));
        z(z < 0) = 0;
        surf(ax, X, Y, z, 'FaceColor', 'interp', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    end

    % ---- 球形禁飞区(存在时) ----
    if ~isempty(model.ball) && numel(model.ball) == 4
        [Xb, Yb, Zb] = sphere(50);
        bc = model.ball(1:3); br = model.ball(4);
        bz = bc(3) + Zb * br; bz(bz < 0) = 0;
        surf(ax, bc(1) + Xb * br, bc(2) + Yb * br, bz, 'FaceColor', [1 0 1], ...
            'EdgeColor', 'none', 'FaceAlpha', 1.0);
    end

    % ---- 圆柱体(带封顶, 存在时) ----
    if ~isempty(model.cylinder.pos) && model.cylinder.r > 0
        cyl_r = model.cylinder.r;
        [Xc, Yc, Zc] = cylinder(cyl_r, 50);
        theta = linspace(0, 2 * pi, 50);
        for i = 1:size(model.cylinder.pos, 1)
            x0 = model.cylinder.pos(i, 1); y0 = model.cylinder.pos(i, 2); h = model.cylinder.pos(i, 3);
            surf(ax, x0 + Xc, y0 + Yc, h * Zc, 'FaceColor', [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
            fill3(ax, x0 + cyl_r * cos(theta), y0 + cyl_r * sin(theta), repmat(h, size(theta)), ...
                [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
        end
    end
end

% 最优路径(红色)
plot3(ax, best_path(:, 1), best_path(:, 2), best_path(:, 3), '-', ...
    'Color', [1 0 0], 'LineWidth', 3);

% 起点/终点小球
draw_marker_sphere(ax, model.start, 5, [0 0.8 0]);
draw_marker_sphere(ax, model.goal,  5, [0 0 1]);

xlabel(ax, 'X/m', 'FontSize', label_font_size);
ylabel(ax, 'Y/m', 'FontSize', label_font_size);
zlabel(ax, 'Z/m', 'FontSize', label_font_size);
title(ax, sprintf('ICSSA 改进混沌麻雀搜索算法路径规划（%s）', map_name_of(map_type)), 'FontSize', title_font_size);
xlim(ax, [0 200]); ylim(ax, [0 200]); zlim(ax, [0 200]);
xticks(ax, 0:20:200); yticks(ax, 0:20:200); zticks(ax, 0:20:200);
daspect(ax, [1 1 1]);
set(ax, 'FontSize', tick_font_size, 'Color', 'white');
grid(ax, 'on');
view(ax, 45, 35);
camproj(ax, 'orthographic');

lg(1) = plot3(ax, nan, nan, nan, '-', 'Color', [1 0 0], 'LineWidth', 3);
lg(2) = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'k');
lg(3) = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0 0 1], 'MarkerEdgeColor', 'k');
legend(ax, lg, {'最优路径(无碰撞)', '起点', '终点'}, 'Location', 'northeast', 'FontSize', 12);
hold(ax, 'off');
end

function draw_building_box(ax, b)
% 绘一栋立方体高楼(4 侧面 + 顶盖), 底在 z=0; 越高越偏蓝紫
x0 = b(1); x1 = b(2); y0 = b(3); y1 = b(4); h = b(5);
t = min(max(h/150, 0), 1);
col = [0.4 - 0.15*t, 0.48 - 0.10*t, 0.72 + 0.10*t];
verts_x = [x0, x1, x1, x0, x0, x1, x1, x0];
verts_y = [y0, y0, y1, y1, y0, y0, y1, y1];
verts_z = [0, 0, 0, 0, h, h, h, h];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax, 'Vertices', [verts_x(:), verts_y(:), verts_z(:)], 'Faces', faces, ...
      'FaceColor', col, 'EdgeColor', 'k', 'LineWidth', 0.5, 'FaceAlpha', 1.0);
patch(ax, 'XData', [x0 x1 x1 x0], 'YData', [y0 y0 y1 y1], 'ZData', [h h h h], ...
      'FaceColor', col .* 1.2, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
end

function str = map_name_of(map_type)
switch map_type
    case 'city', str = '城市地图';
    case 'hill', str = '丘陵地图';
    otherwise,   str = '山体障碍地图';
end
end

function wps = city_feasible_seed(K, start, goal)
% 生成城市地图 K 个可行中间航点(有知初始化种子): 沿街道低空穿行, 不翻越楼顶。
% 城市建筑按 6x6 网格排布(与 map/city_map.m 的 rng(26) 一致), 第 3 行与第 3 列
% 整行列留空形成贯穿主干道, 四周留 14m 边缘空档。存在一条必然无碰撞的低空 L 形航线:
%   start(10,10,60) --左边缘北上--> (10,85.67,60) --主干道东行--> (190,85.67,60)
%                   --右边缘南下--> goal(190,190,60)
% 85.67 为中央横向主干道中心 y 坐标, 全程无楼、净空充足。
AVENUE_Y = 85.6667;
z = start(3);
verts = [start;
         start(1), AVENUE_Y, z;   % 沿左边缘到主干道
         goal(1),  AVENUE_Y, z;   % 沿主干道到右边缘
         goal];                   % 沿右边缘到终点
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

function draw_marker_sphere(ax, center, radius, color)
[sx, sy, sz] = sphere(30);
surf(ax, center(1) + sx * radius, center(2) + sy * radius, center(3) + sz * radius, ...
    'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
end

function draw_curve(curve, map_type)
figure('Name', sprintf('收敛曲线-%s', map_type), 'Color', 'white', 'Position', [950, 100, 620, 420]);
plot(1:numel(curve), curve, 'LineWidth', 1.8, 'Color', [0 0.45 0.74]);
grid on;
xlabel('迭代次数', 'FontSize', 20);
ylabel('最优综合成本', 'FontSize', 20);
title(sprintf('ICSSA 收敛曲线（%s）', map_name_of(map_type)), 'FontSize', 16);
set(gca, 'FontSize', 20, 'Color', 'white');
end