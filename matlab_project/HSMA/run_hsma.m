function best_path = run_hsma(map_type, do_plot)
% RUN_HSMA  HSMA(混合黏菌算法)多目标三维路径规划主程序
% -------------------------------------------------------------------------
% 功能:
%   1) 按 map_type 构建三维环境(city 城市 / hill 丘陵 / 缺省旧山体地图);
%   2) 用 HSMA 多目标搜索迭代，得到 Pareto 最优航线集;
%   3) 绘制:
%        - 三维地图 + Pareto 航线(红色) + 起点(绿球) + 终点(蓝球);
%        - Pareto 前沿散点图(长度 vs 威胁, 颜色=能耗);
%        - 存档非支配解数量随迭代的收敛曲线。
% -------------------------------------------------------------------------
if nargin < 1 || isempty(map_type), map_type = 'legacy'; end
if nargin < 2 || isempty(do_plot), do_plot = true; end
rng('default');   % 复位随机数生成器（清除可能存在旧生成器）
rng(2025);        % 固定随机种子，便于结果复现

%% 1. 环境模型
model = build_environment(map_type);
fprintf('地图类型: %s   起点: [%g %g %g]   终点: [%g %g %g]\n', ...
        map_type, model.start, model.goal);

%% 2. HSMA 算法参数
params.N        = 40;     % 种群规模
params.tmax     = 150;    % 最大迭代次数
params.K        = 8;      % 中间航点数
params.n_pts    = 350;    % 单条路径采样点数
params.safe_dist= 2.5;      % 威胁代价的安全阈值(m)

% 城市地图: 注入一条必然可行的低空街道穿行剖面作为初始化种子(见 city_feasible_seed),
% 保证外部存档至少有一个无碰撞解, 避免算法找不到可行解而输出穿楼/乱转的兜底路径。
if strcmp(map_type, 'city')
    params.init_wps = city_feasible_seed(params.K, model.start, model.goal);
end

%% 3. 运行 HSMA 多目标优化
fprintf('\n===== 开始 HSMA 多目标路径规划 =====\n');
tic;
[arc_paths, arc_f, hist_front, hist_arc] = hsma_multiobjective(model, params);
elapsed = toc;

% ---- 从 Pareto 集中选出唯一一条"无碰撞且合理"的路径 ----
% 碰撞路径的威胁代价会因硬惩罚而巨大(>1e4)，据此剔除。
free = arc_f(:, 6) < 1e4;            % 威胁代价在第6列, ≈0 => 全程满足安全净空
if any(free)
    f_id = find(free);
    Ff  = arc_f(f_id, :);
    % 在无碰撞解中综合"长度+能耗+不平滑度"选均衡折中解(knee),
    % 避免仅凭长度最短选出"冲天折线/贴面"等畸短路径。
    G   = Ff(:, [1 2 3]);            % 新目标顺序: 1=长度, 2=能耗, 3=不平滑度
    Gn  = (G - min(G)) ./ (max(G) - min(G) + 1e-9);
    [~, li] = min(sum(Gn, 2));
    bestIdx = f_id(li);
else
    [~, bestIdx] = min(sum(arc_f, 2));
    warning(['未找到完全无碰撞路径，已取综合代价最小的一条，', ...
             '请增大迭代次数或减小 safe_dist。']);
end
best_path = arc_paths{bestIdx};
best_f    = arc_f(bestIdx, :);

% 目标: [长度(1), 能耗(2), 不平滑度(3), 飞行时间(4), -安全距离(5), 威胁代价(6)]
fprintf('===== 规划完成 =====\n');
fprintf('输出唯一最优路径(不穿过障碍):\n');
fprintf('  目标1 - 路径长度     = %.2f m\n', best_f(1));
fprintf('  目标2 - 能耗         = %.2f Wh\n', best_f(2));
fprintf('  目标3 - 不平滑度     = %.4f (越小越平滑)\n', best_f(3));
fprintf('  目标4 - 飞行时间     = %.2f s (V=10m/s)\n', best_f(4));
fprintf('  目标5 - 最小安全距离 = %.2f m (路径上距障碍物最近点)\n', -best_f(5));
fprintf('  目标6 - 威胁代价     = %.2f (0 表示全程无碰撞)\n',    best_f(6));
fprintf('算法耗时: %.2f s\n', elapsed);

%% 4. 可视化
if do_plot
    draw_map_and_paths(model, best_path, map_type);   % 三维地图 + 唯一最优路径
    draw_pareto_front(arc_f, map_type);               % Pareto 前沿(参考)
    draw_convergence(hist_front, hist_arc, map_type); % 收敛曲线
end
end

% =========================================================================
function draw_map_and_paths(model, best_path, map_type)
% 绘制与 map.m 一致的三维地图，并叠加唯一最优航线、起点/终点小球
tick_font_size  = 20;
label_font_size = 20;
title_font_size = 16;

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
    % ---- 山体/丘陵/旧地图 ----
    map_x = 0:0.5:200;
    map_y = 0:0.5:200;
    [X, Y] = meshgrid(map_x, map_y);

    for k = 1:size(model.mountains, 1)
        cx = model.mountains(k, 1);
        cy = model.mountains(k, 2);
        A  = model.mountains(k, 3);
        s  = model.mountains(k, 4);
        z = A * exp(-((X - cx).^2 + (Y - cy).^2) / (2 * s^2));
        z(z < 0) = 0;
        surf(ax, X, Y, z, 'FaceColor', 'interp', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    end

    % ---- 球形禁飞区（粉色，存在时）----
    if ~isempty(model.ball) && numel(model.ball) == 4
        n_ball = 50;
        [Xb, Yb, Zb] = sphere(n_ball);
        bc = model.ball(1:3); br = model.ball(4);
        bx = bc(1) + Xb * br; by = bc(2) + Yb * br; bz = bc(3) + Zb * br; bz(bz < 0) = 0;
        surf(ax, bx, by, bz, 'FaceColor', [1 0 1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    end

    % ---- 圆柱体障碍物（带封顶，黄色，存在时）----
    if ~isempty(model.cylinder.pos) && model.cylinder.r > 0
        cyl_r = model.cylinder.r;
        n_cyl = 50;
        [Xc, Yc, Zc] = cylinder(cyl_r, n_cyl);
        theta = linspace(0, 2 * pi, n_cyl);
        for i = 1:size(model.cylinder.pos, 1)
            x0 = model.cylinder.pos(i, 1);
            y0 = model.cylinder.pos(i, 2);
            h  = model.cylinder.pos(i, 3);
            surf(ax, x0 + Xc, y0 + Yc, h * Zc, 'FaceColor', [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
            fill3(ax, x0 + cyl_r * cos(theta), y0 + cyl_r * sin(theta), repmat(h, size(theta)), ...
                [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
        end
    end
end

% ---- 规划出的唯一最优路径（红色）----
plot3(ax, best_path(:, 1), best_path(:, 2), best_path(:, 3), '-', ...
      'Color', [1 0 0], 'LineWidth', 3);

% ---- 起点（绿色球）与终点（蓝色球）----
draw_marker_sphere(ax, model.start, 5, [0 0.8 0]);
draw_marker_sphere(ax, model.goal,  5, [0 0 1]);

% ---- 坐标轴设置（与 map.m 一致）----
xlabel(ax, 'X/m', 'FontSize', label_font_size);
ylabel(ax, 'Y/m', 'FontSize', label_font_size);
zlabel(ax, 'Z/m', 'FontSize', label_font_size);
title(ax, sprintf('HSMA 混合黏菌算法多目标三维路径规划（%s）', map_name_of(map_type)), 'FontSize', title_font_size);
xlim(ax, [0 200]); ylim(ax, [0 200]); zlim(ax, [0 200]);
xticks(ax, 0:20:200); yticks(ax, 0:20:200); zticks(ax, 0:20:200);
daspect(ax, [1 1 1]);
set(ax, 'FontSize', tick_font_size, 'Color', 'white');
grid(ax, 'on');
view(ax, 45, 35);

legend_handles(1) = plot3(ax, nan, nan, nan, '-', 'Color', [1 0 0], 'LineWidth', 3);
legend_handles(2) = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'k');
legend_handles(3) = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0 0 1], 'MarkerEdgeColor', 'k');
legend(ax, legend_handles, {'最优路径(无碰撞)', '起点', '终点'}, ...
    'Location', 'northeast', 'FontSize', 12);
hold(ax, 'off');
end

% =========================================================================
function draw_marker_sphere(ax, center, radius, color)
[sx, sy, sz] = sphere(30);
surf(ax, center(1) + sx * radius, center(2) + sy * radius, center(3) + sz * radius, ...
    'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
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
    case 'city',     str = '城市地图';
    case 'hill',     str = '丘陵地图';
    case 'obstacle', str = '障碍物地图';
    otherwise,       str = '山体障碍地图';
end
end

function wps = city_feasible_seed(K, start, goal)
% 生成城市地图 K 个可行中间航点(有知初始化种子): 沿街道低空穿行, 不翻越楼顶。
% 城市建筑按 6x6 网格排布(与 map/city_map.m 的 rng(26) 一致), 且第 3 行(r==3)与
% 第 3 列(c==3)整行整列留空, 形成贯穿东西/南北的主干道; 四周留 14m 边缘空档。
% 因此存在一条必然无碰撞的低空 L 形街道航线(全程高度 60m, 即起终点高度):
%   start(10,10,60) --左边缘北上--> (10,85.67,60) --主干道东行--> (190,85.67,60)
%                   --右边缘南下--> goal(190,190,60)
% 其中 85.67 为中央主干道(row3 空档)中心 y 坐标, 全程无楼、净空充足。
% 以等弧长在折线上采样 K 个中间航点。
AVENUE_Y = 85.6667;   % 中央横向主干道中心(第 3 行空档)
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

% =========================================================================
function draw_pareto_front(arc_f, map_type)
figure('Name', sprintf('Pareto前沿-%s', map_type), 'Color', 'white', 'Position', [980, 200, 620, 460]);
% Pareto 前沿: 长度 vs 飞行时间, 颜色表示能耗(第2列)
scatter(arc_f(:, 1), arc_f(:, 4), 45, arc_f(:, 2), 'filled');
colorbar;
xlabel('路径长度 / m', 'FontSize', 20);
ylabel('飞行时间 / s', 'FontSize', 20);
title(sprintf('HSMA Pareto 前沿（%s，颜色 = 能耗）', map_name_of(map_type)), 'FontSize', 16);
set(gca, 'FontSize', 20, 'Color', 'white');
grid on;
end

% =========================================================================
function draw_convergence(hist_front, hist_arc, map_type)
figure('Name', sprintf('收敛曲线-%s', map_type), 'Color', 'white', 'Position', [980, 700, 620, 420]);
t = 1:numel(hist_arc);
plot(t, hist_arc, 'b-', 'LineWidth', 1.8); hold on;
plot(t, hist_front, 'r--', 'LineWidth', 1.8);
grid on;
xlabel('迭代次数', 'FontSize', 20);
ylabel('非支配解数量', 'FontSize', 20);
title(sprintf('HSMA 收敛曲线（%s）', map_name_of(map_type)), 'FontSize', 16);
legend({'外部存档规模', '每代种群非支配解数'}, 'Location', 'northwest', 'FontSize', 12);
set(gca, 'FontSize', 20, 'Color', 'white');
end