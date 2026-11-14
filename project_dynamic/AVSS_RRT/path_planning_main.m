function path_planning_main()
% PATH_PLANNING_MAIN  基于混沌优化算法(COA)的无人机三维路径规划主程序
% -------------------------------------------------------------------------
% 参考文献: "Path Planning Strategy and Performance Analysis of UAV Based on
%           Chaotic Optimization Algorithm" (CAIBDA 2025)
%
% 功能:
%   1) 构建与 map.m 一致的三维地形/障碍物地图；
%   2) 使用混沌优化算法(COA)规划从起点到终点的三维路径；
%   3) 在三维地图上绘制:
%        - 红色路线    : 规划出的最优路径
%        - 绿色小球    : 起点 [10, 10, 50]
%        - 蓝色小球    : 终点 [190, 190, 80]
%
% 直接运行本文件即可: >> path_planning_main
% -------------------------------------------------------------------------
clc; close all;
rng(2025);   % 固定随机种子，便于结果复现

%% 1. 载入地图模型
model = map_model();
fprintf('起点: [%g %g %g]   终点: [%g %g %g]\n', model.start, model.goal);

%% 2. COA 算法参数
params.pop_size        = 100;   % 种群规模
params.num_points      = 8;     % 中间航点数量（不含起点/终点）
params.max_iter        = 300;   % 最大迭代次数
params.tournament_size = 3;     % 锦标赛选择规模
params.pc              = 0.9;   % 交叉概率
params.pm              = 0.12;  % 变异概率（每个基因）
params.penalty         = 800;   % 单个碰撞采样点的惩罚值 P
params.check_step      = 2;     % 碰撞检测离散步长(m)

%% 3. 运行混沌优化算法
fprintf('\n===== 开始混沌优化路径规划 =====\n');
tic;
[best_path, best_fitness, curve] = chaos_path_planning(model, params);
elapsed = toc;

% 校验最终路径是否无碰撞
pts = densify_path(best_path, params.check_step);
[collide, n_hit] = check_collision(model, pts);
path_len = sum(sqrt(sum(diff(best_path, 1, 1).^2, 2)));

fprintf('\n===== 规划完成 =====\n');
fprintf('最优适应度 F = %.3f\n', best_fitness);
fprintf('路径总长 L = %.3f m\n', path_len);
fprintf('是否碰撞: %s (碰撞采样点数 = %d)\n', mat2str(collide), n_hit);
fprintf('耗时: %.2f s\n', elapsed);

%% 4. 绘制三维地图 + 规划路径
draw_map_and_path(model, best_path);

%% 5. 绘制收敛曲线
figure('Color', 'white', 'Position', [950, 100, 560, 420]);
plot(curve, 'LineWidth', 1.8, 'Color', [0 0.45 0.74]);
grid on;
xlabel('迭代次数', 'FontSize', 20);
ylabel('最优适应度', 'FontSize', 20);
title('混沌优化算法收敛曲线', 'FontSize', 16);
set(gca, 'FontSize', 20);
end

% =========================================================================
function draw_map_and_path(model, best_path)
% 绘制与 map.m 一致的三维地图，并叠加规划路径、起点/终点小球
% -------------------------------------------------------------------------
tick_font_size  = 20;
label_font_size = 20;
title_font_size = 16;

% ---- 三维网格 ----
map_x = 0:0.5:200;
map_y = 0:0.5:200;
[X, Y] = meshgrid(map_x, map_y);

figure('Position', [100, 100, 850, 800], 'Color', 'white');

% ---- 三座高斯山体（与 map.m 一致）----
noise = zeros(size(X));   % 绘图使用平滑地形，便于观察路径
for k = 1:size(model.mountains, 1)
    cx    = model.mountains(k, 1);
    cy    = model.mountains(k, 2);
    A     = model.mountains(k, 3);
    sigma = model.mountains(k, 4);
    z = A * exp(-((X - cx).^2 + (Y - cy).^2) / (2 * sigma^2)) + noise;
    z(z < 0) = 0;
    surf(X, Y, z, 'FaceColor', 'interp', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    hold on;
end

% ---- 球形禁飞区 ----
n_ball = 50;
[Xb, Yb, Zb] = sphere(n_ball);
c = model.ball.center; r = model.ball.radius;
bx = c(1) + Xb * r;
by = c(2) + Yb * r;
bz = c(3) + Zb * r; bz(bz < 0) = 0;
surf(bx, by, bz, 'FaceColor', [1 0 1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);

% ---- 圆柱体障碍物（带封顶）----
cyl_r = model.cylinder.radius;
n_cyl = 50;
[Xc, Yc, Zc] = cylinder(cyl_r, n_cyl);
theta = linspace(0, 2*pi, n_cyl);
for i = 1:size(model.cylinder.pos, 1)
    x0 = model.cylinder.pos(i, 1);
    y0 = model.cylinder.pos(i, 2);
    h  = model.cylinder.pos(i, 3);
    surf(x0 + Xc, y0 + Yc, h * Zc, 'FaceColor', [1 1 0], ...
        'EdgeColor', 'none', 'FaceAlpha', 1.0);
    fill3(x0 + cyl_r*cos(theta), y0 + cyl_r*sin(theta), repmat(h, size(theta)), ...
        [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
end

% ---- 规划出的红色路径 ----
% 用 pchip 平滑一下路径，画出的红线更连续美观
sp = smooth_path(best_path, 300);
plot3(sp(:,1), sp(:,2), sp(:,3), '-', 'Color', [1 0 0], 'LineWidth', 3);

% ---- 起点（绿色球）与终点（蓝色球）----
draw_marker_sphere(model.start, 5, [0 0.8 0]);   % 绿色起点
draw_marker_sphere(model.goal,  5, [0 0 1]);     % 蓝色终点

% ---- 坐标轴设置（与 map.m 一致）----
xlabel('X/m', 'FontSize', label_font_size);
ylabel('Y/m', 'FontSize', label_font_size);
zlabel('Z/m', 'FontSize', label_font_size);
title('混沌优化算法三维路径规划', 'FontSize', title_font_size);
xlim([0 200]); ylim([0 200]); zlim([0 200]);
% 三个坐标轴的刻度间隔均设为 20（显示 0,20,40,...,200）
xticks(0:20:200); yticks(0:20:200); zticks(0:20:200);
daspect([1 1 1]);
set(gca, 'FontSize', tick_font_size, 'Color', 'white');
grid on;
view(45, 35);

% 图例
legend_handles(1) = plot3(nan, nan, nan, '-', 'Color', [1 0 0], 'LineWidth', 3);
legend_handles(2) = plot3(nan, nan, nan, 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', [0 0.8 0], 'MarkerEdgeColor', 'k');
legend_handles(3) = plot3(nan, nan, nan, 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', [0 0 1], 'MarkerEdgeColor', 'k');
legend(legend_handles, {'规划路径', '起点', '终点'}, ...
    'Location', 'northeast', 'FontSize', 12);
hold off;
end

% =========================================================================
function draw_marker_sphere(center, radius, color)
% 在指定位置绘制一个实心小球（用于起点/终点标记）
[sx, sy, sz] = sphere(30);
surf(center(1) + sx * radius, center(2) + sy * radius, center(3) + sz * radius, ...
    'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
end

% =========================================================================
function sp = smooth_path(path, n)
% 用累积弦长参数化 + pchip 插值对折线路径平滑，便于绘制连续红线。
% 采用 pchip（保形分段三次插值）而非 spline，可避免样条在拐点处过冲，
% 使红线不会在起点附近出现回勾等视觉伪影。
m = size(path, 1);
if m < 3
    sp = path;
    return;
end
% 以累积弦长作为参数，插值更贴合实际路径几何
seg = sqrt(sum(diff(path, 1, 1).^2, 2));
t   = [0; cumsum(seg)];
tt  = linspace(t(1), t(end), n);
sp  = zeros(n, 3);
for d = 1:3
    sp(:, d) = pchip(t, path(:, d), tt);
end
end

% =========================================================================
function pts = densify_path(path, step)
% 将折线路径按步长 step 加密（供最终碰撞校验）
pts = path(1, :);
for i = 1:size(path, 1) - 1
    a = path(i, :); b = path(i + 1, :);
    d = norm(b - a);
    n = max(2, ceil(d / step));
    t = linspace(0, 1, n)';
    seg = a + t .* (b - a);
    pts = [pts; seg(2:end, :)]; %#ok<AGROW>
end
end
