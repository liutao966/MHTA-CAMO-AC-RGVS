function AVSS_RRT()
% ================================================================
%  AVSS-RRT (Adaptive Variable Step Size RRT) 三维无人机路径规划
% ----------------------------------------------------------------
%  参考文献:
%   [1] H. Zhang et al., "An Improved Goal-bias RRT algorithm for
%       Unmanned Aerial Vehicle Path Planning," IEEE ICMA 2024.
%   [2] "Hierarchical Control Framework for UAV Hybrid Path Planning
%       in Unstructured Terrain" (AVSS-RRT 对比算法)。
%
%  算法要点:
%   1) Goal-bias 采样: 以概率 p_goal 直接采样终点, 引导树朝目标扩展;
%   2) 自适应可变步长 (Adaptive Variable Step Size, AVSS):
%      步长随"离最近障碍物距离"和"离目标距离"自适应变化——
%      空旷区域步长大(快速探索), 障碍物附近步长小(精细避障);
%   3) 无人机机体约束: 碰撞检测保留安全距离, 最终用三次样条平滑。
%
%  地形: 采用 map.m 中定义的三座高斯山体、球形禁飞区、九根圆柱障碍。
%  起点 [10,10,50] (绿色球), 终点 [190,190,80] (蓝色球), 红色规划路线。
% ================================================================
clear; clc; close all;
rng(1);   % 固定随机种子, 结果可复现

%% ------------------ 1. 地图与障碍物参数 (与 map.m 一致) ----------
world.xlim = [0 200];
world.ylim = [0 200];
world.zlim = [0 200];

% --- 三座高斯山体 [x0, y0, A(峰值), sigma] ---
world.mountains = [ 40 140  90 30;
                   140  40  90 30;
                   140 140 120 30];

% --- 球形禁飞区 [cx, cy, cz, r] ---
world.sphere = [100 100 0 50];

% --- 九根圆柱障碍 [x0, y0, height], 半径统一 ---
world.cyl_r = 5;
world.cylinders = [ 40  70 100;
                    70  40 100;
                    50  50 100;
                    90 160 100;
                   160  80 100;
                    80 180 100;
                   100 180 100;
                   180  60 100;
                   180 100 100];

world.safe = 3.0;        % 无人机安全距离(机体约束)

%% ------------------ 2. 起点 / 终点 -------------------------------
start = [ 10  10 50];
goal  = [190 190 80];

%% ------------------ 3. AVSS-RRT 参数 ----------------------------
params.max_iter   = 8000;   % 最大迭代次数
params.p_goal     = 0.15;   % Goal-bias 采样概率
params.step_min   = 3;      % 最小步长(障碍物附近)
params.step_max   = 18;     % 最大步长(空旷区域)
params.goal_tol   = 8;      % 到达终点的容差
params.edge_res   = 1.5;    % 边碰撞检测的采样分辨率

%% ------------------ 4. 运行 AVSS-RRT -----------------------------
fprintf('AVSS-RRT 开始规划 ...\n');
tic;
[path, tree, success] = avss_rrt_plan(start, goal, world, params);
t_plan = toc;

if ~success
    warning('未能在最大迭代次数内找到路径, 请调整参数或检查地图。');
    if isempty(path); return; end
end
fprintf('规划完成: 用时 %.3f s, 节点数 %d, 原始路径点 %d 个。\n', ...
        t_plan, size(tree.nodes,1), size(path,1));

%% ------------------ 5. 三次样条平滑 (机体约束下的轨迹优化) --------
smooth_path = smooth_path_cubic(path);
% 检查平滑后路径的最小障碍间隙(样条可能略微偏离原折线)
min_clear = min(arrayfun(@(i) dist_to_obstacles(smooth_path(i,:), world), ...
                         1:size(smooth_path,1)));
fprintf('平滑后路径点 %d 个, 路径长度 %.2f m, 最小障碍间隙 %.2f m。\n', ...
        size(smooth_path,1), path_length(smooth_path), min_clear);

%% ------------------ 6. 绘制地图 + 规划结果 -----------------------
draw_world(world);
hold on;

% 规划出的红色路线
plot3(smooth_path(:,1), smooth_path(:,2), smooth_path(:,3), ...
      'r-', 'LineWidth', 3);

% 起点(绿色球) / 终点(蓝色球)
draw_ball(start, 4, [0 0.8 0]);
draw_ball(goal,  4, [0 0 1]);

legend_h = [ plot3(nan,nan,nan,'r-','LineWidth',3), ...
             plot3(nan,nan,nan,'o','MarkerFaceColor',[0 0.8 0], ...
                   'MarkerEdgeColor','none','MarkerSize',10), ...
             plot3(nan,nan,nan,'o','MarkerFaceColor',[0 0 1], ...
                   'MarkerEdgeColor','none','MarkerSize',10)];
legend(legend_h, {'AVSS-RRT 规划路径','起点','终点'}, ...
       'Location','northeast','FontSize',12);

title('AVSS-RRT 三维路径规划', 'FontSize', 16);
view(45, 30);
hold off;
end

% ================================================================
%                         核心算法函数
% ================================================================
function [path, tree, success] = avss_rrt_plan(start, goal, world, params)
% AVSS-RRT 主循环
tree.nodes  = start;      % 节点坐标 (N x 3)
tree.parent = 0;          % 父节点索引 (0 表示根)
success = false;
path = [];

for it = 1:params.max_iter
    % ---- (1) Goal-bias 采样 ----
    if rand < params.p_goal
        x_rand = goal;
    else
        x_rand = sample_free(world);
    end

    % ---- (2) 最近节点 ----
    [x_near, idx_near] = nearest_node(tree.nodes, x_rand);

    % ---- (3) 自适应可变步长 (AVSS) ----
    step = adaptive_step(x_near, goal, world, params);

    % ---- (4) 向采样点扩展 ----
    dir = x_rand - x_near;
    dist = norm(dir);
    if dist < 1e-6; continue; end
    x_new = x_near + (dir / dist) * min(step, dist);

    % 越界检查
    if ~in_bounds(x_new, world); continue; end

    % ---- (5) 边碰撞检测(含安全距离机体约束) ----
    if ~collision_free(x_near, x_new, world, params.edge_res)
        continue;
    end

    % ---- (6) 加入新节点 ----
    tree.nodes(end+1,:)  = x_new;
    tree.parent(end+1,1) = idx_near;

    % ---- (7) 尝试直连终点 ----
    if norm(x_new - goal) <= params.goal_tol
        if collision_free(x_new, goal, world, params.edge_res)
            tree.nodes(end+1,:)  = goal;
            tree.parent(end+1,1) = size(tree.nodes,1)-1;
            success = true;
            break;
        end
    end
end

% ---- 回溯路径 ----
if success
    idx = size(tree.nodes,1);
    while idx ~= 0
        path = [tree.nodes(idx,:); path]; %#ok<AGROW>
        idx = tree.parent(idx);
    end
end
end

% ----------------------------------------------------------------
function step = adaptive_step(x, goal, world, params)
% 自适应可变步长: 综合"离最近障碍距离"与"离目标距离"计算步长。
%   - 距障碍越近 -> 步长越小(精细避障)
%   - 距障碍越远 -> 步长越大(快速探索)
%   - 接近目标时收缩步长, 避免越过终点
d_obs  = dist_to_obstacles(x, world);          % 到最近障碍表面的距离
d_goal = norm(x - goal);

% 障碍距离映射到 [0,1]: 参考尺度取 step_max
r_obs  = min(max((d_obs - world.safe) / params.step_max, 0), 1);

% 基于障碍距离的自适应步长
step = params.step_min + (params.step_max - params.step_min) * r_obs;

% 接近目标时进一步收缩
step = min(step, max(d_goal, params.step_min));
end

% ----------------------------------------------------------------
function x = sample_free(world)
% 在自由空间内均匀采样(拒绝落在障碍内的点)
for k = 1:50
    x = [world.xlim(1) + diff(world.xlim)*rand, ...
         world.ylim(1) + diff(world.ylim)*rand, ...
         world.zlim(1) + diff(world.zlim)*rand];
    if dist_to_obstacles(x, world) > world.safe
        return;
    end
end
% 兜底: 返回随机点(碰撞检测会在边扩展阶段过滤)
end

% ----------------------------------------------------------------
function [x_near, idx] = nearest_node(nodes, x)
d = sum((nodes - x).^2, 2);
[~, idx] = min(d);
x_near = nodes(idx,:);
end

% ----------------------------------------------------------------
function tf = in_bounds(x, world)
tf = x(1) >= world.xlim(1) && x(1) <= world.xlim(2) && ...
     x(2) >= world.ylim(1) && x(2) <= world.ylim(2) && ...
     x(3) >= world.zlim(1) && x(3) <= world.zlim(2);
end

% ----------------------------------------------------------------
function tf = collision_free(a, b, world, res)
% 沿线段 a->b 等距采样, 检查每个点是否满足安全距离约束
L = norm(b - a);
n = max(2, ceil(L / res));
t = linspace(0, 1, n)';
pts = a + t .* (b - a);
for i = 1:n
    if dist_to_obstacles(pts(i,:), world) <= world.safe
        tf = false; return;
    end
end
tf = true;
end

% ----------------------------------------------------------------
function d = dist_to_obstacles(x, world)
% 返回点 x 到"最近障碍表面"的有符号距离(在障碍内部返回负值)。
% 障碍包含: 高斯山体(地形高度场)、球形禁飞区、圆柱群。
d = inf;

% ---- 高斯山体地形: 障碍为 z <= terrain_height(x,y) 的区域 ----
h = terrain_height(x(1), x(2), world);
d = min(d, x(3) - h);        % 高于地形为正, 陷入地形为负

% ---- 球形禁飞区 ----
s = world.sphere;
d_sphere = norm(x - s(1:3)) - s(4);
d = min(d, d_sphere);

% ---- 圆柱障碍 (有限高度圆柱) ----
for i = 1:size(world.cylinders,1)
    c = world.cylinders(i,:);
    dr = norm(x(1:2) - c(1:2)) - world.cyl_r;   % 径向距离(负=在柱内)
    if x(3) <= c(3)                              % 高度在圆柱范围内
        dz = -inf;                               % 侧面主导
        d_cyl = dr;
    else                                         % 高于柱顶
        dz = x(3) - c(3);
        if dr < 0
            d_cyl = dz;                          % 位于柱顶正上方
        else
            d_cyl = hypot(dr, dz);               % 柱顶边缘外侧
        end
    end
    d = min(d, d_cyl);
end
end

% ----------------------------------------------------------------
function h = terrain_height(x, y, world)
% 三座高斯山体叠加后的地形高度(取各山体在该点的最大高度)
h = 0;
for i = 1:size(world.mountains,1)
    m = world.mountains(i,:);
    zi = m(3) * exp(-((x - m(1))^2 + (y - m(2))^2) / (2*m(4)^2));
    h = max(h, zi);
end
end

% ================================================================
%                       路径平滑 & 工具函数
% ================================================================
function sp = smooth_path_cubic(path)
% 三次样条插值平滑路径(Not-A-Knot), 以弦长为参数化变量。
% 先剔除重复/过近的路径点, 避免样条参数出现重复值。
seg = sqrt(sum(diff(path).^2, 2));
keep = [true; seg > 1e-6];
path = path(keep, :);

if size(path,1) < 3
    sp = path; return;
end
% 累积弦长参数(严格递增)
d = [0; cumsum(sqrt(sum(diff(path).^2, 2)))];
tt = linspace(0, d(end), max(200, size(path,1)*20))';
sp = [spline(d, path(:,1), tt), ...
      spline(d, path(:,2), tt), ...
      spline(d, path(:,3), tt)];
end

% ----------------------------------------------------------------
function L = path_length(p)
L = sum(sqrt(sum(diff(p).^2, 2)));
end

% ================================================================
%                          绘图函数
% ================================================================
function draw_world(world)
% 复现 map.m 的三维障碍物地图
tick_font_size  = 20;
label_font_size = 20;

map_x = 0:0.5:200;
map_y = 0:0.5:200;
[X, Y] = meshgrid(map_x, map_y);

figure('Position',[100 100 900 800], 'Color','white');

% --- 三座高斯山体 ---
for i = 1:size(world.mountains,1)
    m = world.mountains(i,:);
    Z = m(3) * exp(-((X - m(1)).^2 + (Y - m(2)).^2) / (2*m(4)^2));
    Z(Z < 0) = 0;
    surf(X, Y, Z, 'FaceColor','interp','EdgeColor','none','FaceAlpha',0.9);
    hold on;
end

% --- 球形禁飞区 ---
s = world.sphere;
[Xs, Ys, Zs] = sphere(50);
bx = s(1) + Xs*s(4);  by = s(2) + Ys*s(4);  bz = s(3) + Zs*s(4);
bz(bz < 0) = 0;
surf(bx, by, bz, 'FaceColor',[1 0 1],'EdgeColor','none','FaceAlpha',1.0);

% --- 圆柱障碍(带封顶) ---
[Xc, Yc, Zc] = cylinder(world.cyl_r, 50);
theta = linspace(0, 2*pi, 50);
for i = 1:size(world.cylinders,1)
    c = world.cylinders(i,:);
    surf(c(1)+Xc, c(2)+Yc, c(3)*Zc, ...
         'FaceColor',[1 1 0],'EdgeColor','none','FaceAlpha',1.0);
    tx = c(1) + world.cyl_r*cos(theta);
    ty = c(2) + world.cyl_r*sin(theta);
    tz = repmat(c(3), size(tx));
    fill3(tx, ty, tz, [1 1 0], 'EdgeColor','none','FaceAlpha',1.0);
end

% --- 坐标轴设置 ---
xlabel('X/m','FontSize',label_font_size);
ylabel('Y/m','FontSize',label_font_size);
zlabel('Z/m','FontSize',label_font_size);
xlim([0 200]); ylim([0 200]); zlim([0 200]);
% 坐标轴刻度间隔设为 20 (0,20,40,...,200)
xticks(0:20:200); yticks(0:20:200); zticks(0:20:200);
daspect([1 1 1]);
set(gca,'FontSize',tick_font_size,'Color','white');
grid on;
end

% ----------------------------------------------------------------
function draw_ball(center, r, color)
% 绘制一个实心小球标记起点/终点
[Xs, Ys, Zs] = sphere(20);
surf(center(1)+Xs*r, center(2)+Ys*r, center(3)+Zs*r, ...
     'FaceColor',color,'EdgeColor','none','FaceAlpha',1.0);
end
