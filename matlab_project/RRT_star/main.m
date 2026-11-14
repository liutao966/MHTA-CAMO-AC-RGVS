%% =========================================================================
% main.m —— RRT* 三维路径规划入口 (直接运行此文件)
% -------------------------------------------------------------------------
% 运行后依次弹出两张图:
%   1) 城市地图(city_map.m 平底+高楼)
%   2) 丘陵地图(Hill_map.m 7 座高斯山体)
% 各图含: 三维地图 + 红色 RRT* 最优路径 + 起点(绿)/终点(蓝) + 20 刻度轴。
% 环境与碰撞/指标经 world 结构按 map_type 分派, 算法主体(RRT*)完全一致。
% 也可单跑: run_rrt_star('city') 或 run_rrt_star('hill')。
% =========================================================================
clear;
clc;
close all;

% 城市地图(起终点 [10,10,60]→[190,190,60], 与 AVSS-RRT 城市地图一致)
run_rrt_star('city');

% 丘陵地图(起终点 [10,10,50]→[190,190,80])
run_rrt_star('hill');

% 障碍物地图(map.m: 3 座山 + 球形禁飞区 + 9 根圆柱, 起终点 [10,10,50]→[190,190,80])
run_rrt_star('obstacle');

%% ====================== 主程序函数 ======================
function run_rrt_star(map_type)
% RUN_RRT_STAR  在指定地图上运行 RRT* 三维路径规划
%   map_type - 'city' | 'hill' | (缺省/其他为旧山体地图)
if nargin < 1 || isempty(map_type), map_type = 'legacy'; end

%% --------------------- 1. 核心参数(算法专属, 保持原配置) ---------------------
params.v = 20;                 % 飞行速度 m/s
params.safe_dist = 2.5;        % 安全距离 m (与 HSMA / Improved RRT 统一)
params.max_height = 200;       % 最大飞行高度 m
params.min_height = 5;         % 最低飞行高度 m

params.max_iter = 5000;        % 最大迭代次数
params.step_size = 5;          % 扩展步长 m
params.goal_threshold = 5;     % 目标点到达阈值 m
params.rewire_radius = 15;     % 重布线邻域半径 m
params.goal_sample_rate = 0.1; % 目标点采样概率

params.x_range = [0, 200];
params.y_range = [0, 200];
params.z_range = [params.min_height, params.max_height];

params.energy_base = 1.2;      % 平飞基础能耗系数
params.energy_climb = 2.5;     % 爬升附加能耗系数
params.energy_descend = 0.8;   % 下降附加能耗系数

%% --------------------- 2. 环境构建(按地图分派) ---------------------
world = build_world(map_type, params);
start_point = world.start;
goal_point  = world.goal;

fprintf('地图类型: %s   起点: [%g %g %g]   终点: [%g %g %g]\n', ...
        map_type, start_point, goal_point);

%% --------------------- 3. RRT* 算法主体(与单地图版本完全一致) ---------------------
tree.node = start_point;   % 节点坐标
tree.parent = 0;           % 父节点索引
tree.cost = 0;             % 从起点到该节点的累计代价
node_count = 1;
goal_reached = false;
goal_node_idx = -1;

fprintf('RRT*算法开始迭代，最大迭代次数：%d\n', params.max_iter);
for iter = 1:params.max_iter
    % ---------- 步骤1：随机采样 ----------
    if rand() < params.goal_sample_rate
        sample_point = goal_point;
    else
        sample_point = [
            params.x_range(1) + rand()*(params.x_range(2)-params.x_range(1)), ...
            params.y_range(1) + rand()*(params.y_range(2)-params.y_range(1)), ...
            params.z_range(1) + rand()*(params.z_range(2)-params.z_range(1))
        ];
    end

    % ---------- 步骤2：最近节点 ----------
    node_array = reshape([tree.node], 3, [])';
    dist_array = vecnorm(node_array - sample_point, 2, 2);
    [~, nearest_idx] = min(dist_array);
    nearest_node = tree(nearest_idx).node;

    % ---------- 步骤3：扩展新节点 ----------
    direction = sample_point - nearest_node;
    direction = direction / norm(direction);
    new_node = nearest_node + direction * params.step_size;

    % ---------- 步骤4：路径段碰撞检测 ----------
    if check_segment_collision(nearest_node, new_node, params, world)
        continue;
    end

    % ---------- 步骤5：选择最优父节点 ----------
    new_node_array = reshape([tree.node], 3, [])';
    dist_to_new = vecnorm(new_node_array - new_node, 2, 2);
    neighbor_idx = find(dist_to_new <= params.rewire_radius);

    min_cost = tree(nearest_idx).cost + norm(new_node - nearest_node);
    best_parent_idx = nearest_idx;

    for i = 1:length(neighbor_idx)
        idx = neighbor_idx(i);
        temp_cost = tree(idx).cost + norm(new_node - tree(idx).node);
        if temp_cost < min_cost && ~check_segment_collision(tree(idx).node, new_node, params, world)
            min_cost = temp_cost;
            best_parent_idx = idx;
        end
    end

    % ---------- 步骤6：加入树中 ----------
    node_count = node_count + 1;
    tree(node_count).node = new_node;
    tree(node_count).parent = best_parent_idx;
    tree(node_count).cost = min_cost;

    % ---------- 步骤7：重布线邻域节点 ----------
    for i = 1:length(neighbor_idx)
        idx = neighbor_idx(i);
        temp_cost = tree(node_count).cost + norm(tree(idx).node - new_node);
        if temp_cost < tree(idx).cost && ~check_segment_collision(new_node, tree(idx).node, params, world)
            tree(idx).parent = node_count;
            tree(idx).cost = temp_cost;
        end
    end

    % ---------- 步骤8：检查是否到达终点 ----------
    if norm(new_node - goal_point) <= params.goal_threshold
        if ~check_segment_collision(new_node, goal_point, params, world)
            goal_reached = true;
            node_count = node_count + 1;
            tree(node_count).node = goal_point;
            tree(node_count).parent = node_count - 1;
            tree(node_count).cost = tree(node_count - 1).cost + norm(goal_point - new_node);
            goal_node_idx = node_count;
            fprintf('迭代第%d次，已找到到达终点的路径！\n', iter);
            break;
        end
    end

    if mod(iter, 500) == 0
        fprintf('已完成%d次迭代，当前树节点数：%d\n', iter, node_count);
    end
end

if ~goal_reached
    fprintf('达到最大迭代次数，未找到到达终点的有效路径！\n');
    return;
end

%% --------------------- 4. 路径回溯 ---------------------
fprintf('开始回溯最优路径...\n');
path = [];
current_idx = goal_node_idx;
while current_idx ~= 0
    path = [tree(current_idx).node; path];
    current_idx = tree(current_idx).parent;
end
path_num = size(path, 1);
fprintf('路径回溯完成，路径点数量：%d\n', path_num);

%% --------------------- 5. 多目标指标计算(6 目标, 与 HSMA/Improved RRT 统一) ---------------------
% 目标: [路径长度, 能耗, 不平滑度, 飞行时间, 最小安全距离, 威胁代价]
V_DRONE = 10;   % 统一飞行速度 m/s (与另外两个项目对齐)

path_seg_len = vecnorm(diff(path, 1, 1), 2, 2);
nP = size(path, 1);

% ---- 目标1: 路径长度 + 能耗所需的高度变化 ----
total_length = sum(path_seg_len);
height_change = sum(abs(diff(path(:,3), 1, 1)));

% ---- 所有路径点的障碍距离(目标5&6共用) ----
D_all = zeros(nP, 1);
for i = 1:nP
    D_all(i) = calc_min_obstacle_dist(path(i,:), world);
end

% ---- 目标6: 威胁代价 ----
safe_dist = params.safe_dist;
threat = 0; hard = 0; HARD_PEN = 1e5;
for i = 1:nP
    if D_all(i) < safe_dist
        threat = threat + (safe_dist - D_all(i));
        hard = hard + 1;
    end
end
threat = threat + hard * HARD_PEN;

% ---- 目标2: 能耗(论文模型) ----
turn_angle_sum = 0;
for i = 2:nP - 1
    v1 = path(i,:) - path(i-1,:);
    v2 = path(i+1,:) - path(i,:);
    n1 = norm(v1); n2 = norm(v2);
    if n1 > 1e-6 && n2 > 1e-6
        cos_a = max(min(dot(v1, v2) / (n1 * n2), 1), -1);
        turn_angle_sum = turn_angle_sum + acos(cos_a);
    end
end
accel_sum = sum(abs(diff(path_seg_len)));
energy = total_length * 0.1 + height_change * 0.3 + 0.2 * turn_angle_sum + 0.15 * accel_sum;

% ---- 目标3: 不平滑度 = 1 - mean(cosθ) ----
cos_sum = 0; cnt = 0;
for i = 2:nP - 1
    v1 = path(i,:) - path(i-1,:);
    v2 = path(i+1,:) - path(i,:);
    n1 = norm(v1); n2 = norm(v2);
    if n1 > 1e-6 && n2 > 1e-6
        cos_sum = cos_sum + dot(v1, v2) / (n1 * n2);
        cnt = cnt + 1;
    end
end
smooth = cos_sum / (cnt + 1e-9);

% ---- 目标4: 飞行时间 ----
flight_time = total_length / V_DRONE;

% ---- 目标5: 最小安全距离 ----
d_safe_min = max(min(D_all), 0);

fprintf('===================== 6 目标指标计算结果（%s） =====================\n', map_name_of(map_type));
fprintf('  目标1 - 路径长度     = %.2f m\n', total_length);
fprintf('  目标2 - 能耗         = %.2f Wh\n', energy);
fprintf('  目标3 - 不平滑度     = %.4f (越小越平滑)\n', 1 - smooth);
fprintf('  目标4 - 飞行时间     = %.2f s (V=%gm/s)\n', flight_time, V_DRONE);
fprintf('  目标5 - 最小安全距离 = %.2f m (路径上距障碍物最近点)\n', d_safe_min);
fprintf('  目标6 - 威胁代价     = %.2f (0 表示全程无碰撞)\n', threat);
fprintf('===================================================================\n');

%% --------------------- 6. 地图与路径可视化 ---------------------
draw_result(world, path, start_point, goal_point, map_type);
end

%% ====================== 环境构建 ======================
function world = build_world(map_type, params)
mt = []; no_fly = []; cyl_r = 0; cyl_pos = [];

switch map_type
    case 'city'
        buildings = build_city_buildings();
        start_p = [10, 10, 60];
        goal_p  = [190, 190, 60];
    case 'hill'
        buildings = [];
        mt = [ 40, 140,  90, 30;
               140,  40,  90, 30;
               140, 140, 120, 30;
                90,  90, 120, 30;
                40,  40,  50, 30;
               180,  90, 100, 30;
                90, 180, 100, 30];
        start_p = [10, 10, 50];
        goal_p  = [190, 190, 80];
    case 'obstacle'
        % 障碍物地图(map.m): 3 座高斯山 + 球形禁飞区(r=50) + 9 根圆柱(r=5,h=100)
        buildings = [];
        mt = [ 40, 140,  90, 30;
               140,  40,  90, 30;
               140, 140, 120, 30];
        no_fly = [100, 100, 0, 50];
        cyl_r  = 5;
        cyl_pos = [ 40,  70, 100;  70,  40, 100;  50,  50, 100;
                    90, 160, 100; 160,  80, 100;  80, 180, 100;
                   100, 180, 100; 180,  60, 100; 180, 100, 100];
        start_p = [10, 10, 50];
        goal_p  = [190, 190, 80];
    otherwise
        buildings = [];
        mt = []; no_fly = []; cyl_r = 0; cyl_pos = [];
        start_p = [10, 10, 50];
        goal_p  = [190, 190, 80];
end

world.type = map_type;
world.start = start_p;
world.goal  = goal_p;
world.buildings = buildings;
world.mountains = mt;
world.no_fly    = no_fly;
world.cyl_r     = cyl_r;
world.cyl_pos   = cyl_pos;

% 山体地形网格(仅 city 之外需要; 确定性、无噪声, 保证跨算法一致)
if ~strcmp(map_type, 'city')
    map_x = 0:0.1:200;
    map_y = 0:0.1:200;
    [world.X, world.Y] = meshgrid(map_x, map_y);
    world.Z = zeros(size(world.X));
    for k = 1:size(mt, 1)
        g = mt(k,3) * exp(-((world.X - mt(k,1)).^2 + (world.Y - mt(k,2)).^2) / (2*mt(k,4)^2));
        g(g < 0) = 0;
        world.Z = max(world.Z, g);
    end
else
    world.X = []; world.Y = []; world.Z = [];
end
end

%% ====================== 碰撞检测(按类型分派) ======================
function is_collision = check_collision(point, params, world)
x = point(1); y = point(2); z = point(3);
is_collision = true;

% 1. 边界
if x < params.x_range(1) || x > params.x_range(2) ...
    || y < params.y_range(1) || y > params.y_range(2) ...
    || z < params.z_range(1) || z > params.z_range(2)
    return;
end

s = params.safe_dist;
if strcmp(world.type, 'city')
    % 2a. 地面(平底, z=0)
    if z < s
        return;
    end
    % 2b. 矩形高楼(外扩 s)(含 z<0 兜底)
    for i = 1:size(world.buildings, 1)
        b = world.buildings(i, :);
        if x > b(1)-s && x < b(2)+s && y > b(3)-s && y < b(4)+s && z > -s && z < b(5)+s
            return;
        end
    end
    is_collision = false;
    return;
end

% (非城市) 2. 山体
mountain_h = interp2(world.X, world.Y, world.Z, x, y, 'linear', 0);
if z < mountain_h + s
    return;
end

% 3. 半球禁飞区(存在时)
if ~isempty(world.no_fly)
    dist_to_nofly = sqrt((x - world.no_fly(1))^2 + (y - world.no_fly(2))^2 + (z - world.no_fly(3))^2);
    if dist_to_nofly < world.no_fly(4) + s && z >= world.no_fly(3)
        return;
    end
end

% 4. 圆柱体(存在时)
for i = 1:size(world.cyl_pos, 1)
    cx = world.cyl_pos(i,1); cy = world.cyl_pos(i,2); cz_max = world.cyl_pos(i,3);
    dist_xy = sqrt((x - cx)^2 + (y - cy)^2);
    if dist_xy < world.cyl_r + s && z <= cz_max + s && z >= 0
        return;
    end
end

is_collision = false;
end

function is_segment_collision = check_segment_collision(p1, p2, params, world)
is_segment_collision = true;
seg_length = norm(p2 - p1);
interp_num = max(ceil(seg_length / 0.5), 2);
interp_x = linspace(p1(1), p2(1), interp_num);
interp_y = linspace(p1(2), p2(2), interp_num);
interp_z = linspace(p1(3), p2(3), interp_num);
for i = 1:interp_num
    if check_collision([interp_x(i), interp_y(i), interp_z(i)], params, world)
        return;
    end
end
is_segment_collision = false;
end

%% ====================== 最近障碍距离(指标用) ======================
function min_dist = calc_min_obstacle_dist(point, world)
x = point(1); y = point(2); z = point(3);
dist_list = [];

if strcmp(world.type, 'city')
    dist_list = [dist_list, z];   % 地面
    for i = 1:size(world.buildings, 1)
        dist_list = [dist_list, dist_to_box(point, world.buildings(i,:))]; %#ok<AGROW>
    end
else
    mountain_h = interp2(world.X, world.Y, world.Z, x, y, 'linear', 0);
    dist_list = [dist_list, z - mountain_h]; %#ok<AGROW>
    if ~isempty(world.no_fly)
        dist_nofly = norm(point - world.no_fly(1:3)) - world.no_fly(4);
        dist_list = [dist_list, dist_nofly]; %#ok<AGROW>
    end
    for i = 1:size(world.cyl_pos, 1)
        cx = world.cyl_pos(i,1); cy = world.cyl_pos(i,2); cz_max = world.cyl_pos(i,3);
        dist_xy = sqrt((x - cx)^2 + (y - cy)^2);
        dist_cyl_xy = dist_xy - world.cyl_r;
        if z > cz_max
            dist_cyl_z = z - cz_max;
        else
            dist_cyl_z = 0;
        end
        dist_cyl = sqrt(dist_cyl_xy^2 + dist_cyl_z^2);
        dist_list = [dist_list, dist_cyl]; %#ok<AGROW>
    end
end
min_dist = min(dist_list);
end

function dd = dist_to_box(pt, b)
dx = max(max(b(1) - pt(1), pt(1) - b(2)), 0);
dy = max(max(b(3) - pt(2), pt(2) - b(4)), 0);
if pt(3) < 0
    dz = -pt(3);
elseif pt(3) > b(5)
    dz = pt(3) - b(5);
else
    dz = 0;
end
dd = sqrt(dx^2 + dy^2 + dz^2);
end

%% ====================== 可视化(按类型分派) ======================
function draw_result(world, path, start_point, goal_point, map_type)
tick_font_size  = 20;
label_font_size = 20;
title_font_size = 16;

figure("Position",[100,100,1000,800], "Color", "white");
hold on; grid on; axis equal; view(3);

if strcmp(world.type, 'city')
    % 平底地面 + 矩形高楼
    patch([0 200 200 0],[0 0 200 200],[1e-3 1e-3 1e-3 1e-3],[0.82 0.82 0.78],'EdgeColor','none','FaceAlpha',1.0);
    for i = 1:size(world.buildings, 1)
        draw_building_box(world.buildings(i,:));
    end
else
    % 山体(无噪声, 确定性)
    for k = 1:size(world.mountains, 1)
        zk = world.mountains(k,3) * exp(-((world.X-world.mountains(k,1)).^2 + (world.Y-world.mountains(k,2)).^2)/(2*world.mountains(k,4)^2));
        zk(zk<0) = 0;
        surf(world.X, world.Y, zk, "FaceColor","interp","EdgeColor","none","FaceAlpha",0.9);
    end
    % 半球禁飞区(存在时)
    if ~isempty(world.no_fly)
        n_ball = 50;
        [X_ball,Y_ball,Z_ball] = sphere(n_ball);
        ball_x = world.no_fly(1) + X_ball*world.no_fly(4);
        ball_y = world.no_fly(2) + Y_ball*world.no_fly(4);
        ball_z = world.no_fly(3) + Z_ball*world.no_fly(4);
        ball_z(ball_z<0) = 0;
        surf(ball_x, ball_y, ball_z, "FaceColor",[1,0,1],"EdgeColor","none","FaceAlpha",0.8);
    end
    % 圆柱体(存在时) —— 侧面 + 顶盖 + 底盖
    if size(world.cyl_pos,1) > 0 && world.cyl_r > 0
        n_cyl = 50;
        [X_cyl,Y_cyl,Z_cyl] = cylinder(world.cyl_r, n_cyl);
        theta = linspace(0, 2*pi, n_cyl);
        top_x = world.cyl_r * cos(theta);
        top_y = world.cyl_r * sin(theta);
        for i = 1:size(world.cyl_pos,1)
            p = world.cyl_pos(i,:);
            surf(p(1)+X_cyl, p(2)+Y_cyl, p(3)*Z_cyl, "FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",0.9);
            fill3(p(1)+top_x, p(2)+top_y, repmat(p(3),n_cyl,1), [1,1,0], "EdgeColor","none","FaceAlpha",0.9);
            fill3(p(1)+top_x, p(2)+top_y, zeros(n_cyl,1),     [1,1,0], "EdgeColor","none","FaceAlpha",0.9);
        end
    end
end

plot3(path(:,1), path(:,2), path(:,3), 'r-', 'LineWidth', 3, 'DisplayName', 'RRT*最优路径');
scatter3(start_point(1), start_point(2), start_point(3), 200, 'g', 'filled', 'DisplayName', '起点');
scatter3(goal_point(1), goal_point(2), goal_point(3), 200, 'b', 'filled', 'DisplayName', '终点');

xlim([0,200]); ylim([0,200]); zlim([0,200]); axis manual;

xlabel("X/m", "FontSize", label_font_size);
ylabel("Y/m", "FontSize", label_font_size);
zlabel("Z/m", "FontSize", label_font_size);
title(sprintf("3D地图RRT*全局路径规划结果（%s）", map_name_of(map_type)), "FontSize", title_font_size);

set(gca, "FontSize", tick_font_size, "Color", "white");
set(gca, 'XTick', 0:20:200, 'YTick', 0:20:200, 'ZTick', 0:20:200);
daspect([1 1 1]);
legend('Location', 'best', 'FontSize', tick_font_size);
hold off;
end

function draw_building_box(b)
x0 = b(1); x1 = b(2); y0 = b(3); y1 = b(4); h = b(5);
t = min(max(h/150, 0), 1);
col = [0.4 - 0.15*t, 0.48 - 0.10*t, 0.72 + 0.10*t];
verts_x = [x0, x1, x1, x0, x0, x1, x1, x0];
verts_y = [y0, y0, y1, y1, y0, y0, y1, y1];
verts_z = [0, 0, 0, 0, h, h, h, h];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Vertices',[verts_x(:),verts_y(:),verts_z(:)],'Faces',faces, ...
      'FaceColor',col,'EdgeColor','k','LineWidth',0.5,'FaceAlpha',1.0);
patch([x0 x1 x1 x0],[y0 y0 y1 y1],[h h h h],col.*1.2,'EdgeColor','none','FaceAlpha',1.0);
end

function str = map_name_of(map_type)
switch map_type
    case 'city',     str = '城市地图';
    case 'hill',     str = '丘陵地图';
    case 'obstacle', str = '障碍物地图';
    otherwise,       str = '山体障碍地图';
end
end

%% ====================== 城市高楼(复现 map/city_map.m) ======================
function B = build_city_buildings()
nCol = 6;  nRow = 6;
margin = 14;
gap    = 6;
rng('default'); rng(26);
xc = linspace(margin, 200 - margin, nCol + 1);
yc = linspace(margin, 200 - margin, nRow + 1);
cellWx = xc(2) - xc(1);
cellWy = yc(2) - yc(1);
bldWx = cellWx - gap;
bldWy = cellWy - gap;
roadRow = 3;
roadCol = 3;
plaza = rand(nRow, nCol) < 0.15;
B = [];
for r = 1:nRow
    for c = 1:nCol
        if r == roadRow || c == roadCol || plaza(r, c)
            continue;
        end
        cx = (xc(c) + xc(c+1)) / 2;
        cy = (yc(r) + yc(r+1)) / 2;
        x0 = cx - bldWx/2;  x1 = cx + bldWx/2;
        y0 = cy - bldWy/2;  y1 = cy + bldWy/2;
        h  = 30 + 70 * rand;
        if rand < 0.12
            h = 120 + 30 * rand;
        end
        B(end+1, :) = [x0 x1 y0 y1 h]; %#ok<AGROW>
    end
end
end