clear; clc; close all;
% =========================================================================
% main.m  —— NMOPOS 多目标粒子群三维路径规划 入口脚本
% -------------------------------------------------------------------------
% 直接运行即可，依次在两张地图上执行 NMOPOS 并各弹出两张图(收敛曲线 + 路径)：
%   1) 城市地图(city_map.m 的平底 + 矩形高楼)
%   2) 丘陵地图(Hill_map.m 的 7 座高斯山体)
% 也可单跑: run_nmopos('city') 或 run_nmopos('hill')。
% 环境由 build_environment(map_type) 统一构建，经 env 结构分派碰撞/绘图，
% 算法本体(NMOPOS 主循环)不受地图类型影响。
% =========================================================================
% 城市地图起终点 [10,10,60]→[190,190,60] 与 AVSS-RRT 城市地图保持一致。
% 丘陵地图起终点 [10,10,50]→[190,190,80]。
run_nmopos('city');
run_nmopos('hill');
run_nmopos('obstacle');

%% ====================== 主程序函数 ======================
function run_nmopos(map_type)
% RUN_NMOPOS  在指定地图上运行 NMOPOS 多目标路径规划
%   map_type - 'city' | 'hill' | (缺省/其他为旧地图)

if nargin < 1 || isempty(map_type), map_type = 'legacy'; end

%% ====================== 1. 环境与参数初始化 ======================
env = build_environment(map_type);     % 统一环境结构(含起终点/障碍/安全距离)
map_range = env.map_range;             % [0 200]
safe_dist = env.safe_dist;             % 安全距离 (m)
drone_speed = 10;                      % 无人机速度 (m/s), 与 HSMA/Improved RRT 统一
% 多目标权重（仅用于最终折中解的选择，不参与优化过程）
% 6 目标: [长度, 能耗, 不平滑度, 飞行时间, -安全距离, 威胁代价]
weights = [0.3, 0.25, 0.15, 0.1, 0.1, 0.1];
start_pos = env.start;
goal_pos  = env.goal;
fprintf('地图类型: %s   起点: [%g %g %g]   终点: [%g %g %g]\n', ...
        env.type, start_pos, goal_pos);

%% ====================== 2. NMOPOS 参数设置 ======================
num_ctrl_pts = 14;          % 控制点总数（首尾固定，中间12个可移动）
num_interp_pts = 150;       % 插值点数量
% NMOPOS 参数
n_particles = 60;
max_iter = 300;
archive_size = 50;
inertia_w = 0.5;
c1 = 1.5;
c2 = 1.5;
mutate_prob = 0.1;
mutate_strength = 5.0;
% 优化变量边界：中间控制点的坐标范围
lb = [0, 0, 10];
ub = [200, 200, 180];
dim = (num_ctrl_pts - 2) * 3;
% 初始化粒子位置和速度
pos = zeros(n_particles, dim);
vel = zeros(n_particles, dim);
for i = 1:n_particles
    t = linspace(0, 1, num_ctrl_pts)';
    base_x = start_pos(1) + (goal_pos(1)-start_pos(1)) * t;
    base_y = start_pos(2) + (goal_pos(2)-start_pos(2)) * t;
    base_z = start_pos(3) + (goal_pos(3)-start_pos(3)) * t;
    ctrl_pts = [base_x(2:end-1), base_y(2:end-1), base_z(2:end-1)];
    ctrl_pts = ctrl_pts + randn(size(ctrl_pts)) * 8;
    ctrl_pts = max(lb, min(ub, ctrl_pts));
    pos(i, :) = reshape(ctrl_pts, 1, dim);
    vel(i, :) = randn(1, dim) * 3;
end
pbest_pos = pos;
pbest_obj = inf(n_particles, 6);
archive = struct('pos', [], 'obj', []);
%% ====================== 3. 适应度函数（含惩罚项） ======================
fitness_func = @(ctrl_vars) evaluate_path_with_penalty(ctrl_vars, start_pos, goal_pos, ...
    num_ctrl_pts, num_interp_pts, env, drone_speed);
to_ctrl_pts = @(vars) [start_pos; reshape(vars, [], 3); goal_pos];
%% ====================== 4. NMOPOS 主循环 ======================
fprintf('正在执行NMOPOS多目标优化（含安全距离惩罚）...\n');
fitness_history = zeros(max_iter, 1);
for iter = 1:max_iter
    obj_all = zeros(n_particles, 6);
    for i = 1:n_particles
        obj_all(i, :) = fitness_func(pos(i, :));
    end
    
    % 更新个人最优
    for i = 1:n_particles
        if any(pbest_obj(i,:) == inf) || dominates(obj_all(i,:), pbest_obj(i,:))
            pbest_pos(i, :) = pos(i, :);
            pbest_obj(i, :) = obj_all(i, :);
        end
    end
    
    % 更新全局存档
    current_archive = [archive.pos; pos];
    current_obj = [archive.obj; obj_all];
    [~, non_dominated] = pareto_filter(current_archive, current_obj);
    archive.pos = current_archive(non_dominated, :);
    archive.obj = current_obj(non_dominated, :);
    
    if size(archive.pos, 1) > archive_size
        archive = truncate_archive(archive, archive_size);
    end
    
    % 记录收敛指标
    if ~isempty(archive.obj)
        fitness_history(iter) = mean(sum(archive.obj, 2));
    else
        fitness_history(iter) = inf;
    end
    
    % 引导者选择与更新
    if ~isempty(archive.obj)
        crowding = crowding_distance(archive.obj);
        if all(isinf(crowding)) || any(isnan(crowding))
            prob = ones(1, size(archive.obj,1)) / size(archive.obj,1);
        else
            crowding(isinf(crowding)) = max(crowding(isfinite(crowding)));
            prob = crowding / (sum(crowding) + eps);
        end
        
        for i = 1:n_particles
            idx = roulette_wheel(prob);
            if isempty(idx) || idx > size(archive.pos,1)
                idx = randi(size(archive.pos,1));
            end
            gbest = archive.pos(idx, :);
            
            r1 = rand(1, dim); r2 = rand(1, dim);
            vel(i, :) = inertia_w * vel(i, :) + ...
                        c1 * r1 .* (pbest_pos(i, :) - pos(i, :)) + ...
                        c2 * r2 .* (gbest - pos(i, :));
            pos(i, :) = pos(i, :) + vel(i, :);
            
            % 边界裁剪
            for d = 1:dim
                dim_idx = mod(d-1, 3) + 1;
                lb_val = lb(dim_idx);
                ub_val = ub(dim_idx);
                pos(i, d) = max(lb_val, min(ub_val, pos(i, d)));
            end
            
            if rand() < mutate_prob
                pos(i, :) = pos(i, :) + randn(1, dim) * mutate_strength;
                for d = 1:dim
                    dim_idx = mod(d-1, 3) + 1;
                    lb_val = lb(dim_idx);
                    ub_val = ub(dim_idx);
                    pos(i, d) = max(lb_val, min(ub_val, pos(i, d)));
                end
            end
        end
    end
    
    if mod(iter, 20) == 0 || iter == max_iter
        fprintf('迭代 %d/%d, 存档非支配解数量: %d\n', iter, max_iter, size(archive.pos, 1));
    end
end
%% ====================== 5. 从存档中选择最终路径 ======================
if isempty(archive.obj)
    error('未找到可行解，请调整算法参数或安全距离阈值');
end
weighted_sum = archive.obj * weights(:);
[~, final_idx] = min(weighted_sum);
optimal_ctrl_vars = archive.pos(final_idx, :);
optimal_ctrl_pts = to_ctrl_pts(optimal_ctrl_vars);
optimal_path = bspline_path(optimal_ctrl_pts, num_interp_pts);
smooth_path = smooth_path_points(optimal_path, 0.5);
% 强制起点和终点严格等于给定值（双重保险）
smooth_path(1,:) = start_pos;
smooth_path(end,:) = goal_pos;
% 可行性验证（使用原始指标，无惩罚）
[feasible, min_safe] = check_path_feasibility(smooth_path, env);
if feasible
    fprintf('\n[%s] 最终路径可行性：可行（最小安全距离：%.2fm ≥ %.2fm）\n', env.type, min_safe, safe_dist);
else
    fprintf('\n[%s] 最终路径可行性：不可行（最小安全距离：%.2fm < %.2fm）\n', env.type, min_safe, safe_dist);
end
% 正确获取六项指标(无惩罚)
[path_len, energy, roughness, flight_time, d_safe_min, threat] = compute_path_metrics(smooth_path, env, drone_speed);
%% ====================== 6. 输出结果 ======================
fprintf('\n==================== NMOPOS 多目标优化结果（%s） ====================\n', map_name_of(env));
fprintf('  目标1 - 路径长度     = %.2f m\n', path_len);
fprintf('  目标2 - 能耗         = %.2f Wh\n', energy);
fprintf('  目标3 - 不平滑度     = %.4f (越小越平滑)\n', roughness);
fprintf('  目标4 - 飞行时间     = %.2f s (V=%gm/s)\n', flight_time, drone_speed);
fprintf('  目标5 - 最小安全距离 = %.2f m (路径上距障碍物最近点)\n', d_safe_min);
fprintf('  目标6 - 威胁代价     = %.2f (0 表示全程无碰撞)\n', threat);
fprintf('==================================================================\n');
% 绘制收敛曲线
figure('Name', ['NMOPOS收敛曲线-', env.type]);
iter_valid = 1:max_iter;
hist_valid = fitness_history;
hist_valid(isinf(hist_valid)) = NaN;
plot(iter_valid, hist_valid, 'b-', 'LineWidth', 1.5);
xlabel('迭代次数'); ylabel('Pareto前沿平均目标值');
title(['NMOPOS收敛曲线（', map_name_of(env), '）']); grid on;
% 可视化路径和环境
visualize_environment(env);
visualize_path(smooth_path, start_pos, goal_pos);
end

%% ====================== 环境构建 ======================
function env = build_environment(map_type)
% 统一构建环境结构 env，供适应度/可行性/绘图共用。
env.map_range = [0, 200];
env.safe_dist = 2.5;

switch map_type
    case 'city'
        % ---- 城市地图: 平底地面 + 矩形高楼 (复现 map/city_map.m) ----
        env.type = 'city';
        env.start = [10,  10,  60];    % 与 AVSS-RRT 城市地图一致
        env.goal  = [190, 190, 60];    % 与 AVSS-RRT 城市地图一致
        env.buildings = build_city_buildings();
        env.mountains   = [];
        env.sphere_center = [];
        env.sphere_radius = 0;
        env.cyl_centers   = [];
        env.cyl_radius    = 0;
        env.cyl_height    = 0;
        env.terrain_func  = @(x, y) zeros(size(x));   % 平底

    case 'hill'
        % ---- 丘陵地图: 7 座高斯山体 (复现 map/Hill_map.m, 确定性) ----
        env.type = 'hill';
        env.start = [10, 10, 50];
        env.goal  = [190, 190, 80];
        env.buildings = [];
        env.mountains = [ 40, 140,  90, 30;
                          140,  40,  90, 30;
                          140, 140, 120, 30;
                           90,  90, 120, 30;
                           40,  40,  50, 30;
                          180,  90, 100, 30;
                           90, 180, 100, 30];
        env.sphere_center = [];
        env.sphere_radius = 0;
        env.cyl_centers   = [];
        env.cyl_radius    = 0;
        env.cyl_height    = 0;
        env.terrain_func  = @(x, y) compute_terrain(x, y, env.mountains);

    case 'obstacle'
        % ---- 障碍物地图: map.m 的 3 座山 + 球形禁飞区 + 9 根圆柱 ----
        env.type = 'obstacle';
        env.start = [10, 10, 50];
        env.goal  = [190, 190, 80];
        env.buildings = [];
        env.mountains = [ 40, 140,  90, 30;
                          140,  40,  90, 30;
                          140, 140, 120, 30];
        env.sphere_center = [100, 100, 0];
        env.sphere_radius = 50;
        env.cyl_centers = [ 40,  70, 100;  70,  40, 100;  50,  50, 100;
                            90, 160, 100; 160,  80, 100;  80, 180, 100;
                           100, 180, 100; 180,  60, 100; 180, 100, 100];
        env.cyl_radius  = 5;
        env.cyl_height  = 100;
        env.terrain_func = @(x, y) compute_terrain(x, y, env.mountains);

    otherwise
        env.type = 'legacy';
        env.start = [10, 10, 50];
        env.goal  = [190, 190, 80];
        env.buildings = [];
        env.mountains = [ 40, 140,  90, 30;
                          140,  40,  90, 30;
                          140, 140, 120, 30];
        env.sphere_center = [100, 100, 0];
        env.sphere_radius = 50;
        env.cyl_centers = [ 40,  70, 100;  70,  40, 100;  50,  50, 100;
                            90, 160, 100; 160,  80, 100;  80, 180, 100;
                           100, 180, 100; 180,  60, 100; 180, 100, 100];
        env.cyl_radius  = 5;
        env.cyl_height  = 100;
        env.terrain_func = @(x, y) compute_terrain(x, y, env.mountains);
end
end

%% ====================== 环境查询（按类型分派） ======================
function d = min_obstacle_distance(pt, env)
% 点 pt 到最近障碍表面的真实间隙（安全距离在调用处与 env.safe_dist 比较）
if strcmp(env.type, 'city')
    d = pt(3);                       % 地面净空
    for i = 1:size(env.buildings, 1)
        d = min(d, dist_to_box(pt, env.buildings(i,:)));
    end
    return;
end

% ---- 山体/丘陵/旧地图 ----
d = pt(3) - env.terrain_func(pt(1), pt(2));
if env.sphere_radius > 0
    d = min(d, norm(pt - env.sphere_center) - env.sphere_radius);
end
for j = 1:size(env.cyl_centers, 1)
    center = env.cyl_centers(j, 1:2);
    h = env.cyl_centers(j, 3);
    dist_horiz = sqrt((pt(1)-center(1))^2 + (pt(2)-center(2))^2) - env.cyl_radius;
    if pt(3) >= 0 && pt(3) <= h
        dist_vert = 0;
    else
        dist_vert = min(abs(pt(3)-0), abs(pt(3)-h));
    end
    d = min(d, max(dist_horiz, dist_vert));
end
end

function dd = dist_to_box(pt, b)
% 点 pt 到轴对齐立方体 b=[x0 x1 y0 y1 h] (z 从 0 到 h) 最近表面的距离
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

function z = compute_terrain(x, y, mountains)
% 高斯山体叠加(确定性, 无噪声)。mountains 为 Kx4 [x0 y0 A sigma]
z = zeros(size(x));
for i = 1:size(mountains, 1)
    x0 = mountains(i, 1); y0 = mountains(i, 2);
    A  = mountains(i, 3); sigma = mountains(i, 4);
    z = z + A * exp(-((x - x0).^2 + (y - y0).^2) / (2 * sigma^2));
end
z(z < 0) = 0;
end

function B = build_city_buildings()
% 复现 map/city_map.m 的高楼排布(同一 rng(26) 种子), 返回 Nx5
% 每行 = [x0 x1 y0 y1 h], z 从 0 到 h 的立方体高楼。
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
roadRow = 3;   % 中间一行留空作主干道
roadCol = 3;   % 中间一列留空作主干道
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
            h = 120 + 30 * rand;   % 地标超高楼
        end
        B(end+1, :) = [x0 x1 y0 y1 h]; %#ok<AGROW>
    end
end
end

%% ====================== 带惩罚的评估 ======================
function obj = evaluate_path_with_penalty(ctrl_vars, start_pos, goal_pos, num_ctrl_pts, num_interp_pts, ...
    env, drone_speed)
    ctrl_pts = [start_pos; reshape(ctrl_vars, [], 3); goal_pos];
    path_pts = bspline_path(ctrl_pts, num_interp_pts);
    % 6 目标: [L, E, 1-cos̄, time, -d_min, threat]
    [L, E, rough, tm, negD, threat] = compute_path_metrics(path_pts, env, drone_speed);
    obj = [L, E, rough, tm, negD, threat];
    % 连续化碰撞惩罚: 穿透深度加权, 让"部分绕开/抬升"的解在所有目标上
    % 都劣于无碰撞解, 给优化器稳定的梯度信号(原 p_scale=800)
    pen = 0;
    for i = 1:size(path_pts, 1)
        d = min_obstacle_distance(path_pts(i,:), env);
        if d < env.safe_dist
            pen = pen + (env.safe_dist - d);
        end
    end
    p_scale = 800;
    obj(1) = obj(1) + p_scale * pen;   % L
    obj(2) = obj(2) + p_scale * pen;   % E
    obj(3) = obj(3) + p_scale * pen;   % 1-cos̄
    obj(4) = obj(4) + p_scale * pen;   % time
    obj(6) = obj(6) + p_scale * pen;   % threat
end

%% ====================== 原始指标计算（无惩罚） ======================
% 输出 6 目标: [路径长度, 能耗, 不平滑度(1-cos̄), 飞行时间, -最小安全距离, 威胁代价]
% 多目标统一最小化, 安全距离越大越好 → 取负; 威胁代价=0 表示全程无碰撞
function [length_val, energy, roughness, time_val, neg_d_safe_min, threat] = compute_path_metrics(path_pts, env, drone_speed)
    diffs = diff(path_pts, 1);
    seg_lengths = sqrt(sum(diffs.^2, 2));
    length_val = sum(seg_lengths);
    n = size(path_pts, 1);

    % ---- 高度变化(能耗所需) ----
    height_change = sum(abs(diff(path_pts(:,3), 1, 1)));

    % ---- 全路径障碍距离(目标5&6共用) ----
    D_all = zeros(n, 1);
    threat = 0; hard = 0; HARD_PEN = 1e5;
    safe_dist = env.safe_dist;
    for i = 1:n
        D_all(i) = min_obstacle_distance(path_pts(i,:), env);
        if D_all(i) < safe_dist
            threat = threat + (safe_dist - D_all(i));
            hard = hard + 1;
        end
    end
    threat = threat + hard * HARD_PEN;

    % ---- 能耗(论文模型) ----
    turn_angle_sum = 0;
    for i = 2:n - 1
        v1 = path_pts(i,:) - path_pts(i-1,:);
        v2 = path_pts(i+1,:) - path_pts(i,:);
        n1 = norm(v1); n2 = norm(v2);
        if n1 > 1e-6 && n2 > 1e-6
            cos_a = max(min(dot(v1, v2) / (n1 * n2), 1), -1);
            turn_angle_sum = turn_angle_sum + acos(cos_a);
        end
    end
    accel_sum = sum(abs(diff(seg_lengths)));
    energy = length_val * 0.1 + height_change * 0.3 + 0.2 * turn_angle_sum + 0.15 * accel_sum;

    % ---- 不平滑度 = 1 - mean(cosθ) ----
    cos_sum = 0; cnt = 0;
    for i = 2:n - 1
        v1 = path_pts(i,:) - path_pts(i-1,:);
        v2 = path_pts(i+1,:) - path_pts(i,:);
        n1 = norm(v1); n2 = norm(v2);
        if n1 > 1e-6 && n2 > 1e-6
            cos_sum = cos_sum + dot(v1, v2) / (n1 * n2);
            cnt = cnt + 1;
        end
    end
    smooth = cos_sum / (cnt + 1e-9);

    % ---- 飞行时间 ----
    time_val = length_val / drone_speed;

    % ---- 最小安全距离(取负供多目标最小化框架) ----
    d_safe_min = max(min(D_all), 0);
    neg_d_safe_min = -d_safe_min;

    roughness = 1 - smooth;
end

%% ====================== B 样条曲线（强制端点） ======================
function path_pts = bspline_path(ctrl_pts, num_pts)
    n = size(ctrl_pts, 1);
    if n < 4
        t = linspace(0, 1, num_pts)';
        path_pts = interp1(0:1, ctrl_pts, t, 'linear');
        return;
    end
    t = linspace(0, 1, num_pts);
    path_pts = zeros(num_pts, 3);
    for i = 1:num_pts
        u = t(i);
        seg = floor(u * (n-3)) + 1;
        seg = max(1, min(seg, n-3));
        u_local = (u - (seg-1)/(n-3)) * (n-3);
        u_local = max(0, min(1, u_local));
        P0 = ctrl_pts(seg, :); P1 = ctrl_pts(seg+1, :);
        P2 = ctrl_pts(seg+2, :); P3 = ctrl_pts(seg+3, :);
        b0 = (1-u_local)^3 / 6; b1 = (3*u_local^3 - 6*u_local^2 + 4) / 6;
        b2 = (-3*u_local^3 + 3*u_local^2 + 3*u_local + 1) / 6; b3 = u_local^3 / 6;
        path_pts(i,:) = b0*P0 + b1*P1 + b2*P2 + b3*P3;
    end
    % 强制起点和终点与给定的控制点首尾一致
    path_pts(1,:) = ctrl_pts(1,:);
    path_pts(end,:) = ctrl_pts(end,:);
end

%% ====================== 平滑函数（恢复端点） ======================
function path_smoothed = smooth_path_points(path, alpha)
    if size(path, 1) < 5
        path_smoothed = path;
        return;
    end
    path_smoothed = path;
    for iter = 1:3
        for i = 2:size(path,1)-1
            path_smoothed(i,:) = (1-alpha) * path_smoothed(i,:) + ...
                                 alpha * (path_smoothed(i-1,:) + path_smoothed(i+1,:)) / 2;
        end
    end
    % 恢复起点和终点，防止平滑导致偏移
    path_smoothed(1,:) = path(1,:);
    path_smoothed(end,:) = path(end,:);
end

%% ====================== 多目标支配 / Pareto / 存档 ======================
function [is_dom] = dominates(obj1, obj2)
% 6 目标均为"越小越好", 直接比较: [L, E, 1-cos̄, time, -d_min, threat]
% 目标5(-d_min)已在 compute_path_metrics 内部取负, 越大的安全距离对应越小的负值
    is_dom = all(obj1 <= obj2) && any(obj1 < obj2);
end

function [idx, nondominated] = pareto_filter(pop, objs)
    n = size(pop, 1);
    nondominated = true(n, 1);
    for i = 1:n
        for j = 1:n
            if i ~= j && dominates(objs(j,:), objs(i,:))
                nondominated(i) = false;
                break;
            end
        end
    end
    idx = find(nondominated);
end

function archive = truncate_archive(archive, max_size)
    if size(archive.pos, 1) <= max_size, return; end
    crowd = crowding_distance(archive.obj);
    [~, idx] = sort(crowd, 'descend');
    keep_idx = idx(1:max_size);
    archive.pos = archive.pos(keep_idx, :);
    archive.obj = archive.obj(keep_idx, :);
end

function crowd = crowding_distance(objs)
    n = size(objs, 1);
    if n <= 2, crowd = ones(n, 1); return; end
    m = size(objs, 2);
    crowd = zeros(n, 1);
    for i = 1:m
        [sorted, idx] = sort(objs(:, i));
        crowd(idx(1)) = inf; crowd(idx(end)) = inf;
        for j = 2:n-1
            crowd(idx(j)) = crowd(idx(j)) + (sorted(j+1) - sorted(j-1));
        end
    end
end

function idx = roulette_wheel(prob)
    cum_prob = cumsum(prob);
    r = rand();
    idx = find(cum_prob >= r, 1);
    if isempty(idx), idx = randi(length(prob)); end
end

%% ====================== 路径可行性检查 ======================
function [feasible, min_dist] = check_path_feasibility(path_pts, env)
    min_dist = inf;
    for i = 1:size(path_pts, 1)
        min_dist = min(min_dist, min_obstacle_distance(path_pts(i,:), env));
    end
    feasible = (min_dist >= env.safe_dist);
end

%% ====================== 可视化环境 ======================
function visualize_environment(env)
    tick_font_size  = 20;
    label_font_size = 20;
    title_font_size = 16;

    figure('Name', ['环境与规划路径-', env.type], 'Position', [100,100,800,600], 'Color', 'white');
    title(sprintf('NMOPOS全局路径规划（%s）', map_name_of(env)), 'FontSize', title_font_size);
    hold on; grid on; axis equal;
    xlim([0,200]); ylim([0,200]); zlim([0,200]);
    axis manual;
    xlabel('X (m)', 'FontSize', label_font_size);
    ylabel('Y (m)', 'FontSize', label_font_size);
    zlabel('Z (m)', 'FontSize', label_font_size);
    set(gca, 'Color', 'white', 'FontSize', tick_font_size);
    set(gca, 'XTick', 0:20:200, 'YTick', 0:20:200, 'ZTick', 0:20:200);

    if strcmp(env.type, 'city')
        % ---- 城市: 平底 + 矩形高楼 ----
        patch([0 200 200 0], [0 0 200 200], [1e-3 1e-3 1e-3 1e-3], ...
              [0.82 0.82 0.78], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
        for i = 1:size(env.buildings, 1)
            b = env.buildings(i, :);
            draw_building_box(b(1), b(2), b(3), b(4), b(5));
        end
        colormap parula;
        view(3);
        set(gcf, 'Color', 'w');
        return;
    end

    % ---- 山体/丘陵/旧地图: 高斯山体(绘图加噪声, 与 legacy 原来一致) ----
    map_x_draw = 0:0.1:200;
    map_y_draw = 0:0.1:200;
    [X_draw, Y_draw] = meshgrid(map_x_draw, map_y_draw);
    noise = randn(size(X_draw));
    for i = 1:size(env.mountains, 1)
        x0 = env.mountains(i,1); y0 = env.mountains(i,2);
        A  = env.mountains(i,3); sigma = env.mountains(i,4);
        z  = A * exp(-((X_draw-x0).^2 + (Y_draw-y0).^2) / (2*sigma^2));
        Z  = z + noise; Z(Z < 0) = 0;
        surf(X_draw, Y_draw, Z, 'FaceColor', 'interp', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    end
    shading interp;
    colormap parula;

    % 球形禁飞区（存在时）
    if env.sphere_radius > 0
        [sx, sy, sz] = sphere(40);
        sx = sx * env.sphere_radius + env.sphere_center(1);
        sy = sy * env.sphere_radius + env.sphere_center(2);
        sz = sz * env.sphere_radius + env.sphere_center(3);
        sz(sz < 0) = 0;
        surf(sx, sy, sz, 'FaceColor', [1,0,1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    end

    % 圆柱障碍物（存在时）
    if size(env.cyl_centers, 1) > 0
        [cx, cy, cz] = cylinder(env.cyl_radius, 40);
        for i = 1:size(env.cyl_centers, 1)
            x0 = env.cyl_centers(i,1);
            y0 = env.cyl_centers(i,2);
            h  = env.cyl_centers(i,3);
            surf(x0 + cx, y0 + cy, cz * h, 'FaceColor', [1,1,0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
            theta = linspace(0, 2*pi, 40);
            top_x = x0 + env.cyl_radius * cos(theta);
            top_y = y0 + env.cyl_radius * sin(theta);
            top_z = repmat(h, size(top_x));
            fill3(top_x, top_y, top_z, [1,1,0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
        end
    end

    view(3);
    set(gcf, 'Color', 'w');
end

function draw_building_box(x0, x1, y0, y1, h)
% 绘一栋立方体高楼(4 侧面 + 顶盖), 底在 z=0; 越高越偏蓝紫
t = min(max(h / 150, 0), 1);
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

function str = map_name_of(env)
if strcmp(env.type, 'city')
    str = '城市地图';
elseif strcmp(env.type, 'hill')
    str = '丘陵地图';
elseif strcmp(env.type, 'obstacle')
    str = '障碍物地图';
else
    str = '山体障碍地图';
end
end

%% ====================== 可视化路径 ======================
function visualize_path(path_pts, start_pos, goal_pos)
    plot3(path_pts(:,1), path_pts(:,2), path_pts(:,3), 'Color', [1 0 0], 'LineStyle', '-', 'LineWidth', 2);
    plot3(start_pos(1), start_pos(2), start_pos(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(goal_pos(1), goal_pos(2), goal_pos(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    legend({'规划路径', '起点', '终点'}, 'FontSize', 12);
end