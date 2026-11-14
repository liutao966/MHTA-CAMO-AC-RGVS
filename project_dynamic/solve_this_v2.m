function [smooth_path, camo_ac_elapsed, final_metrics, mhta_elapsed] = solve_this_v2(map_type, do_plot, weights, buildings)
% SOLVE_THIS_V2  本文算法（untitled.m 自包含版）: MHTA全局搜索 + CAMO-AC多目标 + 后处理
% 与 untitled.m 完全一致，仅改成可调用函数并返回最终平滑路径，便于批量/并行调用。
% 输出:
%   smooth_path     - 最终平滑路径 Nx3
%   camo_ac_elapsed - CAMO-AC 优化步骤耗时(s)（CAMO-AC 局部层）
%   final_metrics   - 最终 5 目标值 [L, E, unsmooth, tm, dmin]（AHP 敏感性用）
%   mhta_elapsed   - MHTA 全局搜索耗时(s)（MHTA 全局层）
% 用法: [p, t] = solve_this_v2('obstacle', false)               % 默认权重
%       [p, t, m] = solve_this_v2('obstacle', false, weights)   % 自定义权重
%       [p, t, m] = solve_this_v2('obstacle', false, [], buildings)  % 外部传入高楼（city 与 eval_metrics 对齐）
if nargin < 1 || isempty(map_type), map_type = 'obstacle'; end
if nargin < 2 || isempty(do_plot),  do_plot  = false; end
if nargin < 3 || isempty(weights),  weights  = [0.40, 0.10, 0.10, 0.05, 0.35]; end
warning off;

grid_size_mhta = 3;
safe_dist = 2.5;
drone_speed = 10;

env = build_static_environment(map_type);
if nargin >= 4 && ~isempty(buildings)
    env.buildings = buildings;   % 用调用方（eval_metrics 同一套）的楼
end
[smooth_path, camo_ac_elapsed, final_metrics, mhta_elapsed] = run_single_map(map_type, env, grid_size_mhta, safe_dist, drone_speed, do_plot, weights);
end

%% ====================== 1. 构建环境（与 HSMA 完全一致） ======================
function env = build_static_environment(map_type)
env.map_range = [0, 200];
env.resolution = 0.1;
[env.X, env.Y] = meshgrid(0:env.resolution:200, 0:env.resolution:200);
env.noise = zeros(size(env.X));  % 无噪声确定性地形(与其余7个对比算法一致，保证30次公平比较)

switch map_type
    case 'city'
        env.buildings = build_city_buildings();
        env.mountains = [];
        env.ball = [0 0 0 0];
        env.cyl_r = 0; env.cyl_pos = zeros(0,3);
        env.start = [10, 10, 60];
        env.goal  = [190, 190, 60];
    case 'hill'
        env.buildings = [];
        env.mountains = [40 140 90 30; 140 40 90 30; 140 140 120 30; ...
                         90 90 120 30; 40 40 50 30; 180 90 100 30; 90 180 100 30];
        env.ball = [0 0 0 0];
        env.cyl_r = 0; env.cyl_pos = zeros(0,3);
        env.start = [10, 10, 50];
        env.goal  = [190, 190, 80];
    case 'obstacle'
        env.buildings = [];
        env.mountains = [40 140 90 30; 140 40 90 30; 140 140 120 30];
        env.ball = [100, 100, 0, 50];
        env.cyl_r = 5;
        env.cyl_pos = [40,70,100; 70,40,100; 50,50,100; 90,160,100; 160,80,100; ...
                       80,180,100; 100,180,100; 180,60,100; 180,100,100];
        env.start = [10, 10, 50];
        env.goal  = [190, 190, 80];
    otherwise
        error('未知地图类型: %s', map_type);
end

env.terrain = compute_terrain(env.X, env.Y, env.mountains, map_type, env.noise);
end

function terrain = compute_terrain(X, Y, mountains, map_type, noise)
if strcmp(map_type, 'city')
    terrain = zeros(size(X));
    return;
end
terrain = zeros(size(X));
for k = 1:size(mountains,1)
    cx = mountains(k,1); cy = mountains(k,2); A = mountains(k,3); sigma = mountains(k,4);
    zk = A*exp(-((X-cx).^2+(Y-cy).^2)/(2*sigma*sigma));
    zk = noise + zk;
    zk(zk < 0) = 0;
    terrain = max(terrain, zk);
end
end

function B = build_city_buildings()
nCol = 6; nRow = 6; margin = 14; gap = 6;
s = rng;  rng('default'); rng(26);
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
rng(s);   % 恢复随机状态，避免重置调用方种子
end

%% ====================== 2. 单地图规划主函数 ======================
function [smooth, elapsed, final_metrics, mhta_elapsed] = run_single_map(map_type, env, grid_size, safe_dist, drone_speed, do_plot, weights)
start_pos = env.start; goal_pos = env.goal;

%% Step 1: MHTA 全局搜索
fprintf('-- Step 1: MHTA 全局搜索 ---\n');
t_mhta = tic;
mhta_path = mhta_search(start_pos, goal_pos, env.map_range, grid_size, env, safe_dist);
mhta_elapsed = toc(t_mhta);
waypoints = extract_key_waypoints(mhta_path, grid_size);
fprintf('MHTA 全局搜索节点 %d, 关键锚点 %d\n', size(mhta_path,1), size(waypoints,1));

%% Step 2: CAMO-AC 多目标优化
fprintf('-- Step 2: CAMO-AC 多目标优化 ---\n');
t_camo = tic;
params.pop_size = 30;
params.max_iter = 120;
params.alpha = 0.8; params.beta = 2.5; params.rho = 0.2; params.q0 = 0.85;
params.step_size = 1.2;
params.weights = weights;  % 论文5目标: [长度0.40, 能耗0.10, 平滑0.10, 时间0.05, 安全0.35]
params.drone_speed = drone_speed;

[optimal_path, fitness_history, metrics_history] = camo_ac_optimize(...
    start_pos, goal_pos, waypoints, env, safe_dist, params);
elapsed = toc(t_camo);
fprintf('CAMO-AC 耗时 %.2f s, 最优路径节点 %d\n', elapsed, size(optimal_path,1));

%% Step 3: 后处理
pruned = prune_path_safe(optimal_path, 0.25, env, safe_dist);
smooth = repair_path(pruned, env, safe_dist);
smooth = prune_path_safe(smooth, 0.25, env, safe_dist);
smooth = resample_polyline(smooth, 200);
smooth = repair_path(smooth, env, safe_dist);   % 重采样线性插值可能插进建筑，重采样后必须再修复
smooth(1,:) = start_pos; smooth(end,:) = goal_pos;
fprintf('-- Step 3: 后处理完成, 最终节点 %d ---\n', size(smooth,1));

%% Step 4: 可视化（与 HSMA 完全同风格）
if do_plot
    draw_map_and_path(env, smooth, start_pos, goal_pos, map_type);
end

%% Step 5: 5 目标输出
[L, E, unsmooth, tm, d_min] = compute_five_objectives(smooth, env, safe_dist, drone_speed);
final_metrics = [L, E, unsmooth, tm, d_min];
fprintf('\n========== 5 个目标值 (%s) ==========\n', map_name_of(map_type));
fprintf('  目标1 - 路径长度     = %.2f m\n', L);
fprintf('  目标2 - 能耗         = %.2f Wh (论文模型)\n', E);
fprintf('  目标3 - 不平滑度     = %.4f   (1 - mean(cosθ))\n', unsmooth);
fprintf('  目标4 - 飞行时间     = %.2f s (V=%d m/s)\n', tm, drone_speed);
fprintf('  目标5 - 最小安全距离 = %.2f m\n', d_min);

[L_a, E_a, u_a, t_a, d_a] = compute_five_objectives(mhta_path, env, safe_dist, drone_speed);
fprintf('---------- MHTA 全局搜索 vs CAMO-AC 优化 ----------\n');
fprintf('  路径长度     : MHTA=%.2f | CAMO-AC=%.2f (差 %.2f)\n', L_a, L, L_a-L);
fprintf('  能耗         : MHTA=%.2f | CAMO-AC=%.2f\n', E_a, E);
fprintf('  不平滑度     : MHTA=%.4f | CAMO-AC=%.4f\n', u_a, unsmooth);
fprintf('  飞行时间     : MHTA=%.2f | CAMO-AC=%.2f\n', t_a, tm);
fprintf('  最小安全距离 : MHTA=%.2f | CAMO-AC=%.2f\n', d_a, d_min);
end

%% ====================== 3. 三维 MHTA ======================
function path = mhta_search(start, goal, map_range, grid_size, env, safe_dist)
x_grid = map_range(1):grid_size:map_range(2);
y_grid = map_range(1):grid_size:map_range(2);
z_grid = map_range(1):grid_size:map_range(2);
[~, si(1)] = min(abs(x_grid-start(1))); [~, si(2)] = min(abs(y_grid-start(2))); [~, si(3)] = min(abs(z_grid-start(3)));
[~, gi(1)] = min(abs(x_grid-goal(1)));  [~, gi(2)] = min(abs(y_grid-goal(2)));  [~, gi(3)] = min(abs(z_grid-goal(3)));
g_arr = inf(length(x_grid), length(y_grid), length(z_grid));
f_arr = inf(length(x_grid), length(y_grid), length(z_grid));
par_arr = zeros(length(x_grid), length(y_grid), length(z_grid), 3);
par_arr(:) = NaN;
g_arr(si(1),si(2),si(3)) = 0;
f_arr(si(1),si(2),si(3)) = enhanced_heuristic(start, goal, env, safe_dist);
open_set = si;
% 用逻辑数组 closed/in_open 代替 close_set 矩阵 + ismember(...,'rows')，
% 把每节点 ×26 邻居的 O(节点数) 逐行查找降到 O(1)。其余（sub2ind/min/open_set 顺序）保持原样，
% 因此搜索顺序与结果路径和原版完全一致。
closed  = false(length(x_grid), length(y_grid), length(z_grid));
in_open = false(length(x_grid), length(y_grid), length(z_grid));
in_open(si(1),si(2),si(3)) = true;
dirs = combvec([-1 0 1],[-1 0 1],[-1 0 1])'; dirs(all(dirs==0,2),:) = [];
found = false; iter = 0; max_iter = 3*numel(g_arr);
while ~isempty(open_set) && ~found && iter < max_iter
    iter = iter + 1;
    fc = sub2ind(size(f_arr), open_set(:,1), open_set(:,2), open_set(:,3));
    [~, mi] = min(f_arr(fc));
    cur = open_set(mi,:); open_set(mi,:) = [];
    in_open(cur(1),cur(2),cur(3)) = false;
    closed(cur(1),cur(2),cur(3)) = true;
    if all(cur == gi), found = true; break; end
    cur_pos = [x_grid(cur(1)), y_grid(cur(2)), z_grid(cur(3))];
    for d = 1:size(dirs,1)
        nx = cur + dirs(d,:);
        if any(nx<1) || nx(1)>length(x_grid) || nx(2)>length(y_grid) || nx(3)>length(z_grid), continue; end
        if closed(nx(1),nx(2),nx(3)), continue; end
        np = [x_grid(nx(1)), y_grid(nx(2)), z_grid(nx(3))];
        if ~check_pos_feasible(np, env, safe_dist) || ~check_seg_feasible(cur_pos, np, env, safe_dist), continue; end
        g_new = g_arr(cur(1),cur(2),cur(3)) + norm(np-cur_pos);
        h_new = enhanced_heuristic(np, goal, env, safe_dist);
        f_new = g_new + h_new;
        if f_new < f_arr(nx(1),nx(2),nx(3))
            g_arr(nx(1),nx(2),nx(3)) = g_new;
            f_arr(nx(1),nx(2),nx(3)) = f_new;
            par_arr(nx(1),nx(2),nx(3),:) = cur;
            if ~in_open(nx(1),nx(2),nx(3))
                open_set(end+1,:) = nx; %#ok<AGROW>
                in_open(nx(1),nx(2),nx(3)) = true;
            end
        end
    end
end
if ~found, error('MHTA 未找到可行路径'); end
path = []; cur = gi;
while ~any(isnan(squeeze(par_arr(cur(1),cur(2),cur(3),:))))
    cp = [x_grid(cur(1)), y_grid(cur(2)), z_grid(cur(3))];
    path = [cp; path];
    cur = squeeze(par_arr(cur(1),cur(2),cur(3),:))';
end
path = [start; path]; path(end,:) = goal;
fprintf('MHTA 路径生成成功！节点数 %d\n', size(path,1));
end

%% ====================== 4. 增强启发函数 ======================
function h = enhanced_heuristic(pos, goal, env, safe_dist)
h = norm(pos-goal)*0.9;
if pos(3) < terrain_height(pos(1),pos(2),env) + 9
    h = h + 200/(pos(3)-terrain_height(pos(1),pos(2),env)+1e-6);
end
if env.ball(4) > 0
    bd = norm(pos-env.ball(1:3)) - env.ball(4);
    if bd < 9, h = h + 300/(bd+1e-6); end
end
if env.cyl_r > 0
    for i = 1:size(env.cyl_pos,1)
        cp = env.cyl_pos(i,:);
        dh = norm([pos(1),pos(2)]-[cp(1),cp(2)]);
        if dh < env.cyl_r+9 && pos(3) < cp(3)
            h = h + 250/(dh-env.cyl_r+1e-6);
        end
    end
end
if ~isempty(env.buildings)
    for i = 1:size(env.buildings,1)
        db = dist_to_box(pos, env.buildings(i,:));
        if db < 9
            h = h + 250/(db+1e-6);
        end
    end
end
end

%% ====================== 5. 地形高度查询 ======================
function th = terrain_height(x, y, env)
xi = round((x-0)/env.resolution + 1);
yi = round((y-0)/env.resolution + 1);
if xi>=1 && xi<=size(env.terrain,2) && yi>=1 && yi<=size(env.terrain,1)
    th = env.terrain(yi,xi);
else
    th = 0;
end
end

%% ====================== 6. 节点/线段可行性检查 ======================
function feasible = check_pos_feasible(pos, env, safe_dist)
x = pos(1); y = pos(2); z = pos(3);
if z < terrain_height(x,y,env) + safe_dist, feasible = false; return; end
if env.ball(4) > 0 && norm(pos-env.ball(1:3)) < env.ball(4)+safe_dist, feasible = false; return; end
if env.cyl_r > 0
    for i = 1:size(env.cyl_pos,1)
        cp = env.cyl_pos(i,:);
        if norm([x-cp(1),y-cp(2)]) < env.cyl_r+safe_dist && z>=0 && z<=cp(3)
            feasible = false; return;
        end
    end
end
if ~isempty(env.buildings)
    for i = 1:size(env.buildings,1)
        if dist_to_box(pos, env.buildings(i,:)) < safe_dist
            feasible = false; return;
        end
    end
end
feasible = true;
end

function feasible = check_seg_feasible(p1, p2, env, safe_dist)
for i = 0:9
    pos = p1 + (p2-p1)*i/9;
    if ~check_pos_feasible(pos, env, safe_dist), feasible = false; return; end
end
feasible = true;
end

function dd = dist_to_box(pt, b)
% 点 pt 到轴对齐立方体 b=[x0 x1 y0 y1 h](z 从 0 到 h) 的带符号距离：
%   外部为正(到最近表面距离)，内部为负。与 eval_metrics.m 的 dist_to_box 完全一致。
    x = pt(1); y = pt(2); z = pt(3);
    x0 = b(1); x1 = b(2); y0 = b(3); y1 = b(4); h = b(5);
    inside = (x > x0 && x < x1 && y > y0 && y < y1 && z > 0 && z < h);
    if inside
        dd = -min([x - x0, x1 - x, y - y0, y1 - y, z, h - z]);
        return;
    end
    dx = max([x0 - x, x - x1, 0]);
    dy = max([y0 - y, y - y1, 0]);
    if z < 0, dz = -z; elseif z > h, dz = z - h; else, dz = 0; end
    dd = sqrt(dx^2 + dy^2 + dz^2);
end

%% ====================== 7. 安全距离（点到最近障碍） ======================
function sd = safe_distance(pos, env)
x = pos(1); y = pos(2); z = pos(3);
ds = z - terrain_height(x,y,env);
if env.ball(4) > 0
    ds = min(ds, norm(pos-env.ball(1:3))-env.ball(4));
end
if env.cyl_r > 0
    for i = 1:size(env.cyl_pos,1)
        cp = env.cyl_pos(i,:);
        dh = norm([x-cp(1),y-cp(2)]);
        if z>=0 && z<=cp(3)
            dc = dh-env.cyl_r;
        elseif z<0
            dc = max(dh-env.cyl_r, -z);
        else
            dc = max(dh-env.cyl_r, z-cp(3));
        end
        ds = min(ds, dc);
    end
end
if ~isempty(env.buildings)
    for i = 1:size(env.buildings,1)
        ds = min(ds, dist_to_box(pos, env.buildings(i,:)));
    end
end
sd = max(ds, 0);
end

%% ====================== 8. 关键锚点提取 ======================
function waypoints = extract_key_waypoints(path, grid_size)
if size(path,1)<=2, waypoints = path; return; end
waypoints = path(1,:);
for i = 2:size(path,1)-1
    v1 = path(i,:)-path(i-1,:); v2 = path(i+1,:)-path(i,:);
    n1 = v1/(norm(v1)+1e-6); n2 = v2/(norm(v2)+1e-6);
    ang = acos(max(min(dot(n1,n2),1),-1));
    if ang > 0.25
        waypoints(end+1,:) = path(i,:); %#ok<AGROW>
    end
end
waypoints(end+1,:) = path(end,:); %#ok<AGROW>
fw = waypoints(1,:);
for i = 2:size(waypoints,1)
    if norm(waypoints(i,:)-fw(end,:)) >= grid_size
        fw(end+1,:) = waypoints(i,:); %#ok<AGROW>
    end
end
waypoints = fw;
end

%% ====================== 9. Lévy flight ======================
function step = levy_flight(step_size)
beta = 1.5;
sigma = (gamma(1+beta)*sin(pi*beta/2)/(gamma((1+beta)/2)*beta*2^((beta-1)/2)))^(1/beta);
u = randn(1,3)*sigma; v = randn(1,3);
step = step_size * u ./ (abs(v).^(1/beta)+1e-10);
end

%% ====================== 10. CAMO-AC 主循环 ======================
function [optimal_path, fitness_history, metrics_history] = camo_ac_optimize(start, goal, waypoints, env, safe_dist, params)
pop_size = params.pop_size; max_iter = params.max_iter;
alpha = params.alpha; beta = params.beta; rho = params.rho; q0 = params.q0;
step_size = params.step_size; weights = params.weights; drone_speed = params.drone_speed;

pop = init_pop(pop_size, start, goal, waypoints, env, safe_dist);
[~, metrics] = calc_fitness(pop, env, safe_dist, weights, drone_speed);
fitness = zeros(pop_size, 1);
for i = 1:pop_size, fitness(i) = calc_abs_fit(metrics(i,:), weights); end
[best_fitness, best_idx] = min(fitness);
optimal_path = pop{best_idx};
global_best_metrics = metrics(best_idx,:);
fitness_history = zeros(1, max_iter);
metrics_history = zeros(max_iter, 5);
fitness_history(1) = calc_abs_fit(global_best_metrics, weights);
metrics_history(1,:) = global_best_metrics;

nwp = size(waypoints,1);
tau = ones(nwp-1, 1);
tau_max = 10; tau_min = 0.1;

for iter = 2:max_iter
    for ant = 1:pop_size
        cur_path = start; cur_pos = start;
        for wp = 1:nwp-1
            ws = waypoints(wp,:); we = waypoints(wp+1,:);
            x_r = [min(ws(1),we(1)), max(ws(1),we(1))];
            y_r = [min(ws(2),we(2)), max(ws(2),we(2))];
            z_r = [min(ws(3),we(3)), max(ws(3),we(3))];
            eta = 0.6/(norm(cur_pos-we)+1e-6) + 0.4/(norm(cur_pos-goal)+1e-6);
            if rand < q0
                dir_greedy = (we-cur_pos)/(norm(we-cur_pos)+1e-6);
                dir_safe = safe_direction(cur_pos, env);
                w_len = weights(1); w_safe = weights(5);
                dir = (w_len*dir_greedy + w_safe*dir_safe);
                dir = dir/(norm(dir)+1e-6);
            else
                if iter < max_iter/3, ls = levy_flight(step_size*1.2);
                elseif iter < max_iter*2/3, ls = levy_flight(step_size*0.8);
                else, ls = levy_flight(step_size*0.4)+randn(1,3)*0.03; end
                dir = ls/(norm(ls)+1e-6);
            end
            seg_len = norm(we-ws);
            sa = 0.8+0.2*(1-iter/max_iter);
            np = cur_pos + dir*seg_len*step_size*sa;
            np(1) = max(x_r(1), min(x_r(2), np(1)));
            np(2) = max(y_r(1), min(y_r(2), np(2)));
            np(3) = max(z_r(1), min(z_r(2), np(3)));
            if ~check_pos_feasible(np, env, safe_dist)
                np = adaptive_repair(np, env, safe_dist, we, 0);
            end
            cur_path(end+1,:) = np; cur_pos = np; %#ok<AGROW>
        end
        cur_path(end+1,:) = goal; %#ok<AGROW>
        cur_path = prune_path(cur_path, 0.15);
        cur_path = repair_path(cur_path, env, safe_dist);
        pop{ant} = cur_path;
    end
    [~, metrics] = calc_fitness(pop, env, safe_dist, weights, drone_speed, global_best_metrics);
    fitness = zeros(pop_size, 1);
    for i = 1:pop_size, fitness(i) = calc_abs_fit(metrics(i,:), weights); end
    [cbf, cbi] = min(fitness);
    if cbf < best_fitness
        best_fitness = cbf; optimal_path = pop{cbi};
        global_best_metrics = metrics(cbi,:);
    end
    tau = (1-rho)*tau + 1/(best_fitness+1e-6)*1.8;
    tau = min(max(tau, tau_min), tau_max);
    fitness_history(iter) = calc_abs_fit(global_best_metrics, weights);
    metrics_history(iter,:) = global_best_metrics;
    if mod(iter,40)==0
        fprintf('  iter %d/%d | fitness %.4f | L %.2f d_min %.2f\n', ...
            iter, max_iter, best_fitness, global_best_metrics(1), global_best_metrics(5));
    end
end
end

%% ====================== 11. 种群初始化 ======================
function pop = init_pop(pop_size, start, goal, waypoints, env, safe_dist)
pop = cell(1, pop_size); nwp = size(waypoints,1);
for ant = 1:pop_size
    cur_path = start;
    for wp = 1:nwp
        if wp == 1, ss = start; else, ss = waypoints(wp-1,:); end
        se = waypoints(wp,:);
        sp = gen_seg(ss, se, 8, env, safe_dist);
        cur_path = [cur_path; sp(2:end,:)];
    end
    sp = gen_seg(waypoints(end,:), goal, 8, env, safe_dist);
    cur_path = [cur_path; sp(2:end,:)];
    cur_path = prune_path(cur_path, 0.15);
    cur_path = repair_path(cur_path, env, safe_dist);
    pop{ant} = cur_path;
end
fprintf('种群初始化完成！%d 条可行初始路径\n', pop_size);
end

function seg = gen_seg(ss, se, nn, env, safe_dist)
seg = zeros(nn,3); t = linspace(0,1,nn);
seg(:,1) = ss(1)+t*(se(1)-ss(1));
seg(:,2) = ss(2)+t*(se(2)-ss(2));
seg(:,3) = ss(3)+t*(se(3)-ss(3));
damp = max(norm(se-ss)*0.15, 6);
for i = 2:nn-1, seg(i,:) = seg(i,:) + damp*(rand(1,3)-0.5); end
for i = 1:nn
    if ~check_pos_feasible(seg(i,:), env, safe_dist)
        seg(i,:) = adaptive_repair(seg(i,:), env, safe_dist, se, 0);
    end
end
end

%% ====================== 12. 适应度计算 ======================
function [fitness, metrics] = calc_fitness(pop, env, safe_dist, weights, drone_speed, gbm)
ps = length(pop); metrics = zeros(ps,5); fitness = zeros(ps,1);
for i = 1:ps
    path = pop{i}; n = size(path,1);
    dpath = diff(path,1,1);
    L = sum(sqrt(sum(dpath.^2,2)));
    dall = zeros(n + 5*(n-1), 1);
    for j = 1:n, dall(j) = safe_distance(path(j,:), env); end
    for j = 1:n-1
        for s = 1:5
            pos = path(j,:) + (path(j+1,:)-path(j,:))*(s/6);
            dall(n + 5*(j-1) + s) = safe_distance(pos, env);
        end
    end
    dmin = max(min(dall), 0);
    hc = sum(abs(diff(path(:,3))));
    ta = 0; ac = 0;
    for j = 2:n-1
        v1 = path(j,:)-path(j-1,:); v2 = path(j+1,:)-path(j,:);
        n1 = v1/(norm(v1)+1e-6); n2 = v2/(norm(v2)+1e-6);
        ang = acos(max(min(dot(n1,n2),1),-1));
        ta = ta+ang; ac = ac+2*sin(ang/2);
    end
    E = L*0.1 + hc*0.3 + 0.2*ta + 0.15*ac;
    sc = 0; cnt = 0;
    for j = 2:n-1
        v1 = path(j,:)-path(j-1,:); v2 = path(j+1,:)-path(j,:);
        if norm(v1)>1e-6 && norm(v2)>1e-6
            sc = sc + dot(v1,v2)/(norm(v1)*norm(v2)); cnt = cnt+1;
        end
    end
    unsmooth = 1 - sc/(cnt+1e-6);
    tm = L/drone_speed;
    metrics(i,:) = [L, E, unsmooth, tm, dmin];
end

nm = zeros(size(metrics));
for col = 1:5
    if nargin < 7 || isempty(gbm)
        lo = min(metrics(:,col)); hi = max(metrics(:,col));
    else
        lo = min([min(metrics(:,col)), gbm(col)]);
        hi = max([max(metrics(:,col)), gbm(col)]);
    end
    if hi-lo < 1e-6, nm(:,col) = 0;
    elseif col == 5, nm(:,col) = 1 - (metrics(:,col)-lo)/(hi-lo);
    else, nm(:,col) = (metrics(:,col)-lo)/(hi-lo); end
end
for i = 1:ps
    fitness(i) = weights(1)*nm(i,1)+weights(2)*nm(i,2)+weights(3)*nm(i,3)+...
                 weights(4)*nm(i,4)+weights(5)*nm(i,5);
end
fitness = fitness/(max(fitness)+1e-6);
end

function af = calc_abs_fit(mr, w)
ref = [300, 60, 2, 30, 10];
nm = [min(mr(1)/ref(1),1), min(mr(2)/ref(2),1), min(mr(3)/ref(3),1), ...
      min(mr(4)/ref(4),1), max(0,1-mr(5)/ref(5))];
af = sum(w.*nm);
end

%% ====================== 13. 自适应修复 ======================
function rp = adaptive_repair(pos, env, safe_dist, target, depth)
if nargin < 5, depth = 0; end
if depth > 5, rp = target; return; end
dirs = combvec([-1 0 1],[-1 0 1],[-1 0 1]);
dirs(:,all(dirs==0,1)) = [];
costs = inf(1,size(dirs,2));
for d = 1:size(dirs,2)
    cand = pos + dirs(:,d)'*safe_dist;
    if check_pos_feasible(cand, env, safe_dist)
        costs(d) = 0.7*norm(cand-target) + 0.3*safe_dist;
    end
end
[mc, bd] = min(costs);
if mc ~= inf
    rp = pos + dirs(:,bd)'*safe_dist;
else
    if norm(target-pos)<1e-6, rd = randn(1,3); rd = rd/norm(rd);
    else, rd = (target-pos)/norm(target-pos); end
    rp = pos + rd*safe_dist*2;
    if ~check_pos_feasible(rp, env, safe_dist)
        rp = adaptive_repair(rp, env, safe_dist, target, depth+1);
    end
end
end

%% ====================== 14. 路径修剪 ======================
function pp = prune_path(path, ang_thr)
if size(path,1)<=3, pp = path; return; end
pp = path(1,:);
for i = 2:size(path,1)-1
    v1 = path(i,:)-path(i-1,:); v2 = path(i+1,:)-path(i,:);
    n1 = v1/(norm(v1)+1e-6); n2 = v2/(norm(v2)+1e-6);
    if acos(max(min(dot(n1,n2),1),-1)) > ang_thr
        pp(end+1,:) = path(i,:); %#ok<AGROW>
    end
end
pp(end+1,:) = path(end,:); %#ok<AGROW>
end

%% ====================== 15. 路径修复 ======================
function rp = repair_path(path, env, safe_dist)
rp = path; n = size(rp,1);
for i = 1:n
    if ~check_pos_feasible(rp(i,:), env, safe_dist)
        if i<n, tgt = rp(i+1,:); else, tgt = rp(i-1,:); end
        rp(i,:) = adaptive_repair(rp(i,:), env, safe_dist, tgt, 0);
    end
end
max_len = size(rp,1) * 10;
i = 1; guard = 0;
while i < size(rp,1) && size(rp,1) < max_len && guard < 500
    guard = guard + 1;
    p1 = rp(i,:); p2 = rp(i+1,:);
    if ~check_seg_feasible(p1, p2, env, safe_dist)
        ins = gen_seg(p1, p2, 4, env, safe_dist);
        rp = [rp(1:i,:); ins(2:end-1,:); rp(i+1:end,:)];
    else
        i = i+1;
    end
end
end

%% ====================== 16. 弧长重采样 ======================
function np = resample_polyline(path, npts)
d = [0; cumsum(sqrt(sum(diff(path).^2,2)))];
if d(end)<1e-6 || size(path,1)<2, np = path; return; end
[d,ia] = unique(d,'stable'); path = path(ia,:);
tt = linspace(0,d(end),npts)';
np = [interp1(d,path(:,1),tt), interp1(d,path(:,2),tt), interp1(d,path(:,3),tt)];
end

%% ====================== 17. 可视化（1:1 对齐 map/ 原版三个文件） ======================
function draw_map_and_path(env, path, start, goal, map_type)
figure('Position',[100,100,800,800],'Name',['MHTA+CAMO-AC - ' map_name_of(map_type)],'Color','white');
ax = axes('Parent',gcf); hold(ax,'on');
xlabel(ax,'X/m','FontSize',20); ylabel(ax,'Y/m','FontSize',20); zlabel(ax,'Z/m','FontSize',20);
title(ax,[map_name_of(map_type) ' 路径规划结果'],'FontSize',16);
xlim(ax,[0 200]); ylim(ax,[0 200]); zlim(ax,[0 200]);
xticks(ax,0:20:200); yticks(ax,0:20:200); zticks(ax,0:20:200);
daspect(ax,[1 1 1]);
set(ax,'FontSize',20,'Color','white');
grid(ax,'on');

if strcmp(map_type,'city')
    patch(ax,[0 200 200 0],[0 0 200 200],[1e-3 1e-3 1e-3 1e-3],[0.82 0.82 0.78],...
        'EdgeColor','none','FaceAlpha',1.0);
    for i = 1:size(env.buildings,1)
        draw_building(ax, env.buildings(i,:));
    end
else
    patch(ax,[0 200 200 0],[0 0 200 200],[1e-3 1e-3 1e-3 1e-3],[0.15 0.25 0.45],...
        'EdgeColor','none','FaceAlpha',1.0);
    for k = 1:size(env.mountains,1)
        cx = env.mountains(k,1); cy = env.mountains(k,2);
        A = env.mountains(k,3); s = env.mountains(k,4);
        zk = A*exp(-((env.X-cx).^2+(env.Y-cy).^2)/(2*s*s));
        zk = env.noise + zk;
        zk(zk<0) = 0;
        surf(ax, env.X, env.Y, zk, 'FaceColor','interp','EdgeColor','none','FaceAlpha',0.9);
    end
end

if env.ball(4) > 0
    [Xb,Yb,Zb] = sphere(50);
    surf(ax, env.ball(1)+Xb*env.ball(4), env.ball(2)+Yb*env.ball(4), env.ball(3)+Zb*env.ball(4),...
        'FaceColor',[1 0 1],'EdgeColor','none','FaceAlpha',1.0);
end

if env.cyl_r > 0 && ~isempty(env.cyl_pos)
    theta = linspace(0,2*pi,50);
    for i = 1:size(env.cyl_pos,1)
        cx = env.cyl_pos(i,1); cy = env.cyl_pos(i,2); cz = env.cyl_pos(i,3);
        [Xc,Yc,Zc] = cylinder(env.cyl_r,50);
        surf(ax, cx+Xc, cy+Yc, cz*Zc, 'FaceColor',[1 1 0], 'EdgeColor','none','FaceAlpha',1.0);
        fill3(ax, cx+env.cyl_r*cos(theta), cy+env.cyl_r*sin(theta), repmat(cz,size(theta)),...
            [1 1 0], 'EdgeColor','none','FaceAlpha',1.0);
    end
end

plot3(ax, path(:,1), path(:,2), path(:,3), '-', 'Color',[1 0 0], 'LineWidth', 3);
[sx,sy,sz] = sphere(30);
surf(ax, start(1)+sx*4, start(2)+sy*4, start(3)+sz*4, 'FaceColor',[0 1 0], 'EdgeColor','none');
surf(ax, goal(1)+sx*4, goal(2)+sy*4, goal(3)+sz*4, 'FaceColor',[0 0 1], 'EdgeColor','none');

if strcmp(map_type,'city')
    view(ax,45,30);
else
    view(ax,-37.5,30);
end
camproj(ax,'orthographic');
hold(ax,'off');
end

function draw_building(ax, b)
x0 = b(1); x1 = b(2); y0 = b(3); y1 = b(4); h = b(5);
t = min(max(h/150,0),1);
col = [0.4-0.15*t, 0.48-0.10*t, 0.72+0.10*t];
vx = [x0 x1 x1 x0 x0 x1 x1 x0];
vy = [y0 y0 y1 y1 y0 y0 y1 y1];
vz = [0 0 0 0 h h h h];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax,'Vertices',[vx(:),vy(:),vz(:)],'Faces',faces,...
    'FaceColor',col,'EdgeColor','k','LineWidth',0.5,'FaceAlpha',1.0);
patch(ax,[x0 x1 x1 x0],[y0 y0 y1 y1],[h h h h],col.*1.2,...
    'EdgeColor','none','FaceAlpha',1.0);
end

%% ====================== 18.5 安全方向 + 安全感知剪枝 ======================
function dir = safe_direction(pos, env)
% 远离最近障碍的方向：采样 6 个主方向，选使 clearance 增大的方向
best_dir = [0 0 1]; best_clear = safe_distance(pos, env);
cands = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
step = 2.0;
for k = 1:size(cands,1)
    d = cands(k,:)/norm(cands(k,:));
    c = safe_distance(pos + d*step, env);
    if c > best_clear
        best_clear = c; best_dir = d;
    end
end
dir = best_dir;
end

function pp = prune_path_safe(path, ang_thr, env, safe_dist)
% 与 prune_path 相同，但删除"近直线"节点前先检查：删掉后前后节点直连是否仍安全
if size(path,1)<=3, pp = path; return; end
pp = path(1,:);
for i = 2:size(path,1)-1
    v1 = path(i,:)-path(i-1,:); v2 = path(i+1,:)-path(i,:);
    n1 = v1/(norm(v1)+1e-6); n2 = v2/(norm(v2)+1e-6);
    ang = acos(max(min(dot(n1,n2),1),-1));
    if ang > ang_thr
        pp(end+1,:) = path(i,:);
    elseif ~check_seg_feasible(path(i-1,:), path(i+1,:), env, 8.0)
        pp(end+1,:) = path(i,:);
    end
end
pp(end+1,:) = path(end,:);
end

%% ====================== 19. 5 目标计算 ======================
function [L,E,unsmooth,tm,dmin] = compute_five_objectives(path, env, safe_dist, drone_speed)
n = size(path,1); dpath = diff(path,1,1);
L = sum(sqrt(sum(dpath.^2,2)));
dall = zeros(n + 5*(n-1), 1);
for j = 1:n, dall(j) = safe_distance(path(j,:), env); end
for j = 1:n-1
    for s = 1:5
        pos = path(j,:) + (path(j+1,:)-path(j,:))*(s/6);
        dall(n + 5*(j-1) + s) = safe_distance(pos, env);
    end
end
dmin = max(min(dall),0);
hc = sum(abs(diff(path(:,3)))); ta = 0; ac = 0;
for j = 2:n-1
    v1 = path(j,:)-path(j-1,:); v2 = path(j+1,:)-path(j,:);
    n1 = v1/(norm(v1)+1e-6); n2 = v2/(norm(v2)+1e-6);
    ang = acos(max(min(dot(n1,n2),1),-1));
    ta = ta+ang; ac = ac+2*sin(ang/2);
end
E = L*0.1 + hc*0.3 + 0.2*ta + 0.15*ac;
sc = 0; cnt = 0;
for j = 2:n-1
    v1 = path(j,:)-path(j-1,:); v2 = path(j+1,:)-path(j,:);
    if norm(v1)>1e-6 && norm(v2)>1e-6
        sc = sc + dot(v1,v2)/(norm(v1)*norm(v2)); cnt = cnt+1;
    end
end
unsmooth = 1 - sc/(cnt+1e-6);
tm = L/drone_speed;
end

%% ====================== 20. 地图中文名 ======================
function str = map_name_of(map_type)
switch map_type
    case 'city';     str = '城市地图';
    case 'hill';     str = '丘陵地图';
    case 'obstacle'; str = '障碍物地图';
    otherwise;       str = '未知地图';
end
end