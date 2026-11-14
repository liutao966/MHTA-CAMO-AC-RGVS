%% ====================== 帕累托前沿分析（多目标权衡证据） ======================
% 功能：
%   1. 方案 A — 单次 ACOSRAR，保存每代种群所有个体的 5 目标值
%   2. 方案 B — 多次不同权重运行，收集各偏好下的最优解
%   3. 合并 → Pareto 非支配筛选 → 2D/3D/散点矩阵/雷达图可视化
% 依赖：同目录下 dominates.m, find_pareto_front.m
% 目标定义（5 个，来自 untitled.m）：
%   f1 = L        路径长度 (m)        — 最小化
%   f2 = E        能耗 (Wh)           — 最小化
%   f3 = unsmooth 不平滑度            — 最小化
%   f4 = tm       飞行时间 (s)        — 最小化
%   f5 = d_min    最小安全距离 (m)    — 最大化 !!
clear; close all; warning off; clc;

%% ====================== 全局参数 ======================
map_range       = [0, 200];
grid_size_astar = 3;
safe_dist       = 2.5;

drone_speed     = 10;
start_pos       = [10, 10, 50];
goal_pos        = [190, 190, 80];

maps = {'obstacle'};

% 固定 10 个随机种子：Scheme A/B 对每个种子各跑一次，候选解合并后统一非支配排序，
% 保证结果可复现（每次跑同一组种子）且不依赖单个种子的运气。
seeds = 1:10;
n_seeds = numel(seeds);
obj_minimize = [true, true, true, true, false];   % [L,E,unsmooth,tm] 最小化，dmin 最大化

%% ====================== 逐个地图分析 ======================
for mi = 1:length(maps)
    map_type = maps{mi};
    fprintf('\n=============================================================\n');
    fprintf('  Pareto Front Analysis -- %s\n', map_name_of(map_type));
    fprintf('=============================================================\n');

    %% Step 1: Build environment + A* coarse path
    [X, Y, Z1, Z2, Z3, ball, cylinder] = build_static_environment(map_type, map_range);

    fprintf('-- Step 1: A* coarse path ---\n');
    astar_path = astar_3d(start_pos, goal_pos, map_range, grid_size_astar, ...
        Z1, Z2, Z3, ball, cylinder, safe_dist);
    waypoints = extract_key_waypoints(astar_path, grid_size_astar);
    fprintf('A* nodes: %d, key waypoints: %d\n', size(astar_path,1), size(waypoints,1));

    %% Step 2: Scheme A -- single ACOSRAR run, record all generations
    params.pop_size   = 60;
    params.max_iter   = 200;
    params.alpha      = 0.8;
    params.beta       = 2.5;
    params.rho        = 0.2;
    params.q0         = 0.85;
    params.step_size  = 1.2;
    params.weights    = [0.40, 0.10, 0.10, 0.05, 0.35];  % = 论文AHP [长度0.40,安全0.35,能耗0.10,平滑0.10,时间0.05] 映射到代码顺序[L,E,unsmooth,tm,dmin]
    params.drone_speed = drone_speed;

    fprintf('\n-- Step 2: Scheme A -- %d seeds (default weights, collect all generations) ---\n', n_seeds);
    all_pop_metrics_A = [];   % 全量候选（灰点可视化用）
    front_A = [];             % 各种子的非支配解（最终前沿并集用）
    tic;
    for s = seeds
        rng(s);
        [~, ~, ~, pop_metrics_s] = acosrar_multiobj_record(...
            start_pos, goal_pos, waypoints, Z1, Z2, Z3, ball, cylinder, safe_dist, params);
        all_pop_metrics_A = [all_pop_metrics_A; pop_metrics_s]; %#ok<AGROW>
        [p_idx_s, ~] = find_pareto_front(pop_metrics_s, obj_minimize);
        front_A = [front_A; pop_metrics_s(p_idx_s, :)]; %#ok<AGROW>
        fprintf('  Scheme A seed %d: +%d candidates, %d non-dominated\n', ...
            s, size(pop_metrics_s,1), numel(p_idx_s));
    end
    t_A = toc;
    fprintf('Scheme A: %.1fs elapsed, %d candidates collected across %d seeds\n', ...
        t_A, size(all_pop_metrics_A,1), n_seeds);

    %% Step 3: Scheme B -- multiple runs with different weight sets
    weight_sets = [
        0.55, 0.15, 0.15, 0.05, 0.10;   % 1 长度偏好
        0.12, 0.50, 0.15, 0.08, 0.15;   % 2 能耗偏好
        0.12, 0.12, 0.50, 0.12, 0.14;   % 3 平滑偏好
        0.15, 0.10, 0.10, 0.50, 0.15;   % 4 时间偏好
        0.30, 0.15, 0.15, 0.10, 0.30;   % 5 均衡
        0.20, 0.08, 0.08, 0.06, 0.58;   % 6 安全+长度混合
        0.12, 0.10, 0.10, 0.08, 0.60;   % 7 安全偏好-中
        0.10, 0.08, 0.08, 0.06, 0.68;   % 8 安全偏好-强
    ];
    n_weight_sets = size(weight_sets, 1);

    fprintf('\n-- Step 3: Scheme B -- %d weight sets x %d seeds ---\n', n_weight_sets, n_seeds);
    all_solutions_B = zeros(n_weight_sets * n_seeds, 5);
    optimal_paths_B = cell(n_weight_sets * n_seeds, 1);
    row = 0;

    for wi = 1:n_weight_sets
        params.weights = weight_sets(wi, :);
        for s = seeds
            rng(s);
            fprintf('  Weight set %d/%d, seed %d: [L E u t d] = [%.2f %.2f %.2f %.2f %.2f]  ', ...
                wi, n_weight_sets, s, params.weights);
            tic;
            [op_B, ~, ~, ~] = acosrar_multiobj_record(...
                start_pos, goal_pos, waypoints, Z1, Z2, Z3, ball, cylinder, safe_dist, params);
            toc_str = toc;
            row = row + 1;
            optimal_paths_B{row} = op_B;
            [L, E, u, tm, dmin] = compute_five_objectives(...
                op_B, Z1, Z2, Z3, ball, cylinder, safe_dist, drone_speed);
            all_solutions_B(row, :) = [L, E, u, tm, dmin];
            fprintf('  L=%.1f  E=%.1f  u=%.4f  t=%.1f  dmin=%.2f  (%.1fs)\n', ...
                L, E, u, tm, dmin, toc_str);
        end
    end

    %% Step 4: Merge + filter + Pareto ranking
    % 全量候选（灰点可视化）与前沿候选（各种子非支配并集 + Scheme B）
    full_pool  = [all_pop_metrics_A; all_solutions_B];
    front_pool = [front_A; all_solutions_B];

    full_valid  = full_pool(full_pool(:, 5) >= safe_dist, :);
    front_valid = front_pool(front_pool(:, 5) >= safe_dist, :);

    [pareto_idx, dominated_idx] = find_pareto_front(front_valid, obj_minimize);
    pareto_metrics = front_valid(pareto_idx, :);
    n_pareto = size(pareto_metrics, 1);

    fprintf('\n========== Pareto Ranking Results ==========\n');
    fprintf('  Total candidates collected (%d seeds): %d\n', n_seeds, size(full_pool, 1));
    fprintf('  Valid solutions (safety >= %.1f m)   : %d (%.1f%%)\n', safe_dist, size(full_valid,1), 100*size(full_valid,1)/size(full_pool,1));
    fprintf('  Per-seed non-dominated + Scheme B     : %d\n', size(front_pool, 1));
    fprintf('  Pareto non-dominated (final)         : %d\n', n_pareto);
    fprintf('  Dominated solutions                   : %d\n', length(dominated_idx));

    [~, sort_order] = sort(pareto_metrics(:, 1));
    pareto_metrics = pareto_metrics(sort_order, :);

    fprintf('\n  Pareto front solutions (sorted by Path Length):\n');
    fprintf('  %-6s %-12s %-12s %-12s %-12s %-14s\n', ...
        'Idx','Path L(m)','Energy(Wh)','Non-smooth','Time(s)','Safe Dist(m)');
    for i = 1:n_pareto
        fprintf('  %-6d %-10.2f %-10.2f %-10.4f %-10.2f %-10.2f\n', ...
            i, pareto_metrics(i,1), pareto_metrics(i,2), pareto_metrics(i,3), ...
            pareto_metrics(i,4), pareto_metrics(i,5));
    end

    %% Step 5: Visualization
    visualize_pareto_front(pareto_metrics, full_valid, map_type, start_pos, goal_pos);

end

fprintf('\n====================== Pareto Analysis Complete ======================\n');

%% ###################################################################
%%  以下所有函数均复制自 untitled.m, 仅做必要修改
%% ###################################################################

%% ====================== 环境构建 ======================
function [X, Y, Z1, Z2, Z3, ball, cylinder] = build_static_environment(map_type, map_range)
map_x = map_range(1):0.1:map_range(2);
map_y = map_range(1):0.1:map_range(2);
[X, Y] = meshgrid(map_x, map_y);
noise = zeros(size(X));   % 与统计实验一致：确定性无噪声地形

ball.num = 50;
cylinder.num_cyl = 50;
cylinder.r = 5;
cylinder.positions = zeros(0,3);

switch map_type
    case 'city'
        Z1 = zeros(size(X)) + noise; Z1(Z1<0)=0;
        Z2 = zeros(size(X)) + noise; Z2(Z2<0)=0;
        Z3 = zeros(size(X)) + noise; Z3(Z3<0)=0;
        ball.cx = 999; ball.cy = 999; ball.cz = 0; ball.r = 0;
        cylinder.positions = [40,70,100; 70,40,100; 50,50,100; 90,160,100; 160,80,100; ...
                              80,180,100; 100,180,100; 180,60,100; 180,100,100];
    case 'hill'
        Z1 = 60*exp(-((X-40).^2+(Y-140).^2)/(2*30^2)) + noise;
        Z2 = 60*exp(-((X-140).^2+(Y-40).^2)/(2*30^2)) + noise;
        Z3 = 60*exp(-((X-140).^2+(Y-140).^2)/(2*30^2)) + noise;
        Z1(Z1<0)=0; Z2(Z2<0)=0; Z3(Z3<0)=0;
        ball.cx = 999; ball.cy = 999; ball.cz = 0; ball.r = 0;
    case 'obstacle'
        Z1 = 90*exp(-((X-40).^2+(Y-140).^2)/(2*30^2)) + noise;
        Z2 = 90*exp(-((X-140).^2+(Y-40).^2)/(2*30^2)) + noise;
        Z3 = 120*exp(-((X-140).^2+(Y-140).^2)/(2*30^2)) + noise;
        Z1(Z1<0)=0; Z2(Z2<0)=0; Z3(Z3<0)=0;
        ball.cx = 100; ball.cy = 100; ball.cz = 0; ball.r = 50;
        cylinder.positions = [40,70,100; 70,40,100; 50,50,100; 90,160,100; 160,80,100; ...
                              80,180,100; 100,180,100; 180,60,100; 180,100,100];
    otherwise
        error('Unknown map type: %s', map_type);
end
end

%% ====================== A* 三维路径规划 ======================
function path = astar_3d(start, goal, map_range, grid_size, Z1, Z2, Z3, ball, cylinder, safe_dist)
x_min = map_range(1); x_max = map_range(2);
y_min = map_range(1); y_max = map_range(2);
z_min = map_range(1); z_max = map_range(2);
x_grid = x_min:grid_size:x_max;
y_grid = y_min:grid_size:y_max;
z_grid = z_min:grid_size:z_max;
start_idx(1) = find(abs(x_grid - start(1)) == min(abs(x_grid - start(1))), 1);
start_idx(2) = find(abs(y_grid - start(2)) == min(abs(y_grid - start(2))), 1);
start_idx(3) = find(abs(z_grid - start(3)) == min(abs(z_grid - start(3))), 1);
goal_idx(1)  = find(abs(x_grid - goal(1))  == min(abs(x_grid - goal(1))),  1);
goal_idx(2)  = find(abs(y_grid - goal(2))  == min(abs(y_grid - goal(2))),  1);
goal_idx(3)  = find(abs(z_grid - goal(3))  == min(abs(z_grid - goal(3))),  1);
node_struct.g = inf(length(x_grid), length(y_grid), length(z_grid));
node_struct.h = zeros(length(x_grid), length(y_grid), length(z_grid));
node_struct.f = inf(length(x_grid), length(y_grid), length(z_grid));
node_struct.parent = cell(length(x_grid), length(y_grid), length(z_grid));
node_struct.g(start_idx(1), start_idx(2), start_idx(3)) = 0;
node_struct.h(start_idx(1), start_idx(2), start_idx(3)) = enhanced_heuristic(start, goal, Z1, Z2, Z3, ball, cylinder, safe_dist);
node_struct.f(start_idx(1), start_idx(2), start_idx(3)) = node_struct.g(start_idx(1), start_idx(2), start_idx(3)) + node_struct.h(start_idx(1), start_idx(2), start_idx(3));
node_struct.parent{start_idx(1), start_idx(2), start_idx(3)} = [];
open_list = {start_idx};
close_list = [];
directions = combvec([-1,0,1], [-1,0,1], [-1,0,1])';
directions(all(directions == 0, 2), :) = [];
z_priority_dirs = directions(abs(directions(:,3))>0, :);
xy_dirs = directions(abs(directions(:,3))==0, :);
directions = [z_priority_dirs; xy_dirs];
found = false; iter_count = 0;
max_iter = 2 * length(x_grid)*length(y_grid)*length(z_grid);
while ~isempty(open_list) && ~found && iter_count < max_iter
    iter_count = iter_count + 1;
    f_values = arrayfun(@(i) node_struct.f(open_list{i}(1), open_list{i}(2), open_list{i}(3)), 1:length(open_list));
    [~, min_idx] = min(f_values);
    current_idx = open_list{min_idx}; open_list(min_idx) = [];
    close_list = [close_list; current_idx];
    if all(current_idx == goal_idx); found = true; break; end
    for d = 1:size(directions, 1)
        next_idx = current_idx + directions(d,:);
        if next_idx(1)<1 || next_idx(1)>length(x_grid) || next_idx(2)<1 || next_idx(2)>length(y_grid) || next_idx(3)<1 || next_idx(3)>length(z_grid)
            continue;
        end
        if ismember(next_idx, close_list, 'rows'); continue; end
        current_pos = [x_grid(current_idx(1)), y_grid(current_idx(2)), z_grid(current_idx(3))];
        next_pos    = [x_grid(next_idx(1)), y_grid(next_idx(2)), z_grid(next_idx(3))];
        if ~check_node_feasible(next_pos, Z1, Z2, Z3, ball, cylinder, safe_dist) || ...
           ~check_segment_feasible(current_pos, next_pos, Z1, Z2, Z3, ball, cylinder, safe_dist)
            continue;
        end
        g_new = node_struct.g(current_idx(1), current_idx(2), current_idx(3)) + norm(next_pos - current_pos);
        h_new = enhanced_heuristic(next_pos, goal, Z1, Z2, Z3, ball, cylinder, safe_dist);
        f_new = g_new + h_new;
        if f_new < node_struct.f(next_idx(1), next_idx(2), next_idx(3))
            node_struct.g(next_idx(1), next_idx(2), next_idx(3)) = g_new;
            node_struct.h(next_idx(1), next_idx(2), next_idx(3)) = h_new;
            node_struct.f(next_idx(1), next_idx(2), next_idx(3)) = f_new;
            node_struct.parent{next_idx(1), next_idx(2), next_idx(3)} = current_idx;
            in_open = false;
            for k = 1:length(open_list)
                if all(open_list{k} == next_idx); in_open = true; break; end
            end
            if ~in_open; open_list{end+1} = next_idx; end
        end
    end
end
if found
    path = []; current = goal_idx;
    while ~isempty(node_struct.parent{current(1), current(2), current(3)})
        cp = [x_grid(current(1)), y_grid(current(2)), z_grid(current(3))];
        path = [cp; path];
        current = node_struct.parent{current(1), current(2), current(3)};
    end
    path = [start; path]; path(end,:) = goal;
else
    error('A* 未找到可行路径');
end
end

%% ====================== A* 增强启发函数 ======================
function h = enhanced_heuristic(pos, goal, Z1, Z2, Z3, ball, cylinder, safe_dist)
euclid_dist = norm(pos - goal);
obstacle_penalty = 0;
x_idx = round((pos(1)-0)/0.1 + 1);
y_idx = round((pos(2)-0)/0.1 + 1);
if x_idx>=1 && x_idx<=size(Z1,2) && y_idx>=1 && y_idx<=size(Z1,1)
    mh = max([Z1(y_idx,x_idx), Z2(y_idx,x_idx), Z3(y_idx,x_idx)]);
    if pos(3) - mh < 9
        obstacle_penalty = obstacle_penalty + 200/(pos(3)-mh+1e-6);
    end
end
bd = norm(pos - [ball.cx, ball.cy, ball.cz]);
if ball.r > 0 && bd < ball.r + 9
    obstacle_penalty = obstacle_penalty + 300/(bd-ball.r+1e-6);
end
for i = 1:size(cylinder.positions,1)
    cp = cylinder.positions(i,:);
    dh = norm([pos(1),pos(2)] - [cp(1),cp(2)]);
    if dh < cylinder.r + 9 && pos(3) < cp(3)
        obstacle_penalty = obstacle_penalty + 250/(dh-cylinder.r+1e-6);
    end
end
h = euclid_dist*0.9 + obstacle_penalty;
end

%% ====================== 关键锚点提取 ======================
function waypoints = extract_key_waypoints(astar_path, grid_size)
if size(astar_path,1)<=2; waypoints = astar_path; return; end
waypoints = astar_path(1,:);
angle_threshold = 0.1;
for i = 2:size(astar_path,1)-1
    v1 = astar_path(i,:)-astar_path(i-1,:); v2 = astar_path(i+1,:)-astar_path(i,:);
    n1 = v1/(norm(v1)+1e-6); n2 = v2/(norm(v2)+1e-6);
    if acos(max(min(dot(n1,n2),1),-1)) > angle_threshold
        waypoints = [waypoints; astar_path(i,:)];
    end
end
waypoints = [waypoints; astar_path(end,:)];
fw = waypoints(1,:);
for i = 2:size(waypoints,1)
    if norm(waypoints(i,:)-fw(end,:)) >= grid_size; fw = [fw; waypoints(i,:)]; end
end
waypoints = fw;
end

%% ====================== Lévy flight 步长 ======================
function step = levy_flight(step_size)
beta = 1.5;
sigma = (gamma(1+beta)*sin(pi*beta/2)/(gamma((1+beta)/2)*beta*2^((beta-1)/2)))^(1/beta);
u = randn(1,3)*sigma; v = randn(1,3);
step = step_size * u ./ (abs(v).^(1/beta) + 1e-10);
end

%% ====================== 【核心修改版】ACOSRAR —— 记录每代所有个体的 5 目标值 ======================
function [optimal_path, fitness_history, metrics_history, all_pop_metrics] = ...
    acosrar_multiobj_record(start, goal, waypoints, Z1, Z2, Z3, ball, cylinder, safe_dist, params)

pop_size = params.pop_size;
max_iter = params.max_iter;
alpha = params.alpha; beta = params.beta; rho = params.rho; q0 = params.q0;
step_size = params.step_size; weights = params.weights; drone_speed = params.drone_speed;

pop = initialize_population(pop_size, start, goal, waypoints, Z1, Z2, Z3, ball, cylinder, safe_dist);
[~, metrics] = calculate_pop_fitness_5obj(pop, Z1, Z2, Z3, ball, cylinder, safe_dist, weights, drone_speed);
fitness = zeros(pop_size, 1);
for i = 1:pop_size, fitness(i) = calculate_absolute_fitness(metrics(i,:), weights); end
[best_fitness, best_idx] = min(fitness);
optimal_path = pop{best_idx};
global_best_metrics = metrics(best_idx,:);

fitness_history = zeros(1, max_iter);
metrics_history = zeros(max_iter, 5);
fitness_history(1) = calculate_absolute_fitness(global_best_metrics, weights);
metrics_history(1,:) = global_best_metrics;

% 【新增】收集每代所有个体的 metrics, 用于后续 Pareto 分析
all_pop_metrics = [];
all_pop_metrics = [all_pop_metrics; metrics];

n_waypoints = size(waypoints,1);
pheromone = ones(n_waypoints-1, 1);
pheromone_max = 10; pheromone_min = 0.1;

for iter = 2:max_iter
    for ant = 1:pop_size
        current_path = start; current_pos = start;
        for wp = 1:n_waypoints-1
            wp_start = waypoints(wp,:); wp_end = waypoints(wp+1,:);
            x_min = min(wp_start(1),wp_end(1))-1e-6; x_max = max(wp_start(1),wp_end(1))+1e-6;
            y_min = min(wp_start(2),wp_end(2))-1e-6; y_max = max(wp_start(2),wp_end(2))+1e-6;
            z_min = min(wp_start(3),wp_end(3))-1e-6; z_max = max(wp_start(3),wp_end(3))+1e-6;
            x_len = round((x_max-x_min)/0.1)+1; y_len = round((y_max-y_min)/0.1)+1; z_len = round((z_max-z_min)/0.1)+1;
            max_len = max([x_len,y_len,z_len]);
            x_range = linspace(x_min,x_max,max_len);
            y_range = linspace(y_min,y_max,max_len);
            z_range = linspace(z_min,z_max,max_len);
            tau = pheromone(wp);
            h_wp  = 1/(norm(current_pos-wp_end)+1e-6);
            h_goal= 1/(norm(current_pos-goal)+1e-6);
            eta   = 0.6*h_wp + 0.4*h_goal;
            prob  = (tau^alpha)*(eta^beta);
            if rand < q0
                dir_greedy = (wp_end - current_pos)/(norm(wp_end-current_pos)+1e-6);
                dir_safe = safe_direction(current_pos, Z1, Z2, Z3, ball, cylinder);
                w_len = weights(1); w_safe = weights(5);
                direction = (w_len*dir_greedy + w_safe*dir_safe);
                direction = direction/(norm(direction)+1e-6);
            else
                if iter < max_iter/3
                    levy_step = levy_flight(step_size*1.2);
                elseif iter < max_iter*2/3
                    levy_step = levy_flight(step_size*0.8);
                else
                    levy_step = levy_flight(step_size*0.4) + randn(1,3)*0.03;
                end
                direction = levy_step/(norm(levy_step)+1e-6);
            end
            segment_length = norm(wp_end - wp_start);
            step_adjust = 0.8 + 0.2*(1-iter/max_iter);
            new_pos = current_pos + direction*segment_length*step_size*step_adjust;
            if ~check_node_feasible(new_pos, Z1, Z2, Z3, ball, cylinder, safe_dist)
                new_pos = adaptive_repair(new_pos, Z1, Z2, Z3, ball, cylinder, safe_dist, wp_end);
            end
            new_pos(1) = max(x_range(1), min(x_range(end), new_pos(1)));
            new_pos(2) = max(y_range(1), min(y_range(end), new_pos(2)));
            new_pos(3) = max(z_range(1), min(z_range(end), new_pos(3)));
            current_path = [current_path; new_pos]; current_pos = new_pos;
        end
        current_path = [current_path; goal];
        current_path = prune_path(current_path, 0.15);
        current_path = repair_path(current_path, Z1, Z2, Z3, ball, cylinder, safe_dist);
        pop{ant} = current_path;
    end
    [~, metrics] = calculate_pop_fitness_5obj(pop, Z1, Z2, Z3, ball, cylinder, safe_dist, weights, drone_speed, global_best_metrics);
    fitness = zeros(pop_size, 1);
    for i = 1:pop_size, fitness(i) = calculate_absolute_fitness(metrics(i,:), weights); end
    [cur_best_fit, cur_best_idx] = min(fitness);
    if cur_best_fit < best_fitness
        best_fitness = cur_best_fit; optimal_path = pop{cur_best_idx};
        global_best_metrics = metrics(cur_best_idx,:);
    end
    pheromone = (1-rho)*pheromone;
    delta_tau = 1/(best_fitness+1e-6)*1.8;
    pheromone = pheromone + delta_tau;
    pheromone = min(max(pheromone, pheromone_min), pheromone_max);

    % 【新增】这一代所有个体的 metrics 加入收集池
    all_pop_metrics = [all_pop_metrics; metrics];

    fitness_history(iter) = calculate_absolute_fitness(global_best_metrics, weights);
    metrics_history(iter,:) = global_best_metrics;
end
end

%% ====================== 种群初始化 ======================
function pop = initialize_population(pop_size, start, goal, waypoints, Z1, Z2, Z3, ball, cylinder, safe_dist)
pop = cell(1, pop_size);
n_waypoints = size(waypoints,1);
segment_node_num = 8;
for ant = 1:pop_size
    current_path = start;
    for wp_idx = 1:n_waypoints
        if wp_idx == 1
            seg_start = start; seg_end = waypoints(wp_idx,:);
        else
            seg_start = waypoints(wp_idx-1,:); seg_end = waypoints(wp_idx,:);
        end
        seg_path = generate_segment_path(seg_start, seg_end, segment_node_num, Z1, Z2, Z3, ball, cylinder, safe_dist);
        current_path = [current_path; seg_path(2:end,:)];
    end
    final_seg = generate_segment_path(waypoints(end,:), goal, segment_node_num, Z1, Z2, Z3, ball, cylinder, safe_dist);
    current_path = [current_path; final_seg(2:end,:)];
    current_path = prune_path(current_path, 0.15);
    current_path = repair_path(current_path, Z1, Z2, Z3, ball, cylinder, safe_dist);
    pop{ant} = current_path;
end
end

function seg_path = generate_segment_path(seg_start, seg_end, node_num, Z1, Z2, Z3, ball, cylinder, safe_dist)
seg_path = zeros(node_num, 3);
t = linspace(0,1,node_num);
seg_path(:,1) = seg_start(1) + t*(seg_end(1)-seg_start(1));
seg_path(:,2) = seg_start(2) + t*(seg_end(2)-seg_start(2));
seg_path(:,3) = seg_start(3) + t*(seg_end(3)-seg_start(3));
seg_len = norm(seg_end - seg_start);
disturb_amp = max(seg_len*0.15, 6);
for i = 2:node_num-1
    seg_path(i,:) = seg_path(i,:) + disturb_amp*(rand(1,3)-0.5);
end
for i = 1:node_num
    if ~check_node_feasible(seg_path(i,:), Z1, Z2, Z3, ball, cylinder, safe_dist)
        seg_path(i,:) = adaptive_repair(seg_path(i,:), Z1, Z2, Z3, ball, cylinder, safe_dist, seg_end);
    end
end
end

%% ====================== 【5 目标版】种群适应度计算 ======================
function [fitness, metrics] = calculate_pop_fitness_5obj(pop, Z1, Z2, Z3, ball, cylinder, safe_dist, weights, drone_speed, global_best_metrics)
pop_size = length(pop);
metrics = zeros(pop_size, 5);
fitness = zeros(pop_size, 1);
for i = 1:pop_size
    path = pop{i}; n_points = size(path,1);
    path_diff = diff(path,1,1);
    L = sum(sqrt(sum(path_diff.^2, 2)));
    d_all = zeros(n_points + 5*(n_points-1), 1);
    for j = 1:n_points
        d_all(j) = safe_distance(path(j,:), Z1, Z2, Z3, ball, cylinder);
    end
    for j = 1:n_points-1
        for s = 1:5
            pos = path(j,:) + (path(j+1,:)-path(j,:))*(s/6);
            d_all(n_points + 5*(j-1) + s) = safe_distance(pos, Z1, Z2, Z3, ball, cylinder);
        end
    end
    d_min = max(min(d_all), 0);
    height_change = sum(abs(diff(path(:,3))));
    turn_angle_sum = 0; accel_sum = 0;
    for j = 2:n_points-1
        v1 = path(j,:)-path(j-1,:); v2 = path(j+1,:)-path(j,:);
        n1 = v1/(norm(v1)+1e-6); n2 = v2/(norm(v2)+1e-6);
        ang = acos(max(min(dot(n1,n2),1),-1));
        turn_angle_sum = turn_angle_sum + ang;
        accel_sum = accel_sum + 2*sin(ang/2);
    end
    E = L*0.1 + height_change*0.3 + 0.2*turn_angle_sum + 0.15*accel_sum;
    smooth_cos = 0; count = 0;
    for j = 2:n_points-1
        v1 = path(j,:)-path(j-1,:); v2 = path(j+1,:)-path(j,:);
        if norm(v1)>1e-6 && norm(v2)>1e-6
            smooth_cos = smooth_cos + dot(v1,v2)/(norm(v1)*norm(v2));
            count = count + 1;
        end
    end
    smooth_cos = smooth_cos/(count+1e-6);
    unsmooth = 1 - smooth_cos;
    tm = L/drone_speed;
    metrics(i,:) = [L, E, unsmooth, tm, d_min];
end

norm_metrics = zeros(size(metrics));
if nargin < 11 || isempty(global_best_metrics)
    for col = 1:5
        cm = min(metrics(:,col)); cM = max(metrics(:,col));
        if cM-cm < 1e-6; norm_metrics(:,col)=0;
        elseif col == 5; norm_metrics(:,col) = 1 - (metrics(:,col)-cm)/(cM-cm);
        else; norm_metrics(:,col) = (metrics(:,col)-cm)/(cM-cm); end
    end
else
    for col = 1:5
        cm = min([min(metrics(:,col)), global_best_metrics(col)]);
        cM = max([max(metrics(:,col)), global_best_metrics(col)]);
        if cM-cm < 1e-6; norm_metrics(:,col)=0;
        elseif col == 5; norm_metrics(:,col) = 1 - (metrics(:,col)-cm)/(cM-cm);
        else; norm_metrics(:,col) = (metrics(:,col)-cm)/(cM-cm); end
    end
end

for i = 1:pop_size
    fitness(i) = weights(1)*norm_metrics(i,1) + weights(2)*norm_metrics(i,2) + ...
                 weights(3)*norm_metrics(i,3) + weights(4)*norm_metrics(i,4) + ...
                 weights(5)*norm_metrics(i,5);
end
fitness = fitness/(max(fitness)+1e-6);
end

%% ====================== 绝对适应度 ======================
function abs_fit = calculate_absolute_fitness(metrics_row, weights)
ref_L = 300; ref_E = 60; ref_u = 2; ref_t = ref_L/10; ref_safe = 10;
n_L = min(metrics_row(1)/ref_L, 1);
n_E = min(metrics_row(2)/ref_E, 1);
n_u = min(metrics_row(3)/ref_u, 1);
n_t = min(metrics_row(4)/ref_t, 1);
n_safe = max(0, 1 - metrics_row(5)/ref_safe);
abs_fit = weights(1)*n_L + weights(2)*n_E + weights(3)*n_u + weights(4)*n_t + weights(5)*n_safe;
end

%% ====================== 静态安全距离 ======================
function sd = safe_distance(pos, Z1, Z2, Z3, ball, cylinder)
x = pos(1); y = pos(2); z = pos(3);
ds = [];
xi = round((x-0)/0.1 + 1); yi = round((y-0)/0.1 + 1);
if xi>=1 && xi<=size(Z1,2) && yi>=1 && yi<=size(Z1,1)
    mh = max([Z1(yi,xi), Z2(yi,xi), Z3(yi,xi)]);
    ds = [ds, z-mh];
end
if ball.r > 0
    bd = norm(pos - [ball.cx, ball.cy, ball.cz]) - ball.r;
    ds = [ds, bd];
end
for i = 1:size(cylinder.positions,1)
    cx = cylinder.positions(i,1); cy = cylinder.positions(i,2); cz = cylinder.positions(i,3);
    dh = norm([x-cx, y-cy]);
    if z>=0 && z<=cz; dc = dh - cylinder.r;
    elseif z<0; dc = max(dh-cylinder.r, -z);
    else; dc = max(dh-cylinder.r, z-cz); end
    ds = [ds, dc];
end
sd = max(min(ds), 0);
end

%% ====================== 节点/线段可行性检查 ======================
function feasible = check_node_feasible(pos, Z1, Z2, Z3, ball, cylinder, safe_dist)
x = pos(1); y = pos(2); z = pos(3);
xi = round((x-0)/0.1 + 1); yi = round((y-0)/0.1 + 1);
if xi>=1 && xi<=size(Z1,2) && yi>=1 && yi<=size(Z1,1)
    if z < Z1(yi,xi)+safe_dist || z < Z2(yi,xi)+safe_dist || z < Z3(yi,xi)+safe_dist
        feasible = false; return;
    end
end
if ball.r > 0 && norm([x,y,z]-[ball.cx,ball.cy,ball.cz]) < ball.r+safe_dist
    feasible = false; return;
end
for i = 1:size(cylinder.positions,1)
    cx = cylinder.positions(i,1); cy = cylinder.positions(i,2); cz = cylinder.positions(i,3);
    if norm([x-cx,y-cy]) < cylinder.r+safe_dist && z>=0 && z<=cz
        feasible = false; return;
    end
end
feasible = true;
end

function feasible = check_segment_feasible(p1, p2, Z1, Z2, Z3, ball, cylinder, safe_dist)
num_samples = 10;
for i = 1:num_samples
    pos = p1 + (p2-p1)*(i-1)/(num_samples-1);
    if ~check_node_feasible(pos, Z1, Z2, Z3, ball, cylinder, safe_dist)
        feasible = false; return;
    end
end
feasible = true;
end

%% ====================== 自适应修复 ======================
function repaired_pos = adaptive_repair(pos, Z1, Z2, Z3, ball, cylinder, safe_dist, target)
dirs = combvec([-1,0,1], [-1,0,1], [-1,0,1]);
dirs(:, all(dirs==0,1)) = [];
costs = inf(1, size(dirs,2));
for d = 1:size(dirs,2)
    cand = pos + dirs(:,d)'*safe_dist;
    if check_node_feasible(cand, Z1, Z2, Z3, ball, cylinder, safe_dist)
        costs(d) = 0.7*norm(cand-target) + 0.3*norm(dirs(:,d)'*safe_dist);
    end
end
[min_cost, best_d] = min(costs);
if min_cost ~= inf
    repaired_pos = pos + dirs(:,best_d)'*safe_dist;
else
    if norm(target-pos) < 1e-6
        rd = randn(1,3); rd = rd/norm(rd); repaired_pos = pos + rd*safe_dist*2;
    else
        td = (target-pos)/norm(target-pos); repaired_pos = pos + td*safe_dist*2;
    end
    if ~check_node_feasible(repaired_pos, Z1, Z2, Z3, ball, cylinder, safe_dist)
        repaired_pos = adaptive_repair(repaired_pos, Z1, Z2, Z3, ball, cylinder, safe_dist, target);
    end
end
end

%% ====================== 路径修剪 ======================
function pruned_path = prune_path(original_path, angle_threshold)
if size(original_path,1)<=3; pruned_path = original_path; return; end
pruned_path = original_path(1,:);
for i = 2:size(original_path,1)-1
    v1 = original_path(i,:)-original_path(i-1,:);
    v2 = original_path(i+1,:)-original_path(i,:);
    n1 = v1/(norm(v1)+1e-6); n2 = v2/(norm(v2)+1e-6);
    ang = acos(max(min(dot(n1,n2),1),-1));
    if ang > angle_threshold; pruned_path = [pruned_path; original_path(i,:)]; end
end
pruned_path = [pruned_path; original_path(end,:)];
end

%% ====================== 路径修复 ======================
function repaired_path = repair_path(path, Z1, Z2, Z3, ball, cylinder, safe_dist)
repaired_path = path;
n = size(repaired_path,1);
for i = 1:n
    if ~check_node_feasible(repaired_path(i,:), Z1, Z2, Z3, ball, cylinder, safe_dist)
        if i < n; tgt = repaired_path(i+1,:); else; tgt = repaired_path(i-1,:); end
        repaired_path(i,:) = adaptive_repair(repaired_path(i,:), Z1, Z2, Z3, ball, cylinder, safe_dist, tgt);
    end
end
i = 1;
while i < size(repaired_path,1)
    p1 = repaired_path(i,:); p2 = repaired_path(i+1,:);
    if ~check_segment_feasible(p1, p2, Z1, Z2, Z3, ball, cylinder, safe_dist)
        insert = generate_segment_path(p1, p2, 5, Z1, Z2, Z3, ball, cylinder, safe_dist);
        insert = insert(2:end-1,:);
        repaired_path = [repaired_path(1:i,:); insert; repaired_path(i+1:end,:)];
    else
        i = i+1;
    end
end
end

%% ====================== 安全方向 ======================
function dir = safe_direction(pos, Z1, Z2, Z3, ball, cylinder)
% 远离最近障碍的方向 = 安全距离场梯度（指向 clearance 增大最快的方向）
h = 1.0;
g = zeros(1,3);
for k = 1:3
    e = zeros(1,3); e(k) = h;
    g(k) = safe_distance(pos+e, Z1, Z2, Z3, ball, cylinder) - safe_distance(pos-e, Z1, Z2, Z3, ball, cylinder);
end
dir = g/(norm(g)+1e-6);
end

%% ====================== 5 目标统一计算 ======================
function [L, E, unsmooth, tm, d_min] = compute_five_objectives(path, Z1, Z2, Z3, ball, cylinder, safe_dist, drone_speed)
n = size(path,1);
path_diff = diff(path,1,1);
L = sum(sqrt(sum(path_diff.^2,2)));
d_all = zeros(n + 5*(n-1), 1);
for j = 1:n; d_all(j) = safe_distance(path(j,:), Z1, Z2, Z3, ball, cylinder); end
for j = 1:n-1
    for s = 1:5
        pos = path(j,:) + (path(j+1,:)-path(j,:))*(s/6);
        d_all(n + 5*(j-1) + s) = safe_distance(pos, Z1, Z2, Z3, ball, cylinder);
    end
end
d_min = max(min(d_all), 0);
height_change = sum(abs(diff(path(:,3))));
turn_angle_sum = 0; accel_sum = 0;
for j = 2:n-1
    v1 = path(j,:)-path(j-1,:); v2 = path(j+1,:)-path(j,:);
    n1 = v1/(norm(v1)+1e-6); n2 = v2/(norm(v2)+1e-6);
    ang = acos(max(min(dot(n1,n2),1),-1));
    turn_angle_sum = turn_angle_sum + ang;
    accel_sum = accel_sum + 2*sin(ang/2);
end
E = L*0.1 + height_change*0.3 + 0.2*turn_angle_sum + 0.15*accel_sum;
smooth_cos = 0; cnt = 0;
for j = 2:n-1
    v1 = path(j,:)-path(j-1,:); v2 = path(j+1,:)-path(j,:);
    if norm(v1)>1e-6 && norm(v2)>1e-6
        smooth_cos = smooth_cos + dot(v1,v2)/(norm(v1)*norm(v2));
        cnt = cnt + 1;
    end
end
smooth_cos = smooth_cos/(cnt+1e-6);
unsmooth = 1 - smooth_cos;
tm = L/drone_speed;
end

%% ====================== 地图英文名 ======================
function str = map_name_of(map_type)
switch map_type
    case 'city';     str = 'City Map';
    case 'hill';     str = 'Hilly Map';
    case 'obstacle'; str = 'Obstacle Map';
    otherwise;       str = 'Unknown Map';
end
end

%% ====================== 显示用数据扰动 (仅用于可视化, 不影响计算) ======================
function m_out = add_display_jitter(m_in, jitter_range)
m_out = m_in;
for col = 1:size(m_in, 2)
    col_vals = m_in(:, col);
    col_range = max(col_vals) - min(col_vals);
    if col_range < 1e-6
        m_out(:, col) = col_vals + (rand(size(col_vals))-0.5) * jitter_range;
    end
end
end

%% ###################################################################
%%  帕累托前沿可视化函数
%% ###################################################################

function visualize_pareto_front(pareto_metrics, valid_metrics, map_type, start_pos, goal_pos)

obj_names  = {'Path Length (m)', 'Energy (Wh)', 'Non-smoothness', 'Flight Time (s)', 'Min Safe Distance (m)'};
obj_labels = {'Path Length', 'Energy', 'Non-smoothness', 'Flight Time', 'Safe Distance'};
n_obj = 5;

jitter_range = 0.05;
pareto_disp = add_display_jitter(pareto_metrics, jitter_range);
valid_disp  = add_display_jitter(valid_metrics,  jitter_range);

%% ---- Fig 1: Pairwise scatter matrix ----
fig1 = figure('Position', [50, 50, 1400, 1400], 'Name', ['Pareto Scatter Matrix - ' map_name_of(map_type)], 'Color', 'w');

% 用 tiledlayout 紧凑排列：缩小子图之间及四周的空白，并自动为坐标标签留出空间避免重叠
t = tiledlayout(n_obj, n_obj, 'TileSpacing', 'compact', 'Padding', 'compact');

for i = 1:n_obj
    for j = 1:n_obj
        nexttile;
        hold on; grid on; box on;

        if i == j
            histogram(valid_disp(:, i), 30, 'FaceColor', [0.7 0.8 0.9], 'EdgeColor', 'none');
            xlabel(obj_labels{i}, 'FontSize', 18);
            ylabel('Frequency', 'FontSize', 18);
        else
            scatter(valid_disp(:, j),  valid_disp(:, i),  8,  [0.75 0.75 0.75], 'filled', 'MarkerFaceAlpha', 0.4);
            scatter(pareto_disp(:, j), pareto_disp(:, i), 40, 'r', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
            xlabel(obj_labels{j}, 'FontSize', 18);
            ylabel(obj_labels{i}, 'FontSize', 18);
        end
        set(gca, 'FontSize', 14);
    end
end

% 图例放到整张图上方（横排），不再挤在子图旁边，避免与子图重合
lg = legend('All feasible solutions', 'Pareto-optimal solutions', ...
    'Orientation', 'horizontal', 'FontSize', 14);
lg.Layout.Tile = 'north';

% 自动导出为紧凑矢量 PDF（无明显白边，需 R2020a 及以上）
exportgraphics(fig1, ['Pareto Scatter Matrix - ' map_name_of(map_type) '.pdf'], ...
    'ContentType', 'vector', 'BackgroundColor', 'white');

%% ---- Fig 2: Core trade-off ----
figure('Position', [100, 100, 900, 700], 'Name', ['Path Length vs Safe Distance - ' map_name_of(map_type)], 'Color', 'w');
hold on; grid on; box on;

scatter(valid_disp(:, 1),  valid_disp(:, 5),  15, [0.7 0.7 0.7], 'filled', 'MarkerFaceAlpha', 0.5);
scatter(pareto_disp(:, 1), pareto_disp(:, 5), 60, pareto_disp(:, 2), 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.8);
cb = colorbar; cb.Label.String = 'Energy (Wh)'; cb.Label.FontSize = 14;

[~, order] = sort(pareto_metrics(:, 1));
plot(pareto_metrics(order, 1), pareto_metrics(order, 5), '-r', 'LineWidth', 1.5);

xlabel('Path Length (m)', 'FontSize', 16);
ylabel('Min Safe Distance (m)', 'FontSize', 16);
title(['Pareto Front: Path Length vs. Safe Distance (' map_name_of(map_type) ')'], 'FontSize', 16, 'FontWeight', 'bold');
legend('All candidate solutions', 'Pareto-optimal solutions', 'Location', 'best', 'FontSize', 13);
set(gca, 'FontSize', 13);

%% ---- Fig 3: 3D scatter ----
figure('Position', [150, 150, 900, 700], 'Name', ['3D Pareto - ' map_name_of(map_type)], 'Color', 'w');
hold on; grid on; box on; view(3);

scatter3(valid_disp(:, 1),  valid_disp(:, 5),  valid_disp(:, 2),  15, [0.75 0.75 0.75], 'filled', 'MarkerFaceAlpha', 0.4);
scatter3(pareto_disp(:, 1), pareto_disp(:, 5), pareto_disp(:, 2), 80, pareto_disp(:, 4), 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1);
cb = colorbar; cb.Label.String = 'Flight Time (s)'; cb.Label.FontSize = 14;

xlabel('Path Length (m)', 'FontSize', 16);
ylabel('Min Safe Distance (m)', 'FontSize', 16);
zlabel('Energy (Wh)', 'FontSize', 16);
title(['Pareto Front \cdot 3D Projection (' map_name_of(map_type) ')'], 'FontSize', 16, 'FontWeight', 'bold');
legend('All candidate solutions', 'Pareto-optimal solutions', 'Location', 'best', 'FontSize', 13);
set(gca, 'FontSize', 13);

%% ---- Fig 4: Radar chart — up to 6 representative Pareto solutions ----
figure('Position', [200, 200, 900, 700], 'Name', ['Radar - ' map_name_of(map_type)], 'Color', 'w');
ax_radar = polaraxes; hold(ax_radar, 'on');

n_p = size(pareto_metrics, 1);
n_rep = 6;   % 雷达图显示的代表解个数（5 个目标极值 + 最分散补充）
if n_p > n_rep
    % 1) 每个目标取一个极值点（长度/能耗/不平滑/时间 取最小，安全距离取最大）
    cand = zeros(1, 5);
    [~, cand(1)] = min(pareto_metrics(:,1));   % 最短路径
    [~, cand(2)] = min(pareto_metrics(:,2));   % 最低能耗
    [~, cand(3)] = min(pareto_metrics(:,3));   % 最平滑
    [~, cand(4)] = min(pareto_metrics(:,4));   % 最快
    [~, cand(5)] = max(pareto_metrics(:,5));   % 最安全
    rep_idx = unique(cand);   % 去重（极值点可能重合）

    % 2) 归一化到 [0,1]，不足 n_rep 个时贪心补"离已选集合最远"的点（maximin）
    P = pareto_metrics;
    for col = 1:5
        lo = min(P(:,col)); hi = max(P(:,col));
        if hi - lo < 1e-9, P(:,col) = 0; else, P(:,col) = (P(:,col) - lo) / (hi - lo); end
    end
    while numel(rep_idx) < n_rep
        rest = setdiff(1:n_p, rep_idx);
        best = rest(1); best_d = -inf;
        for i = rest
            d = min(sqrt(sum((P(i,:) - P(rep_idx,:)).^2, 2)));
            if d > best_d, best_d = d; best = i; end
        end
        rep_idx(end+1) = best; %#ok<AGROW>
    end
else
    rep_idx = 1:n_p;
end

norm_radar = zeros(length(rep_idx), n_obj);
for col = 1:n_obj
    col_min = min(pareto_metrics(:, col));
    col_max = max(pareto_metrics(:, col));
    if col_max - col_min < 1e-6
        norm_radar(:, col) = 0.5;
    else
        if col == 5
            norm_radar(:, col) = (pareto_metrics(rep_idx, col) - col_min) / (col_max - col_min);
        else
            norm_radar(:, col) = 1 - (pareto_metrics(rep_idx, col) - col_min) / (col_max - col_min);
        end
    end
end

radar_labels = {'Path Length\downarrow', 'Energy\downarrow', 'Non-smoothness\downarrow', 'Flight Time\downarrow', 'Safe Distance\uparrow'};
colors = lines(length(rep_idx));

theta = linspace(0, 2*pi, n_obj+1);
for k = 1:length(rep_idx)
    vals = [norm_radar(k, :), norm_radar(k, 1)];
    plot(ax_radar, theta, vals, '-o', 'Color', colors(k,:), 'LineWidth', 2, ...
        'MarkerSize', 6, 'MarkerFaceColor', colors(k,:));
    fill(ax_radar, theta, vals, colors(k,:), 'FaceAlpha', 0.08, 'EdgeColor', 'none');
end

ax_radar.ThetaTick = rad2deg(theta(1:end-1));   % 与五边形顶点对齐（0°,72°,144°,216°,288°）
ax_radar.ThetaTickLabel = radar_labels;
ax_radar.RLim = [0 1];
ax_radar.RTick = [0.2 0.4 0.6 0.8 1.0];
ax_radar.RTickLabel = {'0.2','0.4','0.6','0.8','1.0'};
ax_radar.FontSize = 18;

legend_entries = cell(length(rep_idx), 1);
for k = 1:length(rep_idx)
    legend_entries{k} = sprintf('Sol#%d (L=%.1f, d_{min}=%.2f)', rep_idx(k), ...
        pareto_metrics(rep_idx(k),1), pareto_metrics(rep_idx(k),5));
end
legend(ax_radar, legend_entries, 'Location', 'bestoutside', 'FontSize', 13);

title(['Pareto Representative Solutions \cdot Radar Chart (' map_name_of(map_type) ')'], 'FontSize', 16, 'FontWeight', 'bold');
end