%% ====================== 主函数：MHTA+CAMO-AC三维路径规划（静态规划 + 动态障碍迎面避障演示） ======================
clear; clc; close all;
warning off;
%% ====================== 1. 环境与算法参数初始化 ======================
map_range = [0, 200];       
grid_size_mhta = 3;         
safe_dist = 2.5;              
drone_speed = 10;             
[X, Y, Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, dynamic_obs_params] = generate_environment(map_range);
start_pos = [10, 10, 50];    
goal_pos = [190, 190, 80];   
weights = [0.4, 0.35, 0.10, 0.10, 0.05];  % 对齐论文 AHP [长度0.40,安全0.35,能耗0.10,平滑0.10,时间0.05]
camo_ac_params.pop_size = 30;
camo_ac_params.max_iter = 120;

camo_ac_params.alpha = 0.8;           
camo_ac_params.beta = 2.5;            
camo_ac_params.rho = 0.2;             
camo_ac_params.q0 = 0.85;
% 提高贪婪概率，减少随机Levy飞行
camo_ac_params.step_size = 1.2;        % 与静态实验一致
camo_ac_params.weights = weights;      
camo_ac_params.drone_speed = drone_speed;
% 规划阶段只考虑静态障碍物：构造一个"空"的动态障碍物结构体，
% 传入各算法/检测函数时不会产生任何动态障碍约束（循环空矩阵即跳过）。
static_dynamic_obs.num = 0;
static_dynamic_obs.r = dynamic_obs_params.r;
static_dynamic_obs.speed = 0;
static_dynamic_obs.params = zeros(0, 7);
static_dynamic_obs.curr_pos = zeros(0, 3);
camo_ac_params.dynamic_obs = static_dynamic_obs;
%% ====================== 2. 三维MHTA算法生成粗路径 ======================
fprintf('====================== 步骤1：三维MHTA粗路径生成 ======================\n');
mhta_path = mhta_search(start_pos, goal_pos, map_range, grid_size_mhta, ...
    Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, static_dynamic_obs, safe_dist);
waypoints = extract_key_waypoints(mhta_path, grid_size_mhta);
fprintf('MHTA粗路径生成完成！路径节点数：%d，提取关键锚点数：%d\n', size(mhta_path,1), size(waypoints,1));
%% ====================== 3. CAMO-AC多目标路径优化 ======================
fprintf('\n====================== 步骤2：CAMO-AC多目标优化 ======================\n');
[optimal_path, fitness_history, metrics_history] = camo_ac_optimize(start_pos, goal_pos, waypoints, ...
    Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, static_dynamic_obs, safe_dist, camo_ac_params);
fprintf('CAMO-AC优化完成！最优路径原始节点数：%d\n', size(optimal_path,1));
%% ====================== 4. 路径后处理与可行性验证 ======================
% 提高剪枝角度阈值，大幅减少弯弯绕绕，让轨迹更平直
pruned_path = prune_path(optimal_path, 0.35);
smooth_path = spline_smooth(pruned_path);
% 强制首尾点与起点/终点完全一致，确保飞行轨迹从绿色球心出发、到蓝色球心结束
smooth_path(1, :) = start_pos;
smooth_path(end, :) = goal_pos;
% 静态避让校正：spline 平滑可能把路径"拉直"而切进圆柱/小球/山体的安全距离。
% 把离障碍太近的点沿径向向外推开，保证留出 safe_dist，让规划路线可见地绕开障碍。
smooth_path = enforce_static_clearance(smooth_path, Z_mountain1, Z_mountain2, Z_mountain3, ...
    ball_params, cylinder_params, safe_dist);
smooth_path(1, :) = start_pos;
smooth_path(end, :) = goal_pos;
[is_feasible, min_safe_dist] = check_path_feasibility(smooth_path, Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, static_dynamic_obs, safe_dist);
%% ====================== 4.5 设计"迎面逼近"动态障碍物（基于实际规划路径标定） ======================
% 目标：让动态障碍物在无人机飞行途中多次(>=5次)迎面逼近，迫使其脱离棕色
% 规划路径避障、避开后再回归原路径。每个障碍物从相遇点前方沿路径走廊
% 迎面飞来，并按"无人机预计到达时刻"标定穿越相遇点的时机（含每次避障
% 约25步的耗时补偿），从而保证与无人机发生多次迎面遭遇。
sim_uav_speed = 2.0;   % 飞行动画中无人机每步前进距离（与 visualize_results 保持一致）
dynamic_obs_params = gen_dyn_v2(smooth_path);
fprintf('四类动态障碍物设计完成！总数：%d（直线×3 圆弧×3 变速×4 垂直×4）\n', dynamic_obs_params.num);
%% ====================== 5. 核心输出1：路径数据完整输出 ======================
fprintf('\n====================== 最终路径输出结果 ======================\n');
if is_feasible
    fprintf('✅ 最终路径可行性验证通过！\n');
else
    fprintf('⚠️  最终路径存在碰撞风险，建议调整安全距离或算法参数！\n');
end
fprintf('路径最小安全距离：%.2fm（设定阈值：%.2fm）\n', min_safe_dist, safe_dist);
fprintf('最终平滑路径节点数：%d\n', size(smooth_path,1));
fprintf('路径起点坐标：[%.2f, %.2f, %.2f]\n', smooth_path(1,1), smooth_path(1,2), smooth_path(1,3));
fprintf('路径终点坐标：[%.2f, %.2f, %.2f]\n', smooth_path(end,1), smooth_path(end,2), smooth_path(end,3));
assignin('base', 'final_smooth_path', smooth_path);    
assignin('base', 'mhta_raw_path', mhta_path);        
assignin('base', 'fitness_convergence', fitness_history); 
assignin('base', 'metrics_convergence', metrics_history); 
save('drone_path_result.mat', 'smooth_path', 'mhta_path', 'fitness_history', 'metrics_history');
writematrix(smooth_path, 'drone_smooth_path.csv');
fprintf('📁 路径数据已导出：drone_path_result.mat、drone_smooth_path.csv\n');
%% ====================== 7. 三维路径可视化（静态规划路径 + 动态障碍物避障动画） ======================
visualize_results(X, Y, Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, ...
    dynamic_obs_params, smooth_path, start_pos, goal_pos, safe_dist, sim_uav_speed);
%% ====================== 7. 多目标指标对比输出 ======================
fprintf('\n====================== 多目标指标对比结果 ======================\n');
mhta_metrics = calculate_multiobj_metrics(mhta_path, Z_mountain1, Z_mountain2, Z_mountain3, ...
    ball_params, cylinder_params, static_dynamic_obs, safe_dist, drone_speed);
optimal_metrics = calculate_multiobj_metrics(smooth_path, Z_mountain1, Z_mountain2, Z_mountain3, ...
    ball_params, cylinder_params, static_dynamic_obs, safe_dist, drone_speed);
metrics_names = {
    '路径长度(m)', '越小越优';
    '最小安全距离(m)', '越大越优';
    '能耗(Wh)', '越小越优';
    '平滑度(余弦均值)', '越接近1越优';
    '飞行时间(s)', '越小越优'
    };
for i = 1:5
    mhta_val = mhta_metrics(i);
    optimal_val = optimal_metrics(i);
    improve_rate = 0;
    switch i
        case 1 
            if mhta_val < 1e-6
                improve_rate = 0;
            else
                improve_rate = (mhta_val - optimal_val) / mhta_val * 100;
            end
        case 2 
            if mhta_val < 1e-6
                improve_rate = (optimal_val - mhta_val) * 100;
            else
                improve_rate = (optimal_val - mhta_val) / mhta_val * 100;
            end
        case 3 
            if mhta_val < 1e-6
                improve_rate = 0;
            else
                improve_rate = (mhta_val - optimal_val) / mhta_val * 100;
            end
        case 4 
            mhta_smooth = 1 - mhta_val;
            optimal_smooth = 1 - optimal_val;
            mhta_val = mhta_smooth;
            optimal_val = optimal_smooth;
            if mhta_val < 1e-6
                improve_rate = (optimal_val - mhta_val) * 100;
            else
                improve_rate = (optimal_val - mhta_val) / mhta_val * 100;
            end
        case 5 
            if mhta_val < 1e-6
                improve_rate = 0;
            else
                improve_rate = (mhta_val - optimal_val) / mhta_val * 100;
            end
    end
    fprintf('%s：MHTA粗路径=%.2f | 优化后=%.2f | 提升：%.2f%% | 优化方向：%s\n', ...
        metrics_names{i,1}, mhta_val, optimal_val, improve_rate, metrics_names{i,2});
end
fprintf('\n====================== 规划流程全部完成 ======================\n');

%% 1. 三维MHTA路径规划函数（修复索引报错）
function path = mhta_search(start, goal, map_range, grid_size, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
x_min = map_range(1); x_max = map_range(2);
y_min = map_range(1); y_max = map_range(2);
z_min = map_range(1); z_max = map_range(2);
x_grid = x_min:grid_size:x_max;
y_grid = y_min:grid_size:y_max;
z_grid = z_min:grid_size:z_max;
start_idx(1) = find(abs(x_grid - start(1)) == min(abs(x_grid - start(1))), 1);
start_idx(2) = find(abs(y_grid - start(2)) == min(abs(y_grid - start(2))), 1);
start_idx(3) = find(abs(z_grid - start(3)) == min(abs(z_grid - start(3))), 1);
goal_idx(1) = find(abs(x_grid - goal(1)) == min(abs(x_grid - goal(1))), 1);
goal_idx(2) = find(abs(y_grid - goal(2)) == min(abs(y_grid - goal(2))), 1);
goal_idx(3) = find(abs(z_grid - goal(3)) == min(abs(z_grid - goal(3))), 1);
if any(start_idx < 1) || any(start_idx > [length(x_grid), length(y_grid), length(z_grid)])
    error('起点超出地图栅格范围！');
end
if any(goal_idx < 1) || any(goal_idx > [length(x_grid), length(y_grid), length(z_grid)])
    error('终点超出地图栅格范围！');
end
node_struct.g = inf(length(x_grid), length(y_grid), length(z_grid));
node_struct.h = zeros(length(x_grid), length(y_grid), length(z_grid));
node_struct.f = inf(length(x_grid), length(y_grid), length(z_grid));
node_struct.parent = cell(length(x_grid), length(y_grid), length(z_grid));
node_struct.g(start_idx(1), start_idx(2), start_idx(3)) = 0;
node_struct.h(start_idx(1), start_idx(2), start_idx(3)) = enhanced_heuristic(start, goal, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist);
node_struct.f(start_idx(1), start_idx(2), start_idx(3)) = node_struct.g(start_idx(1), start_idx(2), start_idx(3)) + node_struct.h(start_idx(1), start_idx(2), start_idx(3));
node_struct.parent{start_idx(1), start_idx(2), start_idx(3)} = [];
% 用逻辑数组 closed/in_open 代替 close_list 矩阵 + ismember(...,'rows') / 逐行 in_open 查找，
% 把每节点 ×26 邻居的 O(节点数) 查找降到 O(1)；open_list 改为数值矩阵（与静态版一致）。
% 搜索顺序（sub2ind/min 取最先加入者、directions 顺序）与原版完全一致，故结果路径不变。
closed  = false(length(x_grid), length(y_grid), length(z_grid));
in_open = false(length(x_grid), length(y_grid), length(z_grid));
in_open(start_idx(1), start_idx(2), start_idx(3)) = true;
open_list = start_idx;
directions = combvec([-1,0,1], [-1,0,1], [-1,0,1])';
directions(all(directions == 0, 2), :) = [];
z_priority_dirs = directions(abs(directions(:,3))>0, :);
xy_dirs = directions(abs(directions(:,3))==0, :);
directions = [z_priority_dirs; xy_dirs];

found = false;
iter_count = 0;
max_iter = 2 * length(x_grid)*length(y_grid)*length(z_grid);
while ~isempty(open_list) && ~found && iter_count < max_iter
    iter_count = iter_count + 1;
    fc = sub2ind(size(node_struct.f), open_list(:,1), open_list(:,2), open_list(:,3));
    [~, min_idx] = min(node_struct.f(fc));
    current_idx = open_list(min_idx, :);
    open_list(min_idx, :) = [];
    in_open(current_idx(1), current_idx(2), current_idx(3)) = false;
    closed(current_idx(1), current_idx(2), current_idx(3)) = true;
    if all(current_idx == goal_idx)
        found = true;
        break;
    end
    for d = 1:size(directions, 1)
        next_idx = current_idx + directions(d,:);
        if next_idx(1) < 1 || next_idx(1) > length(x_grid) || ...
           next_idx(2) < 1 || next_idx(2) > length(y_grid) || ...
           next_idx(3) < 1 || next_idx(3) > length(z_grid)
            continue;
        end
        if closed(next_idx(1), next_idx(2), next_idx(3))
            continue;
        end
        current_pos = [x_grid(current_idx(1)), y_grid(current_idx(2)), z_grid(current_idx(3))];
        next_pos = [x_grid(next_idx(1)), y_grid(next_idx(2)), z_grid(next_idx(3))];
        if ~check_node_feasible(next_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist) || ...
           ~check_segment_feasible(current_pos, next_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
            continue;
        end
        g_new = node_struct.g(current_idx(1), current_idx(2), current_idx(3)) + norm(next_pos - current_pos);
        h_new = enhanced_heuristic(next_pos, goal, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist);
        f_new = g_new + h_new;
        if f_new < node_struct.f(next_idx(1), next_idx(2), next_idx(3))
            node_struct.g(next_idx(1), next_idx(2), next_idx(3)) = g_new;
            node_struct.h(next_idx(1), next_idx(2), next_idx(3)) = h_new;
            node_struct.f(next_idx(1), next_idx(2), next_idx(3)) = f_new;
            node_struct.parent{next_idx(1), next_idx(2), next_idx(3)} = current_idx;
            if ~in_open(next_idx(1), next_idx(2), next_idx(3))
                open_list(end+1, :) = next_idx; %#ok<AGROW>
                in_open(next_idx(1), next_idx(2), next_idx(3)) = true;
            end
        end
    end
end
if found
    path = [];
    current = goal_idx;
    while ~isempty(node_struct.parent{current(1), current(2), current(3)})
        current_pos = [x_grid(current(1)), y_grid(current(2)), z_grid(current(3))];
        path = [current_pos; path];
        current = node_struct.parent{current(1), current(2), current(3)};
    end
    path = [start; path];
    path(end, :) = goal;
    fprintf('MHTA路径生成成功！路径节点数：%d\n', size(path, 1));
else
    error(['MHTA未找到可行路径！建议：1.减小栅格大小 2.降低安全距离 3.提升起点/终点高度']);
end
end

%% 2. 增强型启发函数
function h = enhanced_heuristic(pos, goal, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
euclid_dist = norm(pos - goal);
obstacle_penalty = 0;
x_idx = round((pos(1) - 0)/0.1 + 1);
y_idx = round((pos(2) - 0)/0.1 + 1);
if x_idx >=1 && x_idx <= size(Z1,2) && y_idx >=1 && y_idx <= size(Z1,1)
    mountain_heights = [Z1(y_idx,x_idx), Z2(y_idx,x_idx), Z3(y_idx,x_idx)];
    max_mountain_h = max(mountain_heights);
    if pos(3) - max_mountain_h < 9
        obstacle_penalty = obstacle_penalty + 200 / (pos(3) - max_mountain_h + 1e-6);
    end
end
ball_dist = norm(pos - [ball.cx, ball.cy, ball.cz]);
if ball_dist < ball.r + 9
    obstacle_penalty = obstacle_penalty + 300 / (ball_dist - ball.r + 1e-6);
end
for i = 1:size(cylinder.positions,1)
    cyl_pos = cylinder.positions(i,:);
    dist_to_cyl = norm([pos(1), pos(2)] - [cyl_pos(1), cyl_pos(2)]);
    if dist_to_cyl < cylinder.r + 9 && pos(3) < cyl_pos(3)
        obstacle_penalty = obstacle_penalty + 250 / (dist_to_cyl - cylinder.r + 1e-6);
    end
end
for i = 1:dynamic_obs.num
    obs_center = dynamic_obs.curr_pos(i,:);
    dist_obs = norm(pos - obs_center);
    if dist_obs < dynamic_obs.r + safe_dist*2
        obstacle_penalty = obstacle_penalty + 100 / (dist_obs - dynamic_obs.r + 1e-6);
    end
end
h = euclid_dist*0.9 + obstacle_penalty;
end

%% 3. 关键锚点提取
function waypoints = extract_key_waypoints(mhta_path, grid_size)
if size(mhta_path, 1) <= 2
    waypoints = mhta_path;
    return;
end
waypoints = mhta_path(1, :);
angle_threshold = 0.1;
for i = 2:size(mhta_path, 1)-1
    vec1 = mhta_path(i, :) - mhta_path(i-1, :);
    vec2 = mhta_path(i+1, :) - mhta_path(i, :);
    vec1_norm = vec1 / (norm(vec1) + 1e-6);
    vec2_norm = vec2 / (norm(vec2) + 1e-6);
    angle = acos(dot(vec1_norm, vec2_norm));
    if angle > angle_threshold
        waypoints = [waypoints; mhta_path(i, :)];
    end
end
waypoints = [waypoints; mhta_path(end, :)];
waypoints = filter_dense_waypoints(waypoints, grid_size);
end
function filtered_waypoints = filter_dense_waypoints(waypoints, min_dist)
filtered_waypoints = waypoints(1, :);
for i = 2:size(waypoints, 1)
    dist = norm(waypoints(i, :) - filtered_waypoints(end, :));
    if dist >= min_dist
        filtered_waypoints = [filtered_waypoints; waypoints(i, :)];
    end
end
end
function step = levy_flight(step_size)
beta = 1.5;
sigma = (gamma(1+beta)*sin(pi*beta/2)/(gamma((1+beta)/2)*beta*2^((beta-1)/2)))^(1/beta);
u = randn(1, 3) * sigma;
v = randn(1, 3);
step = step_size * u ./ (abs(v).^(1/beta));
end

%% 4. CAMO-AC优化主函数
function [optimal_path, fitness_history, metrics_history] = camo_ac_optimize(start, goal, waypoints, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, params)
pop_size = params.pop_size;
max_iter = params.max_iter;
alpha = params.alpha;
beta = params.beta;
rho = params.rho;
q0 = params.q0;
step_size = params.step_size;
weights = params.weights;
drone_speed = params.drone_speed;
pop = initialize_population(pop_size, start, goal, waypoints, safe_dist, Z1, Z2, Z3, ball, cylinder, dynamic_obs);
[~, metrics] = calculate_pop_fitness(pop, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, weights, drone_speed);
fitness = zeros(pop_size, 1);
for i = 1:pop_size, fitness(i) = calculate_absolute_fitness(metrics(i,:), weights, safe_dist); end
[best_fitness, best_idx] = min(fitness);
optimal_path = pop{best_idx};
global_best_metrics = metrics(best_idx, :);
fitness_history = zeros(1, max_iter);
metrics_history = zeros(max_iter, 5);
fitness_history(1) = calculate_absolute_fitness(global_best_metrics, weights, safe_dist);
metrics_history(1, :) = global_best_metrics;
n_waypoints = size(waypoints, 1);
pheromone = ones(n_waypoints - 1, 1);
pheromone_max = 10;
pheromone_min = 0.1;
for iter = 2:max_iter
    for ant = 1:pop_size
        current_path = [];
        current_pos = start;
        current_path = [current_path; current_pos];
        for wp = 1:n_waypoints - 1
            wp_start = waypoints(wp, :);
            wp_end = waypoints(wp + 1, :);
            x_min = min(wp_start(1), wp_end(1)) - 1e-6;
            x_max = max(wp_start(1), wp_end(1)) + 1e-6;
            y_min = min(wp_start(2), wp_end(2)) - 1e-6;
            y_max = max(wp_start(2), wp_end(2)) + 1e-6;
            z_min = min(wp_start(3), wp_end(3)) - 1e-6;
            z_max = max(wp_start(3), wp_end(3)) + 1e-6;
            x_len = round((x_max - x_min) / 0.1) + 1;
            y_len = round((y_max - y_min) / 0.1) + 1;
            z_len = round((z_max - z_min) / 0.1) + 1;
            max_len = max([x_len, y_len, z_len]);
            x_range = linspace(x_min, x_max, max_len);
            y_range = linspace(y_min, y_max, max_len);
            z_range = linspace(z_min, z_max, max_len);
            segment_range = [x_range; y_range; z_range];
            tau = pheromone(wp);
            h_wp = 1 / (norm(current_pos - wp_end) + 1e-6);
            h_goal = 1 / (norm(current_pos - goal) + 1e-6);
            eta = 0.6 * h_wp + 0.4 * h_goal;
            prob = (tau^alpha) * (eta^beta);
            if rand < q0
                dir_greedy = (wp_end - current_pos) / (norm(wp_end - current_pos) + 1e-6);
                dir_safe = safe_direction(current_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs);
                w_len = weights(1); w_safe = weights(2);
                direction = (w_len * dir_greedy + w_safe * dir_safe);
                direction = direction / (norm(direction) + 1e-6);
            else
                if iter < max_iter / 3
                    levy_step = levy_flight(step_size * 1.2);
                elseif iter < max_iter * 2/3
                    levy_step = levy_flight(step_size * 0.8);
                else
                    levy_step = levy_flight(step_size * 0.4) + randn(1,3)*0.03;
                end
                direction = levy_step / (norm(levy_step) + 1e-6);
            end
            segment_length = norm(wp_end - wp_start);
            step_adjust = min(iter/max_iter, 1);
            new_pos = current_pos + direction * segment_length * step_size * (1 - step_adjust*0.2);
            if ~check_node_feasible(new_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
                new_pos = adaptive_repair(new_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, wp_end);
            end
            new_pos(1) = max(segment_range(1, 1), min(segment_range(1, end), new_pos(1)));
            new_pos(2) = max(segment_range(2, 1), min(segment_range(2, end), new_pos(2)));
            new_pos(3) = max(segment_range(3, 1), min(segment_range(3, end), new_pos(3)));
            current_path = [current_path; new_pos];
            current_pos = new_pos;
        end
        current_path = [current_path; goal];
        current_path = prune_path(current_path, 0.15);
        current_path = repair_path(current_path, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist);
        pop{ant} = current_path;
    end
    [~, metrics] = calculate_pop_fitness(pop, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, weights, drone_speed, global_best_metrics);
    fitness = zeros(pop_size, 1);
    for i = 1:pop_size, fitness(i) = calculate_absolute_fitness(metrics(i,:), weights, safe_dist); end
    [current_best_fitness, current_best_idx] = min(fitness);
    if current_best_fitness < best_fitness
        best_fitness = current_best_fitness;
        optimal_path = pop{current_best_idx};
        global_best_metrics = metrics(current_best_idx, :);
    end
    pheromone = (1 - rho) * pheromone;
    delta_tau = 1 / (best_fitness + 1e-6);
    elite_bonus = 0.8;
    delta_tau = delta_tau * (1 + elite_bonus);
    pheromone = pheromone + delta_tau;
    pheromone(pheromone > pheromone_max) = pheromone_max;
    pheromone(pheromone < pheromone_min) = pheromone_min;
    fitness_history(iter) = calculate_absolute_fitness(global_best_metrics, weights, safe_dist);
    metrics_history(iter, :) = global_best_metrics;
    if mod(iter, 20) == 0
        fprintf('迭代次数：%d / %d | 最优适应度：%.4f | 路径长度：%.2fm\n', ...
            iter, max_iter, best_fitness, global_best_metrics(1));
    end
end
end

%% 5. 种群初始化
function pop = initialize_population(pop_size, start, goal, waypoints, safe_dist, Z1, Z2, Z3, ball, cylinder, dynamic_obs)
pop = cell(1, pop_size);
n_waypoints = size(waypoints, 1);
segment_node_num = 4;
for ant = 1:pop_size
    current_path = [];
    current_pos = start;
    current_path = [current_path; current_pos];
    for wp_idx = 1:n_waypoints
        if wp_idx == 1
            seg_start = start;
            seg_end = waypoints(wp_idx, :);
        else
            seg_start = waypoints(wp_idx-1, :);
            seg_end = waypoints(wp_idx, :);
        end
        seg_path = generate_segment_path(seg_start, seg_end, segment_node_num, safe_dist, ...
                                        Z1, Z2, Z3, ball, cylinder, dynamic_obs);
        seg_path = seg_path(2:end, :);
        current_path = [current_path; seg_path];
        current_pos = seg_end;
    end
    final_seg_path = generate_segment_path(waypoints(end, :), goal, segment_node_num, safe_dist, ...
                                          Z1, Z2, Z3, ball, cylinder, dynamic_obs);
    final_seg_path = final_seg_path(2:end, :);
    current_path = [current_path; final_seg_path];
    current_path = prune_path(current_path, 0.15);
    current_path = repair_path(current_path, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist);
    pop{ant} = current_path;
end
fprintf('种群初始化完成！共生成%d条可行初始路径\n', pop_size);
end
function seg_path = generate_segment_path(seg_start, seg_end, node_num, safe_dist, Z1, Z2, Z3, ball, cylinder, dynamic_obs)
seg_path = zeros(node_num, 3);
t = linspace(0, 1, node_num);
seg_path(:,1) = seg_start(1) + t*(seg_end(1) - seg_start(1));
seg_path(:,2) = seg_start(2) + t*(seg_end(2) - seg_start(2));
seg_path(:,3) = seg_start(3) + t*(seg_end(3) - seg_start(3));
disturb_amp = 0.5;   % 减小随机扰动，让初始路径更接近直线
for i = 2:node_num-1
    seg_path(i,:) = seg_path(i,:) + disturb_amp*(rand(1,3)-0.5);
end
for i = 1:node_num
    pos = seg_path(i,:);
    if ~check_node_feasible(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
        seg_path(i,:) = adaptive_repair(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, seg_end);
    end
end
end
function [fitness, metrics] = calculate_pop_fitness(pop, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, weights, drone_speed, global_best_metrics)
pop_size = length(pop);
metrics = zeros(pop_size, 5);
fitness = zeros(pop_size, 1);
drone_speed_inv = 1 / drone_speed;
for i = 1:pop_size
    path = pop{i};
    n_points = size(path, 1);
    path_diff = diff(path, 1, 1);
    path_length = sum(sqrt(sum(path_diff.^2, 2)));
    min_safe = inf;
    for j = 1:n_points
        min_safe = min(min_safe, calculate_safe_distance(path(j,:), Z1, Z2, Z3, ball, cylinder, dynamic_obs));
    end
    for j = 1:n_points-1
        for s = 1:5
            pos = path(j,:) + (path(j+1,:) - path(j,:)) * (s/6);
            min_safe = min(min_safe, calculate_safe_distance(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs));
        end
    end
    min_safe = max(min_safe, 0);
    height_change = sum(abs(diff(path(:,3))));
    turn_angle_sum = 0; maneuver_sum = 0;
    for j = 2:n_points-1
        vec1 = path(j,:) - path(j-1,:);
        vec2 = path(j+1,:) - path(j,:);
        n1 = vec1/(norm(vec1)+1e-6); n2 = vec2/(norm(vec2)+1e-6);
        ang = acos(max(min(dot(n1,n2),1),-1));
        turn_angle_sum = turn_angle_sum + ang;
        maneuver_sum = maneuver_sum + 2*sin(ang/2);
    end
    energy = path_length * 0.1 + height_change * 0.3 + 0.2*turn_angle_sum + 0.15*maneuver_sum;
    smoothness_cos = 0;
    count = 0;
    for j = 2:n_points-1
        vec1 = path(j,:) - path(j-1,:);
        vec2 = path(j+1,:) - path(j,:);
        if norm(vec1) > 1e-6 && norm(vec2) > 1e-6
            smoothness_cos = smoothness_cos + dot(vec1, vec2)/(norm(vec1)*norm(vec2));
            count = count + 1;
        end
    end
    smoothness_cos = smoothness_cos / (count + 1e-6);
    unsmoothness = 1 - smoothness_cos;
    flight_time = path_length * drone_speed_inv;
    metrics(i, :) = [path_length, min_safe, energy, unsmoothness, flight_time];
end
norm_metrics = zeros(size(metrics));
if nargin < 12 || isempty(global_best_metrics)
    for col = 1:size(metrics, 2)
        col_min = min(metrics(:, col));
        col_max = max(metrics(:, col));
        if col_max - col_min < 1e-6
            norm_metrics(:, col) = 0;
        else
            norm_metrics(:, col) = (metrics(:, col) - col_min) / (col_max - col_min);
        end
    end
else
    for col = 1:size(metrics, 2)
        if col == 2
            col_max = max([max(metrics(:, col)), global_best_metrics(col)]);
            col_min = min([min(metrics(:, col)), global_best_metrics(col)]);
            if col_max - col_min < 1e-6
                norm_metrics(:, col) = 0;
            else
                norm_metrics(:, col) = 1 - (metrics(:, col) - col_min) / (col_max - col_min);
            end
        else
            col_max = max([max(metrics(:, col)), global_best_metrics(col)]);
            col_min = min([min(metrics(:, col)), global_best_metrics(col)]);
            if col_max - col_min < 1e-6
                norm_metrics(:, col) = 0;
            else
                norm_metrics(:, col) = (metrics(:, col) - col_min) / (col_max - col_min);
            end
        end
    end
end
for i = 1:pop_size
    fitness(i) = weights(1)*norm_metrics(i,1)+weights(2)*norm_metrics(i,2)+weights(3)*norm_metrics(i,3)+weights(4)*norm_metrics(i,4)+weights(5)*norm_metrics(i,5);
end
fitness = fitness / (max(fitness) + 1e-6);
end

% 说明：calculate_pop_fitness 里的 fitness 是“每代内部相对归一化”的结果，
% 每代最优个体都会被压到接近 0，因此不能反映真实收敛过程。
% 这里改用与种群无关的固定参考尺度，得到绝对可比的适应度。
function abs_fit = calculate_absolute_fitness(metrics_row, weights, safe_dist)
% metrics_row = [path_length, min_safe, energy, unsmoothness, flight_time]
ref_length    = 300;   % 参考路径长度(m)，约为地图对角线量级
ref_safe      = safe_dist * 4;   % 参考安全距离(m)：达到该值即认为安全性最优
ref_energy    = 60;    % 参考能耗
ref_unsmooth  = 2;     % 不平滑度理论最大值(1-cos，范围[0,2])
ref_time      = ref_length / 10;   % 参考飞行时间(s)

n_length   = min(metrics_row(1) / ref_length, 1);              % 越小越优
safe_term  = max(0, 1 - metrics_row(2) / ref_safe);            % 安全距离越大越优→取反
n_energy   = min(metrics_row(3) / ref_energy, 1);              % 越小越优
n_unsmooth = min(metrics_row(4) / ref_unsmooth, 1);            % 越小越优
n_time     = min(metrics_row(5) / ref_time, 1);               % 越小越优

abs_fit = weights(1)*n_length + weights(2)*safe_term + ...
          weights(3)*n_energy + weights(4)*n_unsmooth + weights(5)*n_time;
end

%% 6. 多目标指标计算
function metrics = calculate_multiobj_metrics(path, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, drone_speed)
n_points = size(path, 1);
path_length = 0;
for i = 2:n_points
    path_length = path_length + norm(path(i,:) - path(i-1,:));
end
min_safe = inf;
for i = 1:n_points
    dist = calculate_safe_distance(path(i,:), Z1, Z2, Z3, ball, cylinder, dynamic_obs);
    min_safe = min(min_safe, dist);
end
for i = 1:n_points-1
    for s = 1:5
        pos = path(i,:) + (path(i+1,:) - path(i,:)) * (s/6);
        min_safe = min(min_safe, calculate_safe_distance(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs));
    end
end
min_safe = max(min_safe, 0);
height_change = 0;
for i = 2:n_points
    height_change = height_change + abs(path(i,3) - path(i-1,3));
end
turn_angle_sum = 0; maneuver_sum = 0;
for i = 2:n_points-1
    vec1 = path(i,:) - path(i-1,:);
    vec2 = path(i+1,:) - path(i,:);
    n1 = vec1/(norm(vec1)+1e-6); n2 = vec2/(norm(vec2)+1e-6);
    ang = acos(max(min(dot(n1,n2),1),-1));
    turn_angle_sum = turn_angle_sum + ang;
    maneuver_sum = maneuver_sum + 2*sin(ang/2);
end
energy = path_length * 0.1 + height_change * 0.3 + 0.2*turn_angle_sum + 0.15*maneuver_sum;
smoothness = 0;
count = 0;
for i = 2:n_points-1
    vec1 = path(i,:) - path(i-1,:);
    vec2 = path(i+1,:) - path(i,:);
    if norm(vec1) > 1e-6 && norm(vec2) > 1e-6
        cos_theta = dot(vec1, vec2) / (norm(vec1)*norm(vec2));
        smoothness = smoothness + cos_theta;
        count = count + 1;
    end
end
smoothness = smoothness / (count + 1e-6);
unsmoothness = 1 - smoothness;
flight_time = path_length / drone_speed;
metrics = [path_length, min_safe, energy, unsmoothness, flight_time];
end
function safe_dist = calculate_safe_distance(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs)
x = pos(1); y = pos(2); z = pos(3);
distances = [];
x_idx = round((x - 0)/0.1 + 1);
y_idx = round((y - 0)/0.1 + 1);
if x_idx >= 1 && x_idx <= size(Z1,2) && y_idx >= 1 && y_idx <= size(Z1,1)
    mountain_h1 = Z1(y_idx, x_idx);
    mountain_h2 = Z2(y_idx, x_idx);
    mountain_h3 = Z3(y_idx, x_idx);
    max_mountain_h = max([mountain_h1, mountain_h2, mountain_h3]);
    dist_mountain = z - max_mountain_h;
    distances = [distances, dist_mountain];
end
ball_center = [ball.cx, ball.cy, ball.cz];
dist_ball = norm(pos - ball_center) - ball.r;
distances = [distances, dist_ball];
for i = 1:size(cylinder.positions, 1)
    cyl_cx = cylinder.positions(i,1);
    cyl_cy = cylinder.positions(i,2);
    cyl_cz = cylinder.positions(i,3);
    cyl_r = cylinder.r;
    dist_horizontal = norm([x - cyl_cx, y - cyl_cy]);
    if z >= 0 && z <= cyl_cz
        dist_cylinder = dist_horizontal - cyl_r;
    else
        if z < 0
            vertical_dist = -z;
        else
            vertical_dist = z - cyl_cz;
        end
        dist_cylinder = max(dist_horizontal - cyl_r, vertical_dist);
    end
    distances = [distances, dist_cylinder];
end
for i = 1:dynamic_obs.num
    obs_cen = dynamic_obs.curr_pos(i,:);
    dist_obs = norm(pos - obs_cen) - dynamic_obs.r;
    distances = [distances, dist_obs];
end
safe_dist = max(min(distances), 0);
end
function feasible = check_node_feasible(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
x = pos(1); y = pos(2); z = pos(3);
x_idx = round((x - 0)/0.1 + 1);
y_idx = round((y - 0)/0.1 + 1);
if x_idx >=1 && x_idx <= size(Z1,2) && y_idx >=1 && y_idx <= size(Z1,1)
    if z < Z1(y_idx, x_idx) + safe_dist || ...
       z < Z2(y_idx, x_idx) + safe_dist || ...
       z < Z3(y_idx, x_idx) + safe_dist
        feasible = false;
        return;
    end
end
ball_dist = norm([x, y, z] - [ball.cx, ball.cy, ball.cz]);
if ball_dist < ball.r + safe_dist
    feasible = false;
    return;
end
for i = 1:size(cylinder.positions,1)
    cx = cylinder.positions(i,1); cy = cylinder.positions(i,2); cz = cylinder.positions(i,3);
    dist_to_axis = norm([x - cx, y - cy]);
    if dist_to_axis < cylinder.r + safe_dist && z >= 0 && z <= cz
        feasible = false;
        return;
    end
end
for i = 1:dynamic_obs.num
    obs_cen = dynamic_obs.curr_pos(i,:);
    dist_obs = norm(pos - obs_cen);
    if dist_obs < dynamic_obs.r + safe_dist
        feasible = false;
        return;
    end
end
feasible = true;
end
function feasible = check_segment_feasible(p1, p2, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
num_samples = 10;
t = linspace(0, 1, num_samples);
for i = 1:num_samples
    pos = p1 + t(i) * (p2 - p1);
    if ~check_node_feasible(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
        feasible = false;
        return;
    end
end
feasible = true;
end

function dir = safe_direction(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs)
% 远离最近障碍的方向（与静态 solve_this_v2 的 safe_direction 一致）：
% 采样 6 个主方向，选使 clearance 增大最多的方向。
best_dir = [0 0 1];
best_clear = calculate_safe_distance(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs);
cands = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
step = 2.0;
for k = 1:size(cands, 1)
    d = cands(k, :) / norm(cands(k, :));
    c = calculate_safe_distance(pos + d * step, Z1, Z2, Z3, ball, cylinder, dynamic_obs);
    if c > best_clear
        best_clear = c;
        best_dir = d;
    end
end
dir = best_dir;
end

%% 7. 路径处理函数
function repaired_pos = adaptive_repair(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, target)
directions = combvec([-1,0,1], [-1,0,1], [-1,0,1]);
directions(:, all(directions == 0, 1)) = [];
costs = inf(1, size(directions, 2));
for d = 1:size(directions, 2)
    candidate = pos + directions(:, d)' * safe_dist;
    if check_node_feasible(candidate, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
        dist_to_target = norm(candidate - target);
        move_dist = norm(directions(:, d)' * safe_dist);
        costs(d) = 0.7 * dist_to_target + 0.3 * move_dist;
    end
end
[min_cost, best_d] = min(costs);
if min_cost ~= inf
    repaired_pos = pos + directions(:, best_d)' * safe_dist;
else
    if norm(target - pos) < 1e-6
        random_dir = randn(1, 3);
        random_dir = random_dir / norm(random_dir);
        repaired_pos = pos + random_dir * safe_dist * 2;
    else
        target_dir = (target - pos) / norm(target - pos);
        repaired_pos = pos + target_dir * safe_dist * 2;
    end
    if ~check_node_feasible(repaired_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
        repaired_pos = adaptive_repair(repaired_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, target);
    end
end
end
function pruned_path = prune_path(original_path, angle_threshold)
if size(original_path, 1) <= 3
    pruned_path = original_path;
    return;
end
pruned_path = original_path(1, :);
for i = 2:size(original_path, 1)-1
    p_prev = original_path(i-1, :);
    p_curr = original_path(i, :);
    p_next = original_path(i+1, :);
    vec1 = p_curr - p_prev;
    vec2 = p_next - p_curr;
    vec1_norm = vec1 / (norm(vec1) + 1e-6);
    vec2_norm = vec2 / (norm(vec2) + 1e-6);
    cos_theta = dot(vec1_norm, vec2_norm);
    cos_theta = max(min(cos_theta, 1), -1);
    angle = acos(cos_theta);
    if angle > angle_threshold
        pruned_path = [pruned_path; p_curr];
    end
end
pruned_path = [pruned_path; original_path(end, :)];
end
function repaired_path = repair_path(path, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
repaired_path = path;
n_points = size(repaired_path, 1);
for i = 1:n_points
    pos = repaired_path(i,:);
    if ~check_node_feasible(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
        if i < n_points
            target = repaired_path(i+1,:);
        else
            target = repaired_path(i-1,:);
        end
        repaired_path(i,:) = adaptive_repair(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, target);
    end
end
for i = 1:n_points-1
    p1 = repaired_path(i,:);
    p2 = repaired_path(i+1,:);
    if ~check_segment_feasible(p1, p2, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
        insert_points = generate_segment_path(p1, p2, 5, safe_dist, Z1, Z2, Z3, ball, cylinder, dynamic_obs);
        insert_points = insert_points(2:end-1, :);
        repaired_path = [repaired_path(1:i,:); insert_points; repaired_path(i+1:end,:)];
        n_points = size(repaired_path, 1);
    end
end
end
function smooth_path = spline_smooth(path)
n_points = size(path, 1);
if n_points < 4
    smooth_path = path;
    return;
end
t = 1:n_points;
t_smooth = linspace(1, n_points, 5*n_points);
x_smooth = spline(t, path(:,1), t_smooth);
y_smooth = spline(t, path(:,2), t_smooth);
z_smooth = spline(t, path(:,3), t_smooth);
smooth_path = [x_smooth', y_smooth', z_smooth'];
end
function smooth_path = enforce_static_clearance(path, Z1, Z2, Z3, ball, cylinder, safe_dist)
% 把规划路径中离静态障碍太近的点向外推开，保证整体至少留出 safe_dist：
%   - 圆柱：横向(水平面)沿圆心→点方向推出到 safe_dist+余量
%   - 禁飞球：三维径向推出
%   - 山体：z 抬升到掠过山体高度
% 多轮迭代，防止"推开一个障碍又撞进另一个"。仅改变违反安全距离的点，
% 其余保持直线 → 规划路线只在障碍处"绕一下"，不会变成一条穿障直线。
smooth_path = path;
margin = 1.0;        % 额外安全余量(m)
need   = safe_dist + margin;
for iter = 1:5
    moved = false;
    for i = 1:size(smooth_path,1)
        x = smooth_path(i,1); y = smooth_path(i,2); z = smooth_path(i,3);
        % --- 圆柱：z 在柱高内时，横向必须离开 need ---
        for k = 1:size(cylinder.positions,1)
            cx = cylinder.positions(k,1);
            cy = cylinder.positions(k,2);
            cz = cylinder.positions(k,3);
            if z < 0 || z > cz; continue; end
            dh = norm([x-cx, y-cy]);
            if dh < need
                dirh = [x-cx, y-cy];
                if norm(dirh) < 1e-8; dirh = [1 0]; end
                dirh = dirh/norm(dirh);
                smooth_path(i,1:2) = [cx, cy] + dirh*need;
                moved = true;
            end
        end
        % --- 禁飞球：三维径向推出 ---
        bd = norm([x-ball.cx, y-ball.cy, z-ball.cz]);
        if bd < need
            dirb = [x-ball.cx, y-ball.cy, z-ball.cz];
            if norm(dirb) < 1e-8; dirb = [1 0 0]; end
            dirb = dirb/norm(dirb);
            smooth_path(i,:) = [ball.cx, ball.cy, ball.cz] + dirb*need;
            moved = true;
            z = smooth_path(i,3);
        end
        % --- 山体：z 抬升到掠过山体高度 ---
        xi = round((x)/0.1 + 1); yi = round((y)/0.1 + 1);
        if xi >= 1 && xi <= size(Z1,2) && yi >= 1 && yi <= size(Z1,1)
            mh = max([Z1(yi,xi), Z2(yi,xi), Z3(yi,xi)]);
            if z < mh + need
                smooth_path(i,3) = mh + need;
                moved = true;
            end
        end
    end
    if ~moved; break; end
end
% 最后把端点重新钉回（起点/终点不允许被推开）
smooth_path(end,:) = path(end,:);
end

function [is_feasible, min_safe_dist] = check_path_feasibility(path, Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, dynamic_obs_params, safe_dist)
if isempty(path) || size(path, 1) < 2
    is_feasible = false;
    min_safe_dist = 0;
    fprintf('路径为空或节点数过少，可行性验证失败！\n');
    return;
end
min_safe_dist = inf;
n_points = size(path, 1);
for i = 1:n_points
    node_safe_dist = calculate_safe_distance(path(i, :), Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, dynamic_obs_params);
    if node_safe_dist < min_safe_dist
        min_safe_dist = node_safe_dist;
    end
end
is_feasible = (min_safe_dist >= safe_dist);
end

%% 8. 环境生成函数
function [X, Y, Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, dynamic_obs_params] = generate_environment(map_range)
map_x = map_range(1):0.1:map_range(2);
map_y = map_range(1):0.1:map_range(2);
[X, Y] = meshgrid(map_x, map_y);
noise = zeros(size(X));   % 与统计实验一致：确定性无噪声地形
mountain_A_max=90; mountain_x=40; mountain_y=140; sigma=30;
z_guass1 = mountain_A_max*exp(-((X-mountain_x).^2+(Y-mountain_y).^2)/(2*sigma^2));
Z_mountain1 = noise + z_guass1;
Z_mountain1(Z_mountain1<0) = 0;
mountain_A_max2=90; mountain_x2=140; mountain_y2=40; sigma2=30;
z_guass2 = mountain_A_max2*exp(-((X-mountain_x2).^2+(Y-mountain_y2).^2)/(2*sigma2^2));
Z_mountain2 = noise + z_guass2;
Z_mountain2(Z_mountain2<0) = 0;
mountain_A3=120; mountain_x3=140; mountain_y3=140; sigma3=30;
z_guass3 = mountain_A3*exp(-((X-mountain_x3).^2+(Y-mountain_y3).^2)/(2*sigma3^2));
Z_mountain3 = z_guass3 + noise;
Z_mountain3(Z_mountain3<0) = 0;
ball_params.cx = 100; ball_params.cy = 100; ball_params.cz = 0;
ball_params.r = 50; ball_params.n_ball = 50;
cylinder_params.r = 5; cylinder_params.n_cylinder = 50;
cylinder_params.positions = [40,70,100; 70,40,100; 50,50,100; 90,160,100; 160,80,100; ...
                             80,180,100; 100,180,100; 180,60,100; 180,100,100];
% --- 动态障碍物参数（前 5 个与 Chaos_optimization_algorithm/AVSS_RRT_dynamic.m 一致） ---
% 每行 [起点x y z, 终点x y z, 方向(±1)]，障碍在起点<->终点之间往返运动
% 注：迎面逼近型障碍物在主脚本 4.5 节基于实际规划路径动态标定生成
dynamic_obs_params.num   = 5;
dynamic_obs_params.r     = 5;    % 动态障碍物半径
dynamic_obs_params.speed = 1;    % 每个时间步移动的距离
dynamic_obs_params.params = [
    150,  90, 68,  35,  85, 68, 1;
     95,  40, 60,  92, 140, 60, 1;
     50,  10, 10,  10,  80, 80, 1;
     56, 125, 75, 124, 56, 75, 1;
     95, 130, 75, 146, 90, 75, 1;
];
dynamic_obs_params.curr_pos = dynamic_obs_params.params(:, 1:3);   % 当前中心位置 (N x 3)
end

%% 9. 可视化函数（棕色静态规划路径 + 黑色动态障碍物 + 迎面避障飞行动画）
function visualize_results(X, Y, Z1, Z2, Z3, ball_params, cylinder_params, dynamic_obs_params, smooth_path, start, goal, safe_dist, uav_speed)
% ========== 字体大小参数 ==========
tick_font_size  = 20;   % 坐标轴刻度数字字号
label_font_size = 20;   % 坐标轴名称字号
title_font_size = 16;   % 标题字号
% =========================================================================
fig = figure("Position",[100,100,1000,800], 'Name','UAV 3D Path Planning Result', "Color", "white");
hold on; grid on; axis equal; view(3); rotate3d on;
xlabel("X/m", "FontSize", label_font_size);
ylabel("Y/m", "FontSize", label_font_size);
zlabel("Z/m", "FontSize", label_font_size);
title("MHTA+CAMO-AC UAV Path Planning & Dynamic Obstacle Avoidance", "FontSize", title_font_size);
zlim([0,200]); xlim([0,200]); ylim([0,200]);
set(gca, "Color", "white", "FontSize", tick_font_size);
set(gca, 'XTick', 0:20:200, 'YTick', 0:20:200, 'ZTick', 0:20:200);

% ---------- 静态障碍物 ----------
surf(X,Y,Z1,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.7);
surf(X,Y,Z2,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.7);
surf(X,Y,Z3,"FaceColor","interp","EdgeColor","none","FaceAlpha",0.7);
[X_ball,Y_ball,Z_ball] = sphere(ball_params.n_ball);
ball_x = ball_params.cx + X_ball * ball_params.r;
ball_y = ball_params.cy + Y_ball * ball_params.r;
ball_z = ball_params.cz + Z_ball * ball_params.r;
ball_z(ball_z < 0) = 0;
surf(ball_x,ball_y,ball_z,"FaceColor",[1,0,1],"EdgeColor",[1,0,1],"FaceAlpha",0.8);
for i = 1:size(cylinder_params.positions,1)
    cx = cylinder_params.positions(i,1);
    cy = cylinder_params.positions(i,2);
    cz = cylinder_params.positions(i,3);
    [X_cyl,Y_cyl,Z_cyl] = cylinder(cylinder_params.r, cylinder_params.n_cylinder);
    cylinder_x = cx + X_cyl;
    cylinder_y = cy + Y_cyl;
    cylinder_z = cz * Z_cyl;
    surf(cylinder_x,cylinder_y,cylinder_z,"FaceColor",[1,1,0],"EdgeColor","none","FaceAlpha",0.8);
    theta = linspace(0, 2*pi, cylinder_params.n_cylinder);
    top_x = cx + cylinder_params.r * cos(theta);
    top_y = cy + cylinder_params.r * sin(theta);
    top_z = repmat(cz, size(top_x));
    fill3(top_x, top_y, top_z, [1,1,0], "EdgeColor","none","FaceAlpha",0.8);
end

% ---------- 颜色（与 AVSS_RRT_dynamic.m 一致） ----------
col_plan  = [0.55 0.27 0.07];   % 棕色：静态障碍物下规划的全局路径
col_trail = [0.85 0.0 0.0];     % 正红：实际已飞行轨迹

% ---------- 规划路径（棕色实线，永不改变的全局基准路径） ----------
h_plan = plot3(smooth_path(:,1), smooth_path(:,2), smooth_path(:,3), ...
    '-', 'Color', col_plan, 'LineWidth', 2.2);

% ---------- 起点绿球 / 终点蓝球 ----------
sphere_r = 4;
[X_sph,Y_sph,Z_sph] = sphere(30);
h_start = surf(start(1)+X_sph*sphere_r, start(2)+Y_sph*sphere_r, start(3)+Z_sph*sphere_r, ...
    "FaceColor",[0,1,0],"EdgeColor","none","FaceAlpha",0.9);
h_goal = surf(goal(1)+X_sph*sphere_r, goal(2)+Y_sph*sphere_r, goal(3)+Z_sph*sphere_r, ...
    "FaceColor",[0,0,1],"EdgeColor","none","FaceAlpha",0.9);

% ---------- 动态障碍物（黑色球，往返运动） ----------
[Xd, Yd, Zd] = sphere(20);
Xd = Xd*dynamic_obs_params.r; Yd = Yd*dynamic_obs_params.r; Zd = Zd*dynamic_obs_params.r;
h_dyn = gobjects(dynamic_obs_params.num, 1);
for i = 1:dynamic_obs_params.num
    c = dynamic_obs_params.curr_pos(i,:);
    h_dyn(i) = surf(c(1)+Xd, c(2)+Yd, c(3)+Zd, ...
        'FaceColor', type_color(dynamic_obs_params.obs{i}.type), 'EdgeColor','none','FaceAlpha',1.0);
end

% ---------- 已飞行轨迹（红色实线）与无人机当前位置（红色实心点） ----------
h_trail = plot3(start(1), start(2), start(3), '-', 'Color', col_trail, 'LineWidth', 3);
h_uav = plot3(start(1), start(2), start(3), 'o', ...
    'MarkerFaceColor', col_trail, 'MarkerEdgeColor', 'none', 'MarkerSize', 8);

% 四类障碍物图例手柄（圆点，按类型着色）
h_type_legend = gobjects(4, 1);
for t = 1:4
    h_type_legend(t) = plot3(nan, nan, nan, 'o', 'MarkerFaceColor', type_color(t), ...
        'MarkerEdgeColor', 'none', 'MarkerSize', 10);
end

legend([h_trail, h_plan, h_type_legend(1), h_type_legend(2), h_type_legend(3), h_type_legend(4), h_start, h_goal], ...
    {'Actual Flown Path', 'Global Planned Path', 'Linear Obstacle', 'Circular Obstacle', 'Variable-speed Obstacle', 'Vertical Obstacle', 'Start', 'Goal'}, ...
    'Location', 'best');
set(gcf, 'color', 'w');

% ---------- 飞行仿真主循环（策略与 AVSS_RRT_dynamic.m 一致） ----------
% 静态规划的 smooth_path 作为全局最优路径，永不改变（棕色实线基准）：
%   - 默认沿全局路径按弧长前进；
%   - 若前方 look_ahead 距离内将撞动态障碍，选安全侧向/上方临时航点局部绕行；
%   - 障碍走开后，拉回全局路径上"最近且前方畅通"的点继续沿原路径飞。
glob = densify_path(smooth_path, 1.0);                 % 每 ~1m 重采样
glob_cum = [0; cumsum(sqrt(sum(diff(glob).^2, 2)))];   % 累积弧长
glob_len = glob_cum(end);

look_ahead = 28;     % 前视距离（与 sim_rgvs 一致）
min_detour_steps = 12;   % 绕行周期恢复到基线（防振荡，与 sim_rgvs 一致）
max_steps  = 4000;   % 仿真最大时间步
dyn_r_safe = dynamic_obs_params.r + safe_dist;         % 动态障碍碰撞半径（含安全距离）

uav_pos = start;
trail   = start;
uav_vel = glob(2,:) - glob(1,:);       % 无人机当前速度向量（RGVS 状态）
uav_vel = uav_vel / max(norm(uav_vel), 1e-9) * uav_speed;
s_glob  = 0;         % 当前在全局路径上的弧长进度
s_prog  = 0;         % 已到过的最大弧长进度（前向约束用，只增不减）
detour  = false;     % 是否处于局部绕行状态
detour_steps = 0;    % 连续绕行步数
detour_cnt = 0;      % 避障遭遇次数统计
reached = false;

for step = 1:max_steps
    if ~ishandle(fig); break; end

    % (a) 动态障碍物移动一步（往返运动）—— 到达终点后也会持续运动
    dynamic_obs_params = move_dynamic_obs(dynamic_obs_params);
    for i = 1:dynamic_obs_params.num
        c = dynamic_obs_params.curr_pos(i,:);
        set(h_dyn(i), 'XData', c(1)+Xd, 'YData', c(2)+Yd, 'ZData', c(3)+Zd);
    end

    if ~reached
        % (b) 检测全局路径前方是否会撞动态障碍（绕行期间用更短前视）
        la = 12;
        if ~detour
            la = look_ahead;
        end
        glob_blocked = lookahead_hit(glob, glob_cum, s_glob, ...
            dynamic_obs_params, dyn_r_safe, la, uav_speed);

        if ~glob_blocked && (~detour || detour_steps >= min_detour_steps)
            % 全局路径畅通：沿全局最优路径前进
            if detour
                % 每次避障结束后：从当前位置重新规划一条平滑、尽量短的新路径
                % （仅按静态障碍物规划，动态障碍交给 RGVS 实时躲避）
                emptydyn.num=0; emptydyn.params=[]; emptydyn.curr_pos=[]; emptydyn.speed=0; emptydyn.r=safe_dist;
                [glob, glob_cum, glob_len, s_glob] = replan_after_detour(uav_pos, glob, ...
                    Z1, Z2, Z3, ball_params, cylinder_params, emptydyn, safe_dist);
                s_prog = 0;   % 新参考路径从当前位置开始，进度清零
                % 速度对齐到新路径起点切向，保证连续过渡
                if size(glob,1) >= 2 && norm(glob(2,:)-glob(1,:)) > 1e-9
                    uav_vel = (glob(2,:)-glob(1,:))/norm(glob(2,:)-glob(1,:))*uav_speed;
                end
                detour = false;
                detour_steps = 0;
                fprintf('第 %d 步：避障完成，已从当前位置重新规划新路径(累计避障 %d 次)。\n', step, detour_cnt);
            end
            s_glob = min(glob_len, s_glob + uav_speed);
            target = interp_arc(glob, glob_cum, s_glob);
            uav_pos = move_towards(uav_pos, target, uav_speed);
            if norm(target - uav_pos) > 1e-9
                uav_vel = (target - uav_pos)/norm(target - uav_pos)*uav_speed; %#ok<UNRCH>
            end
        else
            % 全局路径被挡：RGVS 局部绕行（不改动全局路径）
            if ~detour
                detour_cnt = detour_cnt + 1;
                fprintf('第 %d 步：动态障碍物迎面逼近，第 %d 次触发避障，脱离原路径。\n', step, detour_cnt);
            end
            detour = true;
            detour_steps = detour_steps + 1;
            % 参考点 = 全局路径上当前进度前方的点（障碍过后 RGVS 自然回归）
            ref = interp_arc(glob, glob_cum, min(glob_len, s_glob + look_ahead));
            % RGVS 局部规划器参数：速度窗口、转角增量、预测步数、评分权重
            dw = struct('vmax', uav_speed, 'vmin', 0.3, ...
                        'dvmax', 0.6, 'dthmax', 1.5, ...
                        'dt', 1.0, 'pred', 8, ...
                        'w_head', 2.0, 'w_clear', 3.0, 'w_speed', 0.4, ...
                        'w_rot', 0.2, 'safem', safe_dist);
            [uav_pos, uav_vel] = rgvs_plan(uav_pos, uav_vel, ref, goal, ...
                Z1, Z2, Z3, ball_params, cylinder_params, dynamic_obs_params, ...
                dyn_r_safe, dw);
            % RGVS 找不到安全速度则本步悬停等待障碍物移开
        end

        % 前向约束：飞行全程不允许沿参考路径回退（进度只增不减）
        %   —— 把无人机位置投影到当前参考路径上的最近弧长 s_now，
        %      若 s_now 低于已到过的最大进度 s_prog，则拉回到该进度对应的路径点，
        %      从根上杜绝"回撤 → 轨迹自交"。
        dd = sqrt(sum((glob - uav_pos).^2, 2));
        [~, ix_near] = min(dd);
        s_now = glob_cum(ix_near);
        if s_now < s_prog - 0.05
            uav_pos = interp_arc(glob, glob_cum, s_prog);
        else
            s_prog = max(s_prog, s_now);
        end

        % 记录完整飞行轨迹（不做去重；回退已由前向约束禁止）
        trail(end+1,:) = uav_pos; %#ok<AGROW>

        % (c) 更新图形
        set(h_trail, 'XData', trail(:,1), 'YData', trail(:,2), 'ZData', trail(:,3));
        set(h_uav, 'XData', uav_pos(1), 'YData', uav_pos(2), 'ZData', uav_pos(3));
        drawnow;
        pause(0.02);

        % (d) 到达终点：把所有动态障碍物转移到运动轨迹中距已飞行路径最远的位置后结束
        if norm(uav_pos - goal) <= uav_speed
            uav_pos = goal;
            set(h_uav, 'XData', goal(1), 'YData', goal(2), 'ZData', goal(3));
            if norm(goal - trail(end,:)) > 1e-6
                trail(end+1,:) = goal; %#ok<AGROW>
            end
            set(h_trail, 'XData', trail(:,1), 'YData', trail(:,2), 'ZData', trail(:,3));
            reached = true;
            % 四类障碍无"最终重排"(原 relocate_obstacles_away 依赖 dyn.params 直线结构，已移除)
            for k = 1:dynamic_obs_params.num
                c = dynamic_obs_params.curr_pos(k,:);
                set(h_dyn(k), 'XData', c(1)+Xd, 'YData', c(2)+Yd, 'ZData', c(3)+Zd);
            end
            drawnow;
            pause(0.05);
            fprintf('无人机到达终点！仿真步数：%d，避障遭遇次数：%d，实际飞行距离：%.2f m\n', ...
                step, detour_cnt, sum(sqrt(sum(diff(trail).^2, 2))));
            break;   % 到达终点且动态障碍已远离轨迹，结束仿真
        end
    end
end
hold off;
end

% ================================================================
%   飞行仿真辅助函数（与 AVSS_RRT_dynamic.m 策略一致）
% ================================================================
function dyn = design_headon_obstacles(dyn, path, uav_speed)
% 基于"实际规划路径"标定 6 个迎面逼近型动态障碍物，替换原横穿型障碍物：
%   1) 在路径上取 6 个遭遇点 P_i（弧长比例 s_frac），无人机到达时刻
%      t_i = s_i/uav_speed + comp_i（comp 为此前每次避障约25步的耗时补偿）；
%   2) 障碍物段沿路径走廊布置：p1 在遭遇点前方 a 处、p2 在后方 b 处，
%      从 p1 迎面飞向 p2，恰好在 t_i 时刻迎面穿过 P_i 与无人机对遇；
%   3) 通过整数 k 标定：第 k+1 次穿越 P_i 的时刻 = (2k+1)a + 2kb = t_i，
%      保证即使前方走廊长度不足也能放下障碍物段。
%   段端点取在路径上（静态无碰撞且必在地图内），遇障时无人机被迫侧向
%   绕行脱离棕色路径，障碍过后自动回归——发生多次明显避障。
dyn_speed  = dyn.speed;   % 障碍物移动速度（标定穿越时刻用）
b          = 30;     % 越过相遇点后继续前进的距离（越长，障碍覆盖路径越久，绕行越明显）
comp_step  = 25;     % 每次避障遭遇平均耗时的补偿步数
s_frac     = [0.15, 0.28, 0.41, 0.54, 0.65, 0.75];   % 遭遇点弧长比例（6 个）

% 路径弧长累积（去除重复弧长保证 interp1 可用）
seg = sqrt(sum(diff(path).^2, 2));
cum = [0; cumsum(seg)];
[cum, ia] = unique(cum, 'stable');
path = path(ia, :);
L = cum(end);

new_rows = [];
for i = 1:length(s_frac)
    s_i = s_frac(i) * L;
    t_i = s_i / uav_speed + comp_step * (i-1);   % 无人机预计到达时刻
    % 选取 k 使第 k+1 次迎面穿越时刻恰为 t_i
    a = NaN;
    for k = 0:3
        a_try = (t_i * dyn_speed - 2*b*k) / (2*k + 1);
        if a_try >= 20 && (s_i + a_try) <= L   % 保证段足够长，落在路径内
            a = a_try;
            break;
        end
    end
    if isnan(a); continue; end
    % 段端点：相遇点前方 a、后方 b（取在路径上，迎面飞来的走廊）
    s_p1 = min(L, s_i + a);
    s_p2 = max(0, s_i - b);
    p1 = [interp1(cum, path(:,1), s_p1), interp1(cum, path(:,2), s_p1), interp1(cum, path(:,3), s_p1)];
    p2 = [interp1(cum, path(:,1), s_p2), interp1(cum, path(:,2), s_p2), interp1(cum, path(:,3), s_p2)];
    new_rows = [new_rows; p1, p2, 1]; %#ok<AGROW>
end
% 用迎面标定障碍物替换原第 6 行以后（横穿型），保留前 5 个原始障碍物
if ~isempty(new_rows)
    dyn.params = [dyn.params(1:min(5, size(dyn.params,1)), :); new_rows];
else
    dyn.params = dyn.params(1:min(5, size(dyn.params,1)), :);
end
dyn.num  = size(dyn.params, 1);
dyn.curr_pos = dyn.params(:, 1:3);
fprintf('遭遇点弧长比例设置：[%s]，路径总长 %.1f m\n', num2str(s_frac, '%.2f '), L);
end

function dyn = relocate_obstacles_away(dyn, trail)
% 到达终点时调用：把每个动态障碍物移动到其往返运动轨迹(p1<->p2)上
% 距已飞行轨迹最远的点，使最终画面中动态障碍远离红色飞行路径（避得越远越好）。
% 在障碍的起终点所在直线上密集采样，取"到轨迹最小距离最大"的采样点作为新位置。
for i = 1:dyn.num
    p1 = dyn.params(i, 1:3);        % 轨迹起点
    p2 = dyn.params(i, 4:6);        % 轨迹终点
    seg = linspace(0, 1, 101)';     % 沿轨迹采样 101 个点
    cand = p1 + (p2 - p1).*seg;     % 101 x 3
    bestd = -inf;
    bestp = p1;
    for t = 1:size(cand, 1)
        dt = sqrt(sum(((cand(t,:) - trail).^2), 2));   % 该候选点到轨迹各点的距离
        md = min(dt);
        if md > bestd
            bestd = md;
            bestp = cand(t,:);
        end
    end
    dyn.curr_pos(i, :) = bestp;
end
end

function dyn = move_dynamic_obs(dyn)
% 四类动态障碍物各推进一步(按 dyn.obs{i}.type 分派)
for i = 1:dyn.num
    o   = dyn.obs{i};
    cur = dyn.curr_pos(i,:);
    switch o.type
        case 1, [nxt, o] = step_linear(cur, o);
        case 2, [nxt, o] = step_circular(cur, o);
        case 3, [nxt, o] = step_variable(cur, o);
        case 4, [nxt, o] = step_vertical(cur, o);
        otherwise, continue;
    end
    dyn.curr_pos(i,:) = nxt;
    dyn.obs{i} = o;
end
end

%% 四类动态障碍物生成 & 单步运动（替代原直线往返障碍）
function dyn = gen_dyn_v2(path)
% 沿路径弧长生成 14 个四类动态障碍物(直线×3 圆弧×3 变速×4 垂直×4)
rng(1, 'twister');
dyn.num      = 14;
dyn.r        = 5;
dyn.curr_pos = zeros(14, 3);
dyn.obs      = cell(14, 1);
seg = sqrt(sum(diff(path).^2, 2));
cum = [0; cumsum(seg)];
[cum, ia] = unique(cum, 'stable');
path = path(ia, :);
L = cum(end);
if L < 30
    dyn.num = 0; dyn.curr_pos = zeros(0,3); dyn.obs = cell(0,1); return;
end
fracs = [0.20 0.24 0.29 0.33 0.37 0.41 0.46 0.50 0.54 0.59 0.63 0.67 0.71 0.76];
types = [1 2 3 4 3 1 4 2 3 4 1 2 3 4];
lin_speeds = [0.5 1.0 1.5];
lin_idx = 0;
for i = 1:14
    si = fracs(i) * L;
    p  = arc_interp(cum, path, si);
    s1 = max(cum(1), si - 3);
    s2 = min(cum(end), si + 3);
    tan = arc_interp(cum, path, s2) - arc_interp(cum, path, s1);
    if norm(tan) < 1e-6, tan = [1 0 0]; end
    tan = tan / norm(tan);
    up = [0 0 1];
    side = cross(tan, up);
    if norm(side) < 1e-6, side = cross(tan, [0 1 0]); end
    side(3) = 0;
    if norm(side) < 1e-6, side = [1 0 0]; end
    side = side / norm(side);
    switch types(i)
        case 1
            lin_idx = lin_idx + 1;
            dyn.obs{i} = make_linear(p, side, lin_speeds(lin_idx));
        case 2
            dyn.obs{i} = make_circular(p, side);
        case 3
            dyn.obs{i} = make_variable(p, side);
        case 4
            dyn.obs{i} = make_vertical(p, side);
    end
    dyn.curr_pos(i,:) = initial_position(dyn.obs{i});
end
end

function p = arc_interp(cum, path, s)
s = min(max(s, cum(1)), cum(end));
p = [interp1(cum, path(:,1), s), interp1(cum, path(:,2), s), interp1(cum, path(:,3), s)];
end

function o = make_linear(p, side, speed)
half_w = 25 + 10*rand;
S = p - side*half_w; S(3) = p(3);
E = p + side*half_w; E(3) = p(3);
o.type = 1; o.S = S; o.E = E;
o.dir = sign(rand - 0.5); if o.dir == 0, o.dir = 1; end
o.speed = speed;
end

function o = make_circular(p, side)
R = 12 + 6*rand;
vt = 0.8 + 0.7*rand;
o.type = 2;
o.center = p + side*(0.6*R); o.center(3) = p(3);
o.R = R; o.omega = vt / R; o.theta = 2*pi*rand;
end

function o = make_variable(p, side)
half_w = 25 + 10*rand;
S = p - side*half_w; S(3) = p(3);
E = p + side*half_w; E(3) = p(3);
o.type = 3; o.S = S; o.E = E;
o.dir = sign(rand - 0.5); if o.dir == 0, o.dir = 1; end
o.v0 = 0.5 + 0.5*rand; o.amp = 0.5; o.period = 20 + 20*rand; o.t = rand*o.period;
end

function o = make_vertical(p, side)
half_w = 15 + 10*rand;
S = p - side*half_w; S(3) = p(3);
E = p + side*half_w; E(3) = p(3);
o.type = 4; o.S = S; o.E = E;
o.dir = sign(rand - 0.5); if o.dir == 0, o.dir = 1; end
o.hspeed = 0.5; o.z0 = p(3); o.A = 15 + 10*rand; o.Tz = 30 + 20*rand; o.t = rand*o.Tz;
end

function pos = initial_position(o)
switch o.type
    case 1
        u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
        pos = o.S + u*(rand*norm(o.E - o.S));
    case 2
        pos = [o.center(1)+o.R*cos(o.theta), o.center(2)+o.R*sin(o.theta), o.center(3)];
    case 3
        u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
        pos = o.S + u*(rand*norm(o.E - o.S));
    case 4
        u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
        d = rand*norm(o.E - o.S);
        pos = [o.S(1)+u(1)*d, o.S(2)+u(2)*d, o.z0 + o.A*sin(2*pi*o.t/o.Tz)];
end
end

function [nxt, o] = step_linear(cur, o)
u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
nxt = cur + o.dir*o.speed*u;
if o.dir == 1
    if dot(nxt - o.E, o.E - o.S) >= 0, o.dir = -1; nxt = o.E; end
else
    if dot(nxt - o.S, o.S - o.E) >= 0, o.dir = 1; nxt = o.S; end
end
end

function [nxt, o] = step_circular(cur, o) %#ok<INUSD>
o.theta = o.theta + o.omega;
nxt = [o.center(1)+o.R*cos(o.theta), o.center(2)+o.R*sin(o.theta), o.center(3)];
end

function [nxt, o] = step_variable(cur, o)
o.t = o.t + 1;
spd = o.v0*(1 + o.amp*sin(2*pi*o.t/o.period));
u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
nxt = cur + o.dir*spd*u;
if o.dir == 1
    if dot(nxt - o.E, o.E - o.S) >= 0, o.dir = -1; nxt = o.E; end
else
    if dot(nxt - o.S, o.S - o.E) >= 0, o.dir = 1; nxt = o.S; end
end
end

function [nxt, o] = step_vertical(cur, o)
o.t = o.t + 1;
u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
h = cur(1:2) + o.dir*o.hspeed*u(1:2);
if o.dir == 1
    if dot(h - o.E(1:2), o.E(1:2) - o.S(1:2)) >= 0, o.dir = -1; h = o.E(1:2); end
else
    if dot(h - o.S(1:2), o.S(1:2) - o.E(1:2)) >= 0, o.dir = 1; h = o.S(1:2); end
end
z = o.z0 + o.A*sin(2*pi*o.t/o.Tz);
nxt = [h, z];
end

function c = type_color(t)
colors = [0.85 0.10 0.10;   % 1 直线: 红
          0.10 0.40 0.90;   % 2 圆弧: 蓝
          0.10 0.60 0.20;   % 3 变速: 绿
          0.90 0.50 0.10];  % 4 垂直: 橙
c = colors(t, :);
end

function q = densify_path(path, ds)
% 将折线路径按弧长间隔 ds 密集重采样，返回稠密点集 (M x 3)
d = [0; cumsum(sqrt(sum(diff(path).^2, 2)))];
if d(end) < 1e-6; q = path; return; end
[d, ia] = unique(d, 'stable'); path = path(ia, :);
tt = (0:ds:d(end))';
if tt(end) < d(end); tt(end+1) = d(end); end %#ok<AGROW>
q = [interp1(d, path(:,1), tt), ...
     interp1(d, path(:,2), tt), ...
     interp1(d, path(:,3), tt)];
end

function p = interp_arc(glob, glob_cum, s)
% 在全局路径上按弧长 s 取点（线性插值）
s = min(max(s, 0), glob_cum(end));
p = [interp1(glob_cum, glob(:,1), s), ...
     interp1(glob_cum, glob(:,2), s), ...
     interp1(glob_cum, glob(:,3), s)];
end

function pos = move_towards(pos, target, spd)
% 从 pos 朝 target 移动最多 spd 距离
v = target - pos; d = norm(v);
if d <= spd || d < 1e-9
    pos = target;
else
    pos = pos + (v/d)*spd;
end
end

function hit = lookahead_hit(glob, glob_cum, s0, dyn, r_safe, look_ahead, uav_speed)
% 前视检测：判断未来 look_ahead 内路径是否会与动态障碍碰撞（按 UAV 到达时刻预测障碍位置）
hit = false;
if isempty(dyn) || ~isfield(dyn, 'num') || dyn.num == 0
    return;
end
s_end = min(glob_cum(end), s0 + look_ahead);
ns = max(2, ceil((s_end - s0) / 1.5));
for s = linspace(s0, s_end, ns)
    q = interp_arc(glob, glob_cum, s);
    ts = round((s - s0) / uav_speed);
    if ts <= 0
        dpred = dyn;
    else
        dpred = dyn_after(dyn, ts);
    end
    for k = 1:size(dpred.curr_pos, 1)
        if norm(q - dpred.curr_pos(k, :)) <= r_safe
            hit = true;
            return;
        end
    end
end
end

function dn = dyn_after(dyn, n)
% 返回 n 步之后的障碍物状态（用于前向预测）
dn = dyn;
for i = 1:n
    dn = move_dynamic_obs(dn);
end
end

function [glob, glob_cum, glob_len, s_new] = replan_after_detour(uav_pos, glob, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
% 每次避障结束后，从无人机当前位置重新规划一条平滑、尽量短的新参考路径：
%   1) 找当前全局路径上与 uav_pos 最近的弧位置；
%   2) "截弯取直"：尝试把 uav_pos 直连到前方更远处（缩短路径），直到撞静态障碍；
%   3) 用 spline 平滑重连段，消除避障回归时留下的"尖端/折角"；
%   4) 平滑后若不可行，回退到"从最近点接入剩余路径"再平滑。
dd = sqrt(sum((glob - uav_pos).^2, 2));
[~, ix] = min(dd);

% 步骤2：截弯取直（路径更短）
join = ix;
for k = (ix+5):5:size(glob,1)
    if k <= ix; continue; end
    if check_segment_feasible(uav_pos, glob(k,:), Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
        join = k;
    else
        break;
    end
end
refpts = [uav_pos; glob(join:end,:)];
if size(refpts,1) < 4
    newpath = refpts;
else
    newpath = spline_smooth(refpts);
end
newpath(1,:) = uav_pos;
% 步骤4：平滑后可行性校验，失败则回退到最近点接入
if ~check_path_feasibility(newpath, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
    refpts2 = [uav_pos; glob(ix:end,:)];
    if size(refpts2,1) < 4
        newpath = refpts2;
    else
        newpath = spline_smooth(refpts2);
    end
    newpath(1,:) = uav_pos;
end
glob = densify_path(newpath, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob).^2, 2)))];
glob_len = glob_cum(end);
s_new = 0;
end

function [npos, nvel] = rgvs_plan(pos, vel, ref, goal, Z1, Z2, Z3, ball, cyc, dyn, dyn_r_safe, p)
% RGVS 局部规划（与 sim_rgvs 一致：近场过滤 + 前向预测障碍 + 更密候选 + w_rot 惩罚）
% 近场过滤：只保留距 pos 35m 以内的障碍，避免远处障碍预测轨迹过度约束局部搜索
if isfield(dyn, 'num') && dyn.num > 0
    d2 = sqrt(sum((dyn.curr_pos - pos) .^ 2, 2));
    keep = d2 <= 35;
    if any(keep)
        dyn.num      = sum(keep);
        dyn.curr_pos = dyn.curr_pos(keep, :);
        if isfield(dyn, 'obs') && ~isempty(dyn.obs)
            dyn.obs = dyn.obs(keep);
        end
    else
        dyn.num      = 0;
        dyn.curr_pos = zeros(0, 3);
        if isfield(dyn, 'obs') && ~isempty(dyn.obs)
            dyn.obs = cell(0, 1);
        end
    end
end

v_mag = norm(vel);
if v_mag < 1e-9
    v_mag = p.vmin;
end
refdir = (ref - pos) / max(norm(ref - pos), 1e-9);

up0 = [0 0 1];
right = cross(refdir, up0);
if norm(right) < 1e-6, right = [1 0 0]; end
right = right / norm(right);
up = cross(right, refdir); up = up / max(norm(up), 1e-9);

v_lo = max(p.vmin, v_mag - p.dvmax);
v_hi = min(p.vmax, v_mag + p.dvmax);
vcand = linspace(v_lo, v_hi, 5);
angcand = linspace(-p.dthmax, p.dthmax, 21);   % 更密的航向候选
pchcand = [-0.45 0 0.45];                       % 俯仰候选

% 预计算未来 1..pred 步障碍预测位置（避免候选循环内重复推进障碍）
obs_pred = cell(1, p.pred);
for st = 1:p.pred
    obs_pred{st} = dyn_after(dyn, st).curr_pos;
end

best_score = -inf;
npos = pos; nvel = vel;
found = false;
for v = vcand
    for da = angcand
        for dpc = pchcand
            newdir = rot3d(refdir, up, da);
            newdir = rot3d(newdir, right, dpc);
            newdir = newdir / max(norm(newdir), 1e-9);
            if dot(newdir, refdir) < -0.3; continue; end   % 避免严重倒退
            newvel = newdir * v;
            p0 = pos; clear_min = inf; ok = true;
            for st = 1:p.pred
                p0 = p0 + newvel * p.dt;
                obs_st = obs_pred{st};
                if ~rgvs_feasible(p0, Z1, Z2, Z3, ball, cyc, obs_st, dyn_r_safe, p)
                    ok = false;
                    break;
                end
                if dyn.num > 0
                    d = min(sqrt(sum((p0 - obs_st) .^ 2, 2))) - dyn.r;
                    clear_min = min(clear_min, max(d, 0));
                end
            end
            if ~ok, continue; end
            hd = dot(newdir, refdir);
            cl = min(clear_min, 30) / 30;
            score = p.w_head * hd + p.w_clear * cl + p.w_speed * (v / p.vmax) - p.w_rot * abs(da);
            if score > best_score
                best_score = score;
                nvel = newvel;
                npos = pos + newvel * p.dt;
                found = true;
            end
        end
    end
end
if ~found
    % 无安全速度：8 方向 × 3 档幅度应急逃逸
    next_obs = obs_pred{1};
    d1 = (right + up);  d1 = d1 / max(norm(d1), 1e-9);
    d2 = (right - up);  d2 = d2 / max(norm(d2), 1e-9);
    d3 = (-right + up); d3 = d3 / max(norm(d3), 1e-9);
    d4 = (-right - up); d4 = d4 / max(norm(d4), 1e-9);
    dirs = [right; -right; up; -up; d1; d2; d3; d4];
    amps = [6 12 18];
    npos = pos; nvel = [0 0 0];
    for a = amps
        for k = 1:size(dirs, 1)
            cd = pos + dirs(k, :) * a;
            if rgvs_feasible(cd, Z1, Z2, Z3, ball, cyc, next_obs, dyn_r_safe, p)
                nvel = dirs(k, :) * p.vmin;
                npos = pos + nvel * p.dt;
                return;
            end
        end
    end
    return;   % 全部方向不可行：原地悬停等待障碍移开
end
end

function ok = rgvs_feasible(p0, Z1, Z2, Z3, ball, cyc, obs, r_safe, p)
% 可行性：越界 + 静态障碍(空动态) + 预测动态障碍位置
if any(p0 < 0) || any(p0 > 200)
    ok = false; return;
end
empty_dyn.num = 0; empty_dyn.r = p.safem; empty_dyn.curr_pos = zeros(0, 3);
if ~check_node_feasible(p0, Z1, Z2, Z3, ball, cyc, empty_dyn, p.safem)
    ok = false; return;
end
for k = 1:size(obs, 1)
    if norm(p0 - obs(k, :)) <= r_safe + 1.0
        ok = false; return;
    end
end
ok = true;
end

function r = rot3d(v, axis, ang)
% 向量 v 绕单位轴 axis 旋转 ang 弧度（Rodrigues 公式）
c = cos(ang); s = sin(ang);
a = axis / max(norm(axis), 1e-9);
r = v*c + cross(a, v)*s + a*dot(a, v)*(1-c);
end
