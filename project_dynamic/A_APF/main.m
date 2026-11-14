%% ====================== 主函数：A*+APF三维路径规划（适配map.txt动态障碍物地图） ======================
% 算法流程：
%   1. A*全局粗路径规划（栅格搜索 + 增强启发函数）
%   2. APF人工势场局部优化（引力-斥力梯度下降，在A*锚点间导航）
%   3. 路径后处理（剪枝 + B样条平滑 + 可行性验证）
%   4. 三维可视化 + 飞行仿真（动态障碍反应式避障）
% 参考文献：An Improved UAV 3D Path Planning Method Based on BiRRT-APF
%           Theta*-APF三维路径规划方法
clear; clc; close all;
warning off;

%% ====================== 1. 环境与算法参数初始化 ======================
map_range = [0, 200];
grid_size_astar = 3;
safe_dist = 2.5;
drone_speed = 10;

[X, Y, Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, dynamic_obs_params] = generate_environment(map_range);

start_pos = [10, 10, 50];
goal_pos = [190, 190, 80];

% APF人工势场参数
apf_params.k_att = 3.0;          % 引力增益系数
apf_params.k_rep = 6.0;          % 斥力增益系数
apf_params.rho0 = 25;            % 障碍物影响范围(m)
apf_params.step_size = 1.5;      % 单步移动步长(m)
apf_params.goal_threshold = 3.0; % 到达锚点判定阈值(m)
apf_params.max_iter = 8000;      % 每段最大迭代次数
apf_params.max_rep_force = 80;   % 单个障碍最大斥力上限(防止数值爆炸)

% 规划阶段只考虑静态障碍物
static_dynamic_obs.num = 0;
static_dynamic_obs.r = dynamic_obs_params.r;
static_dynamic_obs.speed = 0;
static_dynamic_obs.params = zeros(0, 7);
static_dynamic_obs.curr_pos = zeros(0, 3);

%% ====================== 2. 三维A*算法生成粗路径 ======================
fprintf('====================== 步骤1：三维A*粗路径生成 ======================\n');
astar_path = astar_3d(start_pos, goal_pos, map_range, grid_size_astar, ...
    Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, static_dynamic_obs, safe_dist);
waypoints = extract_key_waypoints(astar_path, grid_size_astar);
fprintf('A*粗路径生成完成！路径节点数：%d，提取关键锚点数：%d\n', size(astar_path,1), size(waypoints,1));

%% ====================== 3. APF人工势场局部优化 ======================
fprintf('\n====================== 步骤2：APF人工势场优化 ======================\n');
[apf_path, apf_total_steps] = apf_navigate(start_pos, goal_pos, waypoints, ...
    Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, static_dynamic_obs, safe_dist, apf_params);
fprintf('APF优化完成！原始路径节点数：%d，总步数：%d\n', size(apf_path,1), apf_total_steps);

% 安全保障：APF路径过短时回退到A*路径
if size(apf_path, 1) < 4
    warning('APF路径节点过少，回退到A*粗路径作为基线');
    apf_path = astar_path;
end

%% ====================== 4. 路径后处理与可行性验证 ======================
% APF路径过长时先降采样，防止剪枝耗时过长
if size(apf_path, 1) > 3000
    step_ds = ceil(size(apf_path, 1) / 3000);
    apf_path = apf_path(1:step_ds:end, :);
    apf_path = [apf_path; goal_pos];
end
pruned_path = prune_path(apf_path, 0.25);
smooth_path = spline_smooth(pruned_path);
% 强制首尾点与起点/终点完全一致
smooth_path(1, :) = start_pos;
smooth_path(end, :) = goal_pos;
[is_feasible, min_safe_dist] = check_path_feasibility(smooth_path, Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, static_dynamic_obs, safe_dist);

%% ====================== 4b. 布置横穿规划路径的动态障碍物 ======================
% 沿实际规划出的路径布置 6 个横穿型动态障碍物：每个障碍物的往返线段都
% 真实"穿过"棕色规划路线上的交会点(横向摆动 ±25m)，使动态障碍物轨迹与
% 静态规划路线形成多次清晰相交；无人机飞行到此必然触发避让→绕行→回归，
% 已飞行路径呈现明显的避障机动过程。
dynamic_obs_params = gen_dyn_v2(smooth_path);
fprintf('已沿规划路径布置 14 个四类动态障碍物（直线×3 圆弧×3 变速×4 垂直×4）\n');

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
assignin('base', 'astar_raw_path', astar_path);
assignin('base', 'apf_raw_path', apf_path);
save('drone_path_result.mat', 'smooth_path', 'astar_path', 'apf_path');
writematrix(smooth_path, 'drone_smooth_path.csv');
fprintf('📁 路径数据已导出：drone_path_result.mat、drone_smooth_path.csv\n');

%% ====================== 6. 三维路径可视化（动画，阻塞） ======================
visualize_results(X, Y, Z_mountain1, Z_mountain2, Z_mountain3, ball_params, cylinder_params, dynamic_obs_params, ...
    astar_path, smooth_path, start_pos, goal_pos);

%% ====================== 7. 多目标指标对比输出 ======================
fprintf('\n====================== 多目标指标对比结果 ======================\n');
astar_metrics = calculate_multiobj_metrics(astar_path, Z_mountain1, Z_mountain2, Z_mountain3, ...
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
    astar_val = astar_metrics(i);
    optimal_val = optimal_metrics(i);
    improve_rate = 0;
    switch i
        case 1
            if astar_val < 1e-6
                improve_rate = 0;
            else
                improve_rate = (astar_val - optimal_val) / astar_val * 100;
            end
        case 2
            if astar_val < 1e-6
                improve_rate = (optimal_val - astar_val) * 100;
            else
                improve_rate = (optimal_val - astar_val) / astar_val * 100;
            end
        case 3
            if astar_val < 1e-6
                improve_rate = 0;
            else
                improve_rate = (astar_val - optimal_val) / astar_val * 100;
            end
        case 4
            astar_smooth = 1 - astar_val;
            optimal_smooth = 1 - optimal_val;
            astar_val = astar_smooth;
            optimal_val = optimal_smooth;
            if astar_val < 1e-6
                improve_rate = (optimal_val - astar_val) * 100;
            else
                improve_rate = (optimal_val - astar_val) / astar_val * 100;
            end
        case 5
            if astar_val < 1e-6
                improve_rate = 0;
            else
                improve_rate = (astar_val - optimal_val) / astar_val * 100;
            end
    end
    fprintf('%s：A*粗路径=%.2f | APF优化后=%.2f | 提升：%.2f%% | 优化方向：%s\n', ...
        metrics_names{i,1}, astar_val, optimal_val, improve_rate, metrics_names{i,2});
end
fprintf('\n====================== 规划流程全部完成 ======================\n');

%% ====================== 局部函数 ======================

%% 1. 三维A*路径规划函数
function path = astar_3d(start, goal, map_range, grid_size, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
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
open_list = {start_idx};
close_list = [];
% 生成26邻域方向（替代combvec，无需通信工具箱）
[d1, d2, d3] = ndgrid([-1,0,1], [-1,0,1], [-1,0,1]);
directions = [d1(:), d2(:), d3(:)];
directions(all(directions == 0, 2), :) = [];
z_priority_dirs = directions(abs(directions(:,3))>0, :);
xy_dirs = directions(abs(directions(:,3))==0, :);
directions = [z_priority_dirs; xy_dirs];

found = false;
iter_count = 0;
max_iter = 2 * length(x_grid)*length(y_grid)*length(z_grid);
while ~isempty(open_list) && ~found && iter_count < max_iter
    iter_count = iter_count + 1;
    f_values = arrayfun(@(i) node_struct.f(open_list{i}(1), open_list{i}(2), open_list{i}(3)), 1:length(open_list));
    [~, min_idx] = min(f_values);
    current_idx = open_list{min_idx};
    open_list(min_idx) = [];
    close_list = [close_list; current_idx];
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
        if ismember(next_idx, close_list, 'rows')
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
            in_open = false;
            for k = 1:length(open_list)
                if all(open_list{k} == next_idx)
                    in_open = true;
                    break;
                end
            end
            if ~in_open
                open_list{end+1} = next_idx;
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
    fprintf('A*路径生成成功！路径节点数：%d\n', size(path, 1));
else
    error(['A*未找到可行路径！建议：1.减小栅格大小 2.降低安全距离 3.提升起点/终点高度']);
end
end

%% 2. 增强型启发函数（A*启发函数中加入障碍物惩罚项）
function h = enhanced_heuristic(pos, goal, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
euclid_dist = norm(pos - goal);
obstacle_penalty = 0;
x_idx = round((pos(1) - 0)/0.1 + 1);
y_idx = round((pos(2) - 0)/0.1 + 1);
if x_idx >=1 && x_idx <= size(Z1,2) && y_idx >=1 && y_idx <= size(Z1,1)
    mountain_heights = [Z1(y_idx,x_idx), Z2(y_idx,x_idx), Z3(y_idx,x_idx)];
    min_mountain_h = min(mountain_heights);
    if pos(3) - min_mountain_h < safe_dist*2
        obstacle_penalty = obstacle_penalty + 70 / (pos(3) - min_mountain_h + 1e-6);
    end
end
ball_dist = norm(pos - [ball.cx, ball.cy, ball.cz]);
if ball_dist < ball.r + safe_dist*3
    obstacle_penalty = obstacle_penalty + 150 / (ball_dist - ball.r + 1e-6);
end
for i = 1:size(cylinder.positions,1)
    cyl_pos = cylinder.positions(i,:);
    dist_to_cyl = norm([pos(1), pos(2)] - [cyl_pos(1), cyl_pos(2)]);
    if dist_to_cyl < cylinder.r + safe_dist*2 && pos(3) < cyl_pos(3)
        obstacle_penalty = obstacle_penalty + 120 / (dist_to_cyl - cylinder.r + 1e-6);
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
function waypoints = extract_key_waypoints(astar_path, grid_size)
if size(astar_path, 1) <= 2
    waypoints = astar_path;
    return;
end
waypoints = astar_path(1, :);
angle_threshold = 0.1;
for i = 2:size(astar_path, 1)-1
    vec1 = astar_path(i, :) - astar_path(i-1, :);
    vec2 = astar_path(i+1, :) - astar_path(i, :);
    vec1_norm = vec1 / (norm(vec1) + 1e-6);
    vec2_norm = vec2 / (norm(vec2) + 1e-6);
    angle = acos(dot(vec1_norm, vec2_norm));
    if angle > angle_threshold
        waypoints = [waypoints; astar_path(i, :)];
    end
end
waypoints = [waypoints; astar_path(end, :)];
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

%% 4. APF人工势场导航函数（核心：在A*锚点间用梯度下降生成优化路径）
function [path, total_steps] = apf_navigate(start, goal, waypoints, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, params)
k_att = params.k_att;
k_rep = params.k_rep;
rho0 = params.rho0;
step_size = params.step_size;
goal_threshold = params.goal_threshold;
max_iter = params.max_iter;
max_rep_force = params.max_rep_force;

% 导航目标序列：起点 → 各锚点 → 终点
all_targets = [start; waypoints; goal];
n_targets = size(all_targets, 1);

path = start;
current_pos = start;
total_steps = 0;

for wp_idx = 2:n_targets
    current_target = all_targets(wp_idx, :);
    iter = 0;
    prev_dist = inf;
    stuck_count = 0;
    no_progress_count = 0;

    while norm(current_target - current_pos) > goal_threshold && iter < max_iter
        iter = iter + 1;
        total_steps = total_steps + 1;

        % === 引力（Attractive Force） ===
        % 改进型引力：远距离等速吸引，近距离二次衰减，避免远处引力过大
        vec_to_target = current_target - current_pos;
        dist_to_target = norm(vec_to_target);
        if dist_to_target > 1e-6
            if dist_to_target <= rho0
                % 近距离：线性引力
                F_att = k_att * vec_to_target;
            else
                % 远距离：等速引力（截断）
                F_att = k_att * rho0 * vec_to_target / dist_to_target;
            end
        else
            F_att = [0, 0, 0];
        end

        % === 斥力（Repulsive Force） ===
        F_rep = compute_apf_repulsion(current_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, rho0, k_rep, max_rep_force);

        % === 合力 ===
        F_total = F_att + F_rep;
        force_mag = norm(F_total);

        % === 局部极小值逃逸机制 ===
        if force_mag < 1e-4
            % 合力几乎为零 → 陷入局部极小值
            stuck_count = stuck_count + 1;
            if norm(F_att) > 1e-6
                % 取引力方向的切向扰动（与引力垂直）
                tangent = cross(F_att, [0, 0, 1]);
                if norm(tangent) < 1e-6
                    tangent = cross(F_att, [0, 1, 0]);
                end
                if norm(tangent) > 1e-6
                    tangent = tangent / norm(tangent);
                    F_total = F_total + tangent * k_rep * min(stuck_count, 8) * 0.5;
                    force_mag = norm(F_total);
                end
            end
        else
            stuck_count = 0;
        end

        % 无进展检测：连续多步距离几乎不变 → 强制跳向目标方向
        if abs(dist_to_target - prev_dist) < 0.005
            no_progress_count = no_progress_count + 1;
        else
            no_progress_count = 0;
        end
        if no_progress_count > 30
            % 强制向目标方向跨步
            dir_force = (current_target - current_pos) / (norm(current_target - current_pos) + 1e-6);
            next_pos = current_pos + dir_force * step_size * 3;
            if check_node_feasible(next_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist) && ...
               check_segment_feasible(current_pos, next_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
                current_pos = next_pos;
                path = [path; current_pos];
            end
            prev_dist = norm(current_target - current_pos);
            no_progress_count = 0;
            continue;
        end

        % === 沿合力方向前进一步 ===
        if force_mag > 1e-6
            step_dir = F_total / force_mag;
            next_pos = current_pos + step_dir * step_size;

            % 可行性检查：新位置 + 连接线段均需可行
            if check_node_feasible(next_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist) && ...
               check_segment_feasible(current_pos, next_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
                current_pos = next_pos;
                path = [path; current_pos];
            else
                % 不可行 → 自适应修复
                repaired = adaptive_repair(next_pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, current_target);
                if check_node_feasible(repaired, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist) && ...
                   check_segment_feasible(current_pos, repaired, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist)
                    current_pos = repaired;
                    path = [path; current_pos];
                end
            end
        end

        prev_dist = dist_to_target;
    end

    % 锚点间衔接：确保到达当前锚点
    if wp_idx < n_targets
        current_pos = current_target;
        path = [path; current_pos];
    end
end

% 确保终点
path = [path; goal];
path(1, :) = start;
end

%% 5. APF斥力计算函数
function F_rep = compute_apf_repulsion(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, rho0, k_rep, max_force)
F_rep = [0, 0, 0];

% 1. 山体地形斥力（向上推）
x_idx = round((pos(1) - 0)/0.1 + 1);
y_idx = round((pos(2) - 0)/0.1 + 1);
if x_idx >= 1 && x_idx <= size(Z1,2) && y_idx >= 1 && y_idx <= size(Z1,1)
    mountain_heights = [Z1(y_idx,x_idx), Z2(y_idx,x_idx), Z3(y_idx,x_idx)];
    min_mountain_h = min(mountain_heights);
    dist_mountain = pos(3) - min_mountain_h;
    if dist_mountain < rho0 && dist_mountain > 0
        magnitude = k_rep * (rho0 - dist_mountain) / rho0;
        magnitude = min(magnitude, max_force);
        F_rep = F_rep + [0, 0, 1] * magnitude;
    end
end

% 2. 球体障碍斥力
ball_center = [ball.cx, ball.cy, ball.cz];
vec_from_ball = pos - ball_center;
dist_ball_center = norm(vec_from_ball);
dist_ball_surface = dist_ball_center - ball.r;
if dist_ball_surface < rho0 && dist_ball_surface > 0 && dist_ball_center > 1e-6
    direction = vec_from_ball / dist_ball_center;
    magnitude = k_rep * (rho0 - dist_ball_surface) / rho0;
    magnitude = min(magnitude, max_force);
    F_rep = F_rep + direction * magnitude;
end

% 3. 圆柱体障碍斥力
for i = 1:size(cylinder.positions, 1)
    cyl_pos = cylinder.positions(i, :);
    vec_from_cyl = pos - cyl_pos;
    dist_horizontal = norm([vec_from_cyl(1), vec_from_cyl(2)]);

    if pos(3) >= 0 && pos(3) <= cyl_pos(3)
        % 在圆柱高度范围内 → 水平方向斥力
        dist_cyl = dist_horizontal - cylinder.r;
        if dist_cyl < rho0 && dist_cyl > 0 && dist_horizontal > 1e-6
            direction = [vec_from_cyl(1), vec_from_cyl(2), 0] / dist_horizontal;
            magnitude = k_rep * (rho0 - dist_cyl) / rho0;
            magnitude = min(magnitude, max_force);
            F_rep = F_rep + direction * magnitude;
        end
    else
        % 在圆柱上方/下方 → 三维方向斥力
        dist_3d = norm(vec_from_cyl);
        dist_surface = dist_3d - cylinder.r;
        if dist_surface < rho0 && dist_surface > 0 && dist_3d > 1e-6
            direction = vec_from_cyl / dist_3d;
            magnitude = k_rep * (rho0 - dist_surface) / rho0;
            magnitude = min(magnitude, max_force);
            F_rep = F_rep + direction * magnitude;
        end
    end
end

% 4. 动态障碍斥力（规划阶段通常无动态障碍）
for i = 1:dynamic_obs.num
    obs_center = dynamic_obs.curr_pos(i, :);
    vec_from_obs = pos - obs_center;
    dist_center = norm(vec_from_obs);
    dist_surface = dist_center - dynamic_obs.r;
    if dist_surface < rho0 && dist_surface > 0 && dist_center > 1e-6
        direction = vec_from_obs / dist_center;
        magnitude = k_rep * (rho0 - dist_surface) / rho0;
        magnitude = min(magnitude, max_force);
        F_rep = F_rep + direction * magnitude;
    end
end
end

%% 6. 节点可行性检查
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

%% 7. 线段可行性检查
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

%% 8. 自适应修复函数
function repaired_pos = adaptive_repair(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs, safe_dist, target)
% 生成26邻域方向（替代combvec）
[d1, d2, d3] = ndgrid([-1,0,1], [-1,0,1], [-1,0,1]);
directions = [d1(:), d2(:), d3(:)]';
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

%% 9. 路径剪枝函数
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

%% 10. 路径修复函数
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

%% 11. 段路径生成函数
function seg_path = generate_segment_path(seg_start, seg_end, node_num, safe_dist, Z1, Z2, Z3, ball, cylinder, dynamic_obs)
seg_path = zeros(node_num, 3);
t = linspace(0, 1, node_num);
seg_path(:,1) = seg_start(1) + t*(seg_end(1) - seg_start(1));
seg_path(:,2) = seg_start(2) + t*(seg_end(2) - seg_start(2));
seg_path(:,3) = seg_start(3) + t*(seg_end(3) - seg_start(3));
disturb_amp = 1.5;
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

%% 12. B样条平滑函数
function smooth_path = spline_smooth(path)
n_points = size(path, 1);
if n_points < 4
    smooth_path = path;
    return;
end
t = 1:n_points;
t_smooth = linspace(1, n_points, 3*n_points);
x_smooth = spline(t, path(:,1), t_smooth);
y_smooth = spline(t, path(:,2), t_smooth);
z_smooth = spline(t, path(:,3), t_smooth);
smooth_path = [x_smooth', y_smooth', z_smooth'];
end

%% 13. 路径可行性验证
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

%% 14. 安全距离计算
function safe_dist = calculate_safe_distance(pos, Z1, Z2, Z3, ball, cylinder, dynamic_obs)
x = pos(1); y = pos(2); z = pos(3);
distances = [];
x_idx = round((x - 0)/0.1 + 1);
y_idx = round((y - 0)/0.1 + 1);
if x_idx >= 1 && x_idx <= size(Z1,2) && y_idx >= 1 && y_idx <= size(Z1,1)
    mountain_h1 = Z1(y_idx, x_idx);
    mountain_h2 = Z2(y_idx, x_idx);
    mountain_h3 = Z3(y_idx, x_idx);
    min_mountain_h = min([mountain_h1, mountain_h2, mountain_h3]);
    dist_mountain = z - min_mountain_h;
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

%% 15. 多目标指标计算
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
min_safe = max(min_safe, 0);
height_change = 0;
for i = 2:n_points
    height_change = height_change + abs(path(i,3) - path(i-1,3));
end
energy = path_length * 0.1 + height_change * 0.3;
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

%% 16. 环境生成函数
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
dynamic_obs_params.num = 5;
dynamic_obs_params.r = 5;
dynamic_obs_params.speed = 1;
% 基础动态障碍物（与 untitled.m 一致）。另在路径规划完成后，由
% add_traversing_obstacles 沿"实际规划出的路径"追加若干横穿路径型的
% 动态障碍物，确保障碍物往返轨迹与静态规划路线多处交合、避障过程明显。
dynamic_obs_params.params = [
    150,  90, 68,  35,  85, 68, 1;
     95,  40, 60,  92, 140, 60, 1;
     50,  10, 10,  10,  80, 80, 1;
     56, 125, 75, 124, 56, 75, 1;
     95, 130, 75, 146, 90, 75, 1;
];
dynamic_obs_params.curr_pos = dynamic_obs_params.params(:,1:3);
end

%% 16b. 沿规划路径布置"横穿型"动态障碍物
function dyn = add_traversing_obstacles(dyn, path, n_new)
% 沿实际规划路径布置 n_new 个横穿型动态障碍物。
% 每个障碍物的往返线段真实穿过棕色规划路线上的一个交会点 p：
%   以 p 处的路径切向 t 与法向 n 建立局部坐标系，
%   线段的两个端点分别位于路径两侧(横向 ±W=25m)，并沿切向错开 ±L=12m，
%   于是整条线段从路径一侧横穿到另一侧、且经过路径线上 p 点附近。
%   障碍物始终在这条线段上往返 → 无人机飞过 p 时必然与其迎面交错，
%   触发"脱离规划轨迹→侧向绕行→障碍物掠过→回归轨迹"的完整避障展示。
% 交会点沿弧长均匀分布(15% ~ 85%)，彼此间距足够，绕行过程清晰独立。
% ---- 弧长参数化（剔除重复点，保证 cum 严格递增） ----
seg = sqrt(sum(diff(path).^2, 2));
keep = [true; seg > 1e-9];
path = path(keep, :);
seg = sqrt(sum(diff(path).^2, 2));
cum = [0; cumsum(seg)];
L_total = cum(end);
fracs = linspace(0.15, 0.85, n_new);
W = 25;   % 横向摆动幅度(m)
L = 12;   % 沿切向错开(m)
fprintf('---- 横穿型动态障碍物布置（往返线段均穿过规划路线交会点） ----\n');
for k = 1:n_new
    s_k = L_total * fracs(k);
    p = traverse_point(path, cum, s_k);          % 交会点(落在路径上)
    t_k = traverse_tangent(path, cum, s_k);      % 路径单位切向
    n_u = traverse_normal(t_k);                  % 路径单位法向
    if mod(k, 2) == 0
        side = -1;                              % 左右交替布置
    else
        side = 1;
    end
    S = p + n_u * ( W * side) + t_k *  L;
    E = p + n_u * (-W * side) + t_k * -L;
    dyn.params = [dyn.params; S, E, 1];   %#ok<AGROW>
    dyn.curr_pos = [dyn.curr_pos; S];     %#ok<AGROW>
    dyn.num = dyn.num + 1;
    fprintf('  横穿障碍物 #%d：交会点[%.0f,%.0f,%.0f]，往返[%.0f,%.0f,%.0f]<->[%.0f,%.0f,%.0f] 横穿路径\n', ...
        k, p(1), p(2), p(3), S(1), S(2), S(3), E(1), E(2), E(3));
end
end

function p = traverse_point(path, cum, s)
% 取路径上弧长 s 处的坐标（线性插值）
s = min(max(s, cum(1)), cum(end));
p = [interp1(cum, path(:,1), s), interp1(cum, path(:,2), s), interp1(cum, path(:,3), s)];
end

function t = traverse_tangent(path, cum, s)
% 取路径上弧长 s 处的单位切向量
ds = 3;
p1 = traverse_point(path, cum, s - ds);
p2 = traverse_point(path, cum, s + ds);
t = p2 - p1;
t = t / (norm(t) + 1e-9);
end

function n = traverse_normal(t)
% 求路径切向 t 的一个单位法向（与竖直方向叉乘，退化为与x轴叉乘）
nz = cross(t, [0, 0, 1]);
if norm(nz) < 1e-3
    nz = cross(t, [1, 0, 0]);
end
n = nz / norm(nz);
end

%% 17. 可视化函数（满足要求：终点蓝球、棕色实线静态规划路径、红色实线飞行轨迹、飞行仿真动画）
function visualize_results(X, Y, Z1, Z2, Z3, ball_params, cylinder_params, dynamic_obs_params, astar_path, smooth_path, start, goal)
% ========== 字体大小参数（与 main_new_map.m 一致，用户可自由调节） ==========
tick_font_size  = 20;
label_font_size = 20;
title_font_size = 16;
% =========================================================================
fig = figure("Position",[100,100,1000,800], 'Name','UAV 3D Path Planning Result', "Color", "white");
hold on; grid on; axis equal; view(3); rotate3d on;
xlabel("X/m", "FontSize", label_font_size);
ylabel("Y/m", "FontSize", label_font_size);
zlabel("Z/m", "FontSize", label_font_size);
title("A*+APF UAV 3D Path Planning", "FontSize", title_font_size);
zlim([0,200]); xlim([0,200]); ylim([0,200]);
set(gca, "Color", "white", "FontSize", tick_font_size);
set(gca, 'XTick', 0:20:200, 'YTick', 0:20:200, 'ZTick', 0:20:200);
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

% 起点：绿色球体
sphere_r = 4;
[X_sph,Y_sph,Z_sph] = sphere(30);
start_x = start(1) + X_sph * sphere_r;
start_y = start(2) + Y_sph * sphere_r;
start_z = start(3) + Z_sph * sphere_r;
surf(start_x,start_y,start_z,"FaceColor",[0,1,0],"EdgeColor","none","FaceAlpha",0.9, 'DisplayName', 'Start');
% 终点：蓝色球体
goal_r = sphere_r;
goal_x = goal(1) + X_sph * goal_r;
goal_y = goal(2) + Y_sph * goal_r;
goal_z = goal(3) + Z_sph * goal_r;
surf(goal_x,goal_y,goal_z,"FaceColor",[0,0,1],"EdgeColor","none","FaceAlpha",0.9, 'DisplayName', 'Goal');

legend('Location', 'best');
set(gcf, 'color', 'w');

n_ball = ball_params.n_ball;
[X_dynamic, Y_dynamic, Z_dynamic] = sphere(n_ball);
X_dynamic = X_dynamic * dynamic_obs_params.r;
Y_dynamic = Y_dynamic * dynamic_obs_params.r;
Z_dynamic = Z_dynamic * dynamic_obs_params.r;
dynamic_handles = gobjects(dynamic_obs_params.num, 1);
for i = 1:dynamic_obs_params.num
    c = dynamic_obs_params.curr_pos(i, :);
    dynamic_handles(i) = surf(c(1)+X_dynamic, c(2)+Y_dynamic, c(3)+Z_dynamic,...
        "FaceColor", type_color(dynamic_obs_params.obs{i}.type), "EdgeColor","none","FaceAlpha",1.0);
end

% ================= 飞行仿真：沿规划轨迹飞行 + 动态障碍反应式避障 =================
% 棕色实线：静态障碍物下规划得到的全局最优路径（规划基准线，不随动态障碍改变）
% 棕色避开地图中其他物体颜色：山体(蓝绿黄渐变)、禁飞区(品红)、圆柱(黄)、
% 起点(绿)、终点(蓝)、动态障碍(黑)
plan_color = [0.55, 0.27, 0.07];   % 棕色：静态规划路径
plot3(smooth_path(:,1), smooth_path(:,2), smooth_path(:,3), ...
    '-', 'Color', plan_color, 'LineWidth', 2.2, 'DisplayName', 'Planned Path');

actual_color = [1, 0, 0];
actual_traj = smooth_path(1, :);
actual_line = plot3(actual_traj(1,1), actual_traj(1,2), actual_traj(1,3), ...
    '-', 'Color', actual_color, 'LineWidth', 3, 'DisplayName', 'Actual Flown Path');
drone_marker = plot3(actual_traj(1,1), actual_traj(1,2), actual_traj(1,3), ...
    'o', 'MarkerSize', 9, 'MarkerFaceColor', [1,1,1], 'MarkerEdgeColor', 'k', 'DisplayName', 'UAV');
legend('Location', 'best');

avoid_safe = dynamic_obs_params.r + 8;   % 触发避障的安全距离（13m）
avoid_gain = 3.5;                        % 侧向绕行的偏移强度（越大绕行越明显）
path_idx = 1;
n_path = size(smooth_path, 1);
curr_pos = smooth_path(1, :);
reached_end = false;    % 无人机是否已到达终点的标志

while ishandle(fig)
    % ---------- 1. 更新动态障碍物位置（四类运动） ----------
    dynamic_obs_params = move_dynamic(dynamic_obs_params);
    for i = 1:dynamic_obs_params.num
        set(dynamic_handles(i), 'XData', dynamic_obs_params.curr_pos(i,1)+X_dynamic, ...
                                'YData', dynamic_obs_params.curr_pos(i,2)+Y_dynamic, ...
                                'ZData', dynamic_obs_params.curr_pos(i,3)+Z_dynamic);
    end

    % ---------- 2. 无人机沿规划轨迹飞行 + 反应式避障 ----------
    if path_idx < n_path
        target_pt = smooth_path(path_idx + 1, :);
        travel_dir = target_pt - curr_pos;
        seg_len = norm(travel_dir);
        if seg_len < 1e-6
            path_idx = path_idx + 1;
        else
            travel_dir = travel_dir / seg_len;
            desired_next = target_pt;
            avoid_offset = [0, 0, 0];
            for i = 1:dynamic_obs_params.num
                obs = dynamic_obs_params.curr_pos(i, :);
                d_obs = norm(desired_next - obs);
                if d_obs < avoid_safe
                    to_drone = curr_pos - obs;
                    lateral = to_drone - dot(to_drone, travel_dir) * travel_dir;
                    if norm(lateral) < 1e-6
                        lateral = cross(travel_dir, [0, 0, 1]);
                        if norm(lateral) < 1e-6
                            lateral = cross(travel_dir, [0, 1, 0]);
                        end
                    end
                    lateral = lateral / norm(lateral);
                    push = avoid_gain * (avoid_safe - d_obs);   % 越近推得越远
                    avoid_offset = avoid_offset + lateral * push;
                end
            end
            desired_next = target_pt + avoid_offset;
            step_len = min(2.0, norm(desired_next - curr_pos));
            move_dir = (desired_next - curr_pos);
            if norm(move_dir) > 1e-6
                move_dir = move_dir / norm(move_dir);
            end
            curr_pos = curr_pos + move_dir * step_len;
            if norm(curr_pos - target_pt) < 2.0 * 0.8 && all(avoid_offset == 0)
                path_idx = path_idx + 1;
            end
        end
        actual_traj = [actual_traj; curr_pos];
        set(actual_line, 'XData', actual_traj(:,1), 'YData', actual_traj(:,2), 'ZData', actual_traj(:,3));
        set(drone_marker, 'XData', curr_pos(1), 'YData', curr_pos(2), 'ZData', curr_pos(3));
    end

    % ---------- 3. 无人机到达终点 ----------
    if path_idx >= n_path && ~reached_end
        reached_end = true;
    end

    drawnow;
    if reached_end
        break;
    end
    pause(0.01);
end
hold off;
end

function d = min_dist2_traj(p, traj)
% 计算空间点 p 到已飞行轨迹 traj 各点的最短欧氏距离（辅助：终点停靠避开轨迹）
diff = p - traj;                       % Nx3
d = min(sqrt(sum(diff .^ 2, 2)));
end

%% 18. 四类动态障碍物生成 & 单步运动（替代原直线往返障碍）
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

function dyn = move_dynamic(dyn)
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