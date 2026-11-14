function path = plan_avss(env)
%PLAN_AVSS 生成 AVSS-RRT 静态路径（统一解析环境）。
% 复刻 AVSS_RRT_dynamic.m 中的 avss_rrt_plan + 三次样条平滑，仅静态障碍。
rng(1);

start = env.start; goal = env.goal;

params.max_iter = 10000;
params.p_goal   = 0.15;
params.step_min = 3;
params.step_max = 18;
params.goal_tol = 8;
params.edge_res = 1.5;

[path, ~, success] = avss_rrt_plan(start, goal, env, params);
if ~success || isempty(path)
    error('AVSS-RRT 静态规划失败，请调整参数。');
end
path = smooth_path_cubic(path);
end

% ================= AVSS-RRT 核心 =================
function [path, tree, success] = avss_rrt_plan(start, goal, env, params)
tree.nodes  = start;
tree.parent = 0;
success = false;
path = [];

for it = 1:params.max_iter
    if rand < params.p_goal
        x_rand = goal;
    else
        x_rand = sample_free(env);
    end
    [x_near, idx_near] = nearest_node(tree.nodes, x_rand);
    step = adaptive_step(x_near, goal, env, params);
    dir = x_rand - x_near;
    dist = norm(dir);
    if dist < 1e-6
        continue;
    end
    x_new = x_near + (dir / dist) * min(step, dist);
    if ~in_bounds(x_new, env)
        continue;
    end
    if ~collision_free(x_near, x_new, env, params.edge_res)
        continue;
    end
    tree.nodes(end + 1, :)  = x_new;
    tree.parent(end + 1, 1) = idx_near;
    if norm(x_new - goal) <= params.goal_tol
        if collision_free(x_new, goal, env, params.edge_res)
            tree.nodes(end + 1, :)  = goal;
            tree.parent(end + 1, 1) = size(tree.nodes, 1) - 1;
            success = true;
            break;
        end
    end
end

if success
    idx = size(tree.nodes, 1);
    while idx ~= 0
        path = [tree.nodes(idx, :); path]; %#ok<AGROW>
        idx = tree.parent(idx);
    end
end
end

function step = adaptive_step(x, goal, env, params)
d_obs  = env_dist(x, env);
d_goal = norm(x - goal);
r_obs  = min(max((d_obs - env.safe) / params.step_max, 0), 1);
step   = params.step_min + (params.step_max - params.step_min) * r_obs;
step   = min(step, max(d_goal, params.step_min));
end

function x = sample_free(env)
for k = 1:50
    x = [env.map(1) + diff(env.map) * rand, ...
         env.map(1) + diff(env.map) * rand, ...
         env.map(1) + diff(env.map) * rand];
    if env_dist(x, env) > env.safe
        return;
    end
end
end

function [x_near, idx] = nearest_node(nodes, x)
d = sum((nodes - x) .^ 2, 2);
[~, idx] = min(d);
x_near = nodes(idx, :);
end

function tf = in_bounds(x, env)
tf = x(1) >= env.map(1) && x(1) <= env.map(2) && ...
     x(2) >= env.map(1) && x(2) <= env.map(2) && ...
     x(3) >= env.map(1) && x(3) <= env.map(2);
end

function tf = collision_free(a, b, env, res)
L = norm(b - a);
n = max(2, ceil(L / res));
t = linspace(0, 1, n)';
pts = a + t .* (b - a);
for i = 1:n
    if env_dist(pts(i, :), env) <= env.safe
        tf = false;
        return;
    end
end
tf = true;
end

function sp = smooth_path_cubic(path)
seg = sqrt(sum(diff(path) .^ 2, 2));
keep = [true; seg > 1e-6];
path = path(keep, :);
if size(path, 1) < 3
    sp = path;
    return;
end
d = [0; cumsum(sqrt(sum(diff(path) .^ 2, 2)))];
tt = linspace(0, d(end), max(200, size(path, 1) * 20))';
sp = [spline(d, path(:, 1), tt), ...
      spline(d, path(:, 2), tt), ...
      spline(d, path(:, 3), tt)];
end