function path = plan_aapf(env)
%PLAN_AAPF 生成 A*+APF 静态路径（统一解析环境）。
% 流程：三维A*粗路径 -> 提取关键锚点 -> APF人工势场局部优化 -> 剪枝 -> 样条平滑。
rng(1);

safe = env.safe;
gs = 6;    % A* 栅格步长（三个A*类算法统一，控制搜索规模）

astar_path = mhta_env(env.start, env.goal, env, gs, safe);
waypoints = extract_key_waypoints(astar_path, gs);

apf_path = apf_navigate(env.start, env.goal, waypoints, env, safe);
if size(apf_path, 1) < 4
    apf_path = astar_path;   % APF 过短时回退到 A* 路径
end

path = prune_path(apf_path, 0.25);
path = smooth_spline(path);
path = enforce_clearance(path, env);   % 平滑后修复被拉近障碍的点
path(1, :)   = env.start;
path(end, :) = env.goal;
end

% ================= 关键锚点提取 =================
function waypoints = extract_key_waypoints(astar_path, gs)
if size(astar_path, 1) <= 2
    waypoints = astar_path;
    return;
end
waypoints = astar_path(1, :);
th = 0.1;
for i = 2:size(astar_path, 1) - 1
    v1 = astar_path(i, :) - astar_path(i - 1, :);
    v2 = astar_path(i + 1, :) - astar_path(i, :);
    c = dot(v1, v2) / (norm(v1) * norm(v2) + 1e-9);
    c = max(min(c, 1), -1);
    if acos(c) > th
        waypoints = [waypoints; astar_path(i, :)]; %#ok<AGROW>
    end
end
waypoints = [waypoints; astar_path(end, :)];
% 滤除过近锚点
flt = waypoints(1, :);
for i = 2:size(waypoints, 1)
    if norm(waypoints(i, :) - flt(end, :)) >= gs
        flt = [flt; waypoints(i, :)]; %#ok<AGROW>
    end
end
waypoints = flt;
end

% ================= APF 人工势场局部优化 =================
function [path, total_steps] = apf_navigate(start, goal, waypoints, env, safe)
k_att = 3.0; k_rep = 6.0; rho0 = 25; step_size = 1.5;
goal_threshold = 3.0; max_iter = 8000; max_rep = 80;

all_targets = [start; waypoints; goal];
n_targets = size(all_targets, 1);
path = start;
cur = start;
total_steps = 0;

for wp = 2:n_targets
    target = all_targets(wp, :);
    iter = 0;
    stuck = 0;
    while norm(target - cur) > goal_threshold && iter < max_iter
        iter = iter + 1;
        total_steps = total_steps + 1;
        vec = target - cur;
        d = norm(vec);
        if d <= rho0
            Fatt = k_att * vec;
        else
            Fatt = k_att * rho0 * vec / d;
        end
        Frep = apf_repulsion(cur, env, k_rep, rho0, max_rep);
        F = Fatt + Frep;
        if norm(F) < 1e-4
            % 局部极小：沿引力切向扰动
            stuck = stuck + 1;
            tg = cross(Fatt, [0 0 1]);
            if norm(tg) < 1e-6
                tg = cross(Fatt, [0 1 0]);
            end
            if norm(tg) > 1e-6
                tg = tg / norm(tg);
                F = F + tg * k_rep * min(stuck, 8) * 0.5;
            end
        else
            stuck = 0;
        end
        if norm(F) > 1e-6
            dir = F / norm(F);
            nxt = cur + dir * step_size;
            if seg_free_static(cur, nxt, env, safe)
                cur = nxt;
                path = [path; cur]; %#ok<AGROW>
            end
        end
    end
    if wp < n_targets
        cur = target;
        path = [path; cur]; %#ok<AGROW>
    end
end
path = [path; goal];
path(1, :) = start;
end

function Frep = apf_repulsion(pos, env, k_rep, rho0, max_rep)
d0 = env_dist(pos, env);
if d0 >= rho0
    Frep = [0 0 0];
    return;
end
% 有符号距离的数值梯度（指向远离最近障碍方向）
e = 0.6;
g = zeros(1, 3);
for k = 1:3
    p1 = pos; p2 = pos;
    p1(k) = p1(k) + e;
    p2(k) = p2(k) - e;
    g(k) = (env_dist(p1, env) - env_dist(p2, env)) / (2 * e);
end
if norm(g) < 1e-9
    Frep = [0 0 0];
    return;
end
mag = min(k_rep * (rho0 - d0) / rho0, max_rep);
Frep = g / norm(g) * mag;
end