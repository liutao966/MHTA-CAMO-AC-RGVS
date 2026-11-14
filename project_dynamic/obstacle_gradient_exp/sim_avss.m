function [success, trail, status] = sim_avss(path, dyn, start, goal, env, sigma)
%SIM_AVSS AVSS-RRT 算法的前瞻绕行动态避障仿真（无头版，保留原策略）。
% 策略：沿全局路径按弧长前进；前方 look_ahead 内将撞动态障碍时，选择侧向/上方临时航点绕行。
% 可选第 6 参数 sigma：障碍物位置观测噪声标准差(m)，默认 0；决策用观测、碰撞用真实。
if nargin < 6, sigma = 0; end
uav_speed  = 2.0;
look_ahead = 12;
max_steps  = 3000;

glob = densify_path(path, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);

pos = start;
trail = start;
s_glob = 0;
detour = false;
detour_cnt = 0;
success = false;
status = 0;    % 0=超时 1=到达 2=碰撞

for step = 1:max_steps
    dyn = move_obstacles(dyn);
    r_safe = dyn.r + env.safe;

    % 传感不确定性：观测位置 = 真实位置 + 高斯噪声；决策用观测，碰撞用真实
    if sigma > 0
        dyn_obs = dyn;
        dyn_obs.curr = dyn.curr + sigma * randn(size(dyn.curr));
    else
        dyn_obs = dyn;
    end

    blocked = lookahead_hit(glob, glob_cum, s_glob, dyn_obs.curr, r_safe, look_ahead);

    if ~blocked
        if detour
            s_glob = regain_arc(glob, glob_cum, pos, s_glob, dyn_obs.curr, r_safe, look_ahead);
            detour = false;
        end
        s_glob = min(glob_len, s_glob + uav_speed);
        target = interp_arc(glob, glob_cum, s_glob);
        pos = move_towards(pos, target, uav_speed);
    else
        if ~detour
            detour_cnt = detour_cnt + 1; %#ok<NASGU>
            detour = true;
        end
        aim = interp_arc(glob, glob_cum, min(glob_len, s_glob + look_ahead));
        wp = pick_detour(pos, aim, goal, env, dyn_obs.curr, r_safe, uav_speed);
        if ~isempty(wp)
            pos = move_towards(pos, wp, uav_speed);
        end
    end
    trail = [trail; pos]; %#ok<AGROW>

    if dyn_min_dist(pos, dyn) < env.safe || env_dist(pos, env) < env.safe
        success = false;
        status = 2;
        return;
    end

    if norm(pos - goal) <= uav_speed || (s_glob >= glob_len - 1e-6 && norm(pos - goal) <= 8)
        trail = [trail; goal]; %#ok<AGROW>
        success = true;
        status = 1;
        return;
    end
end
success = false;   % 超时
end

% ================= 工具函数 =================
function q = densify_path(path, ds)
d = [0; cumsum(sqrt(sum(diff(path) .^ 2, 2)))];
if d(end) < 1e-6
    q = path; return;
end
[d, ia] = unique(d, 'stable'); path = path(ia, :);
tt = (0:ds:d(end))';
if tt(end) < d(end)
    tt(end + 1) = d(end); %#ok<AGROW>
end
q = [interp1(d, path(:, 1), tt), interp1(d, path(:, 2), tt), interp1(d, path(:, 3), tt)];
end

function p = interp_arc(glob, glob_cum, s)
s = min(max(s, 0), glob_cum(end));
p = [interp1(glob_cum, glob(:, 1), s), ...
     interp1(glob_cum, glob(:, 2), s), ...
     interp1(glob_cum, glob(:, 3), s)];
end

function pos = move_towards(pos, target, spd)
v = target - pos; d = norm(v);
if d <= spd || d < 1e-9
    pos = target;
else
    pos = pos + (v / d) * spd;
end
end

function hit = lookahead_hit(glob, glob_cum, s0, dyn_pos, r_safe, look_ahead)
hit = false;
if isempty(dyn_pos)
    return;
end
s_end = min(glob_cum(end), s0 + look_ahead);
ns = max(2, ceil((s_end - s0) / 1.5));
for s = linspace(s0, s_end, ns)
    q = interp_arc(glob, glob_cum, s);
    for k = 1:size(dyn_pos, 1)
        if norm(q - dyn_pos(k, :)) <= r_safe
            hit = true;
            return;
        end
    end
end
end

function s_new = regain_arc(glob, glob_cum, pos, s_cur, dyn_pos, r_safe, look_ahead)
best_s = s_cur; best_d = inf;
s_from = s_cur;
s_to = min(glob_cum(end), s_cur + look_ahead + 20);
for s = linspace(s_from, s_to, 40)
    p = interp_arc(glob, glob_cum, s);
    if norm(p - pos) < best_d && ~lookahead_hit(glob, glob_cum, s, dyn_pos, r_safe, look_ahead)
        best_d = norm(p - pos);
        best_s = s;
    end
end
s_new = best_s;
end

function wp = pick_detour(pos, aim, goal, env, dyn_pos, r_safe, spd)
wp = [];
fwd = aim - pos;
if norm(fwd) < 1e-6
    fwd = goal - pos;
end
if norm(fwd) < 1e-6
    return;
end
fwd = fwd / norm(fwd);
up0 = [0 0 1];
right = cross(fwd, up0);
if norm(right) < 1e-6
    right = [1 0 0];
end
right = right / norm(right);
up = cross(right, fwd); up = up / norm(up);
amp = [8 14 20];
dirs = [right; -right; up; -up; right + up; right - up; -right + up; -right - up];
dirs = dirs ./ vecnorm(dirs, 2, 2);
best = -inf;
for a = amp
    for j = 1:size(dirs, 1)
        cand = pos + fwd * spd + dirs(j, :) * a;
        if any(cand < env.map(1)) || any(cand > env.map(2))
            continue;
        end
        if ~seg_free_dyn(pos, cand, env, dyn_pos, r_safe)
            continue;
        end
        score = -norm(cand - goal) - 0.3 * a;
        if score > best
            best = score; wp = cand;
        end
    end
    if ~isempty(wp)
        break;
    end
end
end

function tf = seg_free_dyn(a, b, env, dyn_pos, r_safe)
L = norm(b - a);
n = max(2, ceil(L / 1.5));
t = linspace(0, 1, n)';
pts = a + t .* (b - a);
for i = 1:n
    if env_dist(pts(i, :), env) < env.safe
        tf = false; return;
    end
    for k = 1:size(dyn_pos, 1)
        if norm(pts(i, :) - dyn_pos(k, :)) <= r_safe
            tf = false; return;
        end
    end
end
tf = true;
end