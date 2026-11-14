function [success, trail, status] = sim_aapf(path, dyn, start, goal, env, sigma)
%SIM_AAPF A*+APF 算法的反应式动态避障仿真（无头版）。
% 可选第 6 参数 sigma：障碍物位置观测噪声标准差(m)，默认 0；决策用观测、碰撞用真实。
if nargin < 6, sigma = 0; end
uav_speed = 2.0;
max_steps = 4000;

k_att   = 2.0;
k_rep   = 1.5;
rho_rep = 8.0;

glob = densify_path(path, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);
r_safe = dyn.r + env.safe;

pos = start;
trail = start;
s_glob = 0;
success = false;
status = 0;    % 0=超时 1=到达 2=碰撞

for step = 1:max_steps
    dyn = move_obstacles(dyn);

    % 传感不确定性：观测位置 = 真实位置 + 高斯噪声；决策用观测，碰撞用真实
    if sigma > 0
        dyn_obs = dyn;
        dyn_obs.curr = dyn.curr + sigma * randn(size(dyn.curr));
    else
        dyn_obs = dyn;
    end

    aim = interp_arc(glob, glob_cum, min(glob_len, s_glob + 5));

    Fatt = aim - pos;
    if norm(Fatt) < 1e-6
        Fatt = goal - pos;
    end
    Fatt = k_att * Fatt / max(norm(Fatt), 1e-9);

    fwd = (aim - pos);
    if norm(fwd) < 1e-6
        fwd = goal - pos;
    end
    if norm(fwd) < 1e-6
        fwd = [1 0 0];
    end
    fwd = fwd / norm(fwd);

    Frep = [0 0 0];
    for i = 1:dyn_obs.num
        to = pos - dyn_obs.curr(i, :);
        d = norm(to) - dyn.r;
        if d < rho_rep
            lat = to - dot(to, fwd) * fwd;
            if norm(lat) < 1e-6
                lat = cross(fwd, [0 0 1]);
                if norm(lat) < 1e-6
                    lat = cross(fwd, [0 1 0]);
                end
            end
            lat = lat / norm(lat);
            Frep = Frep + k_rep * (rho_rep - d) * lat;
        end
    end

    F = Fatt + Frep;
    if norm(F) < 1e-6
        F = goal - pos;
    end
    if norm(F) < 1e-6
        F = [1 0 0];
    end
    dirmv = F / norm(F);
    npos = pos + dirmv * uav_speed;

    if ~step_safe(pos, npos, env, dyn_obs, r_safe)
        npos = safe_detour(pos, aim, goal, env, dyn_obs, r_safe, uav_speed);
        if isempty(npos)
            fprintf('[A_APF DIAG] 避障失败 step=%d 距目标=%.2f\n', step, norm(pos - goal));
            success = false;
            status = 2;
            return;
        end
    end

    pos = npos;
    trail = [trail; pos]; %#ok<AGROW>

    dd = sqrt(sum((glob - pos) .^ 2, 2));
    [~, ix_near] = min(dd);
    s_glob = glob_cum(ix_near);

    if dyn_min_dist(pos, dyn) < env.safe || env_dist(pos, env) < env.safe
        fprintf('[A_APF DIAG] 碰撞失败 step=%d 障碍数=%d 动态间隙=%.2f 静态间隙=%.2f 位置=(%.1f,%.1f,%.1f)\n', ...
            step, dyn.num, dyn_min_dist(pos, dyn), env_dist(pos, env), pos(1), pos(2), pos(3));
        success = false;
        status = 2;
        return;
    end

    if norm(pos - goal) <= uav_speed
        trail = [trail; goal]; %#ok<AGROW>
        success = true;
        status = 1;
        return;
    end
end
fprintf('[A_APF DIAG] 超时 step=%d 距目标=%.2f s_glob=%.1f/%.1f\n', ...
    max_steps, norm(pos - goal), s_glob, glob_len);
success = false;
end

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

function ok = step_safe(a, b, env, dyn, r_safe)
L = norm(b - a);
n = max(2, ceil(L / 1.0));
t = linspace(0, 1, n)';
pts = a + t .* (b - a);
for i = 1:n
    if env_dist(pts(i, :), env) < env.safe
        ok = false; return;
    end
    for k = 1:dyn.num
        if norm(pts(i, :) - dyn.curr(k, :)) <= r_safe
            ok = false; return;
        end
    end
end
ok = true;
end

function npos = safe_detour(pos, aim, goal, env, dyn, r_safe, spd)
fwd = aim - pos;
if norm(fwd) < 1e-6
    fwd = goal - pos;
end
if norm(fwd) < 1e-6
    fwd = [1 0 0];
end
fwd = fwd / norm(fwd);
right = cross(fwd, [0 0 1]);
if norm(right) < 1e-6
    right = [1 0 0];
end
right = right / norm(right);

amp = [5 8];
dirs = [right; -right];
dirs = dirs ./ vecnorm(dirs, 2, 2);
npos = [];
for a = amp
    for j = 1:size(dirs, 1)
        cand = pos + dirs(j, :) * a;
        if any(cand < 0) || any(cand > 200)
            continue;
        end
        if step_safe(pos, cand, env, dyn, r_safe)
            npos = pos + dirs(j, :) * 2.0;
            return;
        end
    end
end
end