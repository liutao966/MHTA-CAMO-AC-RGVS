function [success, trail, status] = sim_dwa(path, dyn, start, goal, env, sigma)
%SIM_DWA 动态窗口法（DWA）动态避障仿真（无头版，纯 DWA 基线）。
% 策略：每步朝参考路径前视点取引导方向，在由加速度约束限定的"动态窗口"内采样
%       候选速度，逐候选前向预测 pred 步并打分（航向 + 净空 + 速度），取最优。
%       全程纯 DWA，无 detour / 全局重规划 / 样条回连，区别于 sim_rgvs 中
%       仅在绕行阶段调用 DWA 的 RGVS 分层策略。
% 可选第 6 参数 sigma：障碍物位置观测噪声标准差(m)，默认 0；决策用观测、碰撞用真实。
% 接口： [success, trail] = sim_dwa(path, dyn, start, goal, env, sigma)
if nargin < 6, sigma = 0; end
uav_speed  = 2.0;
look_ahead = 28;    % 前视距离(m)，与 sim_rgvs 一致（公平）
max_steps  = 4000;

r_safe = dyn.r + env.safe;

% 动态窗口 + 打分参数（与 RGVS 内嵌 DWA 同源，保证可比）
p.vmax   = uav_speed;
p.vmin   = 0.3;
p.dvmax  = 0.6;     % 单步速度变化上限（加速度约束）
p.dthmax = 1.5;     % 航向候选范围(rad)
p.dt     = 1.0;
p.pred   = 8;       % 前向预测步数
p.w_head = 1.0;
p.w_clear = 6.0;
p.w_speed = 0.4;
p.safem  = env.safe;

glob = densify_path(path, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);

pos   = start;
vel   = (glob(2, :) - glob(1, :)) / max(norm(glob(2, :) - glob(1, :)), 1e-9) * uav_speed;
trail = start;
s_glob = 0;
s_prog = 0;
detour = false;
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

    glob_blocked = lookahead_hit(glob, glob_cum, s_glob, dyn_obs, r_safe + 1.5, look_ahead, uav_speed);

    if ~glob_blocked
        detour = false;
        s_glob = min(glob_len, s_glob + uav_speed);
        target = interp_arc(glob, glob_cum, s_glob);
        pos = move_towards(pos, target, uav_speed);
        if norm(target - pos) > 1e-9
            vel = (target - pos) / norm(target - pos) * uav_speed;
        end
    else
        detour = true;
        ref = interp_arc(glob, glob_cum, min(glob_len, s_glob + look_ahead));
        [pos, vel] = dwa_step(pos, vel, ref, goal, env, dyn_obs, r_safe, p);
    end

    dd = sqrt(sum((glob - pos) .^ 2, 2));
    [~, ix_near] = min(dd);
    s_now = glob_cum(ix_near);
    if ~detour && s_now < s_prog - 0.05
        pos = interp_arc(glob, glob_cum, s_prog);
    else
        s_prog = max(s_prog, s_now);
    end
    % detour 模式下 s_glob 由投影驱动，保证绕行期间仍在前进（与 sim_rgvs 一致）
    if detour
        s_glob = min(glob_len, max(s_glob, min(s_now, s_glob + uav_speed)));
    end

    trail = [trail; pos]; %#ok<AGROW>

    if dyn_min_dist(pos, dyn) < env.safe || env_dist(pos, env) < env.safe
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
success = false;
end

% ================= 单步 DWA =================
function [npos, nvel] = dwa_step(pos, vel, ref, goal, env, dyn, r_safe, p)
% 近场过滤：只保留距 pos 35m 以内的障碍
if isfield(dyn, 'num') && dyn.num > 0
    d2 = sqrt(sum((dyn.curr - pos) .^ 2, 2));
    keep = d2 <= 35;
    if any(keep)
        dyn.num  = sum(keep);
        dyn.curr = dyn.curr(keep, :);
        if isfield(dyn, 'obs') && ~isempty(dyn.obs)
            dyn.obs = dyn.obs(keep);
        else
            dyn.params = dyn.params(keep, :);
        end
    else
        dyn.num  = 0;
        dyn.curr = zeros(0, 3);
        if isfield(dyn, 'obs') && ~isempty(dyn.obs)
            dyn.obs = cell(0, 1);
        else
            dyn.params = zeros(0, 7);
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
if norm(right) < 1e-6
    right = [1 0 0];
end
right = right / norm(right);
up = cross(right, refdir); up = up / max(norm(up), 1e-9);

% 动态窗口：当前速度 ± 加速度可达区间
v_lo = max(p.vmin, v_mag - p.dvmax);
v_hi = min(p.vmax, v_mag + p.dvmax);
vcand   = linspace(v_lo, v_hi, 5);
angcand = linspace(-p.dthmax, p.dthmax, 21);
pchcand = [-0.45 0 0.45];

% 预计算未来 1..pred 步障碍预测位置
obs_pred = cell(1, p.pred);
for st = 1:p.pred
    obs_pred{st} = dyn_after(dyn, st).curr;
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
            if dot(newdir, refdir) < -0.3
                continue;
            end
            newvel = newdir * v;
            p0 = pos; clear_min = inf; ok = true;
            for st = 1:p.pred
                p0 = p0 + newvel * p.dt;
                if ~dwa_feasible(p0, env, obs_pred{st}, r_safe)
                    ok = false;
                    break;
                end
                if dyn.num > 0
                    d = min(sqrt(sum((p0 - obs_pred{st}) .^ 2, 2))) - dyn.r;
                    clear_min = min(clear_min, max(d, 0));
                end
            end
            if ~ok
                continue;
            end
            hd = dot(newdir, refdir);
            cl = min(clear_min, 30) / 30;
            score = p.w_head * hd + p.w_clear * cl + p.w_speed * (v / p.vmax) - 0.05 * abs(da);
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
    % 无安全速度：8 方向逃逸，取第一个可静态/动态安全的偏移
    next_obs = obs_pred{1};
    d1 = (right + up);  d1 = d1 / max(norm(d1), 1e-9);
    d2 = (right - up);  d2 = d2 / max(norm(d2), 1e-9);
    d3 = (-right + up); d3 = d3 / max(norm(d3), 1e-9);
    d4 = (-right - up); d4 = d4 / max(norm(d4), 1e-9);
    dirs = [right; -right; up; -up; d1; d2; d3; d4];
    for k = 1:size(dirs, 1)
        cd = pos + dirs(k, :) * 6;
        if dwa_feasible(cd, env, next_obs, r_safe)
            nvel = dirs(k, :) * p.vmin;
            npos = pos + nvel * p.dt;
            return;
        end
    end
    nvel = [0 0 0];   % 原地悬停等待
    npos = pos;
end
end

function ok = dwa_feasible(p0, env, obs, r_safe)
if any(p0 < 0) || any(p0 > 200)
    ok = false;
    return;
end
if env_dist(p0, env) < env.safe + 0.5
    ok = false;
    return;
end
for k = 1:size(obs, 1)
    if norm(p0 - obs(k, :)) <= r_safe + 1.0
        ok = false;
        return;
    end
end
ok = true;
end

% ================= 工具函数 =================
function dn = dyn_after(dyn, n)
dn = dyn;
for i = 1:n
    dn = move_obstacles(dn);
end
end

function q = densify_path(path, ds)
d = [0; cumsum(sqrt(sum(diff(path) .^ 2, 2)))];
if d(end) < 1e-6
    q = path;
    return;
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

function s = nearest_arc(glob, glob_cum, pos)
[~, ix] = min(sum((glob - pos) .^ 2, 2));
s = glob_cum(ix);
end

function pos = move_towards(pos, target, spd)
v = target - pos; d = norm(v);
if d <= spd || d < 1e-9
    pos = target;
else
    pos = pos + (v / d) * spd;
end
end

function hit = lookahead_hit(glob, glob_cum, s0, dyn, r_safe, look_ahead, uav_speed)
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
    for k = 1:size(dpred.curr, 1)
        if norm(q - dpred.curr(k, :)) <= r_safe
            hit = true;
            return;
        end
    end
end
end

function r = rot3d(v, axis, ang)
c = cos(ang); s = sin(ang);
a = axis / max(norm(axis), 1e-9);
r = v * c + cross(a, v) * s + a * dot(a, v) * (1 - c);
end
