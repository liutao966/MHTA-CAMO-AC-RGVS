function [success, trail, status] = sim_vo(path, dyn, start, goal, env, sigma)
%SIM_VO 速度障碍法（VO）动态避障仿真（无头版）。
% 可选第 6 参数 sigma：障碍物位置观测噪声标准差(m)，默认 0；决策用观测、碰撞用真实。
if nargin < 6, sigma = 0; end
% 策略：每步朝全局参考路径的前视点取"偏好速度"，在速度空间采样候选速度，
%       用有限时间窗 tau 的碰撞锥判据排除会与动态障碍碰撞的速度，选最贴近偏好
%       且同时满足静态避障的最优速度。纯反应式、无全局重规划、无动力学约束。
% 接口与 sim_aapf / sim_avss / sim_rgvs 完全一致：
%   [success, trail] = sim_vo(path, dyn, start, goal, env)
uav_speed  = 2.0;
look_ahead = 28;    % 前视距离(m)，与 sim_rgvs 一致（公平：同一探测视野）
tau        = 15;    % VO 碰撞时间窗（步）
near_radius = 40;   % 近场障碍过滤半径(m)
max_steps  = 4000;

r_safe = dyn.r + env.safe;

glob = densify_path(path, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);

pos   = start;
vel   = (glob(2, :) - glob(1, :)) / max(norm(glob(2, :) - glob(1, :)), 1e-9) * uav_speed;
trail = start;
s_glob = 0;       % 沿全局路径的弧长进度
s_prog = 0;       % 已到达的最大弧长(前向约束)
detour = false;   % 是否处于避障绕行状态
success = false;
status = 0;       % 0=超时 1=到达 2=碰撞

for step = 1:max_steps
    dyn = move_obstacles(dyn);

    % 传感不确定性：观测位置 = 真实位置 + 高斯噪声；决策用观测，碰撞用真实
    if sigma > 0
        dyn_obs = dyn;
        dyn_obs.curr = dyn.curr + sigma * randn(size(dyn.curr));
    else
        dyn_obs = dyn;
    end

    % 前方路径是否被动态障碍挡住(前视检测)
    glob_blocked = lookahead_hit(glob, glob_cum, s_glob, dyn_obs, r_safe + 1.5, look_ahead, uav_speed);

    if ~glob_blocked
        % 沿全局路径前进
        detour = false;
        s_glob = min(glob_len, s_glob + uav_speed);
        target = interp_arc(glob, glob_cum, s_glob);
        pos = move_towards(pos, target, uav_speed);
        if norm(target - pos) > 1e-9
            vel = (target - pos) / norm(target - pos) * uav_speed;
        end
    else
        % 避障：偏好方向 = 全局路径前方点，用 VO 在速度空间选安全速度
        detour = true;
        ref = interp_arc(glob, glob_cum, min(glob_len, s_glob + look_ahead));
        v_pref = (ref - pos) / max(norm(ref - pos), 1e-9) * uav_speed;

        obs_pos = zeros(0, 3);
        obs_vel = zeros(0, 3);
        for i = 1:dyn_obs.num
            if norm(dyn_obs.curr(i, :) - pos) <= near_radius
                obs_pos(end + 1, :) = dyn_obs.curr(i, :); %#ok<AGROW>
                obs_vel(end + 1, :) = obs_velocity(dyn_obs, i); %#ok<AGROW>
            end
        end

        vel = vo_select(pos, v_pref, obs_pos, obs_vel, r_safe, tau, env);
        pos = pos + vel;
    end

    % 前向约束：正常巡线时不允许沿参考路径回退；绕行期间不吸附回路径
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
success = false;   % 超时
end

% ================= VO 速度选择 =================
function vel = vo_select(pos, v_pref, obs_pos, obs_vel, r_safe, tau, env)
uav_speed = norm(v_pref);
if uav_speed < 1e-9
    uav_speed = 2.0;
end
pref_dir = v_pref / max(norm(v_pref), 1e-9);

% 先试偏好速度本身
if ~any_vo_collision(pos, v_pref, obs_pos, obs_vel, r_safe, tau) ...
        && step_static_free(pos, v_pref, env, env.safe)
    vel = v_pref;
    return;
end

% 速度空间采样：速度档 × 水平航向 × 俯仰（候选数与 DWA 基线保持同量级）
up0 = [0 0 1];
right = cross(pref_dir, up0);
if norm(right) < 1e-6
    right = [1 0 0];
end
right = right / norm(right);
up = cross(right, pref_dir); up = up / max(norm(up), 1e-9);

speeds = uav_speed * [1.0 0.7 0.5 0.3];
yaws   = linspace(-1.5, 1.5, 21);
pitchs = [-0.45 0 0.45];

best = [];
best_score = -inf;
for sp = speeds
    for ya = yaws
        for pc = pitchs
            dir = rot3d(pref_dir, up, ya);
            dir = rot3d(dir, right, pc);
            dir = dir / max(norm(dir), 1e-9);
            if dot(dir, pref_dir) < -0.2
                continue;
            end
            v = dir * sp;
            if any_vo_collision(pos, v, obs_pos, obs_vel, r_safe, tau)
                continue;
            end
            if ~step_static_free(pos, v, env, env.safe)
                continue;
            end
            score = dot(dir, pref_dir) + 0.4 * (sp / uav_speed) - 0.02 * abs(ya);
            if score > best_score
                best_score = score;
                best = v;
            end
        end
    end
end
if ~isempty(best)
    vel = best;
    return;
end

% 应急逃逸：8 个大幅偏移方向，取第一个同时满足动态/静态安全的小步速度
d1 = (right + up);  d1 = d1 / max(norm(d1), 1e-9);
d2 = (right - up);  d2 = d2 / max(norm(d2), 1e-9);
d3 = (-right + up); d3 = d3 / max(norm(d3), 1e-9);
d4 = (-right - up); d4 = d4 / max(norm(d4), 1e-9);
escape_dirs = [right; -right; up; -up; d1; d2; d3; d4];
for k = 1:size(escape_dirs, 1)
    v_test = escape_dirs(k, :) * uav_speed * 0.5;
    if ~any_vo_collision(pos, v_test, obs_pos, obs_vel, r_safe, tau) ...
            && step_static_free(pos, v_test, env, env.safe)
        vel = v_test;
        return;
    end
end
vel = [0 0 0];   % 全部方向不可行：原地悬停等待障碍移开
end

% ================= 碰撞锥判据 =================
function tf = any_vo_collision(pos, v, obs_pos, obs_vel, r_safe, tau)
tf = false;
for i = 1:size(obs_pos, 1)
    if in_vo(pos, v, obs_pos(i, :), obs_vel(i, :), r_safe, tau)
        tf = true;
        return;
    end
end
end

function tf = in_vo(pos, v, obs_pos, obs_vel, r_safe, tau)
%IN_VO 有限时间窗速度障碍判据：相对速度在时间 tau 内是否使 UAV 与障碍中心距 <= r_safe。
r = obs_pos - pos;          % UAV -> 障碍
dv = v - obs_vel;           % 相对速度（UAV 减障碍）
d0 = norm(r);
if d0 <= r_safe
    tf = true;
    return;
end
ndv2 = sum(dv .^ 2);
if ndv2 < 1e-12
    tf = false;             % 无相对运动，且当前已不重叠
    return;
end
t_c = dot(r, dv) / ndv2;    % 最近接近时刻
if t_c < 0 || t_c > tau
    tf = false;             % 正在远离，或碰撞发生在时间窗外
    return;
end
d_min = norm(r - dv * t_c);
tf = d_min <= r_safe;
end

% ================= 工具函数 =================
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
% 前视检测：判断未来 look_ahead 内路径是否会与动态障碍碰撞(按 UAV 到达时刻预测障碍位置)
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

function dn = dyn_after(dyn, n)
dn = dyn;
for i = 1:n
    dn = move_obstacles(dn);
end
end

function v = obs_velocity(dyn, i)
%OBS_VELOCITY 返回障碍物 i 的瞬时速度向量（m/步），供 VO 碰撞锥外推使用。
% 四类运动分别给出瞬时速度；VO/RVO 会按"该速度恒定"外推，因此对圆弧/变速/垂直
% 障碍的预测会失准——这正是这些类型打破 VO/RVO 匀速直线假设的方式。
if isfield(dyn, 'obs') && ~isempty(dyn.obs)
    o = dyn.obs{i};
    switch o.type
        case 1
            u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
            v = u * o.speed * o.dir;
        case 2
            v = o.R * o.omega * [-sin(o.theta), cos(o.theta), 0];
        case 3
            u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
            v = u * o.v0 * (1 + o.amp * sin(2 * pi * o.t / o.period)) * o.dir;
        case 4
            u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
            v = u * o.hspeed * o.dir + [0, 0, o.A * (2 * pi / o.Tz) * cos(2 * pi * o.t / o.Tz)];
        otherwise
            v = [0 0 0];
    end
else
    p1 = dyn.params(i, 1:3);
    p2 = dyn.params(i, 4:6);
    u = (p2 - p1) / max(norm(p2 - p1), 1e-9);
    v = u * dyn.speed * dyn.params(i, 7);
end
end

function tf = step_static_free(pos, vel, env, safe)
L = norm(vel);
if L < 1e-9
    tf = env_dist(pos, env) >= safe + 0.5;
    return;
end
n = max(2, ceil(L / 0.5));
t = linspace(0, 1, n)';
pts = pos + t .* vel;
for i = 1:n
    if env_dist(pts(i, :), env) < safe + 0.5
        tf = false;
        return;
    end
end
tf = true;
end

function r = rot3d(v, axis, ang)
c = cos(ang); s = sin(ang);
a = axis / max(norm(axis), 1e-9);
r = v * c + cross(a, v) * s + a * dot(a, v) * (1 - c);
end
