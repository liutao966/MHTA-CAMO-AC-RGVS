function [success, trail, status] = sim_rgvs(path, dyn, start, goal, env, sigma)
%SIM_RGVS MHTA+CAMO-AC 算法（本方法）的 RGVS 动态避障仿真（无头版，保留原策略）。
% 策略：沿全局路径前进 + 前视检测 + RGVS 局部规划器绕行 + 避障后重规划回归。
% 可选第 6 参数 sigma：障碍物位置观测噪声标准差(m)，默认 0；决策用观测、碰撞用真实。
if nargin < 6, sigma = 0; end
uav_speed  = 2.0;
look_ahead = 28;      % 固定前视：一次只锁定最近的一个障碍，避免密集场景多障碍同时触发
max_steps  = 4000;    % 固定最大步数
r_safe = dyn.r + env.safe;

glob = densify_path(path, 1.0);
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);

pos = start;
trail = start;
vel = (glob(2, :) - glob(1, :)) / max(norm(glob(2, :) - glob(1, :)), 1e-9) * uav_speed;
s_glob = 0;
s_prog = 0;
detour = false;
detour_steps = 0;
min_detour_steps = 12;   % 绕行周期恢复到基线：太短会导致未绕开障碍就回到路径、高频振荡
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

    % 绕行期间用更短前视(12m)判断能否回到路径；巡线时用正常前视 28m 提前探测。
    la = 12;
    if ~detour
        la = look_ahead;
    end
    glob_blocked = lookahead_hit(glob, glob_cum, s_glob, dyn_obs, r_safe + 1.5, la, uav_speed);

    if ~glob_blocked && (~detour || detour_steps >= min_detour_steps)
        % 退出绕行回到路径：仅在连续绕行足够步数后才允许
        if detour
            [glob, glob_cum, glob_len, s_glob] = replan_after_detour(pos, glob, env);
            s_prog = 0;
            if size(glob, 1) >= 2 && norm(glob(2, :) - glob(1, :)) > 1e-9
                vel = (glob(2, :) - glob(1, :)) / norm(glob(2, :) - glob(1, :)) * uav_speed;
            end
            detour = false;
            detour_steps = 0;
        end
        s_glob = min(glob_len, s_glob + uav_speed);
        target = interp_arc(glob, glob_cum, s_glob);
        pos = move_towards(pos, target, uav_speed);
        if norm(target - pos) > 1e-9
            vel = (target - pos) / norm(target - pos) * uav_speed;
        end
    else
        detour = true;
        detour_steps = detour_steps + 1;
        % 绕行期间参考点不按固定速度前冲，而是用无人机在全局路径上的
        % 最近投影来推进（见下方 s_now 计算），避免多障碍密集场景下
        % s_glob 剪刀差式前冲、ref 被拉到远处、RGVS 走出大弯路。
        ref = interp_arc(glob, glob_cum, min(glob_len, s_glob + look_ahead));
        dw = struct('vmax', uav_speed, 'vmin', 0.3, 'dvmax', 0.6, 'dthmax', 1.5, ...
                    'dt', 1.0, 'pred', 8, 'w_head', 2.0, 'w_clear', 3.0, ...
                    'w_speed', 0.4, 'w_rot', 0.2, 'safem', env.safe);
        [pos, vel] = rgvs_plan(pos, vel, ref, goal, env, dyn_obs, r_safe, dw);
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

    % detour 模式下 s_glob 由 pos 在全局路径上的实际投影驱动，
    % 加上与巡线一致的速度上限 uav_speed，防止 pos 沿大弯路回退时
    % s_glob 不当地前跳。巡线模式仍固定 +uav_speed。
    if detour
        s_glob = min(glob_len, max(s_glob, min(s_now, s_glob + uav_speed)));
    end

    % 边界保护：放在最后，确保最终 pos 始终在 200×200 地图内（三次样条过冲越界的兜底）
    pos = min(max(pos, 0), 200);

    trail = [trail; pos]; %#ok<AGROW>

    d_dyn = dyn_min_dist(pos, dyn);
    d_env = env_dist(pos, env);
    if d_dyn < env.safe || d_env < env.safe
        fprintf('[RGVS DIAG] 碰撞失败 step=%d 障碍数=%d 动态间隙=%.2f 静态间隙=%.2f 位置=(%.1f,%.1f,%.1f)\n', ...
            step, dyn.num, d_dyn, d_env, pos(1), pos(2), pos(3));
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
fprintf('[RGVS DIAG] 超时 step=%d 位置=(%.1f,%.1f,%.1f) 距目标=%.2f\n', ...
    max_steps, pos(1), pos(2), pos(3), norm(pos - goal));
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

function hit = lookahead_hit(glob, glob_cum, s0, dyn, r_safe, look_ahead, uav_speed)
%LOOKAHEAD_HIT 前视检测：判断未来 look_ahead 内路径是否会与动态障碍碰撞。
% 对每个前视弧长点，用“无人机到达该点的时刻”预测障碍物当时的位置，
% 避免用当前障碍位置做整段检测导致的漏检/误报（慢速穿越障碍也会移动）。
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

function [glob, glob_cum, glob_len, s_new] = replan_after_detour(pos, glob, env)
dd = sqrt(sum((glob - pos) .^ 2, 2));
[~, ix] = min(dd);
join = ix;
for k = (ix + 5):5:size(glob, 1)
    if k <= ix
        continue;
    end
    if seg_free_static(pos, glob(k, :), env, 1.0)
        join = k;
    else
        break;
    end
end
refpts = [pos; glob(join:end, :)];
if size(refpts, 1) < 4
    newpath = refpts;
else
    newpath = smooth_spline(refpts);
end
newpath(1, :) = pos;
% 绕行回归路径同样强制静态间隙 >= safe+1，避免末端圆柱区 detour 回归时切角导致贴/撞静态障碍
newpath = enforce_clearance(newpath, env);
glob = densify_path(newpath, 1.0);
% 兜底：若 enforce_clearance 后仍有切角压进禁区，退回无样条原始路径并再次强制间隙
if ~allpoints_ok(glob, env)
    glob = densify_path(enforce_clearance(refpts, env), 1.0);
end
glob_cum = [0; cumsum(sqrt(sum(diff(glob) .^ 2, 2)))];
glob_len = glob_cum(end);
s_new = 0;
end

function ok = allpoints_ok(path, env)
ok = true;
for i = 1:size(path, 1)
    if env_dist(path(i, :), env) < env.safe + 0.5
        ok = false;
        return;
    end
end
end

function [npos, nvel] = rgvs_plan(pos, vel, ref, goal, env, dyn, r_safe, p)
% 近场过滤：只保留距 pos 35m 以内的障碍，避免远处障碍预测轨迹过度约束局部搜索
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

v_lo = max(p.vmin, v_mag - p.dvmax);
v_hi = min(p.vmax, v_mag + p.dvmax);
vcand = linspace(v_lo, v_hi, 5);   % 去掉 v=0，强制向前/绕行，避免原地悬停拉低路径增量
angcand = linspace(-p.dthmax, p.dthmax, 21);   % 更密的航向候选，狭窄通道更易找到可行解
pchcand = [-0.45 0 0.45];   % 恢复基线俯仰候选，避免过度爬升导致漂移/超时

% 预计算未来 1..pred 步各障碍物的预测位置（避免候选循环内重复推进障碍）
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
                obs_st = obs_pred{st};
                if ~rgvs_feasible(p0, env, obs_st, r_safe)
                    ok = false;
                    break;
                end
                if dyn.num > 0
                    d = min(sqrt(sum((p0 - obs_st) .^ 2, 2))) - dyn.r;
                    clear_min = min(clear_min, max(d, 0));
                end
            end
            if ~ok
                continue;
            end
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
    % 无安全速度：一次性大幅侧向偏移（8 方向 × 3 档幅度），等效 pick_detour 式逃逸
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
            if rgvs_feasible(cd, env, next_obs, r_safe)
                nvel = dirs(k, :) * p.vmin;   % 朝安全偏移方向小步推进（不瞬移）
                npos = pos + nvel * p.dt;
                return;
            end
        end
    end
    return;   % 全部方向都不可行：原地悬停等待障碍移开
end
end

function ok = rgvs_feasible(p0, env, obs, r_safe)
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

function dn = dyn_after(dyn, n)
% DYN_AFTER 返回 n 步之后的障碍物状态（用于 RGVS 预测障碍运动）。
dn = dyn;
for i = 1:n
    dn = move_obstacles(dn);
end
end

function r = rot3d(v, axis, ang)
c = cos(ang); s = sin(ang);
a = axis / max(norm(axis), 1e-9);
r = v * c + cross(a, v) * s + a * dot(a, v) * (1 - c);
end