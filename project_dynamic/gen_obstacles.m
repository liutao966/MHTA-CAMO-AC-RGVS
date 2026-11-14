function dyn = gen_obstacles(count, seed, path, env)
%GEN_OBSTACLES 生成 count 个沿给定路径“横向穿越”的动态障碍物。
% 每个障碍物在遭遇点处沿路径水平法向做 S<->E 往返运动，周期性穿越规划路径。
% 相比“迎面高速拦截”，这里采用慢速穿越 + 加宽走廊 + 相位抖动，保证：
%   - 障碍物确实会与路径相交（真实触发避障，而非瞬间掠过）；
%   - 但移动慢（0.5 m/步，无人机 2 m/步），且相位随机，使障碍物间歇性地
%     挡在路径上，所有算法均有可解窗口（避免 0% 成功率的“必死”场景）。
rng(seed, 'twister');

dyn.num   = count;
dyn.r     = 5;      % 动态障碍物半径(m)
dyn.speed = 0.5;    % 每一时间步移动距离(m)（慢速穿越，给 UAV 留反应时间）
uav_speed = 2.0;    % 无人机前进速率，与各 sim 文件一致

% 路径弧长参数化（去重保证 interp1 可用）
seg = sqrt(sum(diff(path) .^ 2, 2));
cum = [0; cumsum(seg)];
[cum, ia] = unique(cum, 'stable');
path = path(ia, :);
L  = cum(end);

if count < 1 || L < 30
    dyn.params = zeros(0, 7);
    dyn.curr   = zeros(0, 3);
    return;
end

% 遭遇点沿弧长均匀分布 + 小抖动，并保证最小弧长间距，
% 避免高障碍数时两障碍重叠在同一路径段形成不可穿越的"墙"。
fracs = linspace(0.22, 0.78, count)';
fracs = sort(fracs + 0.02 * (rand(count, 1) - 0.5));
% 最小弧长间距自适应：保证 count 个障碍能塞进 [0.22,0.78] 窗口。
% 否则 10 个障碍时 0.08×9=0.72 >窗口0.56，被迫外扩后再 clamp 到 0.85，
% 会在路径末端把两个障碍挤到同一位置(重叠成"墙")，导致 10 障碍档异常下降。
min_gap = min(0.08, 0.62 / max(count - 1, 1));
for i = 2:count
    if fracs(i) - fracs(i - 1) < min_gap
        fracs(i) = fracs(i - 1) + min_gap;
    end
end
fracs = min(max(fracs, 0.15), 0.85);

params = zeros(count, 7);
curr0  = zeros(count, 3);
nc = 0;
for i = 1:count
    si = fracs(i) * L;
    p  = arc_interp(cum, path, si);

    % 夹缝过滤：穿越点离静态障碍太近则沿路径小幅重选，保证 DWA 有横向绕行空间
    ntry = 0;
    while env_dist(p, env) < 20 && ntry < 5
        si = min(L, max(0, si + L * 0.015 * (rand() - 0.5)));
        p  = arc_interp(cum, path, si);
        ntry = ntry + 1;
    end

    % 路径局部切向（用邻近弧长点差分）
    s1 = max(cum(1), si - 3);
    s2 = min(cum(end), si + 3);
    tan = arc_interp(cum, path, s2) - arc_interp(cum, path, s1);
    if norm(tan) < 1e-6
        continue;
    end
    tan = tan / norm(tan);

    % 水平横向方向：垂直于切向水平投影；若近纯垂直则退化为切向×y轴
    up = [0 0 1];
    side = cross(tan, up);
    if norm(side) < 1e-6
        side = cross(tan, [0 1 0]);
    end
    side(3) = 0;                  % 保持水平横向穿越
    if norm(side) < 1e-6
        side = [1 0 0];
    end
    side = side / norm(side);

    half_w = 25 + 10 * rand;      % 穿越半宽 25~35 m（加宽走廊，充分绕行空间）

    S = p - side * half_w;
    E = p + side * half_w;
    S(3) = p(3); E(3) = p(3);     % 与路径同高度穿越
    segL = norm(E - S);
    if segL < 1e-6
        continue;
    end
    u = (E - S) / segL;

    % 相位标定 + ±15% 随机抖动：障碍物在无人机到达时刻附近可靠地穿越路径
    % （避免±50%抖动导致一半种子障碍已越过、路径增量≈0的假成功），保留少量随机性。
    t_arrive = (si / uav_speed) * (0.85 + 0.3 * rand());

    % 遭遇点 p 在弦 S->E 上的投影距离（此处应为 half_w）
    W = dot(p - S, u);
    W = min(max(W, 0), segL);

    % 三角波相位：障碍物在 t_arrive 时应位于距 S 为 W 处（周期 2*segL）
    d0 = mod(W - dyn.speed * t_arrive, 2 * segL);
    if d0 > segL
        d0 = 2 * segL - d0;
        dir0 = -1;
    else
        dir0 = 1;
    end

    nc = nc + 1;
    params(nc, :) = [S, E, dir0];
    curr0(nc, :)  = S + u * d0;
end

dyn.num    = nc;
dyn.params = params(1:nc, :);
dyn.curr   = curr0(1:nc, :);
end

function p = arc_interp(cum, path, s)
s = min(max(s, cum(1)), cum(end));
p = [interp1(cum, path(:, 1), s), ...
     interp1(cum, path(:, 2), s), ...
     interp1(cum, path(:, 3), s)];
end