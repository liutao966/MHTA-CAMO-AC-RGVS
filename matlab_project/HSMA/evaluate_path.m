function [f, P] = evaluate_path(waypoints, model, n_pts, safe_dist)
% EVALUATE_PATH  由中间航点构建平滑路径并计算多目标函数
% -------------------------------------------------------------------------
% 输入:
%   waypoints : K×3  中间航点（不含起点/终点）
%   model     : 环境模型（build_environment 返回值）
%   n_pts     : 平滑后路径采样点数
%   safe_dist : 威胁代价的安全阈值(m)
% 输出:
%   f : [路径长度, 能耗, 不平滑度, 飞行时间, -最小安全距离, 威胁代价]  六个目标(均最小化)
%       第5维取负是因为多目标框架统一最小化, 安全距离越大越好
%   P : 平滑后的完整路径 N×3（起点→…→终点）
% -------------------------------------------------------------------------
V_DRONE = 10;   % 无人机恒定飞行速度 m/s

start = model.start;
goal  = model.goal;
P = path_from_waypoints(start, waypoints, goal, n_pts);
nP = size(P, 1);

% ---- 目标1: 路径长度 + 能耗所需数据 ----
seg_len = zeros(nP - 1, 1);
height_change = 0;
for i = 2:nP
    d = P(i, :) - P(i - 1, :);
    seg_len(i - 1) = norm(d);
    height_change = height_change + abs(d(3));
end
L = sum(seg_len);

% ---- 目标6: 威胁代价 ----
D_all = zeros(nP, 1);
threat = 0; hard = 0; HARD_PEN = 1e5;
for i = 1:nP
    D_all(i) = safe_dist_at_point(P(i, :), model);
    if D_all(i) < safe_dist
        threat = threat + (safe_dist - D_all(i));
        hard = hard + 1;
    end
end
threat = threat + hard * HARD_PEN;

% ---- 目标2: 能耗（论文完整模型）----
turn_angle_sum = 0; accel_sum = 0;
for i = 2:nP - 1
    v1 = P(i, :) - P(i - 1, :);
    v2 = P(i + 1, :) - P(i, :);
    n1 = norm(v1); n2 = norm(v2);
    if n1 > 1e-6 && n2 > 1e-6
        cos_a = max(min(dot(v1, v2) / (n1 * n2), 1), -1);
        ang = acos(cos_a);
        turn_angle_sum = turn_angle_sum + ang;
        accel_sum = accel_sum + 2 * sin(ang / 2);
    end
end

energy = L * 0.1 + height_change * 0.3 + 0.2 * turn_angle_sum + 0.15 * accel_sum;

% ---- 目标3: 不平滑度 = 1 - 相邻段夹角余弦均值 ----
cos_sum = 0; cnt = 0;
for i = 2:nP - 1
    v1 = P(i, :) - P(i - 1, :);
    v2 = P(i + 1, :) - P(i, :);
    n1 = norm(v1); n2 = norm(v2);
    if n1 > 1e-6 && n2 > 1e-6
        cos_sum = cos_sum + dot(v1, v2) / (n1 * n2);
        cnt = cnt + 1;
    end
end
smooth = cos_sum / (cnt + 1e-9);

% ---- 目标4: 飞行时间 ----
flight_time = L / V_DRONE;

% ---- 目标5: 最小安全距离 ----
% 多目标统一最小化, 安全距离越大越好 → 取负
d_safe_min = max(min(D_all), 0);
neg_safe   = -d_safe_min;

f = [L, energy, 1 - smooth, flight_time, neg_safe, threat];
end

% =========================================================================
function D = safe_dist_at_point(pos, model)
% 计算指定点到所有障碍物的最小安全距离（可正可负，越小越危险）
% 返回与 obstacles 的最小带符号距离（负值表示已在障碍物内部）。
x = pos(1); y = pos(2); z = pos(3);
dList = [];

% 城市地图: 平底地面(z=0) + 矩形高楼
if isfield(model, 'type') && strcmp(model.type, 'city')
    dList = [dList, z];                          %#ok<AGROW>  地面净空
    for i = 1:size(model.buildings, 1)
        dList = [dList, dist_to_box(pos, model.buildings(i, :))]; %#ok<AGROW>
    end
    D = min(dList);
    return;
end

% 1. 高斯山体: 点到地表高度差（z - 地表高度）
dList = [dList, z - terrain_height(x, y, model)]; %#ok<AGROW>

% 2. 球形禁飞区: 到球心距离 - 半径
if ~isempty(model.ball) && numel(model.ball) == 4
    dList = [dList, norm(pos - model.ball(1:3)) - model.ball(4)]; %#ok<AGROW>
end

% 3. 圆柱体: 区分是否在圆柱高度范围内
for i = 1:size(model.cylinder.pos, 1)
    cx = model.cylinder.pos(i, 1);
    cy = model.cylinder.pos(i, 2);
    cz = model.cylinder.pos(i, 3);
    r  = model.cylinder.r;
    horiz = norm([x - cx, y - cy]);
    if z >= 0 && z <= cz
        dCyl = horiz - r;
    else
        if z < 0
            vert = -z;
        else
            vert = z - cz;
        end
        dCyl = max(horiz - r, vert);
    end
    dList = [dList, dCyl]; %#ok<AGROW>
end
D = min(dList);
end

function dd = dist_to_box(pt, b)
% 点 pt 到轴对齐立方体 b=[x0 x1 y0 y1 h] (z 从 0 到 h) 的有符号距离:
%   外部为正(到最近表面距离), 内部为负(-到最近墙面距离)。
%   必须用负值表示内部, 否则威胁代价的硬惩罚(D<0)无法识别穿楼点,
%   会误把穿过高楼的直线路径判成"无碰撞"。
x  = pt(1); y = pt(2); z = pt(3);
x0 = b(1); x1 = b(2); y0 = b(3); y1 = b(4); h = b(5);

inside = (x > x0 && x < x1 && y > y0 && y < y1 && z > 0 && z < h);
if inside
    d_faces = min([x - x0, x1 - x, y - y0, y1 - y, z, h - z]);
    dd = -d_faces;
    return;
end

dx = max([x0 - x, x - x1, 0]);
dy = max([y0 - y, y - y1, 0]);
if z < 0
    dz = -z;
elseif z > h
    dz = z - h;
else
    dz = 0;
end
dd = sqrt(dx^2 + dy^2 + dz^2);
end

% =========================================================================
function h = terrain_height(x, y, model)
% 解析计算 (x,y) 处的地表高度 = 若干座高斯山的并集(取最大), 各自截断于 0。
% 城市地图为平底, 高度恒为 0。
if isfield(model, 'type') && strcmp(model.type, 'city')
    h = zeros(size(x));
    return;
end
h = 0;
for k = 1:size(model.mountains, 1)
    cx = model.mountains(k, 1);
    cy = model.mountains(k, 2);
    A  = model.mountains(k, 3);
    s  = model.mountains(k, 4);
    g  = A * exp(-((x - cx).^2 + (y - cy).^2) / (2 * s^2));
    h  = max(h, max(g, 0));
end
end

% =========================================================================
function P = path_from_waypoints(start, waypoints, goal, n_pts)
% 用累积弦长参数化 + pchip（保形三次插值）将折线路径平滑为 n_pts 个点。
% 注意: 航点可能因算法更新产生连续重复/重合点, 导致 pchip 断点 t 出现重复值,
%       chckxy 会报 "第一个输入必须包含唯一的值"。故先去除连续重复点,
%       并对段长强制下限(1e-6), 保证断点严格递增。
full = [start; waypoints; goal];

% 1) 去除连续重复(零长度段)航点
seg = sqrt(sum(diff(full, 1, 1).^2, 2));
keep = [true; seg > 1e-6];
full = full(keep, :);

% 2) 极端情形: 去重后不足 2 点(几乎全部重合), 直接首尾线性插值
if size(full, 1) < 2
    tt = linspace(0, 1, n_pts)';
    P  = start + (goal - start) .* tt;
    return;
end

% 3) 重算段长并强制最小段长, 保证 t 严格递增
seg = sqrt(sum(diff(full, 1, 1).^2, 2));
seg = max(seg, 1e-6);
t   = [0; cumsum(seg)];
tt  = linspace(t(1), t(end), n_pts);
P = zeros(n_pts, 3);
for d = 1:3
    P(:, d) = pchip(t, full(:, d), tt);
end
end