function [cost, stats] = evaluate_path(waypoints, model, n_pts, safe_dist)
% EVALUATE_PATH  由中间航点构建平滑路径并计算单目标综合成本
% -------------------------------------------------------------------------
% 综合成本 cost = 0.4*长度 + 0.3*威胁 + 0.2*能耗 + 0.1*不平滑度（均最小化）
%   长度(软)+威胁(含硬惩罚: 穿障叠加 1e5/点, 保证输出路径不穿障碍)
% stats = [长度, 威胁, 能耗, 不平滑度, 最小安全距离, 飞行时间]
% -------------------------------------------------------------------------
P = path_from_waypoints(model.start, waypoints, model.goal, n_pts);

V_FLIGHT = 10;

% ---- 路径长度 与 高度变化 ----
L = 0; hc = 0;
for i = 2:size(P, 1)
    d = P(i, :) - P(i - 1, :);
    L = L + norm(d);
    hc = hc + abs(d(3));
end

% ---- 威胁代价: 软惩罚 + 越界(贴面/穿障)硬惩罚 + 最小安全距离 ----
HARD_PEN = 1e5;
threat = 0;
hard = 0;
min_clearance = inf;
for i = 1:size(P, 1)
    D = safe_dist_at_point(P(i, :), model);
    min_clearance = min(min_clearance, D);
    if D < safe_dist
        threat = threat + (safe_dist - D);
        hard   = hard + 1;
    end
end
threat = threat + hard * HARD_PEN;
if isinf(min_clearance), min_clearance = 0; end

% ---- 不平滑度 = 1 - 相邻段夹角余弦均值 ----
cosSum = 0; cnt = 0;
turn_angle_sum = 0; accel_sum = 0;   % 转向角累计 / 加速项累计
for i = 2:size(P, 1) - 1
    v1 = P(i, :) - P(i - 1, :);
    v2 = P(i + 1, :) - P(i, :);
    n1 = norm(v1); n2 = norm(v2);
    if n1 > 1e-6 && n2 > 1e-6
        c = dot(v1, v2) / (n1 * n2);
        cosSum = cosSum + c;
        cnt = cnt + 1;
        ang = acos(max(min(c, 1), -1));
        turn_angle_sum = turn_angle_sum + ang;
        accel_sum = accel_sum + 2 * sin(ang / 2);
    end
end
smooth = cosSum / (cnt + 1e-9);
rough  = 1 - smooth;

% ---- 能耗: 论文改进模型(长度×0.1 + 高度变化×0.3 + 转向×0.2 + 加速×0.15) ----
energy = L * 0.1 + hc * 0.3 + 0.2 * turn_angle_sum + 0.15 * accel_sum;

% ---- 飞行时间 ----
flight_time = L / V_FLIGHT;

% ---- 综合成本 ----
w = [0.4, 0.3, 0.2, 0.1];
obj = [L, threat, energy, rough];
cost  = w * obj';
stats = [L, threat, energy, rough, min_clearance, flight_time];
end

% =========================================================================
function D = safe_dist_at_point(pos, model)
% 计算点到所有障碍物的最小带符号距离（负值=在障碍内部）
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

% 高斯山体: 点到地表高度差（z - 地表高度）
dList = [dList, z - terrain_height(x, y, model)]; %#ok<AGROW>

% 球形禁飞区: 到球心距离 - 半径（存在时）
if ~isempty(model.ball) && numel(model.ball) == 4
    dList = [dList, norm(pos - model.ball(1:3)) - model.ball(4)]; %#ok<AGROW>
end

% 圆柱体: 区分是否在圆柱高度范围内（存在时）
for i = 1:size(model.cylinder.pos, 1)
    cx = model.cylinder.pos(i, 1);
    cy = model.cylinder.pos(i, 2);
    cz = model.cylinder.pos(i, 3);
    r  = model.cylinder.r;
    horiz = norm([x - cx, y - cy]);
    if z >= 0 && z <= cz
        dCyl = horiz - r;
    else
        vert = -z; if z > cz, vert = z - cz; end
        dCyl = max(horiz - r, vert);
    end
    dList = [dList, dCyl]; %#ok<AGROW>
end
D = min(dList);
end

function dd = dist_to_box(pt, b)
% 点 pt 到轴对齐立方体 b=[x0 x1 y0 y1 h](z 从 0 到 h) 的有符号距离:
%   外部为正(到最近表面距离), 内部为负(-到最近墙面距离)。
%   必须用负值表示内部, 否则威胁代价的硬惩罚无法识别穿楼点。
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
% 解析计算 (x,y) 处的地表高度 = 若干座高斯山的并集(取最大), 各自截断于0。
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
% 累积弦长参数化 + pchip 平滑。
% 航点可能出现连续重复/重合点, 导致 pchip 断点 t 重复而报
% "第一个输入必须包含唯一的值", 故先去除连续重复点并对段长强制下限。
full = [start; waypoints; goal];

% 1) 去除连续重复(零长度段)航点
seg = sqrt(sum(diff(full, 1, 1).^2, 2));
keep = [true; seg > 1e-6];
full = full(keep, :);

% 2) 极端情形: 去重后不足 2 点, 直接首尾线性插值
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