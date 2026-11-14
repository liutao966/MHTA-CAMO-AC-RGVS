function smooth = run_acosrar(map_type, do_plot, buildings)
% RUN_ACOSRAR  ACOSRAR 三维路径规划主程序（按地图类型 city/hill 分派）
% -------------------------------------------------------------------------
% 功能:
%   1) 按 map_type 构建三维环境(城市高楼 / 丘陵山体 / 旧地图);
%   2) 双向蚁群搜索(起点/终点同时出发, 5x5x5 邻域, 信息素-启发式-平滑-偏航加权);
%   3) 目标函数含连续碰撞裕度惩罚(不穿障), 自适应信息素更新 + 双向交汇合并;
%   4) 后处理(弧长重采样 + 障碍栅格兜底吸附, 不切角穿障);
%   5) 绘制: 三维地图 + 唯一最优路径(红) + 起点(绿球) + 终点(蓝球) + 收敛曲线。
%     坐标轴单位固定为 20, 正交投影保证三轴垂直。
% -------------------------------------------------------------------------
if nargin < 1 || isempty(map_type), map_type = 'legacy'; end
if nargin < 2 || isempty(do_plot), do_plot = true; end

map_range = [0, 200];
grid_step = 2;                 % 栅格步长(m), 降低计算量
safe_dist = 2.5;                 % 障碍安全净空(m)
smooth_nodes = 200;            % 最终路径采样点数

model = build_environment(map_type);
if nargin >= 3 && ~isempty(buildings)
    model.buildings = buildings;   % 用调用方（eval_metrics 同一套）的楼
end
start = model.start;
goal  = model.goal;

x_nodes = map_range(1):grid_step:map_range(2);
y_nodes = map_range(1):grid_step:map_range(2);
z_nodes = map_range(1):grid_step:map_range(2);
nx = numel(x_nodes); ny = numel(y_nodes); nz = numel(z_nodes);

% ---- ACOSRAR 参数 ----
ant_num     = 50;              % 蚂蚁数量
max_iter    = 150;             % 最大迭代次数
beta        = 2.0;             % 启发式因子
K           = 100;             % 信息素强度
alpha_val   = 1.2;             % 信息素重要度
rho         = 0.2;             % 局部信息素挥发
rho_global  = 0.3;             % 全局信息素挥发
smooth_weight = 0.5;           % 平滑权重
max_search  = 400;             % 单只蚂蚁最大搜索步数
wL = 0.6; wR = 0.25; wE = 0.15; w_dist_line = 0.1;

dir_0 = (goal - start) / norm(goal - start);
max_dir_angle   = 120;         % 方向约束(宽松, 允许绕行)
max_dist_to_line = 150;        % 偏离直线约束(宽松)
meet_threshold = grid_step * 2;

fprintf('===== ACOSRAR 三维路径规划（%s）=====\n', map_name_of(map_type));
fprintf('起点 [%g %g %g]   终点 [%g %g %g]\n', start, goal);

%% 1. 构建障碍栅格(栅格顺序: x, y, z)
obstacle = build_obstacle(model, x_nodes, y_nodes, z_nodes, safe_dist);

%% 2. 起终点栅格索引
[si, sj, sk] = nearest_xyz(start, x_nodes, y_nodes, z_nodes);
[ei, ej, ek] = nearest_xyz(goal,  x_nodes, y_nodes, z_nodes);
if obstacle(si, sj, sk), error('起点位于障碍/禁飞区内，请调整起点坐标！'); end
if obstacle(ei, ej, ek), error('终点位于障碍/禁飞区内，请调整终点坐标！'); end

%% 3. 信息素初始化
tau_L = ones(nx, ny, nz);
tau_R = ones(nx, ny, nz);
tau_E = ones(nx, ny, nz);
% 预铺起点-终点直线高信息素(仅落在自由栅格)
line_steps = round(norm(goal - start) / grid_step);
for step = 0:line_steps
    p = start + step * grid_step * dir_0;
    if p(1) < map_range(1) || p(1) > map_range(2) || ...
       p(2) < map_range(1) || p(2) > map_range(2) || ...
       p(3) < map_range(1) || p(3) > map_range(2)
        continue;
    end
    [pi, pj, pk] = nearest_xyz(p, x_nodes, y_nodes, z_nodes);
    if ~obstacle(pi, pj, pk), tau_L(pi, pj, pk) = 8; end
end
if ~obstacle(ei, ej, ek), tau_L(ei, ej, ek) = 6; end
tau_total = 0.6 * tau_L + 0.25 * tau_R + 0.15 * tau_E;

%% 4. ACOSRAR 主循环
global_best_path = [];
global_best_F    = inf;
iter_best    = nan(max_iter, 1);
global_hist  = nan(max_iter, 1);
ant_num_start = floor(ant_num / 2);

fprintf('开始 ACOSRAR 蚁群搜索...\n');
tic;
for iter = 1:max_iter
    % ---- 4.1 自适应状态识别 ----
    if mod(iter, 5) == 0
        scout_num = round(ant_num * 0.1);
        R = 0;
        for s = 1:scout_num
            for samp = 1:50
                i = randi(nx); j = randi(ny); k = randi(nz);
                nb = obstacle(max(1,i-1):min(nx,i+1), max(1,j-1):min(ny,j+1), max(1,k-1):min(nz,k+1));
                Tj = sum(nb(:)) / numel(nb);
                Pj = max(0, min(norm(cross([x_nodes(i),y_nodes(j),z_nodes(k)] - start, dir_0)) / 30, 0.4));
                R = R + Tj + Pj;
            end
        end
        R = R / (scout_num * 50);
        if R < 0.3,        alpha_val = 1.8; rho = 0.15;
        elseif R < 0.6,    alpha_val = 1.2; rho = 0.2;
        else,              alpha_val = 0.9; rho = 0.22;
        end
    end

    ant_paths = cell(ant_num, 1);
    ant_F     = inf(ant_num, 1);

    % ---- 4.2 起点出发的蚂蚁 ----
    for ant = 1:ant_num_start
        fp = search_ant(start, goal, obstacle, x_nodes, y_nodes, z_nodes, ...
            nx, ny, nz, dir_0, max_dir_angle, max_dist_to_line, tau_total, ...
            alpha_val, beta, w_dist_line, smooth_weight, max_search);
        ant_paths{ant} = fp;
        ant_F(ant) = compute_F(fp, start, goal, model, safe_dist, dir_0, wL, wR, wE, w_dist_line);
    end

    % ---- 4.3 终点出发的蚂蚁(反向搜索, 原始路径为 goal->start) ----
    dir_rev = -dir_0;
    for ant = ant_num_start+1:ant_num
        bp = search_ant(goal, start, obstacle, x_nodes, y_nodes, z_nodes, ...
            nx, ny, nz, dir_rev, max_dir_angle, max_dist_to_line, tau_total, ...
            alpha_val, beta, w_dist_line, smooth_weight, max_search);
        ant_paths{ant} = bp;
        if ~isempty(bp)
            ant_F(ant) = compute_F(flip(bp, 1), start, goal, model, safe_dist, dir_0, wL, wR, wE, w_dist_line);
        end
    end

    % ---- 4.4 双向路径交汇检测 ----
    if iter > 10
        for s = 1:ant_num_start
            fp = ant_paths{s};
            if isempty(fp) || size(fp, 1) < 3, continue; end
            for e = ant_num_start+1:ant_num
                bp = ant_paths{e};
                if isempty(bp) || size(bp, 1) < 3, continue; end
                if norm(fp(end, :) - bp(end, :)) <= meet_threshold
                    bpf = flip(bp, 1);
                    merged = [fp; bpf(2:end, :)];
                    Fm = compute_F(merged, start, goal, model, safe_dist, dir_0, wL, wR, wE, w_dist_line);
                    if Fm < global_best_F
                        global_best_F = Fm;
                        global_best_path = merged;
                    end
                end
            end
        end
    end

    % ---- 4.5 信息素更新 ----
    for ant = 1:ant_num
        p = ant_paths{ant};
        if isempty(p) || size(p, 1) < 2 || ~isfinite(ant_F(ant)), continue; end
        delta = K / ant_F(ant);
        for q = 1:size(p, 1)
            [qi, qj, qk] = nearest_xyz(p(q, :), x_nodes, y_nodes, z_nodes);
            tau_L(qi, qj, qk) = (1 - rho) * tau_L(qi, qj, qk) + rho * delta;
            tau_R(qi, qj, qk) = (1 - rho) * tau_R(qi, qj, qk) + rho * delta;
            tau_E(qi, qj, qk) = (1 - rho) * tau_E(qi, qj, qk) + rho * delta;
        end
    end
    tau_total = 0.6 * tau_L + 0.25 * tau_R + 0.15 * tau_E;

    % ---- 4.6 全局最优更新 ----
    fin = isfinite(ant_F);
    if any(fin)
        [minF, mi] = min(ant_F(fin));
        idxs = find(fin);
        if minF < global_best_F
            global_best_F = minF;
            bi = idxs(mi);
            if bi > ant_num_start && ~isempty(ant_paths{bi})
                global_best_path = flip(ant_paths{bi}, 1);   % 反向蚂蚁存的是 goal->start, 翻转
            else
                global_best_path = ant_paths{bi};
            end
        end
    end

    iter_best(iter)   = min(ant_F);       % 无效时 min(inf)=inf
    global_hist(iter) = global_best_F;
    if mod(iter, 20) == 0
        fprintf('迭代 %d/%d: 当前最优 F=%.2f\n', iter, max_iter, global_best_F);
    end
end
elapsed = toc;
fprintf('ACOSRAR 搜索完成: 耗时 %.2f s, 全局最优 F=%.2f\n', elapsed, global_best_F);

%% 5. 后处理: 弧长重采样 + 兜底吸附(不用贝塞尔, 避免切角穿障)
if isempty(global_best_path) || size(global_best_path, 1) < 2
    warning('未找到可行路径，请放宽参数(增大 max_dir_angle/max_dist_to_line/max_iter)。');
    smooth = [];
    return;
end
smooth = resample_polyline(global_best_path, smooth_nodes);
smooth = snap_to_free(smooth, obstacle, x_nodes, y_nodes, z_nodes);
smooth = repair_path_geo(smooth, model, safe_dist, obstacle, x_nodes, y_nodes, z_nodes);

%% 6. 可视化
if do_plot
    draw_map_and_path(model, smooth, map_type);
    draw_convergence(iter_best, global_hist, max_iter);
end

%% 7. 结果输出
raw_len = sum(sqrt(sum(diff(global_best_path).^2, 2)));
smooth_len = sum(sqrt(sum(diff(smooth).^2, 2)));
fprintf('===== ACOSRAR 规划完成 =====\n');
fprintf('原始路径长度   : %.2f m (节点 %d)\n', raw_len, size(global_best_path, 1));
fprintf('平滑后路径长度 : %.2f m (节点 %d)\n', smooth_len, size(smooth, 1));
fprintf('终点偏差       : %.2f m\n', norm(smooth(end, :) - goal));
fprintf('路径最大高度   : %.2f m\n', max(smooth(:, 3)));

%% 8. 计算并打印 6 个目标值
f6 = compute_six_objectives(smooth, model, safe_dist);
fprintf('--- 6 个目标值 ---\n');
fprintf('  目标1 - 路径长度     = %.2f m\n', f6(1));
fprintf('  目标2 - 能耗         = %.2f Wh\n', f6(2));
fprintf('  目标3 - 不平滑度     = %.4f (越小越平滑)\n', f6(3));
fprintf('  目标4 - 飞行时间     = %.2f s (V=10m/s)\n', f6(4));
fprintf('  目标5 - 最小安全距离 = %.2f m (路径上距障碍物最近点)\n', f6(5));
fprintf('  目标6 - 威胁代价     = %.2f (0 表示全程无碰撞)\n',    f6(6));
end

% =========================================================================
function [ix, iy, iz] = nearest_xyz(p, xs, ys, zs)
[~, ix] = min(abs(xs - p(1)));
[~, iy] = min(abs(ys - p(2)));
[~, iz] = min(abs(zs - p(3)));
end

% =========================================================================
function obs = build_obstacle(model, xs, ys, zs, safe_dist)
% 构建障碍栅格(栅格顺序: x, y, z), obs(ix,iy,iz)
nx = numel(xs); ny = numel(ys); nz = numel(zs);
[X3, Y3] = ndgrid(xs, ys);   % nx x ny
terrain = zeros(nx, ny);
for m = 1:size(model.mountains, 1)
    cx = model.mountains(m,1); cy = model.mountains(m,2);
    A  = model.mountains(m,3); s  = model.mountains(m,4);
    terrain = max(terrain, A * exp(-((X3 - cx).^2 + (Y3 - cy).^2) / (2 * s^2)));
end

obs = false(nx, ny, nz);
for k = 1:nz
    z = zs(k);
    % 地形(城市平底地面亦受此约束)
    obs(:, :, k) = (z < terrain + safe_dist);
    % 高楼
    for b = 1:size(model.buildings, 1)
        x0 = model.buildings(b,1); x1 = model.buildings(b,2);
        y0 = model.buildings(b,3); y1 = model.buildings(b,4);
        h  = model.buildings(b,5);
        if z <= h + safe_dist
            inXY = (X3 >= x0 - safe_dist) & (X3 <= x1 + safe_dist) & ...
                   (Y3 >= y0 - safe_dist) & (Y3 <= y1 + safe_dist);
            obs(:, :, k) = obs(:, :, k) | inXY;
        end
    end
    % 圆柱体
    if ~isempty(model.cylinder.pos) && model.cylinder.r > 0
        cr = model.cylinder.r;
        for c = 1:size(model.cylinder.pos, 1)
            cx = model.cylinder.pos(c,1); cy = model.cylinder.pos(c,2); h = model.cylinder.pos(c,3);
            if z >= 0 && z <= h
                obs(:, :, k) = obs(:, :, k) | (((X3 - cx).^2 + (Y3 - cy).^2) < (cr + safe_dist)^2);
            end
        end
    end
end
% 球形禁飞区
if ~isempty(model.ball) && numel(model.ball) == 4
    cx = model.ball(1); cy = model.ball(2); cz = model.ball(3); br = model.ball(4);
    dz = reshape(zs - cz, 1, 1, nz);
    d2 = (X3 - cx).^2 + (Y3 - cy).^2 + dz.^2;
    obs = obs | (d2 < (br + safe_dist)^2);
end
end

% =========================================================================
function path = search_ant(from, to, obstacle, xs, ys, zs, nx, ny, nz, ...
    dir_ref, max_dir_angle, max_dist_to_line, tau, alpha_val, beta, w_dist_line, smooth_weight, max_search)
% 单只蚂蚁随机搜索: 从 from 向 to 前进, 返回原始路径(可能未达 to)
path = from;
cur = from;
[ix, iy, iz] = nearest_xyz(cur, xs, ys, zs);
visited = false(nx, ny, nz);
visited(ix, iy, iz) = true;

for sc = 1:max_search
    if norm(cur - to) <= 0.5, break; end
    % 收集候选(3x3x3 邻域)
    cand = zeros(0, 4);   % [nix, niy, niz, dist_to_line]
    for ddx = -1:1
        for ddy = -1:1
            for ddz = -1:1
                if ddx == 0 && ddy == 0 && ddz == 0, continue; end
                nix = ix + ddx; niy = iy + ddy; niz = iz + ddz;
                if nix < 1 || nix > nx || niy < 1 || niy > ny || niz < 1 || niz > nz, continue; end
                if visited(nix, niy, niz) || obstacle(nix, niy, niz), continue; end
                cn = [xs(nix), ys(niy), zs(niz)];
                dc = (cn - cur) / norm(cn - cur + eps);
                ang = acos(max(min(dot(dc, dir_ref), 1), -1)) * 180 / pi;
                dtoline = norm(cross(cn - from, dir_ref));
                if ang <= max_dir_angle && dtoline <= max_dist_to_line
                    cand(end+1, :) = [nix, niy, niz, dtoline]; %#ok<AGROW>
                end
            end
        end
    end

    if isempty(cand)
        if size(path, 1) > 1
            path = path(1:end-1, :);      % 回溯
            cur = path(end, :);
            [ix, iy, iz] = nearest_xyz(cur, xs, ys, zs);
        else
            break;
        end
        continue;
    end

    % 转移概率
    prob = zeros(size(cand, 1), 1);
    for c = 1:size(cand, 1)
        cix = cand(c,1); ciy = cand(c,2); ciz = cand(c,3); dtoline = cand(c,4);
        cn = [xs(cix), ys(ciy), zs(ciz)];
        tv = tau(cix, ciy, ciz);
        eta = 1 / (norm(cn - to) + eps) * 1.5;
        sf = 1;
        if size(path, 1) >= 2
            v1 = cur - path(end-1, :); v2 = cn - cur;
            a = acos(max(min(dot(v1, v2) / (norm(v1) * norm(v2) + eps), 1), -1)) * 180 / pi;
            sf = max(0, cos(a * pi / 180));
        end
        dp = 1 / (1 + w_dist_line * dtoline);
        prob(c) = max(0, tv^alpha_val * eta^beta * sf^smooth_weight * dp);
    end
    sp = sum(prob);
    if sp < 1e-10, prob = ones(size(prob)) / numel(prob); else, prob = prob / sp; end

    sel = randsample(size(cand, 1), 1, true, prob);
    ix = cand(sel,1); iy = cand(sel,2); iz = cand(sel,3);
    cur = [xs(ix), ys(iy), zs(iz)];
    path(end+1, :) = cur; %#ok<AGROW>
    visited(ix, iy, iz) = true;
end
if size(path, 1) < 2, path = []; end
end

% =========================================================================
function F = compute_F(path, start, goal, model, safe_dist, dir_0, wL, wR, wE, w_dist_line)
if isempty(path) || size(path, 1) < 2, F = inf; return; end
L = sum(sqrt(sum(diff(path).^2, 2)));
R = 0; dlt = 0;
for p = 1:size(path, 1)
    m = min_safe_margin(path(p, :), model, safe_dist);
    R = R + max(0, -m);
    dlt = dlt + norm(cross(path(p, :) - start, dir_0));
end
R = R / size(path, 1);
dlt = dlt / size(path, 1);
E = sum(abs(diff(path(:, 3))));
F = wL * L + wR * R + wE * E + w_dist_line * dlt + 0.3 * norm(path(end, :) - goal);
end

% =========================================================================
function d = min_safe_margin(pos, model, safe_dist)
% 点到所有障碍表面的最小安全裕度(表面距离 - safe_dist), 为负表示侵入安全区
x = pos(1); y = pos(2); z = pos(3);
d = inf;
% 地形
d = min(d, z - terrain_height(model, x, y) - safe_dist);
% 高楼
for b = 1:size(model.buildings, 1)
    d = min(d, box_signed_dist(pos, model.buildings(b, :)) - safe_dist);
end
% 球
if ~isempty(model.ball) && numel(model.ball) == 4
    d = min(d, norm(pos - model.ball(1:3)) - model.ball(4) - safe_dist);
end
% 圆柱
if ~isempty(model.cylinder.pos) && model.cylinder.r > 0
    cr = model.cylinder.r;
    for c = 1:size(model.cylinder.pos, 1)
        d = min(d, cylinder_signed_dist(pos, model.cylinder.pos(c, :), cr) - safe_dist);
    end
end
end

% =========================================================================
function h = terrain_height(model, x, y)
h = 0;
for m = 1:size(model.mountains, 1)
    cx = model.mountains(m,1); cy = model.mountains(m,2);
    A  = model.mountains(m,3); s  = model.mountains(m,4);
    h = max(h, A * exp(-((x - cx)^2 + (y - cy)^2) / (2 * s^2)));
end
end

% =========================================================================
function d = box_signed_dist(pos, b)
% 点到盒(高楼)表面的有符号距离(内部为负)
x0 = b(1); x1 = b(2); y0 = b(3); y1 = b(4); h = b(5);
cx = min(max(pos(1), x0), x1);
cy = min(max(pos(2), y0), y1);
cz = min(max(pos(3), 0),  h);
inside = pos(1) >= x0 && pos(1) <= x1 && pos(2) >= y0 && pos(2) <= y1 && pos(3) >= 0 && pos(3) <= h;
d = norm(pos - [cx, cy, cz]);
if inside, d = -d; end
end

% =========================================================================
function d = cylinder_signed_dist(pos, c, r)
% 点到竖直圆柱(底面 z=0, 高 h=c(3), 半径 r)的有符号距离
cx = c(1); cy = c(2); h = c(3);
dx = norm([pos(1) - cx, pos(2) - cy]) - r;
dy = abs(pos(3) - h / 2) - h / 2;
d = norm([max(dx, 0), max(dy, 0)]) + min(max(dx, dy), 0);
end

% =========================================================================
function path = resample_polyline(poly, n_pts)
% 沿折线按弧长均匀重采样(线性插值), 仅对逐段安全路径使用
if size(poly, 1) < 2
    path = repmat(poly(1, :), n_pts, 1);
    return;
end
seg = sqrt(sum(diff(poly, 1, 1).^2, 2));
cum = [0; cumsum(seg)];
total = cum(end);
if total < 1e-9
    path = repmat(poly(1, :), n_pts, 1);
    return;
end
u = linspace(0, total, n_pts);
path = zeros(n_pts, 3);
for i = 1:n_pts
    j = find(u(i) >= cum(1:end-1) & u(i) <= cum(2:end), 1);
    if isempty(j), j = numel(seg); end
    if seg(j) < 1e-9
        path(i, :) = poly(j, :);
    else
        f = (u(i) - cum(j)) / seg(j);
        path(i, :) = (1 - f) * poly(j, :) + f * poly(j+1, :);
    end
end
path(1, :)   = poly(1, :);
path(end, :) = poly(end, :);
end

% =========================================================================
function path = snap_to_free(path, obstacle, xs, ys, zs)
% 兜底防穿障: 将仍落在障碍栅格内的点吸附到最近自由栅格
nx = numel(xs); ny = numel(ys); nz = numel(zs);
for p = 1:size(path, 1)
    [~, ix] = min(abs(xs - path(p,1)));
    [~, iy] = min(abs(ys - path(p,2)));
    [~, iz] = min(abs(zs - path(p,3)));
    ix = min(max(ix,1), nx); iy = min(max(iy,1), ny); iz = min(max(iz,1), nz);
    if ~obstacle(ix, iy, iz), continue; end
    [fix, fiy, fiz] = nearest_free(obstacle, ix, iy, iz, nx, ny, nz);
    path(p, 1) = xs(fix);
    path(p, 2) = ys(fiy);
    path(p, 3) = zs(fiz);
end
end

function [fix, fiy, fiz] = nearest_free(obstacle, ix, iy, iz, nx, ny, nz)
for rad = 1:max([nx, ny, nz])
    for dx = -rad:rad
        for dy = -rad:rad
            for dz = -rad:rad
                nix = ix + dx; niy = iy + dy; niz = iz + dz;
                if nix < 1 || nix > nx || niy < 1 || niy > ny || niz < 1 || niz > nz, continue; end
                if ~obstacle(nix, niy, niz), fix = nix; fiy = niy; fiz = niz; return; end
            end
        end
    end
end
fix = ix; fiy = iy; fiz = iz;
end

% =========================================================================
function path = repair_path_geo(path, model, safe_dist, obstacle, xs, ys, zs)
% 线段级几何修复：确保重采样/吸附后的路径每条线段都不穿楼（净空 >= safe_dist）。
% 思路与本文算法 solve_this_v2.m 的 repair_path 一致：先修节点，再逐段细分穿障线段，
% 但只用本文件自带的 obstacle_clearance / nearest_free，无额外工具箱依赖。
if size(path, 1) < 2, return; end
nx = numel(xs); ny = numel(ys); nz = numel(zs);

% 1) 节点级：净空不足的节点吸附到最近自由栅格
for i = 1:size(path, 1)
    if obstacle_clearance(path(i, :), model) < safe_dist
        [~, ix] = min(abs(xs - path(i,1)));
        [~, iy] = min(abs(ys - path(i,2)));
        [~, iz] = min(abs(zs - path(i,3)));
        ix = min(max(ix,1), nx); iy = min(max(iy,1), ny); iz = min(max(iz,1), nz);
        [fix, fiy, fiz] = nearest_free(obstacle, ix, iy, iz, nx, ny, nz);
        path(i, :) = [xs(fix), ys(fiy), zs(fiz)];
    end
end

% 2) 线段级：逐段加密检查，穿障处在中点插入吸附后的绕行点
max_len = size(path, 1) * 20;
i = 1; guard = 0;
while i < size(path, 1) && size(path, 1) < max_len && guard < 2000
    guard = guard + 1;
    p1 = path(i, :); p2 = path(i+1, :);
    if seg_min_clearance(p1, p2, model) < safe_dist
        mid = (p1 + p2) / 2;
        [~, ix] = min(abs(xs - mid(1)));
        [~, iy] = min(abs(ys - mid(2)));
        [~, iz] = min(abs(zs - mid(3)));
        ix = min(max(ix,1), nx); iy = min(max(iy,1), ny); iz = min(max(iz,1), nz);
        [fix, fiy, fiz] = nearest_free(obstacle, ix, iy, iz, nx, ny, nz);
        mid = [xs(fix), ys(fiy), zs(fiz)];
        if norm(mid - p1) < 1e-6 || norm(mid - p2) < 1e-6
            i = i + 1;   % 中点与端点重合(退化成无法再细分)，跳过避免死循环
        else
            path = [path(1:i, :); mid; path(i+1:end, :)]; %#ok<AGROW>
        end
    else
        i = i + 1;
    end
end
end

function d = seg_min_clearance(p1, p2, model)
% 线段上最窄净空（按段长加密采样，至少 9 点）
n = max(9, ceil(norm(p2 - p1) / 0.5));
d = inf;
for k = 0:n
    pos = p1 + (p2 - p1) * (k / n);
    d = min(d, obstacle_clearance(pos, model));
end
end

% =========================================================================
function draw_map_and_path(model, path, map_type)
tick_font_size  = 20;
label_font_size = 20;
title_font_size = 16;

figure('Name', sprintf('ACOSRAR 环境与路径-%s', map_type), 'Position', [100, 100, 850, 800], 'Color', 'white');
ax = gca;
hold(ax, 'on');

if strcmp(model.type, 'city')
    % ---- 城市: 平底地面 + 矩形高楼 ----
    patch(ax, [0 200 200 0], [0 0 200 200], [1e-3 1e-3 1e-3 1e-3], ...
          [0.82 0.82 0.78], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    for i = 1:size(model.buildings, 1)
        draw_building_box(ax, model.buildings(i, :));
    end
else
    % ---- 山体/丘陵/旧地图 ----
    map_x = 0:0.5:200;
    map_y = 0:0.5:200;
    [X, Y] = meshgrid(map_x, map_y);
    for k = 1:size(model.mountains, 1)
        cx = model.mountains(k, 1); cy = model.mountains(k, 2);
        A  = model.mountains(k, 3); s  = model.mountains(k, 4);
        z = A * exp(-((X - cx).^2 + (Y - cy).^2) / (2 * s^2));
        z(z < 0) = 0;
        surf(ax, X, Y, z, 'FaceColor', 'interp', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    end
    if ~isempty(model.ball) && numel(model.ball) == 4
        [Xb, Yb, Zb] = sphere(50);
        bc = model.ball(1:3); br = model.ball(4);
        bx = bc(1) + Xb * br; by = bc(2) + Yb * br; bz = bc(3) + Zb * br; bz(bz < 0) = 0;
        surf(ax, bx, by, bz, 'FaceColor', [1 0 1], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    end
    if ~isempty(model.cylinder.pos) && model.cylinder.r > 0
        cyl_r = model.cylinder.r;
        [Xc, Yc, Zc] = cylinder(cyl_r, 50);
        theta = linspace(0, 2 * pi, 50);
        for i = 1:size(model.cylinder.pos, 1)
            x0 = model.cylinder.pos(i, 1); y0 = model.cylinder.pos(i, 2); h = model.cylinder.pos(i, 3);
            surf(ax, x0 + Xc, y0 + Yc, h * Zc, 'FaceColor', [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
            fill3(ax, x0 + cyl_r * cos(theta), y0 + cyl_r * sin(theta), repmat(h, size(theta)), ...
                [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
            fill3(ax, x0 + cyl_r * cos(theta), y0 + cyl_r * sin(theta), zeros(size(theta)), ...
                [1 1 0], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
        end
    end
end

% ---- 唯一最优路径(红色) ----
plot3(ax, path(:, 1), path(:, 2), path(:, 3), '-', 'Color', [1 0 0], 'LineWidth', 3);

% ---- 起点(绿球) / 终点(蓝球) ----
draw_marker_sphere(ax, model.start, 5, [0 0.8 0]);
draw_marker_sphere(ax, model.goal,  5, [0 0 1]);

% ---- 坐标轴设置 ----
xlabel(ax, 'X/m', 'FontSize', label_font_size);
ylabel(ax, 'Y/m', 'FontSize', label_font_size);
zlabel(ax, 'Z/m', 'FontSize', label_font_size);
title(ax, sprintf('ACOSRAR 三维路径规划（%s）', map_name_of(map_type)), 'FontSize', title_font_size);
xlim(ax, [0 200]); ylim(ax, [0 200]); zlim(ax, [0 200]);
xticks(ax, 0:20:200); yticks(ax, 0:20:200); zticks(ax, 0:20:200);
daspect(ax, [1 1 1]);
set(ax, 'FontSize', tick_font_size, 'Color', 'white');
grid(ax, 'on');
view(ax, 45, 35);
camproj(ax, 'orthographic');
hold(ax, 'off');
end

% =========================================================================
function draw_convergence(iter_best, global_hist, max_iter)
figure('Position', [100, 100, 800, 400]);
iter_best(~isfinite(iter_best)) = NaN;
global_hist(~isfinite(global_hist)) = NaN;
plot(1:max_iter, iter_best, 'b-o', 'LineWidth', 1, 'MarkerSize', 3, 'DisplayName', '每代最优');
hold on; grid on;
plot(1:max_iter, global_hist, 'r-*', 'LineWidth', 1.5, 'MarkerSize', 4, 'DisplayName', '全局最优');
xlabel('迭代次数', 'FontSize', 11);
ylabel('目标函数值 F', 'FontSize', 11);
title('ACOSRAR 收敛曲线', 'FontSize', 12);
legend('Location', 'best', 'FontSize', 10);
end

% =========================================================================
function draw_marker_sphere(ax, center, radius, color)
[sx, sy, sz] = sphere(30);
surf(ax, center(1) + sx * radius, center(2) + sy * radius, center(3) + sz * radius, ...
    'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
end

function draw_building_box(ax, b)
x0 = b(1); x1 = b(2); y0 = b(3); y1 = b(4); h = b(5);
t = min(max(h/150, 0), 1);
col = [0.4 - 0.15*t, 0.48 - 0.10*t, 0.72 + 0.10*t];
verts_x = [x0, x1, x1, x0, x0, x1, x1, x0];
verts_y = [y0, y0, y1, y1, y0, y0, y1, y1];
verts_z = [0, 0, 0, 0, h, h, h, h];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch(ax, 'Vertices', [verts_x(:), verts_y(:), verts_z(:)], 'Faces', faces, ...
      'FaceColor', col, 'EdgeColor', 'k', 'LineWidth', 0.5, 'FaceAlpha', 1.0);
patch(ax, 'XData', [x0 x1 x1 x0], 'YData', [y0 y0 y1 y1], 'ZData', [h h h h], ...
      'FaceColor', col .* 1.2, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
end

function f6 = compute_six_objectives(P, model, safe_dist)
% COMPUTE_SIX_OBJECTIVES  对一条平滑路径计算 6 个目标值
%   [路径长度, 能耗, 不平滑度, 飞行时间, 最小安全距离, 威胁代价]
% 复用本文件已有的 min_safe_margin / terrain_height / cylinder_signed_dist 等
V_DRONE = 10;
nP = size(P, 1);
seg_len = zeros(nP - 1, 1);
height_change = 0;
for i = 2:nP
    d = P(i, :) - P(i - 1, :);
    seg_len(i - 1) = norm(d);
    height_change = height_change + abs(d(3));
end
L = sum(seg_len);

D_all = zeros(nP, 1);
threat = 0; hard = 0; HARD_PEN = 1e5;
for i = 1:nP
    D_all(i) = obstacle_clearance(P(i, :), model);
    if D_all(i) < safe_dist
        threat = threat + (safe_dist - D_all(i));
        hard = hard + 1;
    end
end
threat = threat + hard * HARD_PEN;
d_safe_min = max(min(D_all), 0);

turn_angle_sum = 0; accel_sum = 0;
cos_sum = 0; cnt = 0;
for i = 2:nP - 1
    v1 = P(i, :) - P(i - 1, :);
    v2 = P(i + 1, :) - P(i, :);
    n1 = norm(v1); n2 = norm(v2);
    if n1 > 1e-6 && n2 > 1e-6
        cos_a = max(min(dot(v1, v2) / (n1 * n2), 1), -1);
        ang = acos(cos_a);
        turn_angle_sum = turn_angle_sum + ang;
        accel_sum = accel_sum + 2 * sin(ang / 2);
        cos_sum = cos_sum + cos_a;
        cnt = cnt + 1;
    end
end
energy = L * 0.1 + height_change * 0.3 + 0.2 * turn_angle_sum + 0.15 * accel_sum;
smooth = cos_sum / (cnt + 1e-9);
flight_time = L / V_DRONE;

f6 = [L, energy, 1 - smooth, flight_time, d_safe_min, threat];
end

% =========================================================================
function d = obstacle_clearance(pos, model)
% 点到所有障碍物表面的最短净空距离(正值=净空, 负值=已侵入障碍)
d = inf;
d = min(d, pos(3) - terrain_height(model, pos(1), pos(2)));
for b = 1:size(model.buildings, 1)
    d = min(d, box_signed_dist(pos, model.buildings(b, :)));
end
if ~isempty(model.ball) && numel(model.ball) == 4
    d = min(d, norm(pos - model.ball(1:3)) - model.ball(4));
end
if ~isempty(model.cylinder.pos) && model.cylinder.r > 0
    cr = model.cylinder.r;
    for c = 1:size(model.cylinder.pos, 1)
        d = min(d, cylinder_signed_dist(pos, model.cylinder.pos(c, :), cr));
    end
end
end

function str = map_name_of(map_type)
switch map_type
    case 'city',     str = '城市地图';
    case 'hill',     str = '丘陵地图';
    case 'obstacle', str = '障碍物地图';
    otherwise,       str = '山体障碍地图';
end
end