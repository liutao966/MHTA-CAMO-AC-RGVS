function path = octree_plan_world(map_type, buildings)
% OCTREE_PLAN_WORLD  从 run_octree.m 抽出的独立可调用封装(Octree重叠检测 + 3D A*)。
% 输出：path = Nx3 世界坐标 FLC碰撞路径；失败返回空 []。
% 说明：确定性算法(环境/栅格/A* 均无随机性)，批处理只需调用一次。
% map_type - 'obstacle' / 'city' / 'hill'（缺省 legacy，与 run_octree.m 一致）
% buildings - 可选，外部传入的高楼 Nx5（与 eval_metrics 用同一套楼，避免各自生成不一致）

    if nargin < 1 || isempty(map_type), map_type = 'legacy'; end

    sigma = 4;         % 安全裕度
    res   = 4;         % A* 导航栅格分辨率（原 12.5 太粗，22m 高楼只占约 2 格，穿楼）

    model = octree_build_environment(map_type);
    if nargin >= 2
        model.buildings = buildings;   % 用调用方的楼（obstacle/hill 下为 []，city 下与 eval_metrics 同一套）
    end
    nocc  = grid_occupancy(model, res, sigma);
    path  = a_star_grid(nocc, res, model.xyz_lo, model.start, model.goal);
    if ~isempty(path) && strcmp(model.type, 'city')
        path = repair_city_path(path, model.buildings, model.safe_dist);
    end
end

% ===== 以下 a_star_grid / grid_occupancy / idx_of / euclid / smooth_path 由 run_octree.m 局部函数抽取 =====
function occ = grid_occupancy(model, res, sigma)
    lo = model.xyz_lo; hi = model.xyz_hi;
    nx = round((hi(1) - lo(1)) / res);
    ny = round((hi(2) - lo(2)) / res);
    nz = round((hi(3) - lo(3)) / res);
    occ = false(nx, ny, nz);
    for i = 1:nx
        cx = lo(1) + (i - 0.5) * res;
        for j = 1:ny
            cy = lo(2) + (j - 0.5) * res;
            for k = 1:nz
                cz = lo(3) + (k - 0.5) * res;
                if cube_occ([cx cy cz], res / 2, model, sigma)
                    occ(i, j, k) = true;
                end
            end
        end
    end
end

function path = a_star_grid(occ, res, lo, start, goal)
    dims = size(occ);
    si = idx_of(lo, res, start, dims); gi = idx_of(lo, res, goal, dims);
    goalIdx = gi;

    if occ(si(1), si(2), si(3)) || occ(gi(1), gi(2), gi(3))
        path = []; return;
    end

    [dx, dy, dz] = ndgrid(-1:1, -1:1, -1:1);
    nn = [dx(:) dy(:) dz(:)]; nn(14, :) = [];
    g = inf(numel(occ), 1); f = inf(numel(occ), 1); par = zeros(numel(occ), 1);
    ci = @(a, b, c) sub2ind(dims, a, b, c);
    sid = ci(si(1), si(2), si(3)); gid = ci(gi(1), gi(2), gi(3));
    g(sid) = 0; f(sid) = euclid(si, gi);
    open = false(numel(occ), 1); open(sid) = true;
    closed = false(numel(occ), 1);
    found = false;

    while ~found
        op = find(open);
        if isempty(op), break; end
        [~, m] = min(f(op)); u = op(m);
        open(u) = false; closed(u) = true;
        [ux, uy, uz] = ind2sub(dims, u);
        for q = 1:size(nn, 1)
            nx = ux + nn(q, 1); ny = uy + nn(q, 2); nz = uz + nn(q, 3);
            if nx < 1 || nx > dims(1) || ny < 1 || ny > dims(2) || nz < 1 || nz > dims(3), continue; end
            if occ(nx, ny, nz), continue; end
            v = ci(nx, ny, nz);
            if closed(v), continue; end
            step = euclid([ux uy uz], [nx ny nz]);
            if g(u) + step < g(v)
                g(v) = g(u) + step; par(v) = u;
                f(v) = g(v) + euclid([nx ny nz], goalIdx);
                open(v) = true;
            end
        end
        if closed(gid), found = true; end
    end

    if ~found, path = []; return; end

    seq = []; c = gid;
    while c ~= 0 && c ~= sid
        seq = [c seq]; c = par(c); %#ok<AGROW>
    end
    seq = [sid, seq, gid];
    verts = zeros(numel(seq), 3);
    for q = 1:numel(seq)
        [a, b, c3] = ind2sub(dims, seq(q));
        verts(q, :) = lo + [a - 0.5, b - 0.5, c3 - 0.5] .* res;
    end
    path = smooth_path([start; verts; goal], 300);
end

function id = idx_of(lo, res, p, dims)
    id = max(1, min(dims, floor((p - lo) / res) + 1));
end

function d = euclid(a, b)
    d = norm(double(b) - double(a));
end

function sp = smooth_path(path, n)
    keep = [true; sqrt(sum(diff(path, 1, 1).^2, 2)) > 1e-9];
    path = path(keep, :);
    m = size(path, 1);
    if m < 3, sp = path; return; end
    seg = sqrt(sum(diff(path, 1, 1).^2, 2));
    t = [0; cumsum(seg)];
    tt = linspace(t(1), t(end), n);
    sp = zeros(n, 3);
    for d = 1:3
        sp(:, d) = interp1(t, path(:, d), tt, 'linear');   % 线性插值不穿楼（pchip 三次会在楼角过冲）
    end
end

% =========================================================================
function path = repair_city_path(path, buildings, safe_dist)
% 几何修复：把 pchip 平滑后穿楼/贴楼的点推出到净空 >= safe_dist（节点级 + 线段级细分）
if isempty(buildings) || size(path, 1) < 2, return; end
for i = 1:size(path, 1)
    path(i, :) = city_push(path(i, :), buildings, safe_dist);
end
i = 1; guard = 0;
while i < size(path, 1) && guard < 5000
    guard = guard + 1;
    if city_seg_clearance(path(i, :), path(i+1, :), buildings) < safe_dist
        mid = city_push((path(i, :) + path(i+1, :)) / 2, buildings, safe_dist);
        if norm(mid - path(i, :)) < 1e-9 || norm(mid - path(i+1, :)) < 1e-9
            i = i + 1;
        else
            path = [path(1:i, :); mid; path(i+1:end, :)]; %#ok<AGROW>
        end
    else
        i = i + 1;
    end
end
end

function d = city_seg_clearance(p1, p2, buildings)
n = max(9, ceil(norm(p2 - p1) / 0.5));
d = inf;
for k = 0:n
    d = min(d, city_clearance(p1 + (p2 - p1) * (k / n), buildings));
end
end

function d = city_clearance(p, buildings)
d = inf;
for b = 1:size(buildings, 1)
    d = min(d, city_box_sd(p, buildings(b, :)));
end
end

function dd = city_box_sd(p, b)
x = p(1); y = p(2); z = p(3);
x0 = b(1); x1 = b(2); y0 = b(3); y1 = b(4); h = b(5);
if x > x0 && x < x1 && y > y0 && y < y1 && z > 0 && z < h
    dd = -min([x - x0, x1 - x, y - y0, y1 - y, z, h - z]);
    return;
end
dx = max([x0 - x, x - x1, 0]); dy = max([y0 - y, y - y1, 0]);
if z < 0, dz = -z; elseif z > h, dz = z - h; else, dz = 0; end
dd = sqrt(dx^2 + dy^2 + dz^2);
end

function p = city_push(p, buildings, safe_dist)
for b = 1:size(buildings, 1)
    bb = buildings(b, :);
    x0 = bb(1); x1 = bb(2); y0 = bb(3); y1 = bb(4); h = bb(5);
    x = p(1); y = p(2); z = p(3);
    if x > x0 && x < x1 && y > y0 && y < y1 && z > 0 && z < h
        pen = [x - x0, x1 - x, y - y0, y1 - y, h - z]; [~, mi] = min(pen);
        switch mi
            case 1, p(1) = x0 - safe_dist;
            case 2, p(1) = x1 + safe_dist;
            case 3, p(2) = y0 - safe_dist;
            case 4, p(2) = y1 + safe_dist;
            case 5, p(3) = h + safe_dist;
        end
        continue;
    end
    dx = max([x0 - x, x - x1, 0]); dy = max([y0 - y, y - y1, 0]);
    if z < 0, dz = -z; elseif z > h, dz = z - h; else, dz = 0; end
    d = sqrt(dx^2 + dy^2 + dz^2);
    if d >= safe_dist, continue; end
    if dx >= dy && dx >= dz
        if x < x0, p(1) = x0 - safe_dist; else, p(1) = x1 + safe_dist; end
    elseif dy >= dx && dy >= dz
        if y < y0, p(2) = y0 - safe_dist; else, p(2) = y1 + safe_dist; end
    else
        if z < 0, p(3) = -safe_dist; else, p(3) = h + safe_dist; end
    end
end
end