function smooth_path = nmopos_plan_world(start_pos, goal_pos, map_type)
% NMOPOS_PLAN_WORLD  从 NMOPOS/main.m 抽出的独立可调用封装(多目标 PSO)。
% 输入：start_pos / goal_pos = 1x3; map_type - 'obstacle'/'city'/'hill'; 输出 smooth_path = Nx3 世界坐标路径。
% 说明：环境确定性，随机性来自 PSO 采样，批处理换种子可复现。
    if nargin < 3 || isempty(map_type), map_type = 'obstacle'; end
    map_range = [0, 200];
    safe_dist = 2.5;
    drone_speed = 20;
    weights = [0.4, 0.35, 0.15, 0.15, 0.05];

    num_ctrl_pts = 12;
    num_interp_pts = 150;
    n_particles = 40;
    max_iter = 150;
    archive_size = 50;
    inertia_w = 0.5; c1 = 1.5; c2 = 1.5;
    mutate_prob = 0.1; mutate_strength = 5.0;
    lb = [0, 0, 10]; ub = [200, 200, 180];
    dim = (num_ctrl_pts - 2) * 3;

    [terrain_func, sphere_params, cylinder_params, buildings] = generate_environment(map_type);

    pos = zeros(n_particles, dim); vel = zeros(n_particles, dim);
    for i = 1:n_particles
        t = linspace(0, 1, num_ctrl_pts)';
        base_x = start_pos(1) + (goal_pos(1) - start_pos(1)) * t;
        base_y = start_pos(2) + (goal_pos(2) - start_pos(2)) * t;
        base_z = start_pos(3) + (goal_pos(3) - start_pos(3)) * t;
        ctrl_pts = [base_x(2:end-1), base_y(2:end-1), base_z(2:end-1)];
        ctrl_pts = ctrl_pts + randn(size(ctrl_pts)) * 8;
        ctrl_pts = max(lb, min(ub, ctrl_pts));
        pos(i, :) = reshape(ctrl_pts, 1, dim);
        vel(i, :) = randn(1, dim) * 3;
    end
    pbest_pos = pos; pbest_obj = inf(n_particles, 5);
    archive = struct('pos', [], 'obj', []);

    eval_f = @(v) evaluate_path_with_penalty(v, start_pos, goal_pos, num_ctrl_pts, num_interp_pts, ...
        terrain_func, sphere_params, cylinder_params, buildings, safe_dist, drone_speed);
    to_ctrl_pts = @(vars) [start_pos; reshape(vars, [], 3); goal_pos];

    for iter = 1:max_iter
        obj_all = zeros(n_particles, 5);
        for i = 1:n_particles
            obj_all(i, :) = eval_f(pos(i, :));
        end
        for i = 1:n_particles
            if any(pbest_obj(i,:) == inf) || dominates(obj_all(i,:), pbest_obj(i,:))
                pbest_pos(i, :) = pos(i, :); pbest_obj(i, :) = obj_all(i, :);
            end
        end
        current_archive = [archive.pos; pos]; current_obj = [archive.obj; obj_all];
        [~, non_dominated] = pareto_filter(current_archive, current_obj);
        archive.pos = current_archive(non_dominated, :); archive.obj = current_obj(non_dominated, :);
        if size(archive.pos, 1) > archive_size
            archive = truncate_archive(archive, archive_size);
        end
        if ~isempty(archive.obj)
            crowding = crowding_distance(archive.obj);
            if all(isinf(crowding)) || any(isnan(crowding))
                prob = ones(1, size(archive.obj, 1)) / size(archive.obj, 1);
            else
                crowding(isinf(crowding)) = max(crowding(isfinite(crowding)));
                prob = crowding / (sum(crowding) + eps);
            end
            for i = 1:n_particles
                idx = roulette_wheel(prob);
                if isempty(idx) || idx > size(archive.pos, 1), idx = randi(size(archive.pos, 1)); end
                gbest = archive.pos(idx, :);
                r1 = rand(1, dim); r2 = rand(1, dim);
                vel(i, :) = inertia_w * vel(i, :) + c1 * r1 .* (pbest_pos(i, :) - pos(i, :)) + c2 * r2 .* (gbest - pos(i, :));
                pos(i, :) = pos(i, :) + vel(i, :);
                for d = 1:dim
                    dim_idx = mod(d - 1, 3) + 1;
                    pos(i, d) = max(lb(dim_idx), min(ub(dim_idx), pos(i, d)));
                end
                if rand() < mutate_prob
                    pos(i, :) = pos(i, :) + randn(1, dim) * mutate_strength;
                    for d = 1:dim
                        dim_idx = mod(d - 1, 3) + 1;
                        pos(i, d) = max(lb(dim_idx), min(ub(dim_idx), pos(i, d)));
                    end
                end
            end
        end
    end

    if isempty(archive.obj)
        smooth_path = []; return;
    end
    weighted_sum = archive.obj * weights(:);
    [~, final_idx] = min(weighted_sum);
    optimal_ctrl_pts = to_ctrl_pts(archive.pos(final_idx, :));
    optimal_path = bspline_path(optimal_ctrl_pts, num_interp_pts);
    smooth_path = smooth_path_points(optimal_path, 0.5);
    smooth_path(1, :) = start_pos; smooth_path(end, :) = goal_pos;
    smooth_path = repair_city_path(smooth_path, buildings, safe_dist);
end

% ---------------- 以下全部为 NMOPOS/main.m 的辅助子函数 ----------------
function obj = evaluate_path_with_penalty(ctrl_vars, start_pos, goal_pos, num_ctrl_pts, num_interp_pts, terrain_func, sphere_params, cylinder_params, buildings, safe_dist, drone_speed)
    ctrl_pts = [start_pos; reshape(ctrl_vars, [], 3); goal_pos];
    path_pts = bspline_path(ctrl_pts, num_interp_pts);
    [len, min_safe, eng, sm, tm] = compute_path_metrics(path_pts, terrain_func, sphere_params, cylinder_params, buildings, safe_dist, drone_speed);
    if min_safe < safe_dist
        penalty = 1e6;
        len = len + penalty; eng = eng + penalty; sm = sm - penalty; tm = tm + penalty;
    end
    obj = [len, min_safe, eng, sm, tm];
end

function [length_val, min_safe, energy, smoothness, time_val] = compute_path_metrics(path_pts, terrain_func, sphere_params, cylinder_params, buildings, safe_dist, drone_speed)
    diffs = diff(path_pts, 1);
    seg_lengths = sqrt(sum(diffs.^2, 2));
    length_val = sum(seg_lengths);
    min_safe = inf;
    npts = size(path_pts, 1);
    for i = 1:npts
        for s = 0:5
            if s == 0
                pt = path_pts(i, :);
            else
                if i == npts, continue; end
                pt = path_pts(i, :) + (path_pts(i+1, :) - path_pts(i, :)) * (s/6);
            end
            dist_terrain = pt(3) - terrain_func(pt(1), pt(2));
            dist_sphere = norm(pt - sphere_params.center) - sphere_params.radius;
            dist_cyl = inf;
            for j = 1:size(cylinder_params.centers, 1)
                center = cylinder_params.centers(j, 1:2); h = cylinder_params.centers(j, 3);
                dx = pt(1) - center(1); dy = pt(2) - center(2);
                dist_horiz = sqrt(dx^2 + dy^2) - cylinder_params.radius;
                if pt(3) >= 0 && pt(3) <= h, dist_vert = 0;
                else, dist_vert = min(abs(pt(3)-0), abs(pt(3)-h)); end
                dist_cyl = min(dist_cyl, max(dist_horiz, dist_vert));
            end
            dist_bld = inf;
            for j = 1:size(buildings, 1)
                dist_bld = min(dist_bld, dist_to_box(pt, buildings(j, :)));
            end
            min_safe = min(min_safe, min([dist_terrain, dist_sphere, dist_cyl, dist_bld]));
        end
    end
    if isinf(min_safe), min_safe = 1e6; end
    energy = length_val;
    n = size(path_pts, 1); cos_angles = zeros(n - 2, 1);
    for i = 2:n - 1
        v1 = path_pts(i, :) - path_pts(i-1, :); v2 = path_pts(i+1, :) - path_pts(i, :);
        if norm(v1) > eps && norm(v2) > eps, cos_angles(i-1) = dot(v1, v2) / (norm(v1) * norm(v2));
        else, cos_angles(i-1) = 1; end
    end
    smoothness = mean(cos_angles);
    time_val = length_val / drone_speed;
end

function [terrain_func, sphere_params, cylinder_params, buildings] = generate_environment(map_type)
    switch map_type
        case 'city'
            terrain_func = @(x, y) zeros(size(x));
            sphere_params.center = [0 0 0]; sphere_params.radius = -inf;
            cylinder_params.centers = zeros(0,3); cylinder_params.radius = 5; cylinder_params.height = 100;
            buildings = build_city_buildings();
        case 'hill'
            peaks = struct('A', {90,90,120,120,50,100,100}, 'x0', {40,140,140,90,40,180,90}, ...
                           'y0', {140,40,140,90,40,90,180}, 'sigma', {30,30,30,30,30,30,30});
            terrain_func = @(x, y) compute_terrain(x, y, peaks);
            sphere_params.center = [0 0 0]; sphere_params.radius = -inf;
            cylinder_params.centers = zeros(0,3); cylinder_params.radius = 5; cylinder_params.height = 100;
            buildings = [];
        otherwise  % obstacle
            peaks = struct('A', {90, 90, 120}, 'x0', {40, 140, 140}, 'y0', {140, 40, 140}, 'sigma', {30, 30, 30});
            terrain_func = @(x, y) compute_terrain(x, y, peaks);
            sphere_params.center = [100, 100, 0]; sphere_params.radius = 50;
            cylinder_centers = [40,70,100; 70,40,100; 50,50,100; 90,160,100; ...
                160,80,100; 80,180,100; 100,180,100; 180,60,100; 180,100,100];
            cylinder_params.centers = cylinder_centers; cylinder_params.radius = 5; cylinder_params.height = 100;
            buildings = [];
    end
end

function B = build_city_buildings()
    nCol = 6; nRow = 6; margin = 14; gap = 6;
    s = rng;  rng('default'); rng(26);
    xc = linspace(margin, 200-margin, nCol+1);
    yc = linspace(margin, 200-margin, nRow+1);
    cellWx = xc(2)-xc(1); cellWy = yc(2)-yc(1);
    bldWx = cellWx-gap; bldWy = cellWy-gap;
    roadRow = 3; roadCol = 3;
    plaza = rand(nRow, nCol) < 0.15;
    B = [];
    for r = 1:nRow
        for c = 1:nCol
            if r == roadRow || c == roadCol || plaza(r,c), continue; end
            cx = (xc(c)+xc(c+1))/2; cy = (yc(r)+yc(r+1))/2;
            x0 = cx-bldWx/2; x1 = cx+bldWx/2;
            y0 = cy-bldWy/2; y1 = cy+bldWy/2;
            h = 30 + 70*rand;
            if rand < 0.12; h = 120 + 30*rand; end
            B(end+1,:) = [x0 x1 y0 y1 h]; %#ok<AGROW>
        end
    end
rng(s);   % 恢复随机状态，避免重置调用方种子
end

function dd = dist_to_box(pt, b)
    x = pt(1); y = pt(2); z = pt(3);
    x0 = b(1); x1 = b(2); y0 = b(3); y1 = b(4); h = b(5);
    inside = (x > x0 && x < x1 && y > y0 && y < y1 && z > 0 && z < h);
    if inside
        dd = -min([x - x0, x1 - x, y - y0, y1 - y, z, h - z]);
        return;
    end
    dx = max([x0 - x, x - x1, 0]);
    dy = max([y0 - y, y - y1, 0]);
    if z < 0, dz = -z; elseif z > h, dz = z - h; else, dz = 0; end
    dd = sqrt(dx^2 + dy^2 + dz^2);
end

function z = compute_terrain(x, y, peaks)
    z = zeros(size(x));
    for i = 1:length(peaks)
        z = z + peaks(i).A * exp(-((x - peaks(i).x0).^2 + (y - peaks(i).y0).^2) / (2 * peaks(i).sigma^2));
    end
    z(z < 0) = 0;
end

function path_pts = bspline_path(ctrl_pts, num_pts)
    n = size(ctrl_pts, 1);
    if n < 4
        t = linspace(0, 1, num_pts)';
        path_pts = interp1(0:1, ctrl_pts, t, 'linear'); return;
    end
    t = linspace(0, 1, num_pts); path_pts = zeros(num_pts, 3);
    for i = 1:num_pts
        u = t(i); seg = floor(u * (n - 3)) + 1; seg = max(1, min(seg, n - 3));
        u_local = max(0, min(1, (u - (seg-1)/(n-3)) * (n-3)));
        P0 = ctrl_pts(seg, :); P1 = ctrl_pts(seg+1, :); P2 = ctrl_pts(seg+2, :); P3 = ctrl_pts(seg+3, :);
        b0 = (1-u_local)^3 / 6; b1 = (3*u_local^3 - 6*u_local^2 + 4) / 6;
        b2 = (-3*u_local^3 + 3*u_local^2 + 3*u_local + 1) / 6; b3 = u_local^3 / 6;
        path_pts(i, :) = b0*P0 + b1*P1 + b2*P2 + b3*P3;
    end
    path_pts(1, :) = ctrl_pts(1, :); path_pts(end, :) = ctrl_pts(end, :);
end

function path_smoothed = smooth_path_points(path, alpha)
    if size(path, 1) < 5, path_smoothed = path; return; end
    path_smoothed = path;
    for iter = 1:3
        for i = 2:size(path, 1) - 1
            path_smoothed(i, :) = (1-alpha) * path_smoothed(i, :) + alpha * (path_smoothed(i-1, :) + path_smoothed(i+1, :)) / 2;
        end
    end
    path_smoothed(1, :) = path(1, :); path_smoothed(end, :) = path(end, :);
end

function is_dom = dominates(obj1, obj2)
    f1 = obj1; f2 = obj2; f1(2) = -obj1(2); f2(2) = -obj2(2); f1(4) = -obj1(4); f2(4) = -obj2(4);
    is_dom = all(f1 <= f2) && any(f1 < f2);
end

function [tmp_dom, nondominated] = pareto_filter(pop, objs)
    tmp_dom = [];                       % 首个输出仅占位用(调用端丢弃), 必须显式赋值
    n = size(pop, 1); nondominated = true(n, 1);
    for i = 1:n
        for j = 1:n
            if i ~= j && dominates(objs(j, :), objs(i, :)), nondominated(i) = false; break; end
        end
    end
end

function archive = truncate_archive(archive, max_size)
    if size(archive.pos, 1) <= max_size, return; end
    crowd = crowding_distance(archive.obj);
    [~, idx] = sort(crowd, 'descend');
    keep_idx = idx(1:max_size);
    archive.pos = archive.pos(keep_idx, :); archive.obj = archive.obj(keep_idx, :);
end

function crowd = crowding_distance(objs)
    n = size(objs, 1);
    if n <= 2, crowd = ones(n, 1); return; end
    m = size(objs, 2); crowd = zeros(n, 1);
    for i = 1:m
        [sorted, idx] = sort(objs(:, i));
        crowd(idx(1)) = inf; crowd(idx(end)) = inf;
        for j = 2:n - 1, crowd(idx(j)) = crowd(idx(j)) + (sorted(j+1) - sorted(j-1)); end
    end
end

function idx = roulette_wheel(prob)
    cum_prob = cumsum(prob); r = rand();
    idx = find(cum_prob >= r, 1);
    if isempty(idx), idx = randi(length(prob)); end
end

% =========================================================================
function path = repair_city_path(path, buildings, safe_dist)
% 几何修复：把 B 样条/拉普拉斯平滑后穿楼/贴楼的点推出到净空 >= safe_dist（节点级 + 线段级细分）
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