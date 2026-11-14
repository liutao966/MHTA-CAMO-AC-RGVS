function path = rrtstar_plan_world(start_point, goal_point, params, map_type)
% RRTSTAR_PLAN_WORLD  从 RRT_star/main.m 抽出的可独立调用的 RRT* 全局规划。
% 输入：start_point / goal_point: 1x3；params: 算法与飞行参数结构体；map_type - 'obstacle'/'city'/'hill'。
% 输出：path = Nx3 世界坐标最优路径；未找到路径时返回空 []。
% 说明：环境确定性构建，用 persistent 按 map_type 缓存一次；批处理换种子时只重采样 RRT*。

    if nargin < 4 || isempty(map_type), map_type = 'obstacle'; end

    persistent PX PY PZ_T pNofly pCyl pBuildings cached_type

    % --------------------- 环境一次性构建（与共享模型一致） ---------------------
    if isempty(PX) || ~strcmp(cached_type, map_type)
        map_x = 0:0.1:200; map_y = 0:0.1:200;
        [PX, PY] = meshgrid(map_x, map_y);
        pBuildings = [];

        switch map_type
            case 'city'
                PZ_T = zeros(size(PX));
                pNofly = struct('r', 0, 'cx', 0, 'cy', 0, 'cz', 0);
                pCyl = struct('r', 5, 'positions', zeros(0,3));
                pBuildings = build_city_buildings();
            case 'hill'
                z1 = 90*exp(-((PX-40).^2+(PY-140).^2)/(2*30^2));  z1(z1<0)=0;
                z2 = 90*exp(-((PX-140).^2+(PY-40).^2)/(2*30^2));  z2(z2<0)=0;
                z3 = 120*exp(-((PX-140).^2+(PY-140).^2)/(2*30^2)); z3(z3<0)=0;
                z4 = 120*exp(-((PX-90).^2+(PY-90).^2)/(2*30^2));  z4(z4<0)=0;
                z5 = 50*exp(-((PX-40).^2+(PY-40).^2)/(2*30^2));   z5(z5<0)=0;
                z6 = 100*exp(-((PX-180).^2+(PY-90).^2)/(2*30^2)); z6(z6<0)=0;
                z7 = 100*exp(-((PX-90).^2+(PY-180).^2)/(2*30^2)); z7(z7<0)=0;
                PZ_T = max(cat(3, z1, z2, z3, z4, z5, z6, z7), [], 3);
                pNofly = struct('r', 0, 'cx', 0, 'cy', 0, 'cz', 0);
                pCyl = struct('r', 5, 'positions', zeros(0,3));
            otherwise  % obstacle
                z1 = 90*exp(-((PX-40).^2+(PY-140).^2)/(2*30*30));  z1(z1<0)=0;
                z2 = 90*exp(-((PX-140).^2+(PY-40).^2)/(2*30*30));  z2(z2<0)=0;
                z3 = 120*exp(-((PX-140).^2+(PY-140).^2)/(2*30*30)); z3(z3<0)=0;
                PZ_T = max(cat(3, z1, z2, z3), [], 3);
                pNofly = struct('r', 50, 'cx', 100, 'cy', 100, 'cz', 0);
                pCyl = struct('r', 5, 'positions', ...
                    [40,70,100; 70,40,100; 50,50,100; 90,160,100; 160,80,100; ...
                     80,180,100; 100,180,100; 180,60,100; 180,100,100]);
        end
        cached_type = map_type;
    end

    % --------------------- RRT* 主体（与 main.m 一致） ---------------------
    tree.node = start_point; tree.parent = 0; tree.cost = 0;
    node_count = 1; goal_reached = false; goal_node_idx = -1;

    for iter = 1:params.max_iter
        if rand() < params.goal_sample_rate
            sample_point = goal_point;
        else
            sample_point = [params.x_range(1) + rand()*diff(params.x_range), ...
                            params.y_range(1) + rand()*diff(params.y_range), ...
                            params.z_range(1) + rand()*diff(params.z_range)];
        end

        node_array = reshape([tree.node], 3, [])';
        dist_array = vecnorm(node_array - sample_point, 2, 2);
        [~, nearest_idx] = min(dist_array);
        nearest_node = tree(nearest_idx).node;

        direction = (sample_point - nearest_node);
        direction = direction / norm(direction);
        new_node = nearest_node + direction * params.step_size;

        if check_segment_collision(nearest_node, new_node, params, PX, PY, PZ_T, pNofly, pCyl, pBuildings)
            continue;
        end

        new_node_array = reshape([tree.node], 3, [])';
        dist_to_new = vecnorm(new_node_array - new_node, 2, 2);
        neighbor_idx = find(dist_to_new <= params.rewire_radius);

        min_cost = tree(nearest_idx).cost + norm(new_node - nearest_node);
        best_parent_idx = nearest_idx;
        for i = 1:length(neighbor_idx)
            idx = neighbor_idx(i);
            temp_cost = tree(idx).cost + norm(new_node - tree(idx).node);
            if temp_cost < min_cost && ~check_segment_collision(tree(idx).node, new_node, params, PX, PY, PZ_T, pNofly, pCyl, pBuildings)
                min_cost = temp_cost; best_parent_idx = idx;
            end
        end

        node_count = node_count + 1;
        tree(node_count).node = new_node;
        tree(node_count).parent = best_parent_idx;
        tree(node_count).cost = min_cost;

        for i = 1:length(neighbor_idx)
            idx = neighbor_idx(i);
            temp_cost = tree(node_count).cost + norm(tree(idx).node - new_node);
            if temp_cost < tree(idx).cost && ~check_segment_collision(new_node, tree(idx).node, params, PX, PY, PZ_T, pNofly, pCyl, pBuildings)
                tree(idx).parent = node_count;
                tree(idx).cost = temp_cost;
            end
        end

        if norm(new_node - goal_point) <= params.goal_threshold
            if ~check_segment_collision(new_node, goal_point, params, PX, PY, PZ_T, pNofly, pCyl, pBuildings)
                goal_reached = true;
                node_count = node_count + 1;
                tree(node_count).node = goal_point;
                tree(node_count).parent = node_count - 1;
                tree(node_count).cost = tree(node_count - 1).cost + norm(goal_point - new_node);
                goal_node_idx = node_count;
                break;
            end
        end
    end

    if ~goal_reached
        path = [];
        return;
    end

    path = [];
    current_idx = goal_node_idx;
    while current_idx ~= 0
        path = [tree(current_idx).node; path];
        current_idx = tree(current_idx).parent;
    end
    path = repair_city_path(path, pBuildings, params.safe_dist);
end

% ============ 碰撞检测子函数 ============
function is_collision = check_collision(point, params, X, Y, z_total_mountain, nofly, cyl, buildings)
    x = point(1); y = point(2); z = point(3);
    is_collision = true;
    if x < params.x_range(1) || x > params.x_range(2) ...
            || y < params.y_range(1) || y > params.y_range(2) ...
            || z < params.z_range(1) || z > params.z_range(2)
        return;
    end
    mountain_h = interp2(X, Y, z_total_mountain, x, y, 'linear', 0);
    if z < mountain_h + params.safe_dist
        return;
    end
    if nofly.r > 0 && norm([x, y, z] - [nofly.cx, nofly.cy, nofly.cz]) < nofly.r + params.safe_dist && z >= nofly.cz
        return;
    end
    for i = 1:size(cyl.positions, 1)
        cx = cyl.positions(i, 1); cy = cyl.positions(i, 2); cz_max = cyl.positions(i, 3);
        if norm([x - cx, y - cy]) < cyl.r + params.safe_dist && z <= cz_max + params.safe_dist && z >= 0
            return;
        end
    end
    for i = 1:size(buildings, 1)
        b = buildings(i, :);
        if dist_to_box(point, b) < params.safe_dist
            return;
        end
    end
    is_collision = false;
end

function is_segment_collision = check_segment_collision(p1, p2, params, X, Y, z_total_mountain, nofly, cyl, buildings)
    is_segment_collision = true;
    seg_length = norm(p2 - p1);
    interp_num = max(ceil(seg_length / 0.5), 2);
    for i = 1:interp_num
        t = (i - 1) / (interp_num - 1);
        point = p1 + t * (p2 - p1);
        if check_collision(point, params, X, Y, z_total_mountain, nofly, cyl, buildings)
            return;
        end
    end
    is_segment_collision = false;
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
% 点 pt 到轴对齐立方体 b=[x0 x1 y0 y1 h](z 从 0 到 h) 的带符号距离：外部为正，内部为负。
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

% =========================================================================
function path = repair_city_path(path, buildings, safe_dist)
% 几何修复：把最终路径上穿楼/贴楼的点推出到净空 >= safe_dist（节点级 + 线段级细分）。
% 仅当 buildings 非空（city）时生效；其余地图为 no-op。
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
