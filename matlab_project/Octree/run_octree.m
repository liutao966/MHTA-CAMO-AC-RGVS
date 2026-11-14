function path = run_octree(map_type, do_plot)
% RUN_OCTREE  Octree(重叠检测) + 3D A* 三维路径规划主程序 (Octree 项目)
% -------------------------------------------------------------------------
% 参考: "Simplified Unstructured-Mesh Based UAV Path Planning Method Using
%        Octree Overlap Detection", 2025.
% 流程:
%   1) 用重叠检测(cube_occ) / 八叉树(octree_build)划分导航空间, 生成危险栅格;
%   2) 在自由栅格上运行 3D A*(式11 f=g+h) 寻找到终点的无碰撞路径;
%   3) 绘制地图 + 占据栅格(浅色方块) + 红色路径 + 起点/终点。
% 输入:
%   map_type - 可选, 'city' / 'hill' / (缺省为旧地图)
% -------------------------------------------------------------------------
if nargin < 1 || isempty(map_type), map_type = 'legacy'; end
if nargin < 2 || isempty(do_plot), do_plot = true; end
fprintf('\n========================================\n');

%% 1. 环境与网格参数
model = octree_build_environment(map_type);
sigma = 4;          % 安全裕度(障碍外扩,m)
res   = 4;          % A* 导航栅格分辨率(m)（原 12.5 太粗，22m 高楼只占约 2 格，穿楼）
mincell = 12.5;     % 八叉树最小单元边长(m)

%% 2. 构造八叉树(自适应, 用于展示障碍/自由空间结构)
leaves = octree_build(model, mincell, sigma);
nOcc = sum([leaves.occ]);
fprintf('八叉树叶子数: %d (其中占据 %d)\n', numel(leaves), nOcc);

%% 3. 生成 A* 导航栅格(按重叠检测判定每格占据)
nocc = grid_occupancy(model, res, sigma);
fprintf('A* 栅格尺寸 %dx%dx%d, 自由格 %d 个\n', size(nocc, 1), size(nocc, 2), ...
    size(nocc, 3), nnz(~nocc));

%% 4. 运行 3D A*
tic;
path = a_star_grid(nocc, res, model.xyz_lo, model.start, model.goal);
elapsed = toc;
fprintf('A* 规划完成(耗时 %.2f s)\n', elapsed);
if isempty(path)
    error('A* 未能在栅格中找到可行路径，请减小 res 或增大 sigma 之外的空间。');
end
fprintf('路径长度 = %.2f m, 无碰撞 = %g\n', path_length(path), ...
    check_free_all(path, model));

%% 5. 计算并打印 6 个目标值
% 目标: [路径长度, 能耗, 不平滑度, 飞行时间, 最小安全距离, 威胁代价]
f6 = compute_six_objectives(path, model);
fprintf('--- 6 个目标值 ---\n');
fprintf('  目标1 - 路径长度     = %.2f m\n', f6(1));
fprintf('  目标2 - 能耗         = %.2f Wh\n', f6(2));
fprintf('  目标3 - 不平滑度     = %.4f (越小越平滑)\n', f6(3));
fprintf('  目标4 - 飞行时间     = %.2f s (V=10m/s)\n', f6(4));
fprintf('  目标5 - 最小安全距离 = %.2f m (路径上距障碍物最近点)\n', f6(5));
fprintf('  目标6 - 威胁代价     = %.2f (0 表示全程无碰撞)\n',    f6(6));

%% 6. 可视化
if do_plot
    draw_result(model, nocc, path);
end
end

% =========================================================================
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

% =========================================================================
function path = a_star_grid(occ, res, lo, start, goal)
dims = size(occ);
si = idx_of(lo, res, start, dims);
gi = idx_of(lo, res, goal,  dims);
goalIdx = gi;

if occ(si(1), si(2), si(3)) || occ(gi(1), gi(2), gi(3))
    error('起点或终点栅格被占据，请调整参数。');
end

% 邻域(26连通)
[dx, dy, dz] = ndgrid(-1:1, -1:1, -1:1);
nn = [dx(:) dy(:) dz(:)]; nn(14, :) = [];   % 去掉(0,0,0)
g = inf(numel(occ), 1); f = inf(numel(occ), 1);
par = zeros(numel(occ), 1);
ci = @(a, b, c) sub2ind(dims, a, b, c);
sid = ci(si(1), si(2), si(3));
gid = ci(gi(1), gi(2), gi(3));
g(sid) = 0;
f(sid) = euclid(si, gi);
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
        if nx < 1 || nx > dims(1) || ny < 1 || ny > dims(2) || nz < 1 || nz > dims(3)
            continue;
        end
        if occ(nx, ny, nz), continue; end
        v = ci(nx, ny, nz);
        if closed(v), continue; end
        step = euclid([ux uy uz], [nx ny nz]);
        if g(u) + step < g(v)
            g(v) = g(u) + step;
            par(v) = u;
            f(v) = g(v) + euclid([nx ny nz], goalIdx);
            open(v) = true;
        end
    end
    if closed(gid), found = true; end
end

if ~found
    path = []; return;
end

% ---- 回溯 ----
seq = [];
c = gid;
while c ~= 0 && c ~= sid
    seq = [c seq]; %#ok<AGROW>
    c = par(c);
end
seq = [sid, seq, gid];
verts = zeros(numel(seq), 3);
for q = 1:numel(seq)
    [a, b, c3] = ind2sub(dims, seq(q));
    verts(q, :) = lo + [a - 0.5, b - 0.5, c3 - 0.5] .* res;
end
% 拼接真实起点/终点并平滑
path = smooth_path([start; verts; goal], 300);
end

function id = idx_of(lo, res, p, dims)
id = max(1, min(dims, floor((p - lo) / res) + 1));
end

function d = euclid(a, b)
d = norm(double(b) - double(a));
end

% =========================================================================
function L = path_length(P)
L = sum(sqrt(sum(diff(P, 1, 1).^2, 2)));
end

function ok = check_free_all(P, model)
ok = true;
for i = 1:size(P, 1)
    if min_dist(P(i, :), model) < 0
        ok = false; return;
    end
end
end

function sp = smooth_path(path, n)
% 剔除连续重合点, 保证 pchip 累积弦长参数单调
keep = [true; sqrt(sum(diff(path, 1, 1).^2, 2)) > 1e-9];
path = path(keep, :);
m = size(path, 1);
if m < 3
    sp = path; return;
end
seg = sqrt(sum(diff(path, 1, 1).^2, 2));
t = [0; cumsum(seg)];
tt = linspace(t(1), t(end), n);
sp = zeros(n, 3);
for d = 1:3
    sp(:, d) = pchip(t, path(:, d), tt);
end
end

% =========================================================================
function f6 = compute_six_objectives(P, model)
% COMPUTE_SIX_OBJECTIVES  对一条平滑路径计算 6 个目标值
%   [路径长度, 能耗, 不平滑度, 飞行时间, 最小安全距离, 威胁代价]
% 内部调用同目录独立文件 min_dist.m 计算点到障碍物带符号距离
V_DRONE = 10;
safe_dist = model.safe_dist;
nP = size(P, 1);

% ---- 目标1: 路径长度 + 能耗所需的高度变化 ----
seg_len = zeros(nP - 1, 1);
height_change = 0;
for i = 2:nP
    d = P(i, :) - P(i - 1, :);
    seg_len(i - 1) = norm(d);
    height_change = height_change + abs(d(3));
end
L = sum(seg_len);

% ---- 目标5&6: 先算全路径点的障碍距离 ----
D_all = zeros(nP, 1);
threat = 0; hard = 0; HARD_PEN = 1e5;
for i = 1:nP
    D_all(i) = min_dist(P(i, :), model);
    if D_all(i) < safe_dist
        threat = threat + (safe_dist - D_all(i));
        hard = hard + 1;
    end
end
threat = threat + hard * HARD_PEN;
d_safe_min = max(min(D_all), 0);

% ---- 目标2: 能耗(论文模型) ----
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

% ---- 目标3: 不平滑度 = 1 - mean(cosθ) ----
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

f6 = [L, energy, 1 - smooth, flight_time, d_safe_min, threat];
end

% =========================================================================
function draw_result(model, occ, path)
tick_font_size  = 20; label_font_size = 20; title_font_size = 16;
map_x = 0:0.5:200; map_y = 0:0.5:200;
[X, Y] = meshgrid(map_x, map_y);
mt = model.mountains;
noise = randn(size(X));

figure('Position', [100, 100, 850, 800], 'Color', 'white');

if isfield(model,'type') && strcmp(model.type,'city')
    draw_city_scene(model);
else
    % 山体(若干座, 通用) 
    for i = 1:size(mt,1)
        zk = mt(i,3)*exp(-((X-mt(i,1)).^2+(Y-mt(i,2)).^2)/(2*mt(i,4)^2)) + noise;
        zk(zk<0)=0;
        surf(X,Y,zk,'FaceColor','interp','EdgeColor','none','FaceAlpha',0.9);
        hold on;
    end
    % 球形禁飞区(存在时)
    if ~isempty(model.ball) && numel(model.ball)==4
        [Xb,Yb,Zb] = sphere(50); bc = model.ball(1:3); br = model.ball(4);
        bz = bc(3)+Zb*br; bz(bz<0)=0;
        surf(bc(1)+Xb*br, bc(2)+Yb*br, bz, 'FaceColor',[1 0 1],'EdgeColor','none','FaceAlpha',1.0);
    end
    % 圆柱体(存在时)
    if size(model.cylinder.pos,1) > 0 && model.cylinder.r > 0
        [Xc,Yc,Zc] = cylinder(model.cylinder.r, 50);
        th = linspace(0,2*pi,50);
        for i = 1:size(model.cylinder.pos,1)
            p = model.cylinder.pos(i,:);
            surf(p(1)+Xc, p(2)+Yc, p(3)*Zc, 'FaceColor',[1 1 0],'EdgeColor','none','FaceAlpha',1.0);
            fill3(p(1)+model.cylinder.r*cos(th), p(2)+model.cylinder.r*sin(th), ...
                repmat(p(3),size(th)), [1 1 0], 'EdgeColor','none');
        end
    end
end

% 占据栅格(浅色方块, octree危险区域) —— 按需显隐: true=显示
show_grid = false;
if show_grid
    res = 12.5; lo = [0 0 30];
    idxs = find(occ);
    for q = 1:numel(idxs)
        [a, b, c3] = ind2sub(size(occ), idxs(q));
        ctr = lo + [a - 0.5, b - 0.5, c3 - 0.5] .* res;
        draw_cube(ctr, res/2, [0.45 0.45 0.45]);
    end
end

% 最优路径(红色)
plot3(path(:,1), path(:,2), path(:,3), '-','Color',[1 0 0],'LineWidth',3);
draw_marker(model.start, 6, [0 0.8 0]);
draw_marker(model.goal,  6, [0 0 1]);

xlabel("X/m","FontSize",label_font_size);
ylabel("Y/m","FontSize",label_font_size);
zlabel("Z/m","FontSize",label_font_size);
title(["Octree 重叠检测 + 3D A* 路径规划（" + map_name_of(model) + "）"],"FontSize",title_font_size);
xlim([0 200]); ylim([0 200]); zlim([0 200]);
xticks(0:20:200); yticks(0:20:200); zticks(0:20:200);
daspect([1 1 1]); set(gca,"FontSize",tick_font_size);
grid on; set(gca,"Color","white");
view(45,30); camproj("orthographic");

ia = plot3(nan,nan,nan,'-','Color',[1 0 0],'LineWidth',3);
ib = plot3(nan,nan,nan,'o','MarkerSize',8,'MarkerFaceColor',[0 0.8 0],'MarkerEdgeColor','k');
ic = plot3(nan,nan,nan,'o','MarkerSize',8,'MarkerFaceColor',[0 0 1],'MarkerEdgeColor','k');
legend([ia ib ic],{'A* 最优路径','起点','终点'},'Location','northeast','FontSize',12);
hold off;
end

function str = map_name_of(model)
if isfield(model,'type') && strcmp(model.type,'city')
    str = "城市地图";
elseif isfield(model,'type') && strcmp(model.type,'hill')
    str = "丘陵地图";
elseif isfield(model,'type') && strcmp(model.type,'obstacle')
    str = "障碍物地图";
else
    str = "山体障碍地图";
end
end

function draw_city_scene(model)
% 绘制 city_map.m 的城市环境: 平底地面 + 矩形高楼
hold on;
patch([0 200 200 0],[0 0 200 200],[1e-3 1e-3 1e-3 1e-3],[0.82 0.82 0.78], ...
      'EdgeColor','none','FaceAlpha',1.0);
B = model.buildings;
for i = 1:size(B,1)
    b = B(i,:);
    draw_building_box(b(1),b(2),b(3),b(4),b(5));
end
end

function draw_building_box(x0,x1,y0,y1,h)
% 绘一栋立方体高楼(4 侧面 + 顶盖), 底在 z=0; 越高越偏蓝紫
t = min(max(h/150,0),1);
col = [0.4-0.15*t, 0.48-0.10*t, 0.72+0.10*t];
verts_x = [x0, x1, x1, x0, x0, x1, x1, x0];
verts_y = [y0, y0, y1, y1, y0, y0, y1, y1];
verts_z = [0, 0, 0, 0, h, h, h, h];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
patch('Vertices',[verts_x(:),verts_y(:),verts_z(:)],'Faces',faces, ...
      'FaceColor',col,'EdgeColor','k','LineWidth',0.5,'FaceAlpha',1.0);
patch([x0 x1 x1 x0],[y0 y0 y1 y1],[h h h h],col.*1.2,'EdgeColor','none','FaceAlpha',1.0);
end

function draw_cube(c, h, col)
verts = [c + (h*[-1 -1 -1]); c + (h*[1 -1 -1]); c + (h*[1 1 -1]); c + (h*[-1 1 -1]); ...
         c + (h*[-1 -1 1]); c + (h*[1 -1 1]); c + (h*[1 1 1]); c + (h*[-1 1 1])];
faces = [1 2 3 4; 2 6 7 3; 6 5 8 7; 5 1 4 8; 1 5 6 2; 4 3 7 8];
patch('Vertices', verts, 'Faces', faces, 'FaceColor', col, ...
    'EdgeColor', 'none', 'FaceAlpha', 0.06);
end

function draw_marker(c, r, col)
[sx,sy,sz] = sphere(30);
surf(c(1)+sx*r, c(2)+sy*r, c(3)+sz*r, 'FaceColor',col,'EdgeColor','none','FaceAlpha',1.0);
end