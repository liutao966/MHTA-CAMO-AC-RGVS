function best_path = run_irrtstar(map_type, do_plot)
% RUN_IRRTSTAR  Improved RRT*(人工势场代价)三维路径规划主程序
% -------------------------------------------------------------------------
% 功能:
%   1) 按 map_type 构建三维环境(city 城市 / hill 丘陵 / 缺省旧山体地图);
%   2) 用改进 RRT*(基于 APF 代价函数)规划一条不穿障碍的最优路径;
%   3) 绘制三维地图 + RRT*树扩张 + 红色最优路径 + 起点(绿)/终点(蓝) + 代价收敛。
% -------------------------------------------------------------------------
if nargin < 1 || isempty(map_type), map_type = 'legacy'; end
if nargin < 2 || isempty(do_plot), do_plot = true; end
rng('default');
rng(2025);      % 固定随机种子, 便于结果复现

%% 1. 环境与参数
model = build_environment(map_type);
params.delta       = 8;     % 扩展步长(m)
params.maxIter     = 2000;  % 最大迭代次数
params.near_radius = 24;    % 最优父节点/重布线搜索半径(m)
params.goal_bias   = 0.10;  % 目标偏向概率
params.safe_dist   = 2.5;     % 碰撞裕度(=UAV半径外扩,m)
params.step        = 2;     % 碰撞采样步长(m)

% 人工势场代价参数(式9/式10)
params.gamma = 0.8;         % 势场权重 γ
params.d0    = 18;          % 斥力影响范围(m)
params.eta1  = 1.0;         % 斥力强度
params.eta2  = 0.3;         % 引力强度
params.dl    = 1;           % 势场采样间隔(m)

%% 2. 运行 Improved RRT*
fprintf('地图类型: %s   起点: [%g %g %g]   终点: [%g %g %g]\n', map_type, model.start, model.goal);
fprintf('\n===== 开始 Improved RRT* 规划 =====\n');
tic;
[best_path, best_cost, tree, parent, curve] = irrtstar_plan(model, params);
elapsed = toc;
fprintf('===== 规划完成(耗时 %.2f s) =====\n', elapsed);
if isempty(best_path)
    error('未能在有限迭代内求得路径，请增大 maxIter 或调整势场参数。');
end
ok = segment_free_all(best_path, model, params.safe_dist);

% ---- 计算并打印 6 个目标值 ----
% 目标: [路径长度(1), 能耗(2), 不平滑度(3), 飞行时间(4), 最小安全距离(5), 威胁代价(6)]
f6 = compute_six_objectives(best_path, model, params.safe_dist);
fprintf('路径长度 = %.2f m, 无碰撞 = %g\n', path_length(best_path), ok);
fprintf('--- 6 个目标值 ---\n');
fprintf('  目标1 - 路径长度     = %.2f m\n', f6(1));
fprintf('  目标2 - 能耗         = %.2f Wh\n', f6(2));
fprintf('  目标3 - 不平滑度     = %.4f (越小越平滑)\n', f6(3));
fprintf('  目标4 - 飞行时间     = %.2f s (V=10m/s)\n', f6(4));
fprintf('  目标5 - 最小安全距离 = %.2f m (路径上距障碍物最近点)\n', f6(5));
fprintf('  目标6 - 威胁代价     = %.2f (0 表示全程无碰撞)\n',    f6(6));
fprintf('算法耗时: %.2f s\n', elapsed);

%% 3. 可视化
if do_plot
    draw_map_and_paths(model, tree, parent, best_path);
    draw_curve(curve, map_type);
end
end

% =========================================================================
function L = path_length(P)
L = sum(sqrt(sum(diff(P, 1, 1).^2, 2)));
end

function ok = segment_free_all(P, model, safe_d)
ok = true;
for i = 1:size(P, 1) - 1
    if ~segment_free(P(i, :), P(i + 1, :), model, safe_d, 2)
        ok = false; return;
    end
end
end

% =========================================================================
function draw_map_and_paths(model, tree, parent, best_path)
tick_font_size  = 20; label_font_size = 20; title_font_size = 16;

figure('Name', sprintf('环境与路径-%s', model.type), 'Position', [100, 100, 850, 800], 'Color', 'white');
ax = gca;
hold(ax, 'on');

if isfield(model, 'type') && strcmp(model.type, 'city')
    % ---- 城市: 平底地面 + 矩形高楼 ----
    patch('Parent', ax, 'XData', [0 200 200 0], 'YData', [0 0 200 200], 'ZData', [1e-3 1e-3 1e-3 1e-3], ...
          'FaceColor', [0.82 0.82 0.78], 'EdgeColor', 'none', 'FaceAlpha', 1.0);
    for i = 1:size(model.buildings, 1)
        draw_building_box(ax, model.buildings(i, :));
    end
else
    % ---- 山体/丘陵/旧地图 ----
    map_x = 0:0.5:200; map_y = 0:0.5:200;
    [X, Y] = meshgrid(map_x, map_y);
    mt = model.mountains;
    noise = randn(size(X));
    for i = 1:size(mt, 1)
        zk = mt(i,3) * exp(-((X-mt(i,1)).^2 + (Y-mt(i,2)).^2)/(2*mt(i,4)^2)) + noise;
        zk(zk<0) = 0;
        surf(ax, X, Y, zk, 'FaceColor','interp','EdgeColor','none','FaceAlpha',0.9);
    end

    % 球形禁飞区(存在时)
    if ~isempty(model.ball) && numel(model.ball) == 4
        [Xb,Yb,Zb] = sphere(50); bc = model.ball(1:3); br = model.ball(4);
        bz = bc(3)+Zb*br; bz(bz<0)=0;
        surf(ax, bc(1)+Xb*br, bc(2)+Yb*br, bz, 'FaceColor',[1 0 1],'EdgeColor','none','FaceAlpha',1.0);
    end

    % 圆柱体(存在时)
    if ~isempty(model.cylinder.pos) && model.cylinder.r > 0
        [Xc,Yc,Zc] = cylinder(model.cylinder.r, 50);
        th = linspace(0,2*pi,50);
        for i = 1:size(model.cylinder.pos,1)
            p = model.cylinder.pos(i,:);
            surf(ax, p(1)+Xc, p(2)+Yc, p(3)*Zc, 'FaceColor',[1 1 0],'EdgeColor','none','FaceAlpha',1.0);
            fill3(ax, p(1)+model.cylinder.r*cos(th), p(2)+model.cylinder.r*sin(th), ...
                repmat(p(3),size(th)), [1 1 0], 'EdgeColor','none');
        end
    end
end

% RRT*树(浅灰细线)——按需显隐: 1=显示, 0=隐藏
show_tree = false;
if show_tree
    tshow = min(size(tree, 1), 4000);
    for i = 2:tshow
        p1 = tree(i, :); p0 = tree(parent(i), :);
        line(ax, [p0(1) p1(1)],[p0(2) p1(2)],[p0(3) p1(3)],'Color',[0.75 0.75 0.75],'LineWidth',0.3);
    end
end

% 最优路径(红色)
plot3(ax, best_path(:,1), best_path(:,2), best_path(:,3), '-','Color',[1 0 0],'LineWidth',3);

draw_marker(ax, model.start, 6, [0 0.8 0]);
draw_marker(ax, model.goal,  6, [0 0 1]);

xlabel(ax, "X/m","FontSize",label_font_size);
ylabel(ax, "Y/m","FontSize",label_font_size);
zlabel(ax, "Z/m","FontSize",label_font_size);
title(ax, sprintf("Improved RRT* 人工势场代价路径规划（%s）", map_name_of(model.type)), "FontSize", title_font_size);
xlim(ax, [0 200]); ylim(ax, [0 200]); zlim(ax, [0 200]);
xticks(ax, 0:20:200); yticks(ax, 0:20:200); zticks(ax, 0:20:200);
daspect(ax, [1 1 1]); set(ax, "FontSize", tick_font_size);
grid(ax, 'on'); set(ax, "Color", "white");
view(ax, 45, 30); camproj(ax, "orthographic");

ia = plot3(ax, nan,nan,nan,'-','Color',[1 0 0],'LineWidth',3);
ib = plot3(ax, nan,nan,nan,'o','MarkerSize',8,'MarkerFaceColor',[0 0.8 0],'MarkerEdgeColor','k');
ic = plot3(ax, nan,nan,nan,'o','MarkerSize',8,'MarkerFaceColor',[0 0 1],'MarkerEdgeColor','k');
legend(ax, [ia ib ic],{'最优路径(无碰撞)','起点','终点'},'Location','northeast','FontSize',12);
hold(ax, 'off');
end

function draw_marker(ax, c, r, col)
[sx,sy,sz] = sphere(30);
surf(ax, c(1)+sx*r, c(2)+sy*r, c(3)+sz*r, 'FaceColor',col,'EdgeColor','none','FaceAlpha',1.0);
end

function draw_building_box(ax, b)
% 绘一栋立方体高楼(4 侧面 + 顶盖), 底在 z=0; 越高越偏蓝紫
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

function str = map_name_of(map_type)
switch map_type
    case 'city',     str = '城市地图';
    case 'hill',     str = '丘陵地图';
    case 'obstacle', str = '障碍物地图';
    otherwise,       str = '山体障碍地图';
end
end

function draw_curve(curve, map_type)
valid = curve(curve > 0);
figure('Name', sprintf('代价收敛曲线-%s', map_type), 'Color','white','Position',[950 100 620 420]);
if isempty(valid)
    return;
end
plot(1:numel(valid), valid, 'LineWidth', 1.8, 'Color', [0 0.45 0.74]);
grid on;
xlabel('迭代次数','FontSize',20);
ylabel('最优路径总代价','FontSize',20);
title(sprintf('Improved RRT* 代价收敛曲线（%s）', map_name_of(map_type)),'FontSize',16);
set(gca,'FontSize',20,'Color','white');
end

% =========================================================================
function f6 = compute_six_objectives(P, model, safe_dist)
% COMPUTE_SIX_OBJECTIVES  对一条平滑路径计算 6 个目标值
%   [路径长度, 能耗, 不平滑度, 飞行时间, 最小安全距离, 威胁代价]
% 内部调用独立文件 min_dist.m 计算点到障碍物带符号距离
V_DRONE = 10;
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

% ---- 目标5&6: 先算全路径点的障碍距离(后面能耗需要) ----
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

% ---- 目标2: 能耗（论文完整版）----
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

% ---- 目标3: 不平滑度 ----
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
% ---- 目标6: 威胁代价 ----
f6 = [L, energy, 1 - smooth, flight_time, d_safe_min, threat];
end