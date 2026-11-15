function AVSS_RRT_dynamic()
% ================================================================
%  AVSS-RRT 动态障碍物三维路径规划与实时避障
% ----------------------------------------------------------------
%  在 AVSS_RRT.m 的基础上, 加入动态障碍物(黑色球, 在设定的两点
%  之间往返运动), 实现无人机沿路径飞行时的实时避障:
%
%   1) 先对"静态障碍物"(高斯山体/球形禁飞区/圆柱群)用 AVSS-RRT
%      规划出一条全局最优路径(棕色实线, 图中始终显示);
%   2) 无人机沿该路径逐步飞行, 每个时间步动态障碍物随之移动;
%   3) 若无人机前方一段距离内(look-ahead)将与某个动态障碍物相遇,
%      则脱离原轨迹向侧向/上方小幅绕行(不改动全局路径);
%   4) 障碍物走开、前方畅通后, 无人机回归全局最优路径继续飞行;
%   5) 动画显示: 棕色实线=规划路径, 红色实线=已飞行轨迹,
%      黑色球=动态障碍物, 绿/蓝球=起点/终点。
%
%  地形: 采用 map.m 中定义的三座高斯山体、球形禁飞区、九根圆柱障碍。
%  起点 [10,10,50] (绿色球), 终点 [190,190,80] (蓝色球)。
% ================================================================
clear; clc; close all;
rng(1);   % 固定随机种子, 结果可复现

%% ------------------ 1. 地图与静态障碍物参数 (与 map.m 一致) ------
world.xlim = [0 200];
world.ylim = [0 200];
world.zlim = [0 200];

% --- 三座高斯山体 [x0, y0, A(峰值), sigma] ---
world.mountains = [ 40 140  90 30;
                   140  40  90 30;
                   140 140 120 30];

% --- 球形禁飞区 [cx, cy, cz, r] ---
world.sphere = [100 100 0 50];

% --- 九根圆柱障碍 [x0, y0, height], 半径统一 ---
world.cyl_r = 5;
world.cylinders = [ 40  70 100;
                    70  40 100;
                    50  50 100;
                    90 160 100;
                   160  80 100;
                    80 180 100;
                   100 180 100;
                   180  60 100;
                   180 100 100];

world.safe = 3.0;        % 无人机安全距离(机体约束)

% --- 动态障碍物初始为空, 用于静态规划阶段 ---
world.dyn = [];          % 每行 [cx cy cz], 无动态障碍时为空

%% ------------------ 2. 动态障碍物参数 (与 untitled.m 一致) -------
% 每行 [起点x y z, 终点x y z, 方向(±1)], 障碍在起点<->终点之间往返
dyn.num   = 11;
dyn.r     = 5;           % 动态障碍物半径
dyn.speed = 1;           % 每个时间步移动的距离
% 前 5 行: 与 untitled.m 一致的原始动态障碍物;
% 后 6 行: 新增障碍物, 其往返线段均"垂直横穿"静态规划路径(在弧长
%          s≈40/100/120/180/220/300 m 处), 使避障过程频繁可见。
dyn.params = [
    150,  90, 68,  35,  85, 68, 1;
     95,  40, 60,  92, 140, 60, 1;
     50,  10, 10,  10,  80, 80, 1;
     56, 125, 75, 124, 56, 75, 1;
     95, 130, 75, 146, 90, 75, 1;
     29,  37, 53,  54, 11, 53, 1;
     61,  74, 73,  81, 52, 73, 1;
     87,  73, 75,  91, 42, 75, 1;
    106, 113, 81, 134, 97, 81, 1;
    154, 124, 99, 139, 95, 99, 1;
    156, 168,102, 184,151,102, 1;
];
dyn.curr = dyn.params(:, 1:3);   % 当前中心位置 (N x 3)

%% ------------------ 3. 起点 / 终点 -------------------------------
start = [ 10  10 50];
goal  = [190 190 80];

%% ------------------ 4. AVSS-RRT 参数 ----------------------------
params.max_iter   = 8000;   % 最大迭代次数
params.p_goal     = 0.15;   % Goal-bias 采样概率
params.step_min   = 3;      % 最小步长(障碍物附近)
params.step_max   = 18;     % 最大步长(空旷区域)
params.goal_tol   = 8;      % 到达终点的容差
params.edge_res   = 1.5;    % 边碰撞检测的采样分辨率

% --- 动态避障相关 ---
sim.uav_speed   = 2.0;      % 无人机每个时间步飞行的距离
sim.look_ahead  = 12;       % 前视距离: 检查前方多远是否会撞动态障碍
sim.max_steps   = 4000;     % 仿真最大时间步(防止死循环)

%% ------------------ 5. 初始路径规划 (仅静态障碍) -----------------
fprintf('AVSS-RRT 初始路径规划(静态障碍物) ...\n');
tic;
[path, ~, success] = avss_rrt_plan(start, goal, world, params);
t_plan = toc;
if ~success || isempty(path)
    error('初始路径规划失败, 请调整参数或检查地图。');
end
path = smooth_path_cubic(path);
fprintf('初始规划完成: 用时 %.3f s, 路径长度 %.2f m。\n', ...
        t_plan, path_length(path));

% 用四类动态障碍物(直线×3 圆弧×3 变速×4 垂直×4)覆盖上方旧硬编码障碍,
% 沿 AVSS-RRT 静态路径弧长生成, 取代原来的 11 个直线往返障碍。
dyn = gen_dyn_v2(path);

%% ------------------ 6. 绘制场景 ----------------------------------
draw_world(world);
hold on;

% 起点(绿色球) / 终点(蓝色球)
draw_ball(start, 4, [0 0.8 0]);
draw_ball(goal,  4, [0 0 1]);

% 规划路径(棕色实线, 静态规划得到的全局最优路径) 与已飞行轨迹(红实线)
% 棕色避开地图中其他物体颜色: 山体(蓝绿黄渐变)、禁飞区(品红)、圆柱(黄)、
% 起点(绿)、终点(蓝)、动态障碍(黑)。
col_plan  = [0.55 0.27 0.07];   % 棕色: 规划路径
col_trail = [0.85 0.0 0.0];     % 正红: 已飞行轨迹
h_plan  = plot3(path(:,1), path(:,2), path(:,3), ...
                '-', 'Color', col_plan, 'LineWidth', 2.2);
h_trail = plot3(start(1), start(2), start(3), ...
                '-',  'Color', col_trail, 'LineWidth', 3);
% 无人机当前位置(红色实心点)
h_uav = plot3(start(1), start(2), start(3), 'o', ...
              'MarkerFaceColor', col_trail, 'MarkerEdgeColor', 'none', ...
              'MarkerSize', 8);

% 动态障碍物句柄(按四类运动着色: 红=直线 蓝=圆弧 绿=变速 橙=垂直)
[Xd, Yd, Zd] = sphere(20);
Xd = Xd*dyn.r; Yd = Yd*dyn.r; Zd = Zd*dyn.r;
h_dyn = gobjects(dyn.num, 1);
for i = 1:dyn.num
    c = dyn.curr(i,:);
    h_dyn(i) = surf(c(1)+Xd, c(2)+Yd, c(3)+Zd, ...
                    'FaceColor', type_color(dyn.obs{i}.type), ...
                    'EdgeColor','none','FaceAlpha',1.0);
end

legend_h = [ plot3(nan,nan,nan,'-','Color',col_trail,'LineWidth',3), ...
             plot3(nan,nan,nan,'-','Color',col_plan,'LineWidth',2.2), ...
             plot3(nan,nan,nan,'o','MarkerFaceColor',type_color(1), ...
                   'MarkerEdgeColor','none','MarkerSize',10), ...
             plot3(nan,nan,nan,'o','MarkerFaceColor',type_color(2), ...
                   'MarkerEdgeColor','none','MarkerSize',10), ...
             plot3(nan,nan,nan,'o','MarkerFaceColor',type_color(3), ...
                   'MarkerEdgeColor','none','MarkerSize',10), ...
             plot3(nan,nan,nan,'o','MarkerFaceColor',type_color(4), ...
                   'MarkerEdgeColor','none','MarkerSize',10), ...
             plot3(nan,nan,nan,'o','MarkerFaceColor',[0 0.8 0], ...
                   'MarkerEdgeColor','none','MarkerSize',10), ...
             plot3(nan,nan,nan,'o','MarkerFaceColor',[0 0 1], ...
                   'MarkerEdgeColor','none','MarkerSize',10)];
legend(legend_h, {'已飞行轨迹','规划路径','直线障碍','圆弧障碍','变速障碍','垂直障碍','起点','终点'}, ...
       'Location','northeast','FontSize',11);
title('AVSS-RRT 动态障碍物路径规划与避障', 'FontSize', 16);
view(45, 30);

%% ------------------ 7. 仿真主循环: 沿全局最优路径飞行 + 局部避障 + 回归 ---
% 策略: 静态规划得到的 path 作为“全局最优路径”, 永不改变(品红实线基准)。
%   - 默认沿全局路径前进(用弧长参数 s_glob 定位);
%   - 若全局路径前方将撞动态障碍, 进入“局部绕行”: 朝侧向/上方偏移一个
%     临时航点躲避, 不改动全局路径;
%   - 障碍走开后, 拉回到全局路径上“最近且前方畅通”的点, 继续沿原路径飞。
fig = gcf;

% 将全局最优路径按弧长密集离散, 便于定位与回归
glob = densify_path(path, 1.0);          % 每 ~1m 一个采样点
glob_cum = [0; cumsum(sqrt(sum(diff(glob).^2, 2)))];   % 累积弧长
glob_len = glob_cum(end);

uav_pos = start;             % 无人机当前位置
trail   = start;            % 已飞行轨迹
s_glob  = 0;                % 当前在全局路径上的弧长进度
detour  = false;            % 是否处于局部绕行状态
detour_cnt = 0;             % 绕行触发次数(统计用)
reached = false;

for step = 1:sim.max_steps
    if ~ishandle(fig); break; end

    % ---- (a) 动态障碍物移动一步(往返运动) ----
    dyn = move_dynamic(dyn);
    world.dyn = dyn.curr;
    dyn_r_safe = dyn.r + world.safe;      % 动态障碍碰撞半径(含安全距离)

    % ---- (b) 检测全局路径前方(从当前进度起)是否会撞动态障碍 ----
    glob_blocked = glob_lookahead_hit(glob, glob_cum, s_glob, ...
                                      dyn.curr, dyn_r_safe, sim.look_ahead);

    if ~glob_blocked
        % ===== 全局路径畅通: 沿全局最优路径前进(必要时先拉回路径) =====
        if detour
            % 刚绕行完毕: 找到全局路径上"最近且前方畅通"的回归点
            s_glob = regain_arc(glob, glob_cum, uav_pos, s_glob, ...
                                dyn.curr, dyn_r_safe, sim.look_ahead);
            detour = false;
            fprintf('第 %d 步: 障碍已避开, 回归全局最优路径。\n', step);
        end
        s_glob = min(glob_len, s_glob + sim.uav_speed);
        target = interp_arc(glob, glob_cum, s_glob);
        uav_pos = move_towards(uav_pos, target, sim.uav_speed);
    else
        % ===== 全局路径被挡: 局部绕行(不改动全局路径) =====
        if ~detour
            detour_cnt = detour_cnt + 1;
            detour = true;
            fprintf('第 %d 步: 全局路径前方有动态障碍, 第 %d 次局部绕行。\n', ...
                    step, detour_cnt);
        end
        % 前瞻的全局路径点作为"绕行想去的方向", 在其周围选一个安全的临时航点
        aim = interp_arc(glob, glob_cum, min(glob_len, s_glob + sim.look_ahead));
        wp = pick_detour_waypoint(uav_pos, aim, goal, world, ...
                                  dyn.curr, dyn_r_safe, sim.uav_speed);
        if isempty(wp)
            % 找不到安全方向 -> 本步悬停等待障碍物移开
        else
            uav_pos = move_towards(uav_pos, wp, sim.uav_speed);
        end
    end
    trail(end+1,:) = uav_pos; %#ok<AGROW>

    % ---- (c) 更新图形 ----
    for i = 1:dyn.num
        c = dyn.curr(i,:);
        set(h_dyn(i), 'XData', c(1)+Xd, 'YData', c(2)+Yd, 'ZData', c(3)+Zd);
    end
    set(h_trail, 'XData', trail(:,1), 'YData', trail(:,2), 'ZData', trail(:,3));
    set(h_uav,   'XData', uav_pos(1), 'YData', uav_pos(2), 'ZData', uav_pos(3));
    drawnow;
    pause(0.02);

    % ---- (d) 到达终点判定 ----
    if norm(uav_pos - goal) <= sim.uav_speed || ...
       (s_glob >= glob_len - 1e-6 && norm(uav_pos - goal) <= params.goal_tol)
        uav_pos = goal;
        set(h_uav, 'XData', goal(1), 'YData', goal(2), 'ZData', goal(3));
        trail(end+1,:) = goal; %#ok<AGROW>
        set(h_trail, 'XData', trail(:,1), 'YData', trail(:,2), 'ZData', trail(:,3));
        drawnow;
        reached = true;
        break;
    end
end

% ---- 最终停止时: 将动态障碍物移到不与已飞行轨迹重合的位置 ----
% 沿每个障碍物的运动线段均匀采样, 取离已飞行轨迹(trail)最远的候选点,
% 使最后一帧黑色障碍球不与红色轨迹重叠。
for i = 1:0   % 四类障碍无"最终重排", 跳过依赖旧 dyn.params 的直线逻辑
    p1 = [0 0 0]; p2 = [0 0 0];
    seg = p2 - p1; L = norm(seg);
    if L > 1e-6
        u = seg / L;
        ss = linspace(0, L, 200)';
        cand = p1 + ss * u;                       % 候选位置 (N x 3)
        dmin = zeros(size(cand,1),1);
        for j = 1:size(cand,1)
            dmin(j) = min(sqrt(sum((trail - cand(j,:)).^2, 2)));
        end
        [~, k] = max(dmin);                       % 离轨迹最远的候选点
        dyn.curr(i,:) = cand(k,:);
    end
    c = dyn.curr(i,:);
    set(h_dyn(i), 'XData', c(1)+Xd, 'YData', c(2)+Yd, 'ZData', c(3)+Zd);
end
drawnow;

if reached
    fprintf('无人机到达终点! 步数 %d, 局部绕行 %d 次, 飞行距离 %.2f m。\n', ...
            step, detour_cnt, path_length(trail));
else
    warning('未在最大步数内到达终点, 可尝试增大 sim.max_steps。');
end
hold off;
end

% ================================================================
%                     动态障碍物运动 & 避障
% ================================================================
function dyn = move_dynamic(dyn)
% 四类动态障碍物各推进一步(按 dyn.obs{i}.type 分派)
for i = 1:dyn.num
    o   = dyn.obs{i};
    cur = dyn.curr(i,:);
    switch o.type
        case 1
            [nxt, o] = step_linear(cur, o);
        case 2
            [nxt, o] = step_circular(cur, o);
        case 3
            [nxt, o] = step_variable(cur, o);
        case 4
            [nxt, o] = step_vertical(cur, o);
        otherwise
            continue;
    end
    dyn.curr(i,:) = nxt;
    dyn.obs{i} = o;
end
end

% ================================================================
%        四类动态障碍物生成 & 单步运动 (替代原直线往返障碍)
% ================================================================
function dyn = gen_dyn_v2(path)
% 沿 AVSS-RRT 路径弧长生成 14 个四类动态障碍物(直线×3 圆弧×3 变速×4 垂直×4)
rng(1, 'twister');

dyn.num  = 14;
dyn.r    = 5;
dyn.curr = zeros(14, 3);
dyn.obs  = cell(14, 1);

seg = sqrt(sum(diff(path).^2, 2));
cum = [0; cumsum(seg)];
[cum, ia] = unique(cum, 'stable');
path = path(ia, :);
L = cum(end);
if L < 30
    dyn.num = 0; dyn.curr = zeros(0,3); dyn.obs = cell(0,1); return;
end

fracs = [0.20 0.24 0.29 0.33 0.37 0.41 0.46 0.50 0.54 0.59 0.63 0.67 0.71 0.76];
types = [1 2 3 4 3 1 4 2 3 4 1 2 3 4];
lin_speeds = [0.5 1.0 1.5];
lin_idx = 0;

for i = 1:14
    si = fracs(i) * L;
    p  = arc_interp(cum, path, si);
    s1 = max(cum(1), si - 3);
    s2 = min(cum(end), si + 3);
    tan = arc_interp(cum, path, s2) - arc_interp(cum, path, s1);
    if norm(tan) < 1e-6, tan = [1 0 0]; end
    tan = tan / norm(tan);
    up = [0 0 1];
    side = cross(tan, up);
    if norm(side) < 1e-6, side = cross(tan, [0 1 0]); end
    side(3) = 0;
    if norm(side) < 1e-6, side = [1 0 0]; end
    side = side / norm(side);

    switch types(i)
        case 1
            lin_idx = lin_idx + 1;
            dyn.obs{i} = make_linear(p, side, lin_speeds(lin_idx));
        case 2
            dyn.obs{i} = make_circular(p, side);
        case 3
            dyn.obs{i} = make_variable(p, side);
        case 4
            dyn.obs{i} = make_vertical(p, side);
    end
    dyn.curr(i,:) = initial_position(dyn.obs{i});
end
end

function p = arc_interp(cum, path, s)
s = min(max(s, cum(1)), cum(end));
p = [interp1(cum, path(:,1), s), interp1(cum, path(:,2), s), interp1(cum, path(:,3), s)];
end

function o = make_linear(p, side, speed)
half_w = 25 + 10*rand;
S = p - side*half_w; S(3) = p(3);
E = p + side*half_w; E(3) = p(3);
o.type = 1; o.S = S; o.E = E;
o.dir = sign(rand - 0.5); if o.dir == 0, o.dir = 1; end
o.speed = speed;
end

function o = make_circular(p, side)
R = 12 + 6*rand;
vt = 0.8 + 0.7*rand;
o.type = 2;
o.center = p + side*(0.6*R); o.center(3) = p(3);
o.R = R; o.omega = vt / R; o.theta = 2*pi*rand;
end

function o = make_variable(p, side)
half_w = 25 + 10*rand;
S = p - side*half_w; S(3) = p(3);
E = p + side*half_w; E(3) = p(3);
o.type = 3; o.S = S; o.E = E;
o.dir = sign(rand - 0.5); if o.dir == 0, o.dir = 1; end
o.v0 = 0.5 + 0.5*rand; o.amp = 0.5; o.period = 20 + 20*rand; o.t = rand*o.period;
end

function o = make_vertical(p, side)
half_w = 15 + 10*rand;
S = p - side*half_w; S(3) = p(3);
E = p + side*half_w; E(3) = p(3);
o.type = 4; o.S = S; o.E = E;
o.dir = sign(rand - 0.5); if o.dir == 0, o.dir = 1; end
o.hspeed = 0.5; o.z0 = p(3); o.A = 15 + 10*rand; o.Tz = 30 + 20*rand; o.t = rand*o.Tz;
end

function pos = initial_position(o)
switch o.type
    case 1
        u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
        pos = o.S + u*(rand*norm(o.E - o.S));
    case 2
        pos = [o.center(1)+o.R*cos(o.theta), o.center(2)+o.R*sin(o.theta), o.center(3)];
    case 3
        u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
        pos = o.S + u*(rand*norm(o.E - o.S));
    case 4
        u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
        d = rand*norm(o.E - o.S);
        pos = [o.S(1)+u(1)*d, o.S(2)+u(2)*d, o.z0 + o.A*sin(2*pi*o.t/o.Tz)];
end
end

function [nxt, o] = step_linear(cur, o)
u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
nxt = cur + o.dir*o.speed*u;
if o.dir == 1
    if dot(nxt - o.E, o.E - o.S) >= 0, o.dir = -1; nxt = o.E; end
else
    if dot(nxt - o.S, o.S - o.E) >= 0, o.dir = 1; nxt = o.S; end
end
end

function [nxt, o] = step_circular(cur, o) %#ok<INUSD>
o.theta = o.theta + o.omega;
nxt = [o.center(1)+o.R*cos(o.theta), o.center(2)+o.R*sin(o.theta), o.center(3)];
end

function [nxt, o] = step_variable(cur, o)
o.t = o.t + 1;
spd = o.v0*(1 + o.amp*sin(2*pi*o.t/o.period));
u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
nxt = cur + o.dir*spd*u;
if o.dir == 1
    if dot(nxt - o.E, o.E - o.S) >= 0, o.dir = -1; nxt = o.E; end
else
    if dot(nxt - o.S, o.S - o.E) >= 0, o.dir = 1; nxt = o.S; end
end
end

function [nxt, o] = step_vertical(cur, o)
o.t = o.t + 1;
u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
h = cur(1:2) + o.dir*o.hspeed*u(1:2);
if o.dir == 1
    if dot(h - o.E(1:2), o.E(1:2) - o.S(1:2)) >= 0, o.dir = -1; h = o.E(1:2); end
else
    if dot(h - o.S(1:2), o.S(1:2) - o.E(1:2)) >= 0, o.dir = 1; h = o.S(1:2); end
end
z = o.z0 + o.A*sin(2*pi*o.t/o.Tz);
nxt = [h, z];
end

function c = type_color(t)
colors = [0.85 0.10 0.10;   % 1 直线: 红
          0.10 0.40 0.90;   % 2 圆弧: 蓝
          0.10 0.60 0.20;   % 3 变速: 绿
          0.90 0.50 0.10];  % 4 垂直: 橙
c = colors(t, :);
end

% ================================================================
%        全局路径跟随 + 局部绕行 + 回归 (工具函数)
% ================================================================
function q = densify_path(path, ds)
% 将折线路径按弧长间隔 ds 密集重采样, 返回稠密点集 (M x 3)
d = [0; cumsum(sqrt(sum(diff(path).^2, 2)))];
if d(end) < 1e-6; q = path; return; end
[d, ia] = unique(d, 'stable'); path = path(ia, :);
tt = (0:ds:d(end))';
if tt(end) < d(end); tt(end+1) = d(end); end
q = [interp1(d, path(:,1), tt), ...
     interp1(d, path(:,2), tt), ...
     interp1(d, path(:,3), tt)];
end

% ----------------------------------------------------------------
function p = interp_arc(glob, glob_cum, s)
% 在全局路径上按弧长 s 取点(线性插值)
s = min(max(s, 0), glob_cum(end));
p = [interp1(glob_cum, glob(:,1), s), ...
     interp1(glob_cum, glob(:,2), s), ...
     interp1(glob_cum, glob(:,3), s)];
end

% ----------------------------------------------------------------
function pos = move_towards(pos, target, spd)
% 从 pos 朝 target 移动最多 spd 距离
v = target - pos; d = norm(v);
if d <= spd || d < 1e-9
    pos = target;
else
    pos = pos + (v/d)*spd;
end
end

% ----------------------------------------------------------------
function hit = glob_lookahead_hit(glob, glob_cum, s0, dyn_pos, dyn_r_safe, look_ahead)
% 沿全局路径从弧长 s0 向前查看 look_ahead 距离, 判断是否会撞动态障碍
hit = false;
if isempty(dyn_pos); return; end
s_end = min(glob_cum(end), s0 + look_ahead);
ns = max(2, ceil((s_end - s0) / 1.5));
for s = linspace(s0, s_end, ns)
    q = interp_arc(glob, glob_cum, s);
    for k = 1:size(dyn_pos,1)
        if norm(q - dyn_pos(k,:)) <= dyn_r_safe
            hit = true; return;
        end
    end
end
end

% ----------------------------------------------------------------
function s_new = regain_arc(glob, glob_cum, uav_pos, s_cur, dyn_pos, dyn_r_safe, look_ahead)
% 障碍避开后, 在全局路径上(当前进度附近向前)寻找“离无人机最近且前方畅通”
% 的回归点弧长。优先回到不倒退的位置, 保证沿原路径继续前进。
best_s = s_cur; best_d = inf;
s_from = s_cur;                       % 不允许倒退
s_to   = min(glob_cum(end), s_cur + look_ahead + 20);
for s = linspace(s_from, s_to, 40)
    p = interp_arc(glob, glob_cum, s);
    if norm(p - uav_pos) < best_d && ...
       ~glob_lookahead_hit(glob, glob_cum, s, dyn_pos, dyn_r_safe, look_ahead)
        best_d = norm(p - uav_pos);
        best_s = s;
    end
end
s_new = best_s;
end

% ----------------------------------------------------------------
function wp = pick_detour_waypoint(uav_pos, aim, goal, world, dyn_pos, dyn_r_safe, spd)
% 局部绕行: 在“朝 aim 前进”的基础上叠加若干候选侧向/垂直偏移, 选一个
% 一步内不撞任何障碍(静态+动态)、且尽量朝目标推进的临时航点。
wp = [];

fwd = aim - uav_pos;
if norm(fwd) < 1e-6; fwd = goal - uav_pos; end
if norm(fwd) < 1e-6; return; end
fwd = fwd / norm(fwd);

% 构造与前进方向垂直的两个基向量(侧向 right、垂直 up)
up0 = [0 0 1];
right = cross(fwd, up0);
if norm(right) < 1e-6; right = [1 0 0]; end
right = right / norm(right);
up = cross(right, fwd); up = up / (norm(up) + 1e-9);

% 候选偏移(单位: 距离), 覆盖左右/上下及其组合, 幅度由小到大
amp = [8 14 20];
dirs = [ right; -right; up; -up; ...
         (right+up); (right-up); (-right+up); (-right-up) ];
dirs = dirs ./ vecnorm(dirs, 2, 2);

best_score = -inf;
for a = amp
    for j = 1:size(dirs,1)
        cand = uav_pos + fwd*spd + dirs(j,:)*a;   % 前进一步 + 侧移
        if ~in_bounds(cand, world); continue; end
        % 一步内路径(uav_pos -> cand)需对静态+动态障碍都安全
        if ~seg_free_dyn(uav_pos, cand, world, dyn_pos, dyn_r_safe); continue; end
        % 评分: 越靠近目标越好, 偏移越小越好
        score = -norm(cand - goal) - 0.3*a;
        if score > best_score
            best_score = score; wp = cand;
        end
    end
    if ~isempty(wp); break; end   % 优先采用幅度最小的可行偏移
end
end

% ----------------------------------------------------------------
function tf = seg_free_dyn(a, b, world, dyn_pos, dyn_r_safe)
% 线段 a->b 对静态障碍(含安全距离)与动态障碍(dyn_r_safe)均无碰撞
L = norm(b - a);
n = max(2, ceil(L / 1.5));
t = linspace(0, 1, n)';
pts = a + t .* (b - a);
for i = 1:n
    if dist_to_obstacles(pts(i,:), world) <= world.safe
        tf = false; return;
    end
    for k = 1:size(dyn_pos,1)
        if norm(pts(i,:) - dyn_pos(k,:)) <= dyn_r_safe
            tf = false; return;
        end
    end
end
tf = true;
end


% ================================================================
%                    AVSS-RRT 核心算法 (含动态障碍)
% ================================================================
function [path, tree, success] = avss_rrt_plan(start, goal, world, params)
% AVSS-RRT 主循环 (碰撞检测同时考虑静态障碍与 world.dyn 动态障碍)
tree.nodes  = start;
tree.parent = 0;
success = false;
path = [];

for it = 1:params.max_iter
    % ---- (1) Goal-bias 采样 ----
    if rand < params.p_goal
        x_rand = goal;
    else
        x_rand = sample_free(world);
    end

    % ---- (2) 最近节点 ----
    [x_near, idx_near] = nearest_node(tree.nodes, x_rand);

    % ---- (3) 自适应可变步长 (AVSS) ----
    step = adaptive_step(x_near, goal, world, params);

    % ---- (4) 向采样点扩展 ----
    dir = x_rand - x_near;
    dist = norm(dir);
    if dist < 1e-6; continue; end
    x_new = x_near + (dir / dist) * min(step, dist);

    if ~in_bounds(x_new, world); continue; end

    % ---- (5) 边碰撞检测(含安全距离机体约束) ----
    if ~collision_free(x_near, x_new, world, params.edge_res)
        continue;
    end

    % ---- (6) 加入新节点 ----
    tree.nodes(end+1,:)  = x_new;
    tree.parent(end+1,1) = idx_near;

    % ---- (7) 尝试直连终点 ----
    if norm(x_new - goal) <= params.goal_tol
        if collision_free(x_new, goal, world, params.edge_res)
            tree.nodes(end+1,:)  = goal;
            tree.parent(end+1,1) = size(tree.nodes,1)-1;
            success = true;
            break;
        end
    end
end

if success
    idx = size(tree.nodes,1);
    while idx ~= 0
        path = [tree.nodes(idx,:); path]; %#ok<AGROW>
        idx = tree.parent(idx);
    end
end
end

% ----------------------------------------------------------------
function step = adaptive_step(x, goal, world, params)
% 自适应可变步长: 综合"离最近障碍距离"与"离目标距离"计算步长
d_obs  = dist_to_obstacles(x, world);
d_goal = norm(x - goal);
r_obs  = min(max((d_obs - world.safe) / params.step_max, 0), 1);
step = params.step_min + (params.step_max - params.step_min) * r_obs;
step = min(step, max(d_goal, params.step_min));
end

% ----------------------------------------------------------------
function x = sample_free(world)
for k = 1:50
    x = [world.xlim(1) + diff(world.xlim)*rand, ...
         world.ylim(1) + diff(world.ylim)*rand, ...
         world.zlim(1) + diff(world.zlim)*rand];
    if dist_to_obstacles(x, world) > world.safe
        return;
    end
end
end

% ----------------------------------------------------------------
function [x_near, idx] = nearest_node(nodes, x)
d = sum((nodes - x).^2, 2);
[~, idx] = min(d);
x_near = nodes(idx,:);
end

% ----------------------------------------------------------------
function tf = in_bounds(x, world)
tf = x(1) >= world.xlim(1) && x(1) <= world.xlim(2) && ...
     x(2) >= world.ylim(1) && x(2) <= world.ylim(2) && ...
     x(3) >= world.zlim(1) && x(3) <= world.zlim(2);
end

% ----------------------------------------------------------------
function tf = collision_free(a, b, world, res)
L = norm(b - a);
n = max(2, ceil(L / res));
t = linspace(0, 1, n)';
pts = a + t .* (b - a);
for i = 1:n
    if dist_to_obstacles(pts(i,:), world) <= world.safe
        tf = false; return;
    end
end
tf = true;
end

% ----------------------------------------------------------------
function d = dist_to_obstacles(x, world)
% 点 x 到"最近障碍表面"的有符号距离(障碍内部为负)。
% 障碍含: 高斯山体、球形禁飞区、圆柱群, 以及当前动态障碍物(world.dyn)。
d = inf;

% ---- 高斯山体地形 ----
h = terrain_height(x(1), x(2), world);
d = min(d, x(3) - h);

% ---- 球形禁飞区 ----
s = world.sphere;
d = min(d, norm(x - s(1:3)) - s(4));

% ---- 圆柱障碍 ----
for i = 1:size(world.cylinders,1)
    c = world.cylinders(i,:);
    dr = norm(x(1:2) - c(1:2)) - world.cyl_r;
    if x(3) <= c(3)
        d_cyl = dr;
    else
        dz = x(3) - c(3);
        if dr < 0
            d_cyl = dz;
        else
            d_cyl = hypot(dr, dz);
        end
    end
    d = min(d, d_cyl);
end

% ---- 动态障碍物(球形) ----
if isfield(world, 'dyn') && ~isempty(world.dyn)
    for i = 1:size(world.dyn,1)
        d = min(d, norm(x - world.dyn(i,:)) - 5);   % 动态障碍半径 5
    end
end
end

% ----------------------------------------------------------------
function h = terrain_height(x, y, world)
h = 0;
for i = 1:size(world.mountains,1)
    m = world.mountains(i,:);
    zi = m(3) * exp(-((x - m(1))^2 + (y - m(2))^2) / (2*m(4)^2));
    h = max(h, zi);
end
end

% ================================================================
%                       路径平滑 & 工具函数
% ================================================================
function sp = smooth_path_cubic(path)
% 三次样条插值平滑, 以弦长参数化; 先剔除过近的重复点
seg = sqrt(sum(diff(path).^2, 2));
keep = [true; seg > 1e-6];
path = path(keep, :);
if size(path,1) < 3
    sp = path; return;
end
d = [0; cumsum(sqrt(sum(diff(path).^2, 2)))];
tt = linspace(0, d(end), max(200, size(path,1)*20))';
sp = [spline(d, path(:,1), tt), ...
      spline(d, path(:,2), tt), ...
      spline(d, path(:,3), tt)];
end

% ----------------------------------------------------------------
function L = path_length(p)
L = sum(sqrt(sum(diff(p).^2, 2)));
end

% ================================================================
%                          绘图函数
% ================================================================
function draw_world(world)
% 复现 map.m 的三维障碍物地图(字体设置与 AVSS_RRT.m 一致)
tick_font_size  = 20;
label_font_size = 20;

map_x = 0:0.5:200;
map_y = 0:0.5:200;
[X, Y] = meshgrid(map_x, map_y);

figure('Position',[100 100 900 800], 'Color','white');

% --- 三座高斯山体 ---
for i = 1:size(world.mountains,1)
    m = world.mountains(i,:);
    Z = m(3) * exp(-((X - m(1)).^2 + (Y - m(2)).^2) / (2*m(4)^2));
    Z(Z < 0) = 0;
    surf(X, Y, Z, 'FaceColor','interp','EdgeColor','none','FaceAlpha',0.9);
    hold on;
end

% --- 球形禁飞区 ---
s = world.sphere;
[Xs, Ys, Zs] = sphere(50);
bx = s(1) + Xs*s(4);  by = s(2) + Ys*s(4);  bz = s(3) + Zs*s(4);
bz(bz < 0) = 0;
surf(bx, by, bz, 'FaceColor',[1 0 1],'EdgeColor','none','FaceAlpha',1.0);

% --- 圆柱障碍(带封顶) ---
[Xc, Yc, Zc] = cylinder(world.cyl_r, 50);
theta = linspace(0, 2*pi, 50);
for i = 1:size(world.cylinders,1)
    c = world.cylinders(i,:);
    surf(c(1)+Xc, c(2)+Yc, c(3)*Zc, ...
         'FaceColor',[1 1 0],'EdgeColor','none','FaceAlpha',1.0);
    tx = c(1) + world.cyl_r*cos(theta);
    ty = c(2) + world.cyl_r*sin(theta);
    tz = repmat(c(3), size(tx));
    fill3(tx, ty, tz, [1 1 0], 'EdgeColor','none','FaceAlpha',1.0);
end

% --- 坐标轴设置 ---
xlabel('X/m','FontSize',label_font_size);
ylabel('Y/m','FontSize',label_font_size);
zlabel('Z/m','FontSize',label_font_size);
xlim([0 200]); ylim([0 200]); zlim([0 200]);
xticks(0:20:200); yticks(0:20:200); zticks(0:20:200);
daspect([1 1 1]);
set(gca,'FontSize',tick_font_size,'Color','white');
grid on;
end

% ----------------------------------------------------------------
function draw_ball(center, r, color)
[Xs, Ys, Zs] = sphere(20);
surf(center(1)+Xs*r, center(2)+Ys*r, center(3)+Zs*r, ...
     'FaceColor',color,'EdgeColor','none','FaceAlpha',1.0);
end
