function dyn = gen_obstacles_v2(seed, path, env, type)
%GEN_OBSTACLES_V2 生成 14 个四类动态障碍物（直线×3 圆弧×3 变速×4 垂直×4）。
% 沿参考路径弧长均匀交错布置，非直线类型占多数（回应 R1-2 只测直线、R3-5 仅水平）。
%
% 类型编码（dyn.obs{i}.type）：
%   1 直线匀速   沿 S<->E 水平往返，恒定速度（分慢/中/快三档）
%   2 匀速圆弧   圆心偏移到路径一侧，水平面内匀速圆周运动（破匀速直线假设）
%   3 变速直线   沿 S<->E 往返，速度正弦调制（破匀速假设）
%   4 垂直运动   水平慢速穿越 + 垂直正弦振荡（破"仅水平"假设）
%
% 数据结构：dyn.num / dyn.r / dyn.curr(num×3) 与旧版兼容；新增 dyn.obs{num} 元胞，
% 每个元素为该障碍物的类型+参数+演化状态。move_obstacles 按 type 分派推进。
if nargin < 4 || isempty(type), type = 0; end   % 0=混合14个；1~4=单类型10个（类型分组实验）
rng(seed, 'twister');

if type == 0
    n = 14;
else
    n = 10;
end
dyn.num   = n;
dyn.r     = 5;
dyn.speed = 1.0;          % 名义速度（新结构不直接用，仅保留字段兼容）
dyn.curr  = zeros(n, 3);
dyn.obs   = cell(n, 1);

% 弧长参数化（去重保证 interp1 可用）
seg = sqrt(sum(diff(path) .^ 2, 2));
cum = [0; cumsum(seg)];
[cum, ia] = unique(cum, 'stable');
path = path(ia, :);
L = cum(end);
if L < 30
    error('gen_obstacles_v2: 参考路径过短，无法布置动态障碍物');
end

if type == 0
    % 14 个遭遇点弧长比例 + 类型（交错排列，避免同类扎堆）
    fracs = [0.20 0.24 0.29 0.33 0.37 0.41 0.46 0.50 0.54 0.59 0.63 0.67 0.71 0.76];
    types = [1 2 3 4 3 1 4 2 3 4 1 2 3 4];
else
    % 单类型：10 个同类型障碍，弧长均匀分布
    fracs = linspace(0.25, 0.75, 10);
    types = type * ones(1, 10);
end

% 直线障碍速度三档（慢 0.5 / 中 1.0 / 快 1.5 m/步），按出现顺序分配
lin_speeds = [0.5 1.0 1.5];
lin_idx = 0;

for i = 1:n
    si = fracs(i) * L;
    p  = arc_interp(cum, path, si);

    % 路径切向（邻近弧长点差分）
    s1 = max(cum(1), si - 3);
    s2 = min(cum(end), si + 3);
    tan = arc_interp(cum, path, s2) - arc_interp(cum, path, s1);
    if norm(tan) < 1e-6
        tan = [1 0 0];
    end
    tan = tan / norm(tan);

    % 水平横向方向（垂直于切向水平投影）
    up = [0 0 1];
    side = cross(tan, up);
    if norm(side) < 1e-6
        side = cross(tan, [0 1 0]);
    end
    side(3) = 0;
    if norm(side) < 1e-6
        side = [1 0 0];
    end
    side = side / norm(side);

    switch types(i)
        case 1
            lin_idx = lin_idx + 1;
            dyn.obs{i} = make_linear(p, side, lin_speeds(mod(lin_idx - 1, 3) + 1));
        case 2
            dyn.obs{i} = make_circular(p, side);
        case 3
            dyn.obs{i} = make_variable(p, side);
        case 4
            dyn.obs{i} = make_vertical(p, side);
    end
    dyn.curr(i, :) = initial_position(dyn.obs{i});
end
end

% ================= 各类障碍构造 =================
function o = make_linear(p, side, speed)
half_w = 25 + 10 * rand;          % 横向穿越半宽 25~35m
S = p - side * half_w; S(3) = p(3);
E = p + side * half_w; E(3) = p(3);
o.type = 1;
o.S = S; o.E = E;
o.dir = sign(rand - 0.5); if o.dir == 0, o.dir = 1; end
o.speed = speed;
end

function o = make_circular(p, side)
R  = 12 + 6 * rand;               % 轨道半径 12~18m
vt = 0.8 + 0.7 * rand;            % 切向速度 0.8~1.5 m/步
o.type = 2;
% 圆心偏移 0.6R，使圆周扫过走廊（最近点 ~0.4R < r_safe=7.5m，确实威胁路径）
o.center = p + side * (0.6 * R);
o.center(3) = p(3);
o.R = R;
o.omega = vt / R;
o.theta = 2 * pi * rand;          % 随机初始相位
end

function o = make_variable(p, side)
half_w = 25 + 10 * rand;
S = p - side * half_w; S(3) = p(3);
E = p + side * half_w; E(3) = p(3);
o.type = 3;
o.S = S; o.E = E;
o.dir = sign(rand - 0.5); if o.dir == 0, o.dir = 1; end
o.v0     = 0.5 + 0.5 * rand;      % 基准速度 0.5~1.0 m/步
o.amp    = 0.5;                   % 速度调制幅度
o.period = 20 + 20 * rand;        % 调制周期 20~40 步
o.t      = rand * o.period;       % 随机初始相位
end

function o = make_vertical(p, side)
half_w = 15 + 10 * rand;          % 水平穿越半宽 15~25m（比直线略窄）
S = p - side * half_w; S(3) = p(3);
E = p + side * half_w; E(3) = p(3);
o.type = 4;
o.S = S; o.E = E;
o.dir = sign(rand - 0.5); if o.dir == 0, o.dir = 1; end
o.hspeed = 0.5;                   % 水平速度
o.z0     = p(3);                  % 基准高度 = 路径高度
o.A      = 15 + 10 * rand;        % 垂直振幅 15~25m
o.Tz     = 30 + 20 * rand;        % 垂直振荡周期 30~50 步
o.t      = rand * o.Tz;
end

% ================= 初始位置 =================
function pos = initial_position(o)
switch o.type
    case 1
        u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
        pos = o.S + u * (rand * norm(o.E - o.S));
    case 2
        pos = [o.center(1) + o.R * cos(o.theta), ...
               o.center(2) + o.R * sin(o.theta), o.center(3)];
    case 3
        u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
        pos = o.S + u * (rand * norm(o.E - o.S));
    case 4
        u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
        d = rand * norm(o.E - o.S);
        pos = [o.S(1) + u(1) * d, o.S(2) + u(2) * d, ...
               o.z0 + o.A * sin(2 * pi * o.t / o.Tz)];
end
end

function p = arc_interp(cum, path, s)
s = min(max(s, cum(1)), cum(end));
p = [interp1(cum, path(:, 1), s), ...
     interp1(cum, path(:, 2), s), ...
     interp1(cum, path(:, 3), s)];
end
