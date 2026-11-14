function [best_path, best_fitness, curve] = chaos_path_planning(model, params)
% CHAOS_PATH_PLANNING  基于混沌优化算法(COA)的三维路径规划
% -------------------------------------------------------------------------
% 参考文献:
%   "Path Planning Strategy and Performance Analysis of UAV Based on
%    Chaotic Optimization Algorithm" (CAIBDA 2025)
%
% 算法要点（严格对应论文 III 节）:
%   A. 初始种群: Logistic 混沌映射  x_{k+1}=4*x_k*(1-x_k)，映射到解空间
%   B. 适应度:   F = L + P  （路径总长 L + 障碍物碰撞惩罚 P）
%   C. 进化操作:
%      a) 选择   —— 锦标赛选择 (tournament selection)
%      b) 交叉   —— 混沌交叉  O1=α·P1+(1-α)·P2, O2=(1-α)·P1+α·P2
%      c) 变异   —— 混沌变异  x' = x + δ·map_dimension
% -------------------------------------------------------------------------
% 输入:
%   model  - map_model() 返回的地图模型
%   params - 算法参数结构体（见 path_planning_main.m）
% 输出:
%   best_path    - (num_points+2)×3 的最优路径（含起点/终点）
%   best_fitness - 最优适应度
%   curve        - 每代最优适应度收敛曲线

start = model.start;
goal  = model.goal;

pop_size   = params.pop_size;      % 种群规模
num_points = params.num_points;    % 每条路径的中间航点数
max_iter   = params.max_iter;      % 最大迭代次数
dim        = num_points * 3;       % 决策变量维度（每航点 x,y,z）

lo = model.map_min;                % 解空间下界
hi = model.map_max;                % 解空间上界
span = hi - lo;                    % 各维度跨度 map_dimension

% =========================================================================
% A. 初始种群生成 —— Logistic 混沌映射
% =========================================================================
% 用 Logistic 映射产生 pop_size*dim 个 [0,1] 混沌变量，再线性映射到解空间
chaos = logistic_sequence(rand(), pop_size * dim);
chaos = reshape(chaos, pop_size, dim);
pop = zeros(pop_size, dim);
for j = 1:dim
    d = mod(j - 1, 3) + 1;        % 该列对应 x/y/z 维
    pop(:, j) = lo(d) + chaos(:, j) * span(d);
end

% 计算初始适应度
fitness = zeros(pop_size, 1);
for i = 1:pop_size
    fitness(i) = path_fitness(decode(pop(i, :), num_points, start, goal), start, goal, model, params);
end
[best_fitness, idx] = min(fitness);
best_ind = pop(idx, :);

curve = zeros(max_iter, 1);

% 维护一个全局混沌流，供交叉/变异使用（论文中 α、δ 均由混沌映射产生）
cx_state = rand();

% =========================================================================
% C. 进化迭代
% =========================================================================
for it = 1:max_iter
    new_pop = zeros(pop_size, dim);

    % 精英保留：最优个体直接进入下一代（保证收敛稳定）
    new_pop(1, :) = best_ind;
    count = 1;

    while count < pop_size
        % ---- a) 锦标赛选择两个父代 ----
        p1 = tournament(pop, fitness, params.tournament_size);
        p2 = tournament(pop, fitness, params.tournament_size);

        % ---- b) 混沌交叉 ----
        if rand() < params.pc
            [cx_state, alpha] = logistic_next(cx_state);   % α 来自混沌映射
            o1 = alpha .* p1 + (1 - alpha) .* p2;
            o2 = (1 - alpha) .* p1 + alpha .* p2;
        else
            o1 = p1;
            o2 = p2;
        end

        % ---- c) 混沌变异 ----
        [cx_state, o1] = chaos_mutation(cx_state, o1, span, lo, params.pm);
        [cx_state, o2] = chaos_mutation(cx_state, o2, span, lo, params.pm);

        % 边界约束
        o1 = clamp(o1, lo, hi);
        o2 = clamp(o2, lo, hi);

        count = count + 1;
        new_pop(count, :) = o1;
        if count < pop_size
            count = count + 1;
            new_pop(count, :) = o2;
        end
    end

    pop = new_pop;

    % ---- 重新评估适应度 ----
    for i = 1:pop_size
        fitness(i) = path_fitness(decode(pop(i, :), num_points, start, goal), start, goal, model, params);
    end

    % ---- 更新全局最优 ----
    [gen_best, idx] = min(fitness);
    if gen_best < best_fitness
        best_fitness = gen_best;
        best_ind = pop(idx, :);
    end

    curve(it) = best_fitness;

    if mod(it, 20) == 0 || it == 1
        fprintf('迭代 %4d / %4d   当前最优适应度 = %.3f\n', it, max_iter, best_fitness);
    end
end

% 组装最终路径（起点 + 中间航点 + 终点）
best_path = [start; decode(best_ind, num_points, start, goal); goal];

% =========================================================================
% D. 路径后处理 —— 拉直/高度平滑/捷径优化（消除“绕大圈、大起伏”）
% =========================================================================
% COA 得到可行路径后，做贪心优化，均在“不碰撞”的可行域内进行，
% 只优化几何形态、不改变 F=L+P 的优化目标：
%   1) 拉绳法(string-pulling)：直线可达则删去中间多余航点，去掉绕行；
%   2) 高度平滑：让各航点高度尽量贴近“起点z→终点z”的平缓基准，
%      仅在基准会撞障碍时才抬升到刚好避开的高度，抑制上下大起伏；
%   3) 再次拉绳法：整理后再拉直一次，得到更短更平顺的路径。
best_path = shortcut_path(best_path, model, params);
best_path = smooth_altitude(best_path, model, params);
best_path = shortcut_path(best_path, model, params);
best_path = smooth_altitude(best_path, model, params);

% 用后处理后的路径重新计算适应度并汇报
best_fitness = eval_path(best_path, model, params);
end

% =========================================================================
% ------------------------------ 辅助函数 ---------------------------------
% =========================================================================

function wp = decode(ind, num_points, start, goal)
% 将行向量个体解码为 num_points×3 的航点矩阵。
% 并按“起点→终点”主方向上的投影对航点排序，保证路径单调前进、
% 不出现来回折返，从而得到更短、更平滑的路径（不改变适应度 F=L+P 定义）。
wp = reshape(ind, 3, num_points)';
dir = goal - start;
if norm(dir) > 0
    proj = (wp - start) * dir(:);     % 各航点在主方向上的投影
    [~, order] = sort(proj);          % 按投影升序排序 -> 沿主方向单调前进
    wp = wp(order, :);
end
end

function f = path_fitness(waypoints, start, goal, model, params)
% 适应度 F = L + P
%   L: 路径总长（相邻航点欧氏距离之和）
%   P: 碰撞惩罚（对整条离散化路径检测碰撞点数量施加惩罚）
path = [start; waypoints; goal];
f = eval_path(path, model, params);
end

function f = eval_path(path, model, params)
% 对整条路径（含起终点）计算适应度 F = L + P
diffs = diff(path, 1, 1);
L = sum(sqrt(sum(diffs.^2, 2)));
pts = densify(path, params.check_step);
[~, n_hit] = check_collision(model, pts);
P = params.penalty * n_hit;
f = L + P;
end

function path = shortcut_path(path, model, params)
% 拉绳法(string-pulling)贪心捷径优化：
%   反复扫描，若 path(i) 与 path(j)(j>i+1) 之间直线无碰撞，则删去中间点。
%   多轮迭代直到无法再删，从而把“绕大圈”拉直成尽量短的无碰撞折线。
step = params.check_step;
changed = true;
while changed
    changed = false;
    i = 1;
    while i < size(path, 1) - 1
        % 从最远点往回找：尽量一次跨过更多中间点
        j = size(path, 1);
        removed = false;
        while j > i + 1
            if seg_free(path(i, :), path(j, :), model, step)
                path(i+1:j-1, :) = [];   % 删去 i 与 j 之间的所有点
                changed = true;
                removed = true;
                break;
            end
            j = j - 1;
        end
        if ~removed
            i = i + 1;
        end
    end
end
end

function ok = seg_free(a, b, model, step)
% 判断直线段 a->b 是否无碰撞（按步长离散采样检测）
d = norm(b - a);
n = max(2, ceil(d / step));
t = linspace(0, 1, n)';
seg = a + t .* (b - a);
ok = ~check_collision(model, seg);
end

function path = smooth_altitude(path, model, params)
% 高度平滑：抑制路径 z 方向的大起伏。
% 思路：以“起点z→终点z”沿弧长的线性插值作为期望高度基准，让每个航点
% 尽量贴近该平缓基准；只有当基准高度会与障碍/地形碰撞时，才用二分法把
% 该点抬升到刚好避开的最低安全高度。这样整体高度曲线尽量平缓单调，
% 不再“先降后升再降”。全程保持无碰撞，不改变 F=L+P 目标。
step = params.check_step;

% ---- 先在过长的段上插入中间点，提高高度调节分辨率 ----
maxseg = 15;
newpath = path(1, :);
for i = 1:size(path, 1) - 1
    a = path(i, :); b = path(i + 1, :);
    k = max(1, ceil(norm(b - a) / maxseg));
    for m = 1:k
        newpath(end + 1, :) = a + (m / k) * (b - a); %#ok<AGROW>
    end
end
path = newpath;
n = size(path, 1);

% ---- 计算沿弧长的线性高度基准 z_base ----
seg = sqrt(sum(diff(path, 1, 1).^2, 2));
s = [0; cumsum(seg)];
z_start = path(1, 3);
z_goal  = path(end, 3);
z_base = z_start + (z_goal - z_start) * (s / s(end));

% ---- 逐个中间航点：先设为基准高度，撞障碍则抬升到最低安全高度 ----
for i = 2:n - 1
    target = z_base(i);
    zfloor = terrain_height(model, path(i, 1), path(i, 2)) + model.safe_margin + 0.5;
    target = max(target, zfloor);

    trial = path(i, :); trial(3) = target;
    if seg_free(path(i-1, :), trial, model, step) && ...
       seg_free(trial, path(i+1, :), model, step)
        path(i, 3) = target;                 % 基准高度即可行，直接采用
    else
        % 基准高度会碰撞：在 [target, 上界] 内二分找最低可行高度
        lo = target; hi = model.map_max(3); best = path(i, 3);
        for b = 1:20
            mid = 0.5 * (lo + hi);
            trial(3) = mid;
            if seg_free(path(i-1, :), trial, model, step) && ...
               seg_free(trial, path(i+1, :), model, step)
                best = mid; hi = mid;        % 可行，尝试更低
            else
                lo = mid;                    % 不可行，抬高
            end
        end
        path(i, 3) = best;
    end
end

% ---- 局部均值平滑：进一步削平残余的小起伏（保证仍无碰撞、不低于安全高度）----
for pass = 1:3
    for i = 2:n - 1
        cand = (path(i-1, 3) + path(i+1, 3)) / 2;       % 邻域平均高度
        zfloor = terrain_height(model, path(i, 1), path(i, 2)) + model.safe_margin + 0.5;
        cand = max(cand, zfloor);
        trial = path(i, :); trial(3) = cand;
        if seg_free(path(i-1, :), trial, model, step) && ...
           seg_free(trial, path(i+1, :), model, step)
            path(i, 3) = cand;
        end
    end
end
end


function pts = densify(path, step)
% 将折线路径按近似步长 step 线性插值加密，得到密集采样点用于碰撞检测
pts = path(1, :);
for i = 1:size(path, 1) - 1
    a = path(i, :);
    b = path(i + 1, :);
    d = norm(b - a);
    n = max(2, ceil(d / step));
    t = linspace(0, 1, n)';
    seg = a + t .* (b - a);
    pts = [pts; seg(2:end, :)]; %#ok<AGROW>
end
end

function p = tournament(pop, fitness, k)
% 锦标赛选择：随机挑 k 个个体，返回适应度最小者
n = size(pop, 1);
idx = randi(n, k, 1);
[~, best] = min(fitness(idx));
p = pop(idx(best), :);
end

function [state, child] = chaos_mutation(state, ind, span, lo, pm)
% 混沌变异：每个基因以概率 pm 变异，新值 = lo + δ*span（δ 来自混沌映射）
child = ind;
for g = 1:numel(ind)
    [state, r] = logistic_next(state);
    if r < pm
        [state, delta] = logistic_next(state);
        d = mod(g - 1, 3) + 1;                 % 该基因所属维度
        child(g) = lo(d) + delta * span(d);    % 论文式(5) 的思想
    end
end
end

function y = clamp(x, lo, hi)
% 将个体各维约束到 [lo,hi]（lo/hi 为 1×3，对 x/y/z 循环使用）
y = x;
for g = 1:numel(x)
    d = mod(g - 1, 3) + 1;
    y(g) = min(max(x(g), lo(d)), hi(d));
end
end

function s = logistic_sequence(x0, n)
% 生成长度为 n 的 Logistic 混沌序列  x_{k+1}=4*x_k*(1-x_k)
x = x0;
if x <= 0 || x >= 1, x = 0.3; end
% 避免落在不动点上
if abs(x - 0.25) < 1e-6, x = 0.3; end
s = zeros(1, n);
for k = 1:n
    x = 4 * x * (1 - x);
    s(k) = x;
end
end

function [state, val] = logistic_next(state)
% Logistic 映射的单步迭代，返回下一个混沌值
if state <= 0 || state >= 1, state = 0.3; end
state = 4 * state * (1 - state);
val = state;
end
