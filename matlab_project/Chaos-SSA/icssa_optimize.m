function [best_path, best_cost, stats_best, curve] = icssa_optimize(model, params)
% ICSSA_OPTIMIZE  改进混沌麻雀搜索算法(ICSSA)三维路径规划
% -------------------------------------------------------------------------
% 参考: "An improved chaos sparrow search algorithm for UAV path planning",
%       Scientific Reports, 2024.
% 四大改进:
%   1) PWLCM 混沌映射初始化种群(Eq.4, p=0.4);
%   2) 非线性动态权重因子 wd 改进发现者更新(Eq.5~6);
%   3) 改进正弦余弦算法改进加入者更新(Eq.7~8);
%   4) 透镜成像反向学习(动态边界, k=0.2+t/M, 贪心保留)(Eq.12~15)。
% 标准 SSA 三角色: 发现者 / 加入者 / 警戒者。
% -------------------------------------------------------------------------
N      = params.N;          % 种群规模
K      = params.K;          % 中间航点数
M      = params.M;          % 最大迭代次数
n_pts  = params.n_pts;      % 路径采样点数
safe_d = params.safe_dist;  % 威胁安全阈值(m)

Dim = 3 * K;
lb0 = model.bound(1); ub0 = model.bound(2);
lbz = model.bound(3); ubz = model.bound(4);
lb  = repmat([lb0 lb0 lbz], 1, K);
ub  = repmat([ub0 ub0 ubz], 1, K);

PD  = 0.2;                  % 发现者比例
SD  = 0.2;                  % 警戒者比例
ST  = 0.8;                  % 安全阈值
pNum = max(2, round(N * PD));
sNum = round(N * SD);

% ---- 1. PWLCM 混沌映射初始化种群 ----
pop = pwlcm_init(N, Dim, lb, ub);

% 有知初始化: 若提供可行种子走廊, 则用"种子 + 有界扰动"重构整个初始种群,
% 使初始解均落在无碰撞走廊附近, 避免随机初始化造成穿楼/贴面/终点乱转。
if isfield(params, 'init_wps') && ~isempty(params.init_wps)
    seed_vec = reshape(params.init_wps', 1, Dim);   % Kx3 -> 1x3K
    for i = 1:N
        noise = (2 * rand(1, Dim) - 1) .* (ub - lb) * 0.12;
        pop(i, :) = min(max(seed_vec + noise, lb), ub);
    end
    pop(1, :) = seed_vec;   % 令种群首个体即为纯种子
end

cost = evaluate_pop(pop, model, n_pts, safe_d, Dim, K);

curve = zeros(M, 1);
best_cost  = inf;
best_path  = [];
stats_best = zeros(1, 6);

for t = 1:M
    % ---- 按适应度升序排列（发现者在前） ----
    [cost, idx] = sort(cost); pop = pop(idx, :);
    Xbest     = pop(1, :);
    Xworst    = pop(N, :);
    fb = cost(1);
    fw = cost(N);

    Q   = randn(1, Dim);
    a1  = rand;                       % α
    wd  = (exp(4 * (1 - t / M)) - 1) / (exp(4 * (1 - t / M)) + 1);   % Eq.5 动态权重
    R2  = rand();

    newPop = pop;

    % ---- 2. 发现者更新(Eq.6) ----
    for i = 1:pNum
        if R2 < ST
            newPop(i, :) = pop(i, :) .* wd .* exp(-i / (a1 * M));
        else
            newPop(i, :) = pop(i, :) + Q;
        end
    end

    % ---- 3. 加入者更新(Eq.7~8, 改进正弦余弦) ----
    r1 = 2 * exp(-30 * (t / M)^5);    % Eq.8 线性衰减
    for i = pNum + 1:N
        r2 = 2 * rand; r3 = 2 * rand; r4 = rand;
        if r4 <= 0.5
            newPop(i, :) = Xbest + r1 * sin(r2) .* abs(r3 * Xbest - pop(i, :));
        else
            newPop(i, :) = Xbest + r1 * cos(r2) .* abs(r3 * Xbest - pop(i, :));
        end
    end

    % ---- 4. 警戒者更新(Eq.3) ----
    for i = N - sNum + 1:N
        beta = randn;
        fi = cost(i);
        if fi > fb
            newPop(i, :) = Xbest + abs(pop(i, :) - Xbest) .* beta;
        else
            kk = -1 + 2 * rand;
            newPop(i, :) = pop(i, :) + kk .* (abs(pop(i, :) - Xbest) ./ (fi - fw + eps));
        end
    end

    % ---- 边界裁剪 ----
    newPop = min(max(newPop, lb), ub);

    % ---- 评估新种群 ----
    newCost = evaluate_pop(newPop, model, n_pts, safe_d, Dim, K);
    pop = newPop; cost = newCost;

    % ---- 5. 透镜成像反向学习 + 贪心(Eq.12~15) ----
    [ct, bi] = min(cost); Xi = pop(bi, :);
    k = 0.2 + t / M;                    % Eq.12 线性增大
    a = min(pop); b = max(pop);         % 动态边界 Eq.14
    xhat = (a + b) / 2 + (a + b) / (2 * k) - Xi ./ k;
    xhat = min(max(xhat, lb), ub);
    ch = objval(xhat, model, n_pts, safe_d);
    if ch < ct
        pop(bi, :) = xhat; cost(bi) = ch;
        ct = ch; Xi = xhat;
    end

    % ---- 记录全局最优 ----
    if ct < best_cost
        best_cost  = ct;
        best_path  = Xi;
        stats_best = objvalStats(Xi, model, n_pts, safe_d);
    end
    curve(t) = best_cost;
end

% ---- 组装输出路径 ----
wps = reshape(best_path, 3, K)';
best_path = rebuild_path(model.start, wps, model.goal, n_pts);
end

% =========================================================================
%---------------------------- 子函数 -------------------------------------
% =========================================================================
function pop = pwlcm_init(N, Dim, lb, ub)
% PWLCM 混沌映射(Eq.4)初始化种群，p=0.4。
p = 0.4;
pop = zeros(N, Dim);
for i = 1:N
    x = rand;
    for j = 1:Dim
        x = pw(x, p);
        pop(i, j) = x;
    end
end
% 映射到 [lb, ub]
pop = lb + (ub - lb) .* pop;
end

function y = pw(x, p)
% 分段线性混沌映射 Eq.4
if x < p
    y = x / p;
elseif x < 0.5
    y = (x - p) / (0.5 - p);
elseif x < 1 - p
    y = (1 - p - x) / (0.5 - p);
else
    y = (1 - x) / p;
end
end

function cost = evaluate_pop(pop, model, n_pts, safe_d, Dim, K)
N = size(pop, 1);
cost = zeros(N, 1);
for i = 1:N
    cost(i) = objval(pop(i, :), model, n_pts, safe_d);
end
end

function c = objval(x, model, n_pts, safe_d)
% x 为一维航点向量(1×3K)，内部 reshape 为 K×3 后评估
K = numel(x) / 3;
wps = reshape(x(:), 3, K)';
c = evaluate_path(wps, model, n_pts, safe_d);
end

function s = objvalStats(x, model, n_pts, safe_d)
K = numel(x) / 3;
wps = reshape(x(:), 3, K)';
[~, s] = evaluate_path(wps, model, n_pts, safe_d);
end

function P = rebuild_path(start, waypoints, goal, n_pts)
% 累积弦长参数化 + pchip 平滑(去除连续重复航点, 保证断点严格递增)。
full = [start; waypoints; goal];
seg = sqrt(sum(diff(full, 1, 1).^2, 2));
keep = [true; seg > 1e-6];
full = full(keep, :);
if size(full, 1) < 2
    tt = linspace(0, 1, n_pts)';
    P  = start + (goal - start) .* tt;
    return;
end
seg = sqrt(sum(diff(full, 1, 1).^2, 2));
seg = max(seg, 1e-6);
t = [0; cumsum(seg)];
tt = linspace(t(1), t(end), n_pts);
P = zeros(n_pts, 3);
for d = 1:3
    P(:, d) = pchip(t, full(:, d), tt);
end
end