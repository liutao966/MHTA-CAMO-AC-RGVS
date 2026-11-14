function [arc_paths, arc_f, hist_front, hist_arc] = hsma_multiobjective(model, params)
% HSMA_MULTIOBJECTIVE  混合黏菌算法(HSMA)多目标三维路径规划
% -------------------------------------------------------------------------
% 原理（参考文献: IEEE Trans. AES 2024, Abdel-Basset et al.）:
%   HSMA 将黏菌算法 SMA 与一种三阶段混合更新策略杂交:
%     当 rand < t/tmax —— 执行 SMA 更新(接近/包裹/抓取食物, Eq.12~18);
%     否则执行混合更新策略(三阶段):
%        Stage-1 (Eq.21): Xi 与随机个体的中点       —— 探索;
%        Stage-2 (Eq.22): Xb + gamma*(Xb-Xi)+(Xr-Xa)*eta —— 定向搜索,
%                        gamma = 1/(1+e^l), l 从 -1 线性降至 -2;
%        Stage-3 (Eq.24): Xb*r2 - Xi*r3*beta*eta + Xb (借鉴 GTO, beta=3) —— 开发;
%   迭代中按 Pareto 支配筛选个体, 外部存档 Arc 保存非支配解。
% 输出:
%   arc_paths : cell，每个元素为非支配解对应的平滑路径 N×3
%   arc_f     : (nArc×nObj) 非支配解的目标值
%   hist_front: (tmax×1) 每代种群中非支配解个数
%   hist_arc  : (tmax×1) 每代存档规模
% -------------------------------------------------------------------------
N        = params.N;          % 种群规模
K        = params.K;          % 中间航点数
tmax     = params.tmax;       % 最大迭代次数
n_pts    = params.n_pts;      % 路径采样点数
safe_d   = params.safe_dist;  % 威胁安全阈值(m)
z_prob   = 0.03;              % SMA 随机重初始化概率
beta     = 3;                 % Stage-3 GTO 系数
max_arc  = 80;                % 存档容量上限

start = model.start;
goal  = model.goal;
lb0   = model.bound(1); ub0 = model.bound(2);
lbz   = model.bound(3); ubz = model.bound(4);
Dim   = 3 * K;
lb    = repmat([lb0 lb0 lbz], 1, K);
ub    = repmat([ub0 ub0 ubz], 1, K);

% ---- 初始化种群（航点向量）----
pop = lb + (ub - lb) .* rand(N, Dim);

% 有知初始化: 若提供可行种子走廊, 则用"种子 + 有界扰动"重构整个初始种群，
% 使初始解均落在无碰撞飞行走廊附近, 避免随机初始化造成穿楼/贴面/终点乱转。
if isfield(params, 'init_wps') && ~isempty(params.init_wps)
    seed_vec = reshape(params.init_wps', 1, Dim);   % Kx3 -> 1x3K
    for i = 1:N
        noise = (2 * rand(1, Dim) - 1) .* (ub - lb) * 0.12;
        pop(i, :) = min(max(seed_vec + noise, lb), ub);
    end
    pop(1, :) = seed_vec;   % 令种群首个体即为纯种子, 保证存档必有可行路径
end

% ---- 目标评估 ----
[F, ~] = evaluate_pop(pop, model, n_pts, safe_d, Dim, K);
[~, bestIdx]  = min(scalar_rank(F));
Xb            = pop(bestIdx, :);

% ---- 外部存档初始化：初始种群非支配解 ----
Arc = pop;
ArcF = F;

hist_front = zeros(tmax, 1);
hist_arc   = zeros(tmax, 1);

% ========================== 主优化循环 ==========================
for t = 1:tmax
    r = rand;
    if r < t / tmax
        % ---------------- 分支 A: SMA 更新 ----------------
        S  = scalar_rank(F);
        DF = min(S);
        [Ssort, Iord] = sort(S);
        bF = Ssort(1); wF = Ssort(end);
        % 权重 W (Eq.16)
        W = zeros(N, 1);
        for i = 1:N
            ratio = (bF - Ssort(i)) / (bF - wF + 1e-9) + 1;
            if i <= N / 2
                W(Iord(i)) = 1 + rand * log(ratio);
            else
                W(Iord(i)) = 1 - rand * log(ratio);
            end
        end
        p = tanh(abs(S - DF));            % 控制参数 p (Eq.13)
        a = atanh(max(1 - t / tmax, 1e-6)); % a = arctanh(1 - t/tmax)
        if isinf(a) || isnan(a), a = 5; end
        vb = -a + 2 * a * rand;           % Eq.14
        vc = 1 - t / tmax;                % Eq. 降至 0

        newPop = zeros(N, Dim);
        for i = 1:N
            r1 = rand;
            if r1 < z_prob
                newPop(i, :) = lb + (ub - lb) .* rand(1, Dim);       % 随机重初始化
            else
                [Xa, Xbnd] = sselect(pop, N, i);
                if rand < p(i)
                    newPop(i, :) = Xb + vb * W(i) * (pop(Xa, :) - pop(Xbnd, :)); % Eq.18 包裹
                else
                    newPop(i, :) = vc .* pop(i, :);                             % 抓取/收缩
                end
            end
        end
    else
        % ---------------- 分支 B: 混合三阶段更新 ----------------
        l    = -1 - (t / tmax);        % l 从 -1 线性降至 -2
        gamma = 1 / (1 + exp(l));      % Eq.23
        rR = randi(N); rA = randi(N);  % 两个随机个体
        newPop = zeros(N, Dim);
        for i = 1:N
            r3 = rand; rr = rand; r1 = rand;
            if r3 < t / tmax
                % Stage-1 (Eq.21): 与随机个体取中点 —— 探索
                newPop(i, :) = 0.5 * (pop(i, :) + pop(rA, :));
            elseif rr < r1
                % Stage-2 (Eq.22): 结合当前解与随机解的差距
                newPop(i, :) = Xb + gamma * (Xb - pop(i, :)) + (pop(rR, :) - pop(rA, :)) * randn;
            else
                % Stage-3 (Eq.24): 借鉴 GTO, 强化开发
                newPop(i, :) = Xb * (2 * rand - 1) - pop(i, :) * (2 * rand - 1) * beta * randn + Xb;
            end
        end
    end

    % ---- 边界裁剪 ----
    newPop = min(max(newPop, lb), ub);

    % ---- 评估新种群 ----
    [Fnew, ~] = evaluate_pop(newPop, model, n_pts, safe_d, Dim, K);

    % ---- Pareto 支配接受: 若旧解支配新解则保留旧解 ----
    keep_old = dominates(F, Fnew);       % F(旧) 支配 Fnew(新)
    chg = ~keep_old;
    pop(chg, :) = newPop(chg, :);
    F(chg, :)   = Fnew(chg, :);

    % ---- 更新当前最优个体(按标量化排名) ----
    [~, bestIdx] = min(scalar_rank(F));
    Xb = pop(bestIdx, :);

    % ---- 更新外部存档（合并种群非支配解 + 拥挤度剪枝） ----
    [Arc, ArcF] = update_archive(Arc, ArcF, pop, F, max_arc);

    hist_front(t) = sum(non_dominated_mask(F));
    hist_arc(t)   = size(Arc, 1);
end

% ---- 返回存档为平滑路径 ----
nArc = size(Arc, 1);
arc_paths = cell(1, nArc);
for k = 1:nArc
    wps = reshape(Arc(k, :), 3, K)';
    [~, arc_paths{k}] = evaluate_path(wps, model, n_pts, safe_d);
end
arc_f = ArcF;
end

% =========================================================================
%---------------------------- 子函数 -------------------------------------
% =========================================================================
function [F, cache] = evaluate_pop(pop, model, n_pts, safe_d, Dim, K)
% 评估种群所有个体，cache 用于返回中间航点（可调用，此处用于对齐接口）。
N = size(pop, 1);
wps_tmp = reshape(pop(1, :), 3, K)';
[f_tmp, ~] = evaluate_path(wps_tmp, model, n_pts, safe_d);
nObj = numel(f_tmp);
F = zeros(N, nObj);
cache = cell(1, N);
F(1, :) = f_tmp;
cache{1} = wps_tmp;
for i = 2:N
    wps = reshape(pop(i, :), 3, K)';
    cache{i} = wps;
    F(i, :) = evaluate_path(wps, model, n_pts, safe_d);
end
end

function S = scalar_rank(F)
% 将多目标归一化为单一标量排名(用于 SMA 权重排序 / 选择当前最优)。
rng_c = max(F) - min(F); rng_c(rng_c < 1e-9) = 1;
Fnorm = (F - min(F)) ./ rng_c;
S = sum(Fnorm, 2);
end

function [a, b] = sselect(pop, N, i)
% 选择两个不同于 i 的随机个体下标。
a = randi(N); b = randi(N);
while a == i, a = randi(N); end
while b == i || b == a, b = randi(N); end
end

function m = dominates(Fa, Fb)
% 向量化: m(k)=1 表示第 k 行 Fa 支配 Fb。(多目标最小化)
% 支配条件: 所有分量 <= 且至少一个 < 。
all_le = all(Fa <= Fb, 2);
any_lt = any(Fa < Fb, 2);
m = all_le & any_lt;
end

function mask = non_dominated_mask(F)
% 返回当前种群中非支配解的掩码（最小化）。
N = size(F, 1);
mask = true(N, 1);
for i = 1:N
    others = all(bsxfun(@le, F, F(i, :)), 2);
    mask(i) = (sum(others & (sum(bsxfun(@lt, F, F(i, :)), 2) > 0))) == 0;
end
end

function [Arc, ArcF] = update_archive(Arc, ArcF, pop, F, max_arc)
% 将当前种群的非支配解并入存档，去重后再做非支配筛选；若超过容量按拥挤度剪枝。
push = non_dominated_mask(F);
allA = [Arc; pop(push, :)];   %#ok<AGROW>
allF = [ArcF; F(push, :)];    %#ok<AGROW>

% 按目标向量去重(保留首次出现)，再求 Pareto 非支配集
[unqF, ia] = unique(allF, 'rows', 'stable');
tmpA = allA(ia, :);
md   = non_dominated_mask(unqF);
Arc  = tmpA(md, :);
ArcF = unqF(md, :);

if size(Arc, 1) > max_arc
    cd = crowding_distance(ArcF);
    [~, keep] = sort(cd, 'descend');
    keep = keep(1:max_arc);
    Arc  = Arc(keep, :);
    ArcF = ArcF(keep, :);
end
end

function cd = crowding_distance(F)
% 拥挤度距离（目标空间密度，用于存档剪枝时保持前沿均匀）。
N = size(F, 1);
nObj = size(F, 2);
if N <= 2
    cd = ones(N, 1);
    return;
end
cd = zeros(N, 1);
Fn = F;
for o = 1:nObj
    [Fs, idx] = sort(Fn(:, o));
    cd(idx(1))   = inf;
    cd(idx(end)) = inf;
    dmin = Fs(end) - Fs(1);
    if dmin < 1e-9, continue; end
    for j = 2:N - 1
        cd(idx(j)) = cd(idx(j)) + (Fs(j + 1) - Fs(j - 1)) / dmin;
    end
end
end