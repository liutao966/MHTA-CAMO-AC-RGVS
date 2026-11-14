function [best_path, best_cost, tree, parent, curve] = irrtstar_plan(model, params)
% IRRTTSTAR_PLAN  Improved RRT*(基于人工势场代价函数的 RRT*)
% -------------------------------------------------------------------------
% 参考: "Dynamically Feasible 3D Path Planning and Optimization for UAVs via
%        an Improved RRT*", 2024.
% 在传统 RRT*(最优父节点选择 + 重布线)基础上, 将代价函数改进为人工势场形式
%   c(x) = min_{x'∈Xnear} { c(x') + |x-x'| + γ·Σ_k U(x_k)·Δl }   (式9)
% 势场值 U(x_k) 采用混合势场(式10):
%   足够靠近障碍(d_obs≤d0): 斥力   U = ½η1·(1/d_obs - 1/d0)²
%   否则                 : 引力   U = ½η2·(|x_goal-x|/|x_goal-x_start|)²
% 使树在扩张时倾向低碰撞风险区域, 同时被目标轻轻吸引, 兼顾安全与效率。
% -------------------------------------------------------------------------
start = model.start;
goal  = model.goal;
bo = model.bound;
lbv = [bo(1) bo(1) bo(3)];
ubv = [bo(2) bo(2) bo(4)];

delta  = params.delta;       % 扩展步长
maxIter= params.maxIter;     % 最大迭代次数
nearR  = params.near_radius; % 最优父节点/重布线搜索半径
goalBias = params.goal_bias; % 目标偏向概率
safe_d = params.safe_dist;   % 碰撞裕度
step   = params.step;        % 碰撞采样步长

% APF 代价参数
gam    = params.gamma;       % 势场权重 γ
d0     = params.d0;          % 斥力影响范围
eta1   = params.eta1;        % 斥力强度
eta2   = params.eta2;        % 引力强度
dl     = params.dl;          % 势场采样间隔

% ---- 初始化 ----
nodes = start;
parent = 0;           % parent 为 0 表示根
gcost = 0;            % 从起点到各节点的累积代价(含势场)
best_cost = inf;
best_node = 0;
curve = zeros(maxIter, 1);

Dgoal = norm(start - goal);
bestFound = inf;

for it = 1:maxIter
    % ---- 采样(含目标偏向) ----
    if rand < goalBias
        rpt = goal;
    else
        rpt = lbv + (ubv - lbv) .* rand(1, 3);
    end

    % ---- 最近邻(按几何距离) ----
    d2 = sum((nodes - rpt).^2, 2);
    [~, nn] = min(d2);
    dir = rpt - nodes(nn, :); nd = norm(dir);
    if nd < 1e-9, continue; end
    Pnew = nodes(nn, :) + dir / nd * min(delta, nd);
    Pnew = min(max(Pnew, lbv), ubv);
    if ~segment_free(nodes(nn, :), Pnew, model, safe_d, step)
        continue;
    end

    % ---- 最优父节点选择(圈内节点, 取含 APF 的代价最小) ----
    [parent_candidate, minc] = best_parent(nodes, gcost, Pnew, model, gam, d0, eta1, eta2, dl, safe_d, step, nearR, start, goal, Dgoal);
    if minc == inf
        continue;   % 若因势场/碰撞找不出合法父节点则跳过
    end

    nodes(end + 1, :) = Pnew; %#ok<AGROW>
    parent(end + 1)   = parent_candidate; %#ok<AGROW>
    gcost(end + 1)    = minc; %#ok<AGROW>
    ni = size(nodes, 1);

    % ---- 重布线: 尝试用新节点作为邻居的更优父节点 ----
    nodes = rewire(nodes, parent, gcost, ni, model, gam, d0, eta1, eta2, dl, safe_d, step, nearR, start, goal, Dgoal);

    % ---- 尝试连接目标 ----
    if segment_free(Pnew, goal, model, safe_d, step)
        cand = gcost(ni) + edge_cost(Pnew, goal, model, gam, d0, eta1, eta2, dl, start, goal, Dgoal);
        if cand < best_cost
            best_cost = cand;
            best_node = ni;
        end
    end
    if best_cost < bestFound
        bestFound = best_cost;
    end
    if bestFound < inf
        curve(it) = bestFound;      % 已发现目标后, 记录当前最优代价
    end
end

tree   = nodes;
parent = parent;
curve  = curve;

% ---- 提取最优路径 ----
if best_node == 0
    best_path = []; best_cost = inf;
    return;
end
seq = [];
c = best_node;
while c ~= 0
    seq = [c, seq]; %#ok<AGROW>
    c = parent(c);
end
path = [nodes(seq, :); goal];
best_path = smooth_path(path, 300);
end

% =========================================================================
function [pc, cmin] = best_parent(nodes, gcost, Pnew, model, gam, d0, eta1, eta2, dl, safe_d, step, nearR, start, goal, Dgoal)
d2 = sum((nodes - Pnew).^2, 2);
nbr = find(d2 <= nearR^2);
cmin = inf; pc = 0;
for j = 1:numel(nbr)
    k = nbr(j);
    if ~segment_free(nodes(k, :), Pnew, model, safe_d, step)
        continue;
    end
    ck = gcost(k) + edge_cost(nodes(k, :), Pnew, model, gam, d0, eta1, eta2, dl, start, goal, Dgoal);
    if ck < cmin
        cmin = ck; pc = k;
    end
end
% 保证总能连上最近邻(至少几何可行)
if pc == 0
    if segment_free(nodes(1, :), Pnew, model, safe_d, step)
        cmin = gcost(1) + edge_cost(nodes(1, :), Pnew, model, gam, d0, eta1, eta2, dl, start, goal, Dgoal);
        pc = 1;
    end
end
end

function nodes = rewire(nodes, parent, gcost, ni, model, gam, d0, eta1, eta2, dl, safe_d, step, nearR, start, goal, Dgoal)
nd = nodes(ni, :);
d2 = sum((nodes - nd).^2, 2);
nbr = find(d2 <= nearR^2);
for j = 1:numel(nbr)
    k = nbr(j);
    if k == ni, continue; end
    if ~segment_free(nd, nodes(k, :), model, safe_d, step)
        continue;
    end
    cand = gcost(ni) + edge_cost(nd, nodes(k, :), model, gam, d0, eta1, eta2, dl, start, goal, Dgoal);
    if cand < gcost(k)
        parent(k) = ni;
        gcost(k)  = cand;
    end
end
end

% =========================================================================
function c = edge_cost(p1, p2, model, gam, d0, eta1, eta2, dl, start, goal, Dgoal)
% 式(9): c = 欧氏距离 + γ·ΣU(x_k)·Δl
ed = norm(p2 - p1);
n  = max(2, ceil(ed / dl));
U  = 0;
for k = 1:n - 1
    q = p1 + (p2 - p1) * (k / n);
    U = U + upot(q, model, d0, eta1, eta2, start, goal, Dgoal);
end
c = ed + gam * U * dl;
end

function u = upot(q, model, d0, eta1, eta2, start, goal, Dgoal)
% 式(10) 混合人工势场
dobs = min_dist(q, model);
if dobs <= d0
    u = 0.5 * eta1 * (1 / max(dobs, 1e-6) - 1 / d0)^2;   % 斥力
else
    u = 0.5 * eta2 * (norm(q - goal) / Dgoal)^2;           % 引力
end
end

% =========================================================================
function sp = smooth_path(path, n)
% pchip 累积弦长平滑
m = size(path, 1);
if m < 3, sp = path; return; end
seg = sqrt(sum(diff(path, 1, 1).^2, 2));
t = [0; cumsum(seg)];
tt = linspace(t(1), t(end), n);
sp = zeros(n, 3);
for d = 1:3
    sp(:, d) = pchip(t, path(:, d), tt);
end
end