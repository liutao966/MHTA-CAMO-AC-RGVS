function [pareto_idx, dominated_idx] = find_pareto_front(metrics, obj_minimize)
%% FIND_PARETO_FRONT 从候选解集中筛选 Pareto 非支配解
% 输入:
%   metrics      - N x m 目标值矩阵 (N 个解, m 个目标)
%   obj_minimize - 1 x m 布尔向量, true=最小化目标, false=最大化目标
% 输出:
%   pareto_idx    - 非支配解的索引
%   dominated_idx - 被支配解的索引

N = size(metrics, 1);
is_dominated = false(N, 1);

for i = 1:N
    if is_dominated(i), continue; end
    for j = 1:N
        if i == j, continue; end
        if dominates(metrics(i,:), metrics(j,:), obj_minimize)
            is_dominated(j) = true;
        end
    end
end

pareto_idx    = find(~is_dominated);
dominated_idx = find(is_dominated);
end