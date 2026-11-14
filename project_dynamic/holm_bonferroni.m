function p_adj = holm_bonferroni(p)
%HOLM_BONFERRONI Holm-Bonferroni 多重比较校正。
% 输入 p：原始 p 值向量（可含 NaN）。
% 输出 p_adj：校正后的 p 值（同顺序，NaN 保留 NaN）。
% 原理：升序排列后，第 i 小的 p 值乘以 (m - i + 1)，并保证单调递增、截断到 1。
p = p(:)';
m = numel(p);
valid = ~isnan(p);
p_adj = nan(1, m);
if ~any(valid)
    return;
end
pv = p(valid);
[~, order] = sort(pv);   % 升序
n = numel(pv);
adj = zeros(1, n);
for i = 1:n
    adj(order(i)) = pv(order(i)) * (n - i + 1);
end
% 单调递增约束
for i = 2:n
    if adj(order(i)) < adj(order(i - 1))
        adj(order(i)) = adj(order(i - 1));
    end
end
adj = min(adj, 1);
p_adj(valid) = adj;
end
