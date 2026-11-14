function ok = segment_free(a, b, model, safe_d, step)
% SEGMENT_FREE  判断线段 a→b 是否全程无碰撞（每个采样点离障碍 ≥ safe_d）
% 障碍边界按 UAV 半径(safe_d)外扩，保证机身不穿障。
d = norm(b - a);
if d < 1e-9, ok = true; return; end
n = max(2, ceil(d / step));
t = (0:n)' / n;
P = a + t .* (b - a);
for k = 1:n + 1
    if min_dist(P(k, :), model) < safe_d
        ok = false; return;
    end
end
ok = true;
end