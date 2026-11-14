function d = dyn_min_dist(p, dyn)
%DYN_MIN_DIST 点 p 到最近动态障碍物表面的距离（中心距 - r）。无动态障碍时返回 inf。
if isempty(dyn) || ~isfield(dyn, 'num') || dyn.num == 0
    d = inf;
    return;
end
d = inf;
for i = 1:dyn.num
    d = min(d, norm(p(:)' - dyn.curr(i, :)) - dyn.r);
end
end