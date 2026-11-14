function [len, energy] = path_metrics(path)
%PATH_METRICS 计算路径长度(m)与能耗(Wh)。
% 能耗公式与论文一致（4 项）：energy = 0.1*L + 0.3*ΔH + 0.2*Σθ + 0.15*Σ2sin(θ/2)。
seg = diff(path, 1, 1);
len = sum(sqrt(sum(seg .^ 2, 2)));
height_change = sum(abs(seg(:, 3)));
turn = 0; maneuver = 0;
for j = 2:size(path, 1) - 1
    v1 = path(j, :) - path(j - 1, :);
    v2 = path(j + 1, :) - path(j, :);
    n1 = v1 / (norm(v1) + 1e-9);
    n2 = v2 / (norm(v2) + 1e-9);
    ang = acos(max(min(dot(n1, n2), 1), -1));
    turn = turn + ang;
    maneuver = maneuver + 2 * sin(ang / 2);
end
energy = len * 0.1 + height_change * 0.3 + 0.2 * turn + 0.15 * maneuver;
end
