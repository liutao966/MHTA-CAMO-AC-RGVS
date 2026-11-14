function e = max_tracking_error(trail, path)
%MAX_TRACKING_ERROR 计算飞行轨迹相对参考路径的最大跟踪误差（m）。
% 对轨迹每个点，取到参考路径最近点的距离，返回其中的最大值。
% 对应论文 Table IX 的 "Maximum Tracking Error (m)" 列。
e = 0;
n = size(trail, 1);
if n == 0 || isempty(path)
    return;
end
for i = 1:n
    d = min(sqrt(sum((path - trail(i, :)) .^ 2, 2)));
    if d > e
        e = d;
    end
end
end
