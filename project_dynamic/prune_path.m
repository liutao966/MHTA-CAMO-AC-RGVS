function out = prune_path(path, th)
%PRUNE_PATH 基于转角阈值的路径剪枝（去除近共线冗余点）。
if size(path, 1) <= 3
    out = path;
    return;
end
out = path(1, :);
for i = 2:size(path, 1) - 1
    v1 = path(i, :) - path(i - 1, :);
    v2 = path(i + 1, :) - path(i, :);
    c = dot(v1, v2) / (norm(v1) * norm(v2) + 1e-9);
    c = max(min(c, 1), -1);
    if acos(c) > th
        out = [out; path(i, :)]; %#ok<AGROW>
    end
end
out = [out; path(end, :)];
end