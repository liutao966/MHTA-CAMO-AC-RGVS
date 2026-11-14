function sp = smooth_spline(path)
%SMOOTH_SPLINE 三次样条平滑（弦长参数化，避免急转弯处过冲冲进障碍）。
n = size(path, 1);
if n < 4
    sp = path;
    return;
end
% 剔除过近的重复点，保证弦长参数单调递增、spline 可用
seg  = sqrt(sum(diff(path) .^ 2, 2));
keep = [true; seg > 1e-6];
path = path(keep, :);
n = size(path, 1);
if n < 4
    sp = path;
    return;
end
d  = [0; cumsum(sqrt(sum(diff(path) .^ 2, 2)))];
ts = linspace(0, d(end), max(3 * n, 200))';
sp = [spline(d, path(:, 1), ts), ...
      spline(d, path(:, 2), ts), ...
      spline(d, path(:, 3), ts)];
end