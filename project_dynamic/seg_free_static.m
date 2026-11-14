function tf = seg_free_static(a, b, env, res)
%SEG_FREE_STATIC 判断线段 a->b 对静态障碍（含安全距离 env.safe）是否无碰撞。
if nargin < 4
    res = 1.5;   % 采样分辨率
end
L = norm(b - a);
n = max(2, ceil(L / res));
t = linspace(0, 1, n)';
pts = a + t .* (b - a);
for i = 1:n
    if env_dist(pts(i, :), env) < env.safe
        tf = false;
        return;
    end
end
tf = true;
end