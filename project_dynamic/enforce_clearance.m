function out = enforce_clearance(path, env)
%ENFORCE_CLEARANCE 把路径中离静态障碍过近的点沿 env_dist 梯度向外推开，
% 保证最小间隙 >= env.safe+margin。目标值与碰撞判定共用同一 env_dist（解析、无噪声）。
% 复刻不同_stage 原始 untitled.m 中 enforce_static_clearance 的作用。
margin = 1.0;
need   = env.safe + margin;

out = path;
n = size(out, 1);
for i = 1:n
    if i == 1 || i == n
        continue;                 % 端点最后单独钉回
    end
    out(i, :) = push_out(out(i, :), env, need);
end

% 端点钉回起点/终点，不允许被推开
out(1, :)   = path(1, :);
out(end, :) = path(end, :);
end

function p = push_out(p, env, need)
% 沿 env_dist 的数值梯度（离开障碍方向）迭代外推，直到间隙 >= need。
for it = 1:30
    d = env_dist(p, env);
    if d >= need
        return;
    end
    e = 0.5;
    g = zeros(1, 3);
    for k = 1:3
        p1 = p; p2 = p;
        p1(k) = p1(k) + e;
        p2(k) = p2(k) - e;
        g(k) = (env_dist(p1, env) - env_dist(p2, env)) / (2 * e);
    end
    if norm(g) < 1e-9
        g = [0 0 1];              % 无梯度时向上抬升（越过障碍最稳妥）
    end
    g = g / norm(g);
    p = p + g * (need - d + 0.1);
end
end