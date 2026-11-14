function d = env_dist(p, env)
%ENV_DIST 计算点 p 到最近静态障碍表面的“有符号距离”（障碍内部为负）。
% 统一解析表达：山体(取最大高程)、球形禁飞区、圆柱群（与 AVSS_RRT_dynamic.m 一致）。
x = p(1); y = p(2); z = p(3);

% 山体地形：取三座山在该 (x,y) 处的最大高程 h，距离 = z - h
h = 0;
for i = 1:size(env.mountains, 1)
    m = env.mountains(i, :);
    h = max(h, m(3) * exp(-((x - m(1))^2 + (y - m(2))^2) / (2 * m(4)^2)));
end
d = z - h;

% 球形禁飞区
d = min(d, norm(p - env.ball(1:3)) - env.ball(4));

% 圆柱障碍
for i = 1:size(env.cylinders, 1)
    c = env.cylinders(i, :);
    dr = norm(p(1:2) - c(1:2)) - env.cyl_r;
    if z <= c(3)
        dc = dr;
    else
        dz = z - c(3);
        if dr < 0
            dc = dz;
        else
            dc = hypot(dr, dz);
        end
    end
    d = min(d, dc);
end
end