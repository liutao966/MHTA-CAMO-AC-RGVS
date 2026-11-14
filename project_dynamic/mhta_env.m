function path = mhta_env(start, goal, env, gs, safe)
%ASTAR_ENV 三维 MHTA 路径搜索（解析环境，线性索引实现，26 邻域，加权 A*）。
% 启发场 H 一次性预计算，min 取 open 中 f 最小者用向量化索引，避免逐点 arrayfun。
xg = env.map(1):gs:env.map(2);
yg = xg; zg = xg;
nx = numel(xg); ny = numel(yg); nz = numel(zg);

% 启发场：0.9 倍欧氏 + 障碍距离惩罚（按类型 200/300/250，与 MHTA-CAMO-AC-RGVS untitled.m 的 enhanced_heuristic 完全一致）
[XX, YY, ZZ] = ndgrid(xg, yg, zg);
H = 0.9 * sqrt((XX - goal(1)).^2 + (YY - goal(2)).^2 + (ZZ - goal(3)).^2);
clear XX YY ZZ;
for ii = 1:nx
    for jj = 1:ny
        for kk = 1:nz
            H(ii, jj, kk) = H(ii, jj, kk) + h_penalty([xg(ii), yg(jj), zg(kk)], env, safe);
        end
    end
end

[~, si] = min(abs(xg - start(1)));
[~, sj] = min(abs(yg - start(2)));
[~, sk] = min(abs(zg - start(3)));
[~, gi] = min(abs(xg - goal(1)));
[~, gj] = min(abs(yg - goal(2)));
[~, gk] = min(abs(zg - goal(3)));

start_l = si + (sj - 1) * nx + (sk - 1) * nx * ny;
goal_l  = gi + (gj - 1) * nx + (gk - 1) * nx * ny;

gval    = inf(nx, ny, nz);
parent  = zeros(nx, ny, nz);
closed  = false(nx, ny, nz);
in_open = false(nx, ny, nz);

[d1, d2, d3] = ndgrid([-1 0 1], [-1 0 1], [-1 0 1]);
D = [d1(:) d2(:) d3(:)];
D(all(D == 0, 2), :) = [];

gval(start_l) = 0;
open = start_l;
in_open(start_l) = true;

found = false;
max_iter = 2 * nx * ny * nz;
iter = 0;

while ~isempty(open) && ~found && iter < max_iter
    iter = iter + 1;
    [~, m] = min(gval(open) + H(open));
    Lc = open(m);
    open(m) = [];
    in_open(Lc) = false;
    if Lc == goal_l
        found = true;
        break;
    end
    if closed(Lc)
        continue;
    end
    closed(Lc) = true;
    ic = mod(Lc - 1, nx) + 1;
    jc = mod(floor((Lc - ic) / nx), ny) + 1;
    kc = floor((Lc - ic - (jc - 1) * nx) / (nx * ny)) + 1;
    cpos = [xg(ic) yg(jc) zg(kc)];
    for d = 1:size(D, 1)
        ii = ic + D(d, 1);
        jj = jc + D(d, 2);
        kk = kc + D(d, 3);
        if ii < 1 || ii > nx || jj < 1 || jj > ny || kk < 1 || kk > nz
            continue;
        end
        Ln = ii + (jj - 1) * nx + (kk - 1) * nx * ny;
        if closed(Ln)
            continue;
        end
        npos = [xg(ii) yg(jj) zg(kk)];
        if env_dist(npos, env) >= safe && seg_free_static(cpos, npos, env, 1.0)
            gnew = gval(Lc) + norm(npos - cpos);
            if gnew < gval(Ln)
                gval(Ln) = gnew;
                parent(Ln) = Lc;
                if ~in_open(Ln)
                    in_open(Ln) = true;
                    open(end + 1) = Ln; %#ok<AGROW>
                end
            end
        end
    end
end

if ~found
    error('A* 未找到可行路径，请减小 gs 或调整参数。');
end

path = [];
cur = goal_l;
while cur ~= 0
    ic = mod(cur - 1, nx) + 1;
    jc = mod(floor((cur - ic) / nx), ny) + 1;
    kc = floor((cur - ic - (jc - 1) * nx) / (nx * ny)) + 1;
    path = [[xg(ic) yg(jc) zg(kc)]; path]; %#ok<AGROW>
    cur = parent(cur);
end
path(1, :)   = start;
path(end, :) = goal;
end

function pen = h_penalty(pos, env, safe)
% 障碍距离惩罚：按类型 200/300/250，与 MHTA-CAMO-AC-RGVS/untitled.m 的 enhanced_heuristic 一致。
x = pos(1); y = pos(2); z = pos(3);
pen = 0;
% 山体：三座山取最大高程
h = 0;
for i = 1:size(env.mountains, 1)
    m = env.mountains(i, :);
    h = max(h, m(3) * exp(-((x - m(1))^2 + (y - m(2))^2) / (2 * m(4)^2)));
end
if z - h < 9
    pen = pen + 200 / (z - h + 1e-6);
end
% 球形禁飞区
bd = norm(pos - env.ball(1:3));
if bd < env.ball(4) + 9
    pen = pen + 300 / (bd - env.ball(4) + 1e-6);
end
% 圆柱
for i = 1:size(env.cylinders, 1)
    c = env.cylinders(i, :);
    dh = norm([x - c(1), y - c(2)]);
    if dh < env.cyl_r + 9 && z < c(3)
        pen = pen + 250 / (dh - env.cyl_r + 1e-6);
    end
end
end