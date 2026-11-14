function m = eval_metrics(p, model, V)
% EVAL_METRICS  统一计算一条路径的 5 个静态指标（规划时间在调用处单独 tic/toc）
% -------------------------------------------------------------------------
% 用途：统计实验(run_stat_exp.m)与各基准/多环境实验共用同一套指标口径，
%       保证跨算法、跨表可比。能耗采用论文改进模型(4 项)，加速项按本文算法
%       (算法copy.m / solve_this_v2.m) 的 2*sin(θ/2) 口径。
% 输入：
%   p      - 路径 Nx3 世界坐标 [x y z]
%   model  - 环境结构体（与 run_stat_exp.m 的 build_env() 一致）：
%            model.mountains    Kx4  [cx, cy, 峰高A, 方差sigma]
%            model.ball         1x4  [cx, cy, cz, 半径r]（无球时用 [0 0 0 0]）
%            model.cylinder.r   圆柱半径
%            model.cylinder.pos Nx3 圆柱中心 [cx, cy, 顶高]
%   V      - 巡航速度 m/s
% 输出：
%   m      - 1x5 [路径长度, 最小安全距离, 能耗, 平滑度, 飞行时间]
%   口径：能耗 = 0.1*L + 0.3*ΔH + 0.2*Σ转向角 + 0.15*Σ(2*sin(θ/2))；
%         平滑度 = 相邻段夹角余弦均值（越接近 1 越平滑）。
    if isempty(p) || size(p, 1) < 2
        m = [nan nan nan nan nan]; return;
    end
    d = diff(p, 1, 1);
    seg = sqrt(sum(d.^2, 2));
    L = sum(seg);

    minsafe = inf;
    for i = 1:size(p, 1)
        minsafe = min(minsafe, node_safe_dist(p(i, :), model));
    end
    for i = 1:size(p, 1)-1
        for s = 1:5
            pos = p(i, :) + (p(i+1, :) - p(i, :))*(s/6);
            minsafe = min(minsafe, node_safe_dist(pos, model));
        end
    end
    minsafe = max(minsafe, 0);

    hc = sum(abs(d(:, 3)));          % 总高度变化 ΔH
    ta = 0; ac = 0;                  % 转向角累计 / 加速项累计
    cos_sum = 0; cnt = 0;
    for i = 2:size(p, 1)-1
        v1 = p(i, :) - p(i-1, :);
        v2 = p(i+1, :) - p(i, :);
        n1 = norm(v1); n2 = norm(v2);
        if n1 > 1e-6 && n2 > 1e-6
            c = dot(v1, v2)/(n1*n2);
            ang = acos(max(min(c, 1), -1));
            ta = ta + ang;
            ac = ac + 2*sin(ang/2);
            cos_sum = cos_sum + c;
            cnt = cnt + 1;
        end
    end
    E = L*0.1 + hc*0.3 + 0.2*ta + 0.15*ac;   % 论文改进能耗模型(4 项)
    S = cos_sum / (cnt + eps);

    m = [L, minsafe, E, S, L/V];
end

% ========================================================================
function d = node_safe_dist(pos, model)
% 点到最近障碍(高斯山 + 球形禁飞区 + 圆柱)的带符号距离；越小越危险，>=0 表示无碰撞
    x = pos(1); y = pos(2); z = pos(3);
    ds = [];
    h = 0;
    for k = 1:size(model.mountains, 1)
        cx = model.mountains(k,1); cy = model.mountains(k,2);
        A  = model.mountains(k,3); sg = model.mountains(k,4);
        g  = A*exp(-((x-cx)^2+(y-cy)^2)/(2*sg^2));
        h  = max(h, max(g, 0));      % 地形 = 多座高斯山的并集(取最大)
    end
    ds(end+1) = z - h;

    if numel(model.ball) == 4 && model.ball(4) > 0
        c = model.ball(1:3); r = model.ball(4);
        ds(end+1) = norm(pos - c) - r;
    end

    if isfield(model, 'cylinder') && ~isempty(model.cylinder.pos)
        for k = 1:size(model.cylinder.pos, 1)
            ccx = model.cylinder.pos(k,1); ccy = model.cylinder.pos(k,2); ch = model.cylinder.pos(k,3);
            r = model.cylinder.r;
            dhor = norm([x-ccx, y-ccy]) - r;
            if z >= 0 && z <= ch
                dcyl = dhor;
            else
                dvert = max(-z, z - ch);
                dcyl  = max(dhor, dvert);
            end
            ds(end+1) = dcyl;
        end
    end

    if isfield(model, 'buildings') && ~isempty(model.buildings)
        for k = 1:size(model.buildings, 1)
            ds(end+1) = dist_to_box(pos, model.buildings(k, :));
        end
    end
    d = min(ds);
end

% ========================================================================
function dd = dist_to_box(pt, b)
% 点 pt 到轴对齐立方体 b=[x0 x1 y0 y1 h](z 从 0 到 h) 的带符号距离：
%   外部为正(到最近表面距离)，内部为负。
    x = pt(1); y = pt(2); z = pt(3);
    x0 = b(1); x1 = b(2); y0 = b(3); y1 = b(4); h = b(5);
    inside = (x > x0 && x < x1 && y > y0 && y < y1 && z > 0 && z < h);
    if inside
        dd = -min([x - x0, x1 - x, y - y0, y1 - y, z, h - z]);
        return;
    end
    dx = max([x0 - x, x - x1, 0]);
    dy = max([y0 - y, y - y1, 0]);
    if z < 0, dz = -z; elseif z > h, dz = z - h; else, dz = 0; end
    dd = sqrt(dx^2 + dy^2 + dz^2);
end
