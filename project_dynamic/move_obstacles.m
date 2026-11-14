function dyn = move_obstacles(dyn)
%MOVE_OBSTACLES 推进所有动态障碍物一步。
% 新版（gen_obstacles_v2）用 dyn.obs{i}.type 分派四类运动；旧版（gen_obstacles 直线
% params 结构）自动回退到原有直线往返逻辑，保证向后兼容。
if isfield(dyn, 'obs') && ~isempty(dyn.obs)
    for i = 1:dyn.num
        o   = dyn.obs{i};
        cur = dyn.curr(i, :);
        switch o.type
            case 1
                [nxt, o] = step_linear(cur, o);
            case 2
                [nxt, o] = step_circular(cur, o);
            case 3
                [nxt, o] = step_variable(cur, o);
            case 4
                [nxt, o] = step_vertical(cur, o);
            otherwise
                continue;
        end
        dyn.curr(i, :) = nxt;
        dyn.obs{i} = o;
    end
else
    % 旧结构：直线 S<->E 往返（向后兼容 gen_obstacles）
    for i = 1:dyn.num
        p1  = dyn.params(i, 1:3);
        p2  = dyn.params(i, 4:6);
        dir = dyn.params(i, 7);
        cur = dyn.curr(i, :);
        total = norm(p2 - p1);
        if total < 1e-6
            continue;
        end
        unit = (p2 - p1) / total;
        nxt  = cur + dir * dyn.speed * unit;
        if dir == 1
            if dot(nxt - p2, p2 - p1) >= 0
                dyn.params(i, 7) = -1;
                nxt = p2;
            end
        else
            if dot(nxt - p1, p1 - p2) >= 0
                dyn.params(i, 7) = 1;
                nxt = p1;
            end
        end
        dyn.curr(i, :) = nxt;
    end
end
end

% ================= 四类单步推进 =================
function [nxt, o] = step_linear(cur, o)
u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
nxt = cur + o.dir * o.speed * u;
if o.dir == 1
    if dot(nxt - o.E, o.E - o.S) >= 0
        o.dir = -1; nxt = o.E;
    end
else
    if dot(nxt - o.S, o.S - o.E) >= 0
        o.dir = 1; nxt = o.S;
    end
end
end

function [nxt, o] = step_circular(cur, o) %#ok<INUSD>
o.theta = o.theta + o.omega;
nxt = [o.center(1) + o.R * cos(o.theta), ...
       o.center(2) + o.R * sin(o.theta), o.center(3)];
end

function [nxt, o] = step_variable(cur, o)
o.t = o.t + 1;
spd = o.v0 * (1 + o.amp * sin(2 * pi * o.t / o.period));
u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
nxt = cur + o.dir * spd * u;
if o.dir == 1
    if dot(nxt - o.E, o.E - o.S) >= 0
        o.dir = -1; nxt = o.E;
    end
else
    if dot(nxt - o.S, o.S - o.E) >= 0
        o.dir = 1; nxt = o.S;
    end
end
end

function [nxt, o] = step_vertical(cur, o)
o.t = o.t + 1;
u = (o.E - o.S) / max(norm(o.E - o.S), 1e-9);
% 水平沿 S<->E 以 hspeed 匀速穿越，垂直方向正弦振荡
h = cur(1:2) + o.dir * o.hspeed * u(1:2);
if o.dir == 1
    if dot(h - o.E(1:2), o.E(1:2) - o.S(1:2)) >= 0
        o.dir = -1; h = o.E(1:2);
    end
else
    if dot(h - o.S(1:2), o.S(1:2) - o.E(1:2)) >= 0
        o.dir = 1; h = o.S(1:2);
    end
end
z = o.z0 + o.A * sin(2 * pi * o.t / o.Tz);
nxt = [h, z];
end
