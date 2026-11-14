function D = min_dist(pos, model)
% MIN_DIST  点到所有障碍物的最小带符号距离(负值=在障碍内部)
%   'city' -> 地面(z=0) + 矩形高楼
%   其他   -> 高斯山 + (可选)禁飞球 + (可选)圆柱
x = pos(1); y = pos(2); z = pos(3);

if isfield(model, 'type') && strcmp(model.type, 'city')
    D = z;                                   % 地面净空
    for i = 1:size(model.buildings, 1)
        D = min(D, dist_to_box(pos, model.buildings(i, :)));
    end
    return;
end

dList = z - terrain_height(x, y, model);      % 高斯山

if ~isempty(model.ball)
    dList = [dList, norm(pos - model.ball(1:3)) - model.ball(4)];  % 禁飞球
end

for i = 1:size(model.cylinder.pos, 1)
    cx = model.cylinder.pos(i, 1); cy = model.cylinder.pos(i, 2);
    cz = model.cylinder.pos(i, 3); r = model.cylinder.r;
    horiz = norm([x - cx, y - cy]);
    if z >= 0 && z <= cz
        dCyl = horiz - r;
    else
        vert = -z; if z > cz, vert = z - cz; end
        dCyl = max(horiz - r, vert);
    end
    dList = [dList, dCyl]; %#ok<AGROW>
end
D = min(dList);
end

% =========================================================================
function dd = dist_to_box(pt, b)
% 点 pt 到轴对齐立方体 b=[x0 x1 y0 y1 h] (z 从 0 到 h) 的带符号距离：内部为负，外部为正
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