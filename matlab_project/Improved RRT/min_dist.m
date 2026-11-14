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

dList = z - terrain_height(x, y, model);     % 高斯山

if ~isempty(model.ball) && numel(model.ball) == 4
    dList = [dList, norm(pos - model.ball(1:3)) - model.ball(4)]; %#ok<AGROW>
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
% 点 pt 到轴对齐立方体 b=[x0 x1 y0 y1 h] (z 从 0 到 h) 最近表面的距离
dx = max(max(b(1) - pt(1), pt(1) - b(2)), 0);
dy = max(max(b(3) - pt(2), pt(2) - b(4)), 0);
if pt(3) < 0
    dz = -pt(3);
elseif pt(3) > b(5)
    dz = pt(3) - b(5);
else
    dz = 0;
end
dd = sqrt(dx^2 + dy^2 + dz^2);
end