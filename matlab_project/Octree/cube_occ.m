function o = cube_occ(center, half, model, sigma)
% CUBE_OCC  八叉树重叠检测：立方体 [center±half]（再外扩 sigma）是否与任一障碍重叠
%   'city' -> 地面(z=0) + 矩形高楼
%   其他   -> 地形(山体) / 球形禁飞区 / 圆柱体(带高度)
%   'hill'/'legacy' 中球/圆柱为空时对应分支自动跳过。
ex   = half + sigma;                    % 外扩后的半边长(含安全裕度 σ)
zmin = center(3) - ex;
zmax = center(3) + ex;

if isfield(model, 'type') && strcmp(model.type, 'city')
    % ---- 城市: 地面(立方体底触及挤出地面) + 高楼 AABB 重叠 ----
    if zmin <= sigma                    % 立方体外扩后触及地面(z=0)
        o = true; return;
    end
    for i = 1:size(model.buildings, 1)
        if aabb_overlap(center, ex, model.buildings(i, :))
            o = true; return;
        end
    end
    o = false; return;
end

% ---- 1) 地形重叠(山体表面穿过立方体) ----
loX = center(1) - half; hiX = center(1) + half;
loY = center(2) - half; hiY = center(2) + half;
gx = linspace(loX, hiX, 3);
gy = linspace(loY, hiY, 3);
for a = 1:3
    for b = 1:3
        h = terrain_height(gx(a), gy(b), model);
        if h >= zmin - sigma          % 山体表面触及或高于立方体底 → 重叠
            o = true; return;
        end
    end
end

% ---- 2) 球形禁飞区(AABB-球体重叠)(存在时) ----
if ~isempty(model.ball) && numel(model.ball) == 4
    bc = model.ball(1:3); r = model.ball(4);
    cp = max(center - ex, min(center + ex, bc));   % 立方体内最近点
    if norm(cp - bc) < r
        o = true; return;
    end
end

% ---- 3) 圆柱体(XY 水平重叠 + Z 区间重叠)(存在时) ----
for i = 1:size(model.cylinder.pos, 1)
    c = model.cylinder.pos(i, :); cr = model.cylinder.r;
    cxp = max(center(1) - ex, min(center(1) + ex, c(1)));
    cyp = max(center(2) - ex, min(center(2) + ex, c(2)));
    if norm([c(1) - cxp, c(2) - cyp]) < cr ...   % 水平距离小于半径
            && zmax >= -sigma && zmin <= c(3) + sigma  % Z 区间重叠
        o = true; return;
    end
end
o = false;
end

% =========================================================================
function ov = aabb_overlap(center, ex, b)
% 立方体 [center-ex, center+ex] 与高楼 b=[x0 x1 y0 y1 h] (z∈[0,h]) 是否重叠
x_ov = (center(1) + ex > b(1)) && (center(1) - ex < b(2));
y_ov = (center(2) + ex > b(3)) && (center(2) - ex < b(4));
z_ov = (center(3) + ex > 0)     && (center(3) - ex < b(5));
ov = x_ov && y_ov && z_ov;
end