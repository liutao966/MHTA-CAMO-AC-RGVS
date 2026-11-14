function model = build_environment(map_type)
% BUILD_ENVIRONMENT  构建三维环境模型(碰撞/评估/绘图共用), 按地图类型分派
%   map_type - 'city' | 'hill' | (缺省/其他为旧山体地图 legacy)
%   model.type      : 地图类型, 供 evaluate_path 分派
%   model.buildings : city 高楼 Nx5 [x0 x1 y0 y1 h] (仅 city)
%   model.mountains : 高斯山 Nx4 [cx cy A sigma]
%   model.ball      : 球形禁飞区 [cx cy cz r] (可为空)
%   model.cylinder  : .r / .pos 圆柱体 (可为空)
%   model.bound     : [xy_min xy_max z_min z_max] 航点搜索边界
if nargin < 1 || isempty(map_type), map_type = 'legacy'; end

model.type = map_type;

switch map_type
    case 'city'
        % ---- 城市地图: 平底 + 矩形高楼(复现 map/city_map.m) ----
        model.buildings = build_city_buildings();
        model.mountains = [];
        model.ball = [];
        model.cylinder.r   = 0;
        model.cylinder.pos = [];
        model.start = [10, 10, 60];
        model.goal  = [190, 190, 60];
        % 低空街道穿行: z 搜索域约束在起终点附近高度(55~65m), 沿途在
        % 楼间道路/主干道间隙里绕行, 不翻越楼顶(高楼 30~150m)。
        model.bound = [0, 200, 55, 65];

    case 'hill'
        % ---- 丘陵地图: 7 座高斯山体(确定性, 无噪声) ----
        model.buildings = [];
        model.mountains = [
            40, 140,  90, 30;
            140,  40,  90, 30;
            140, 140, 120, 30;
             90,  90, 120, 30;
             40,  40,  50, 30;
            180,  90, 100, 30;
             90, 180, 100, 30];
        model.ball = [];
        model.cylinder.r   = 0;
        model.cylinder.pos = [];
        model.start = [10, 10, 50];
        model.goal  = [190, 190, 80];
        model.bound = [0, 200, 50, 200];

    otherwise % legacy
        % ---- 旧默认地图: 三座山体 + 球形禁飞区 + 九根圆柱 ----
        model.buildings = [];
        model.mountains = [
             40, 140,  90, 30;
            140,  40,  90, 30;
            140, 140, 120, 30];
        model.ball = [100, 100, 0, 50];
        model.cylinder.r   = 5;
        model.cylinder.pos = [
            40, 70, 100; 70, 40, 100; 50, 50, 100;  90, 160, 100; 160, 80, 100; ...
            80, 180, 100; 100, 180, 100; 180, 60, 100; 180, 100, 100];
        model.start = [10, 10, 50];
        model.goal  = [190, 190, 80];
        model.bound = [0, 200, 50, 200];
end
end

% =========================================================================
function B = build_city_buildings()
% 复现 map/city_map.m 的高楼排布(同一 rng(26) 种子), 返回 Nx5
% 每行 = [x0 x1 y0 y1 h], z 从 0 到 h 的立方体高楼。
nCol = 6;  nRow = 6;
margin = 14;
gap    = 6;
rng('default'); rng(26);
xc = linspace(margin, 200 - margin, nCol + 1);
yc = linspace(margin, 200 - margin, nRow + 1);
cellWx = xc(2) - xc(1);
cellWy = yc(2) - yc(1);
bldWx = cellWx - gap;
bldWy = cellWy - gap;
roadRow = 3;   % 中间一行留空作主干道
roadCol = 3;   % 中间一列留空作主干道
plaza = rand(nRow, nCol) < 0.15;
B = [];
for r = 1:nRow
    for c = 1:nCol
        if r == roadRow || c == roadCol || plaza(r, c)
            continue;
        end
        cx = (xc(c) + xc(c+1)) / 2;
        cy = (yc(r) + yc(r+1)) / 2;
        x0 = cx - bldWx/2;  x1 = cx + bldWx/2;
        y0 = cy - bldWy/2;  y1 = cy + bldWy/2;
        h  = 30 + 70 * rand;
        if rand < 0.12
            h = 120 + 30 * rand;   % 地标超高楼
        end
        B(end+1, :) = [x0 x1 y0 y1 h]; %#ok<AGROW>
    end
end
end