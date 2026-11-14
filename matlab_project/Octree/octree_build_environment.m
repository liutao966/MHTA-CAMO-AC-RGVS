function model = octree_build_environment(map_type)
% BUILD_ENVIRONMENT  构建三维环境模型 (供碰撞/八叉树/绘图共用)
% -------------------------------------------------------------------------
% 输入:
%   map_type - 可选:
%        'city'  -> 城市地图(city_map.m 平底+高楼)
%        'hill'  -> 丘陵地图(Hill_map.m 7 座高斯山体)
%        缺省/其他 -> 旧默认地图(三座山体+球形+圆柱)
% 不同地图通过 model.type 区分, 由 terrain_height / min_dist / cube_occ
% 分派查询。start/goal 与算法参数在 run_octree 中设定后由调用者使用。
% -------------------------------------------------------------------------
if nargin < 1 || isempty(map_type), map_type = 'legacy'; end

model.type = map_type;

% 通用: 起终点与导航空间
model.start  = [10, 10, 50];
model.goal   = [190, 190, 80];
model.xyz_lo = [0, 0, 30];   % 导航空间下界(避开近地大量障碍)
model.xyz_hi = [200, 200, 200];
model.safe_dist = 2.5;          % 安全距离 m (与 HSMA/NMOPOS/Improved RRT 统一)

switch map_type
    case 'city'
        % ============ 城市地图: 平底 + 矩形高楼 ============
        % 与 map/city_map.m 同一 rng(26) 种子生成(所画=所碰)
        model.buildings = build_city_buildings();   % Nx5 [x0 x1 y0 y1 h]
        model.mountains  = [];
        model.ball       = [];
        model.cylinder.r = [];
        model.cylinder.pos = [];
        model.start = [10, 10, 60];   % 与 AVSS-RRT 城市地图一致
        model.goal  = [190, 190, 60];

    case 'hill'
        % ============ 丘陵地图: 7 座高斯山体(确定性, 无噪声) ============
        model.mountains = [ 40, 140,  90, 30;
                           140,  40,  90, 30;
                           140, 140, 120, 30;
                            90,  90, 120, 30;
                            40,  40,  50, 30;
                           180,  90, 100, 30;
                            90, 180, 100, 30];
        model.ball       = [];
        model.cylinder.r = [];
        model.cylinder.pos = [];

    case 'obstacle'
        % ============ 障碍物地图: map.m 三座山体 + 球形 + 圆柱 ============
        model.mountains = [
            40, 140, 90,  30;
            140,  40, 90,  30;
            140, 140, 120, 30];
        model.ball = [100, 100, 0, 50];
        model.cylinder.r   = 5;
        model.cylinder.pos = [
            40, 70, 100; 70, 40, 100; 50, 50, 100; 90, 160, 100; 160, 80, 100; ...
            80, 180, 100; 100, 180, 100; 180, 60, 100; 180, 100, 100];

    otherwise
        % ============ legacy 保留 ============
        model.mountains = [
            40, 140, 90,  30;
            140,  40, 90,  30;
            140, 140, 120, 30];
        model.ball = [100, 100, 0, 50];
        model.cylinder.r   = 5;
        model.cylinder.pos = [
            40, 70, 100; 70, 40, 100; 50, 50, 100; 90, 160, 100; 160, 80, 100; ...
            80, 180, 100; 100, 180, 100; 180, 60, 100; 180, 100, 100];
end
end

% =========================================================================
function B = build_city_buildings()
% 复现 map/city_map.m 的高楼排布(同一 rng(26) 种子), 返回 Nx5
% 每行 = [x0 x1 y0 y1 h], z 从 0 到 h 的立方体高楼。
nCol = 6;  nRow = 6;
margin = 14;
gap    = 6;
s = rng;  rng('default'); rng(26);
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
rng(s);   % 恢复随机状态，避免重置调用方种子
end