clear;
clc;
% ===================== 字体大小一键调节区（直接修改数字即可）=====================
tick_font_size  = 20;   % 坐标轴刻度数字（0/50/100/150/200）的字号
label_font_size = 20;   % 坐标轴名称（X/m、Y/m、Z/m）的字号
title_font_size = 16;   % 图标题（城市障碍物地图）的字号
% =================================================================================
% 城市环境（200 x 200 x 200）：平坦地面 + 大量矩形高楼（模拟街区/摩天楼）
% 建筑按网格"地块"排布，每栋楼只占据一个互不重叠的地块，楼与楼之间自动留出道路。
% 中央横向预留一条主干道，纵向也间断留空，便于无人机沿街道走廊飞行。
% =================================================================================
% ------------------------- 1. 城市街区网格参数 -------------------------
nCol = 6;   nRow = 6;        % 列数 / 行数（共 36 个地块）
margin = 14;                 % 距地图边缘留白(m)
gap    = 6;                  % 楼与楼之间的道路宽度(m)? 
                             % ==> 地块内建筑比地块小 gap，四周形成道路，保证不重合
rng('default');               % 复位随机数生成器（清除旧生成器）
rng(26);                      % 固定随机种子，便于结果复现

xc = linspace(margin, 200 - margin, nCol + 1);   % 地块纵向边界
yc = linspace(margin, 200 - margin, nRow + 1);   % 地块横向边界
cellWx = xc(2) - xc(1);
cellWy = yc(2) - yc(1);
bldWx  = cellWx - gap;       % 建筑占地面积(宽) <= 地块宽 - 道路
bldWy  = cellWy - gap;

% ------------------------- 2. 创建画布 + 平坦地面 -------------------------
figure("Position",[100,100,800,800],"Color","white");
hold on;
patch([0 200 200 0],[0 0 200 200],[1e-3 1e-3 1e-3 1e-3], ...
      [0.82 0.82 0.78],"EdgeColor","none","FaceAlpha",1.0);

% ------------------------- 3. 逐地块生成高楼（互不重合） -------------------------
% 中央横向主干道: y 方向中间一行(第3行, index=3)全部留空作为主干道
roadRow  = 3;                 % 0-based 主机道所在行(纵向留空)
% 纵向主干道: x 方向中间一列(第3列)也留空
roadCol  = 3;
% 再额外随机留出若干"广场"地块
plaza = rand(nRow, nCol) < 0.15;

for r = 1:nRow
    for c = 1:nCol
        % 道路 / 广场单元不建楼
        if r == roadRow || c == roadCol || plaza(r, c)
            continue;
        end
        % 地块中心
        cx = (xc(c) + xc(c+1)) / 2;
        cy = (yc(r) + yc(r+1)) / 2;
        % 建筑矩形（比地块小 gap，四面留路）
        x0 = cx - bldWx/2;  x1 = cx + bldWx/2;
        y0 = cy - bldWy/2;  y1 = cy + bldWy/2;
        % 高度：30~100 m 普通楼房，少量地标塔楼到 150 m
        h = 30 + 70 * rand;
        if rand < 0.12
            h = 120 + 30 * rand;   % 地标超高楼
        end
        draw_building(x0, x1, y0, y1, h);
    end
end

% ===================== 【核心修改：所有轴设置统一放在绘图完成后】=====================
% 坐标轴名称+字号设置
xlabel("X/m","FontSize",label_font_size);
ylabel("Y/m","FontSize",label_font_size);
zlabel("Z/m","FontSize",label_font_size);

% 标题+字号设置
title("城市障碍物地图","FontSize",title_font_size);

% 【强制设置坐标范围，放在最后，不会被后续操作覆盖】
xlim([0,200]);
ylim([0,200]);
zlim([0,200]);

% 【三个坐标轴单位设置为20】
xticks(0:20:200);
yticks(0:20:200);
zticks(0:20:200);

% 【替代axis equal，保证X/Y/Z等比例，同时不覆盖zlim范围】
daspect([1 1 1]);
% 补充：若觉得Z轴太空，可改为daspect([1 1 0.8])，微调Z轴比例，X/Y仍保持等比例

% 统一设置刻度字号
set(gca,"FontSize",tick_font_size);

% 基础样式设置
grid on;
set(gca,"Color","white");

% 【关键】强制设为 3D 等距视角（水平方位45°、俯仰30°），避免显示成俯视2D图
view(45, 30);
% 正交投影：保持三轴严格平行/垂直，z 轴与 x/y 平面原点精确汇交，
% 避免透视投影造成的 z 轴与 xy 平面在视觉上"不重合/偏移"现象
camproj("orthographic");

hold off;

% =========================================================================
% 局部函数：绘制一栋立方体高楼（4侧面 + 顶盖），z 高度取 h，底在 z=0
% =========================================================================
function draw_building(x0, x1, y0, y1, h)
% 高度越高颜色越偏蓝紫，低楼偏灰
t = min(max(h/150, 0), 1);
col = [0.4 - 0.15*t, 0.48 - 0.10*t, 0.72 + 0.10*t];

verts_x = [x0, x1, x1, x0, x0, x1, x1, x0];
verts_y = [y0, y0, y1, y1, y0, y0, y1, y1];
verts_z = [0, 0, 0, 0, h, h, h, h];
faces   = [
    1 2 3 4;  % 底面
    5 6 7 8;  % 顶面
    1 2 6 5;  % X-正侧面
    2 3 7 6;  % Y-正侧面
    3 4 8 7;  % X-负侧面
    4 1 5 8   % Y-负侧面
];
patch('Vertices', [verts_x(:), verts_y(:), verts_z(:)], 'Faces', faces, ...
      'FaceColor', col, 'EdgeColor', 'k', 'LineWidth', 0.5, 'FaceAlpha', 1.0);

% 顶盖提亮
patch([x0 x1 x1 x0], [y0 y0 y1 y1], [h h h h], col .* 1.2, ...
      'EdgeColor', 'none', 'FaceAlpha', 1.0);
end