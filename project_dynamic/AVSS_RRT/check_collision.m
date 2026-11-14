function [collide, n_hit] = check_collision(model, pts)
% CHECK_COLLISION  检测一条离散化路径 pts 是否与障碍物发生碰撞
%   输入:
%     model - map_model() 返回的模型结构体
%     pts   - N×3 的路径采样点矩阵 [x y z]
%   输出:
%     collide - 逻辑值，路径整体是否碰撞
%     n_hit   - 发生碰撞的采样点数量（用于连续化的惩罚项）
%
% 碰撞判定包含三类障碍物：
%   1) 地形（点 z 低于地形高度即撞山）
%   2) 球形禁飞区（点到球心距离 < 半径）
%   3) 圆柱体（点在圆柱半径内且高度低于圆柱顶）
% 各障碍物均加入安全裕度 safe_margin。

margin = model.safe_margin;
x = pts(:, 1);
y = pts(:, 2);
z = pts(:, 3);

% ---- 1) 地形碰撞：路径点低于地形表面 ----
h_terrain = terrain_height(model, x, y);
hit_terrain = z < (h_terrain + margin);

% ---- 2) 球形禁飞区碰撞 ----
c = model.ball.center;
d_ball = sqrt((x - c(1)).^2 + (y - c(2)).^2 + (z - c(3)).^2);
hit_ball = d_ball < (model.ball.radius + margin);

% ---- 3) 圆柱体碰撞 ----
hit_cyl = false(size(x));
r = model.cylinder.radius + margin;
for k = 1:size(model.cylinder.pos, 1)
    cx = model.cylinder.pos(k, 1);
    cy = model.cylinder.pos(k, 2);
    h  = model.cylinder.pos(k, 3);
    d_xy = sqrt((x - cx).^2 + (y - cy).^2);
    hit_k = (d_xy < r) & (z < h + margin);
    hit_cyl = hit_cyl | hit_k;
end

% ---- 4) 出界判定（超出地图范围也算不可行）----
lo = model.map_min;
hi = model.map_max;
out = (x < lo(1)) | (x > hi(1)) | ...
      (y < lo(2)) | (y > hi(2)) | ...
      (z < lo(3)) | (z > hi(3));

hit = hit_terrain | hit_ball | hit_cyl | out;
n_hit   = sum(hit);
collide = n_hit > 0;
end
