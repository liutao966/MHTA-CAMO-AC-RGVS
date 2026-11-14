function h = terrain_height(model, x, y)
% TERRAIN_HEIGHT  查询 (x,y) 处三座高斯山体叠加得到的地形高度
%   输入 x, y 可为标量或同尺寸数组，返回对应地形高度（不含随机噪声）。
%   与 map.m 的高斯山体公式一致：A*exp(-((x-cx)^2+(y-cy)^2)/(2*sigma^2))
%   为保证碰撞检测的稳定性，此处忽略 map.m 中的随机噪声项 randn。

h = zeros(size(x));
for k = 1:size(model.mountains, 1)
    cx    = model.mountains(k, 1);
    cy    = model.mountains(k, 2);
    A     = model.mountains(k, 3);
    sigma = model.mountains(k, 4);
    h = h + A * exp(-((x - cx).^2 + (y - cy).^2) / (2 * sigma^2));
end
end
