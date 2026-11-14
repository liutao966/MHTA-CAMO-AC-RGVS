function h = terrain_height(x, y, model)
% TERRAIN_HEIGHT  解析地表高度
%   'city' -> 平底, 高度恒为 0
%   其他   -> 若干座高斯山的并集(取最大, 各自截断于0)
if isfield(model, 'type') && strcmp(model.type, 'city')
    h = zeros(size(x));
    return;
end

h = 0;
for k = 1:size(model.mountains, 1)
    cx = model.mountains(k, 1); cy = model.mountains(k, 2);
    A  = model.mountains(k, 3); s = model.mountains(k, 4);
    g  = A * exp(-((x - cx).^2 + (y - cy).^2) / (2 * s^2));
    h  = max(h, max(g, 0));
end
end