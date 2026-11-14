function leaves = octree_build(model, mincell, sigma)
% OCTREE_BUILD  递归构造环境八叉树(导航空间)
% -------------------------------------------------------------------------
% 自顶向下细分红黑八叉树: 根为整个导航空间, 遇"均匀自由"或"均匀占据"
% 区域提前停止细分, 仅障碍边界附近逐层细化到最小单元 mincell。
% 每个叶子记录中心/半长/占据状态, 构成危险栅格(供A*与可视化)。
% -------------------------------------------------------------------------
leaves = struct('center', {}, 'half', {}, 'occ', {});

lo = model.xyz_lo; hi = model.xyz_hi;
stack = struct('lo', lo, 'hi', hi);
while ~isempty(stack)
    nd = stack(end); stack(end) = [];
    c = (nd.lo + nd.hi) / 2;
    half = max(nd.hi - nd.lo) / 2;
    hmax = max(nd.hi - nd.lo);

    % ---- 占用判定（供占据/自由叶子使用，不做外扩）----
    cubeOccupied = cube_occ(c, half, model, 0);   % 与障碍有交集

    if ~cubeOccupied
        % 整块均不与障碍相交 → 均匀自由大叶子
        
        leaves(end + 1) = struct('center', c, 'half', half, 'occ', false); %#ok<AGROW>
        continue;
    end

    % 中心 + 8 角点是否全在障碍内(允许按 sigma 外扩的安全判据)
    pts = corner_pts(c, half);
    occ(1) = point_occ(c, model, sigma); %#ok<AGROW>
    for k = 1:8
        occ(k + 1) = point_occ(pts(k, :), model, sigma); %#ok<AGROW>
    end

    if all(occ)
        % 整体都在障碍内 → 均匀占据大叶子
        leaves(end + 1) = struct('center', c, 'half', half, 'occ', true); %#ok<AGROW>
        continue;
    end

    if hmax <= mincell
        % 边界且已达最小单元 → 按实际占据生成叶子
        leaves(end + 1) = struct('center', c, 'half', half, ...
            'occ', cubeOccupied); %#ok<AGROW>
        continue;
    end

    % ---- 细分 8 个八分体 ----
    dh = half / 2;
    for sx = [-1 1]
        for sy = [-1 1]
            for sz = [-1 1]
                cc = c + [sx sy sz] .* dh;
                stack(end + 1) = struct('lo', cc - dh, 'hi', cc + dh); %#ok<AGROW>
            end
        end
    end
end
end

% =========================================================================
function pts = corner_pts(c, half)
d = [half, half, half];
pts = [c + [-1 -1 -1] .* d; c + [1 -1 -1] .* d; c + [-1 1 -1] .* d; ...
       c + [1 1 -1] .* d; c + [-1 -1 1] .* d; c + [1 -1 1] .* d; ...
       c + [-1 1 1] .* d; c + [1 1 1] .* d];
end

function o = point_occ(p, model, sigma)
o = min_dist(p, model) < sigma;
end