%% minimum_safe_distance_distribution.m
%  箱线图：展示 8 个算法在 30 次独立运行下 "最小安全距离 (Min safe dist)" 的分布。
%  数据来源：stat_results/stat_results_obstacle.xlsx 的 RawData 表。
%
%  目的（回应审稿人 R2-7）：不要只报最小值，要报安全距离的分布。
%  箱线图能直观看到：
%     - Octree 的 16.21 m 是一个退化的确定性值（IQR = 0），是保守绕行造成的；
%     - "This algorithm" 的分布又高又紧（中位数 ~8.9 m，IQR ~0.7 m）。

clear; clc; close all;

%% 1. 读取数据 -------------------------------------------------------------
here     = fileparts(mfilename('fullpath'));            % 本脚本所在目录 .../matlab_project
dataFile = fullfile(here, 'stat_results', 'stat_results_obstacle.xlsx');

% preserve 保留原始列名（含空格、括号），方便按列名访问
opts = detectImportOptions(dataFile, 'Sheet', 'RawData', ...
    'VariableNamingRule', 'preserve');
T = readtable(dataFile, opts);

colSafe = 'Min safe dist (m)';   % 最小安全距离列
colAlgo = 'Algorithm';            % 算法名列

%% 2. 原始算法名 -> 显示标签 ----------------------------------------------
% 与论文 Table V 顺序一致
algoRaw   = {'Octree','ACOSRAR','RRT*','NMOPOS','Chaos-SSA (ICSSA)', ...
             'HSMA','Improved RRT*','This algorithm'};
algoLabel = {'Octree','ACOSRAR','RRT*','NMOPOS','Chaos-SSA', ...
             'HSMA','Improved RRT*','This algorithm'};

nAlgo = numel(algoRaw);
X     = nan(30, nAlgo);           % 每个算法 30 次运行
for i = 1:nAlgo
    sel     = strcmp(strtrim(T.(colAlgo)), algoRaw{i});
    X(:, i) = T.(colSafe)(sel);
end

% （可选）按中位数从大到小排序，让两个"安全"算法靠左，视觉更清晰；
% 若想保持论文 Table V 顺序，注释掉下面两行即可。
[~, sIdx] = sort(median(X, 'omitnan'), 'descend');
X         = X(:, sIdx);
algoLabel = algoLabel(sIdx);

%% 3. 画箱线图 -------------------------------------------------------------
figure('Color', 'w', 'Position', [80 80 960 540]);
h = boxplot(X, 'Labels', algoLabel, 'Widths', 0.6, 'Symbol', '+');
hold on;   % 关键：之后叠加 patch/plot 时才不会清掉箱线图

n       = size(X, 2);
thisIdx = find(strcmp(algoLabel, 'This algorithm'));
octIdx  = find(strcmp(algoLabel, 'Octree'));

% 颜色方案（灰度打印也可区分深浅）
blue = [0.20 0.45 0.74];
red  = [0.82 0.16 0.16];
grey = [0.55 0.55 0.55];

for i = 1:n
    if i == thisIdx
        boxCol  = red;  faceCol = [1.00 0.86 0.86];   % 本文算法：红色高亮
    elseif i == octIdx
        boxCol  = grey; faceCol = [0.92 0.92 0.92];   % Octree：灰色（保守绕行）
    else
        boxCol  = blue; faceCol = [0.80 0.88 0.98];   % 其余算法：蓝色
    end

    % 箱体：用 patch 填充，同时画出边框
    xd = get(h(5, i), 'XData');   % 箱体轮廓 X 坐标
    yd = get(h(5, i), 'YData');   % 箱体轮廓 Y 坐标
    patch(xd, yd, faceCol, 'FaceAlpha', 0.55, 'EdgeColor', boxCol, ...
          'LineWidth', 1.2);

    % 中位线加粗
    set(h(6, i), 'Color', boxCol, 'LineWidth', 2.2);
    % 上下须颜色
    set(h([1 2], i), 'Color', boxCol, 'LineWidth', 1.1);
    % 离群点颜色
    set(h(7, i), 'Color', boxCol, 'MarkerSize', 5);
end

%% 4. 叠加均值（菱形），与论文中 mean±std 对应 ------------------------------
hold on;
mu = mean(X, 'omitnan');
plot(1:n, mu, 'd', 'MarkerFaceColor', [0.15 0.15 0.15], ...
     'MarkerEdgeColor', 'k', 'MarkerSize', 6, 'LineWidth', 1);

%% 5. 标出安全阈值 d_safe = 2.5 m ------------------------------------------
dSafe = 2.5;
yline(dSafe, '--', ['d_{safe} = ' num2str(dSafe) ' m'], ...
      'Color', [0.3 0.3 0.3], 'LineWidth', 1.1, 'FontSize', 10);

%% 6. 坐标轴与文字 ---------------------------------------------------------
ylabel('Minimum safe distance (m)', 'FontSize', 12);
set(gca, 'FontName', 'Times', 'FontSize', 11, 'TickDir', 'out', ...
         'LineWidth', 1, 'Box', 'off');
grid on;  grid minor;
ylim([1.5 11]);   % 预留 Octree (16.21) 与阈值 (2.5) 的空间

% 在每个箱体上方标注中位数值
med = median(X, 'omitnan');
q3  = prctile(X, 75, 1);        % 每列第 75 百分位
for i = 1:n
    text(i, q3(i) + 0.35, sprintf('%.2f', med(i)), ...
         'HorizontalAlignment', 'center', 'FontSize', 9);
end

% 图例
hold on;
hThis = patch(nan, nan, red,  'FaceAlpha', 0.55, 'EdgeColor', red);
hOct  = patch(nan, nan, grey, 'FaceAlpha', 0.55, 'EdgeColor', grey);
hOth  = patch(nan, nan, blue, 'FaceAlpha', 0.55, 'EdgeColor', blue);
hMean = plot(nan, nan, 'dk', 'MarkerFaceColor', 'k');
legend([hThis hOct hOth hMean], ...
       {'This algorithm', 'Octree (conservative detour)', ...
        'Other algorithms', 'Mean'}, ...
       'Location', 'northeast', 'FontSize', 10, 'Box', 'off');

%% 7. 保存高清图（论文插图用）-----------------------------------------------
outFile = fullfile(here, '..', 'static_data', 'safety_distance_distribution.png');
if exist('exportgraphics', 'file')
    exportgraphics(gcf, outFile, 'Resolution', 300);
else
    print(gcf, outFile, '-dpng', '-r300');
end
fprintf('图已保存到: %s\n', outFile);
