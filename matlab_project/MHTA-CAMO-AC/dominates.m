function d = dominates(a, b, obj_minimize)
%% DOMINATES 判断解 a 是否 Pareto 支配解 b
% 输入:
%   a            - 解 a 的目标向量 (1 x m)
%   b            - 解 b 的目标向量 (1 x m)
%   obj_minimize - 1 x m 布尔向量, true=最小化目标, false=最大化目标
% 输出:
%   d            - true 表示 a 支配 b

if nargin < 3
    obj_minimize = true(1, length(a));
end

a_work = a; b_work = b;
for i = 1:length(a)
    if ~obj_minimize(i)
        a_work(i) = -a_work(i);
        b_work(i) = -b_work(i);
    end
end

all_better_or_eq = all(a_work <= b_work);
strictly_better  = any(a_work <  b_work);

d = all_better_or_eq && strictly_better;
end