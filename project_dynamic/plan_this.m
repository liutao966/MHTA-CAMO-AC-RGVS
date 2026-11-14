function path = plan_this(env) %#ok<INUSD>
%PLAN_THIS 生成本文算法静态路径（直接复用权威 solve_this_v2，保证与静态实验一致）。
% solve_this_v2.m 已复制到本目录，自包含，无需 addpath 整个 MHTA-CAMO-AC，避免同名函数遮蔽。
rng(1);
path = solve_this_v2('obstacle', false);
end
