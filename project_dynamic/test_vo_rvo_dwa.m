% =========================================================================
%  VO / RVO / DWA 三算法冒烟测试（四类障碍物 gen_obstacles_v2）
% -------------------------------------------------------------------------
%  验证三个速度空间反应式基线 sim_vo / sim_rvo / sim_dwa 能正常处理四类障碍
%  （直线/圆弧/变速/垂直），并与本方法 sim_rgvs 做个成功率的快速对照。
% =========================================================================
clear; clc; close all;

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'obstacle_gradient_exp'));
addpath(fullfile(root, 'VO'));
addpath(fullfile(root, 'RVO'));
addpath(fullfile(root, 'DWA'));

env   = build_env();
start = env.start; goal = env.goal;

fprintf('生成共同参考路径 (plan_this) ...\n');
ref_path = plan_this(env);

nseed  = 20;
algos   = {'VO', 'RVO', 'DWA', 'This algorithm'};
simFcns = {@sim_vo, @sim_rvo, @sim_dwa, @sim_rgvs};

fprintf('======== VO/RVO/DWA 冒烟测试 (14 四类障碍) ========\n');
for a = 1:numel(algos)
    succ = zeros(nseed, 1);
    for s = 1:nseed
        dyn = gen_obstacles_v2(s, ref_path, env);
        [ok, ~] = simFcns{a}(ref_path, dyn, start, goal, env);
        succ(s) = ok;
    end
    fprintf('[%s] 成功率 = %5.1f%% (%d/%d)\n', algos{a}, mean(succ) * 100, sum(succ), nseed);
end
fprintf('================ 完成 ================\n');
