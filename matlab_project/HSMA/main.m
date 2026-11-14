% =========================================================================
% main.m —— HSMA 多目标路径规划程序入口（直接运行本文件即可）
% -------------------------------------------------------------------------
% HSMA = Hybrid Slime Mould Algorithm（混合黏菌算法）
% 参考: Mohamed Abdel-Basset et al., "A Multiobjective Optimization
%       Algorithm for Safety and Optimality of 3-D Route Planning in UAV",
%       IEEE Trans. AES, 2024.
%
% 运行后依次弹出三组结果(每组 3 张图):
%   1) 城市地图(city_map.m 平底+高楼)
%   2) 丘陵地图(Hill_map.m 7 座高斯山体)
%   3) 障碍物地图(map.m 3 座山+禁飞球+9 根圆柱)
% 每组含: 三维地图+Pareto 航线、Pareto 前沿散点图、收敛曲线。
% 环境与碰撞经 model.type 分派, 算法主体(hsma_multiobjective)完全一致。
% 也可单跑: run_hsma('city') / run_hsma('hill') / run_hsma('obstacle')。
% 6 个目标: [路径长度, 威胁代价, 安全距离成本, 能耗, 不平滑度, 飞行时间]
% =========================================================================
clear;
clc;
close all;

% 城市地图(起终点 [10,10,60]→[190,190,60], 与 AVSS-RRT 城市地图一致)
run_hsma('city');

% 丘陵地图(起终点 [10,10,50]→[190,190,80])
run_hsma('hill');

% 障碍物地图(map.m: 3 座山 + 球形禁飞区 + 9 根圆柱, 起终点 [10,10,50]→[190,190,80])
run_hsma('obstacle');