# 论文代码运行说明

本文件对应论文《Hierarchical Deep-Fusion Path Planning of UAVs in Unstructured Terrain With Global Optimization and Real-Time Dynamic Obstacle Avoidance: A Unified MHTA-CAMO-AC-RGVS Framework》的**每个实验用哪个代码运行、单个算法怎么单独跑**。

---

## 0. 目录结构

```
MHTA-CAMO-AC-RGVS/
├── matlab_project/                     # 静态实验（MHTA 全局层 + CAMO-AC 局部层，论文 IV-A~E、IV-J）
│   ├── run_stat_exp.m                  # 8 算法 × 30 次横向对比（IV-B 山地 / IV-C 城市+丘陵）
│   ├── AHP.m                           # AHP 权重敏感性（IV-D）
|   ├── AHP.txt                         # AHP 权重敏感性实验的终端实验数据
│   ├── eval_metrics.m                  # 统一 5 目标指标计算
│   └── MHTA-CAMO-AC/
│       ├── solve_this_v2.m             # ★ 本文算法静态版（权威，函数版，自包含 MHTA + CAMO-AC）
│       ├── untitled.m                  # ★ 本文算法静态版（脚本版，带可视化）
│       ├── ablation_solver.m           # 消融求解器（IV-J，被 run_ablation.m 调用）
│       ├── run_ablation.m              # 消融实验入口（IV-J）
│       ├── pareto_front_analysis.m     # 帕累托前沿（IV-E，依赖 dominates.m find_pareto_front.m）
│       ├── dominates.m                 # 非支配判断（帕累托依赖）
│       ├── ablation_results            # 消融实验数据文件夹
│       ├── pareto                      # 帕累托实验数据文件夹
│       ├── stat_results                # 30次运行数据文件夹
│       └── find_pareto_front.m         # 帕累托筛选（帕累托依赖）
│
└── project_dynamic/                    # 动态实验（RGVS 反应层 + 动态基线，论文 IV-F~I）
    ├── solve_this_v2.m                 # 本文算法静态版副本（动态基准用它生成全局路径）
    ├── plan_this.m                     # 调 solve_this_v2 生成全局路径（供动态仿真）
    ├── run_dynamic_benchmark.m         # 四类动态障碍主对比（IV-F）
    ├── run_type_grouping.m             # 障碍类型分组实验（IV-G）
    ├── run_sensing_uncertainty.m       # 传感不确定性（IV-H）
    ├── run_realtime.m                  # 实时性实验（IV-I）
    ├── MHTA-CAMO-AC-RGVS               # 本文算法动态版（脚本版，带可视化）
    ├── data                            # 动态实验数据文件夹
    └── obstacle_gradient_exp/
        ├── sim_rgvs.m                  # ★ 本文算法动态层 RGVS（权威）
        ├── sim_aapf.m / sim_avss.m     # 动态基线 A*+APF / AVSS-RRT
        ├── sim_vo.m / sim_rvo.m / sim_dwa.m   # 动态基线 VO / RVO / DWA
        └── gen_obstacles_v2.m          # 四类动态障碍生成器（直线/圆弧/变速/垂直）
```
