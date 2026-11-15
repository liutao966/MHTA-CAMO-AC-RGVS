# 论文代码运行说明

本文件对应论文《Hierarchical Deep-Fusion Path Planning of UAVs in Unstructured Terrain With Global Optimization and Real-Time Dynamic Obstacle Avoidance: A Unified MHTA-CAMO-AC-RGVS Framework》的**每个实验用哪个代码运行、单个算法怎么单独跑**。

---

## 0. 目录结构

```
论文/
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
    ├── MHTA-CAMO-AC-RGVS                 # 实时性实验（IV-I）
    └── obstacle_gradient_exp/
        ├── sim_rgvs.m                  # ★ 本文算法动态层 RGVS（权威）
        ├── sim_aapf.m / sim_avss.m     # 动态基线 A*+APF / AVSS-RRT
        ├── sim_vo.m / sim_rvo.m / sim_dwa.m   # 动态基线 VO / RVO / DWA
        └── gen_obstacles_v2.m          # 四类动态障碍生成器（直线/圆弧/变速/垂直）
```

---

## 1. 论文实验 ↔ 运行代码对照表

| 论文章节 | 实验内容 | 运行代码 | 输出 |
|---|---|---|---|
| IV-A | 实验环境与参数设置 | （各脚本内置，无需单独运行） | — |
| IV-B | 山地场景 8 算法横向对比（30 次均值±std + 显著性 + 箱线图） | `matlab_project/run_stat_exp.m` | `matlab_project/stat_results/stat_results_obstacle.xlsx` |
| IV-C | 城市 / 丘陵多环境泛化（30 次 + 显著性） | `matlab_project/run_stat_exp.m` | `stat_results_city.xlsx` / `stat_results_hill.xlsx` |
| IV-D | AHP 权重敏感性分析 | `matlab_project/AHP.m` | 控制台 + 敏感性数据 |
| IV-E | 帕累托前沿分析 | `matlab_project/MHTA-CAMO-AC/pareto_front_analysis.m` | Pareto 图（3D/散点/雷达） |
| IV-F | 四类动态障碍主对比（6 算法 × 30 seeds） | `project_dynamic/run_dynamic_benchmark.m` | `project_dynamic/dynamic_benchmark_results.xlsx` |
| IV-G | 障碍类型分组实验（4 类型 × 6 算法） | `project_dynamic/run_type_grouping.m` | `project_dynamic/type_grouping_results.xlsx` |
| IV-H | 传感不确定性鲁棒性（σ=0~2 m） | `project_dynamic/run_sensing_uncertainty.m` | `project_dynamic/sensing_uncertainty_results.xlsx` |
| IV-I | 实时性（单步决策耗时，2/6/11 障碍） | `project_dynamic/run_realtime.m` | `project_dynamic/realtime_results.csv` |
| IV-J | 消融实验（full / no_mhta / no_levy / no_repair / no_postproc / no_safety） | `matlab_project/MHTA-CAMO-AC/run_ablation.m` | `MHTA-CAMO-AC/ablation_results/ablation_results.xlsx` |

---

## 2. 各实验运行方式

### 2.1 静态横向对比（IV-B / IV-C）

**代码**：`matlab_project/run_stat_exp.m`

8 个算法在**同一张地图**上各独立运行 30 次，统一算 6 个指标（路径长度 / 最小安全距离 / 能耗 / 飞行时间 / 规划时间 / 平滑度），输出 RawData（30 次原始）+ Summary（均值±std）到 xlsx。

```matlab
cd matlab_project
run_stat_exp()                  % 全部算法 × 30 次
run_stat_exp(1, 5)              % 冒烟测试（只跑 5 次，快速验证）
```

> 8 个算法 = 本文算法 + 7 个静态基线（Octree / ACOSRAR / RRT* / NMOPOS / Chaos-SSA / HSMA / Improved RRT*）。
> 切换地图：脚本内 `map_type`（`obstacle` / `city` / `hill`）控制，三个地图分别生成三份 xlsx。

### 2.2 AHP 权重敏感性（IV-D）

**代码**：`matlab_project/AHP.m`

对论文 5 目标权重 `[0.40, 0.35, 0.10, 0.10, 0.05]` 每个权重单独 ±50%（再归一化），固定种子重跑本文算法，观察各指标稳定性。

```matlab
cd matlab_project
AHP
```

### 2.3 帕累托前沿（IV-E）

**代码**：`matlab_project/MHTA-CAMO-AC/pareto_front_analysis.m`

```matlab
cd matlab_project/MHTA-CAMO-AC
pareto_front_analysis
```

### 2.4 消融实验（IV-J）

**代码**：`matlab_project/MHTA-CAMO-AC/run_ablation.m`

```matlab
cd matlab_project/MHTA-CAMO-AC
run_ablation()                    % 全部 6 变体 × 30 次
run_ablation(1, 5)                % 冒烟测试
run_ablation(1, 30, 'no_safety')  % 只跑单个变体
```

> 6 个变体：`full` / `no_mhta`（去 MHTA 全局层）/ `no_levy` / `no_repair` / `no_postproc` / `no_safety`。

### 2.5 动态主对比（IV-F）

**代码**：`project_dynamic/run_dynamic_benchmark.m`

6 算法 × 30 seeds，14 个四类障碍（直线×3 / 圆弧×3 / 变速×4 / 垂直×4），输出成功率/碰撞率 + 路径/能耗增长率 + 统计显著性（McNemar/Wilcoxon/Cliff's delta/Holm）。

```matlab
cd project_dynamic
run_dynamic_benchmark
```

### 2.6 障碍类型分组（IV-G）

**代码**：`project_dynamic/run_type_grouping.m`

```matlab
cd project_dynamic
run_type_grouping
```

### 2.7 传感不确定性（IV-H）

**代码**：`project_dynamic/run_sensing_uncertainty.m`

```matlab
cd project_dynamic
run_sensing_uncertainty
```

### 2.8 实时性（IV-I）

**代码**：`project_dynamic/run_realtime.m`

```matlab
cd project_dynamic
run_realtime
```

---

## 3. 单个算法单独运行

### 3.1 本文算法 —— 静态版（MHTA + CAMO-AC）

**权威代码**：`matlab_project/MHTA-CAMO-AC/solve_this_v2.m`（函数版，自包含）

```matlab
addpath('matlab_project/MHTA-CAMO-AC')
[p, t_camo, metrics, t_mhta] = solve_this_v2('obstacle', false)
% p      : 最终平滑路径 N×3
% t_camo : CAMO-AC 局部层耗时 (s)
% metrics: 5 目标值 [L, E, unsmooth, tm, dmin]
% t_mhta : MHTA 全局层耗时 (s)

% 可选：自定义权重、外部传入高楼（city）
[p, t, m] = solve_this_v2('obstacle', false, [0.40, 0.10, 0.10, 0.05, 0.35])
[p, t, m] = solve_this_v2('city',  false, [], buildings)
```

**脚本版**（直接运行、带可视化）：`matlab_project/MHTA-CAMO-AC/untitled.m`

```matlab
% 打开 untitled.m，改第 14 行 maps = {'obstacle'} 为 {'city'} / {'hill'} / {'obstacle'}
% 直接 F5 运行
```

### 3.2 本文算法 —— 动态版（RGVS）

**权威代码**：`project_dynamic/obstacle_gradient_exp/sim_rgvs.m`

```matlab
[success, trail, status] = sim_rgvs(path, dyn, start, goal, env, sigma)
% path    : 全局参考路径（由 plan_this → solve_this_v2 生成）
% dyn     : 动态障碍结构体（由 gen_obstacles_v2 生成）
% sigma   : 障碍物观测噪声标准差 (m)，默认 0
% success : 是否到达目标
% status  : 0=超时 1=到达 2=碰撞
```

### 3.3 动态基线

| 算法 | 代码 |
|---|---|
| A*+APF | `project_dynamic/obstacle_gradient_exp/sim_aapf.m` |
| AVSS-RRT | `project_dynamic/obstacle_gradient_exp/sim_avss.m` |
| VO | `project_dynamic/VO/sim_vo.m` |
| RVO | `project_dynamic/RVO/sim_rvo.m` |
| DWA | `project_dynamic/DWA/sim_dwa.m` |

### 3.4 静态基线（7 个）

由 `run_stat_exp.m` 内部统一调用（脚本内的 `solve_Octree` / `solve_ACOSRAR` / `solve_RRT*` / `solve_NMOPOS` / `solve_ChaosSSA` / `solve_HSMA` / `solve_ImprovedRRT*` 适配器），单独跑可参考 `run_stat_exp.m` 里对应适配器的调用方式。

---

## 4. 重要说明（避免混淆）

1. **命名与论文一致**：代码已统一按论文框架命名——**MHTA**（全局搜索，对应 `mhta_search`）、**CAMO-AC**（局部多目标优化，对应 `camo_ac_optimize`）、**RGVS**（动态反应层，对应 `sim_rgvs.m` / `rgvs_plan`）。本文算法 = MHTA + CAMO-AC + RGVS 三层。

2. **静态权威代码**是 `matlab_project/MHTA-CAMO-AC/solve_this_v2.m`（函数版，自包含全部子函数）；`project_dynamic/solve_this_v2.m` 是它的副本，供动态实验生成全局路径用，二者功能一致。

3. **动态权威代码**是 `project_dynamic/obstacle_gradient_exp/sim_rgvs.m`（RGVS 反应层）；`project_dynamic/plan_this.m` 生成全局参考路径。

4. `project_dynamic/MHTA-CAMO-AC-RGVS/untitled.m` 是**独立的"静态+动态迎面避障演示"脚本**（不是基准的一部分，用样条平滑、无 city 楼，同样已统一为 MHTA/CAMO-AC/RGVS 命名），仅供可视化演示，不影响论文数据。

5. **本文算法静态版的关键参数**（与论文一致）：`safe_dist=2.5`、`drone_speed=10`、AHP 权重 `[0.40, 0.10, 0.10, 0.05, 0.35]`（代码顺序 [长度, 能耗, 平滑, 时间, 安全]）、`pop_size=30`、`max_iter=120`、`q0=0.85`、`step_size=1.2`。

6. **本文算法动态版（RGVS）关键参数**（与论文一致）：`uav_speed=2.0`、`look_ahead=28`、`vmin=0.3 / dvmax=0.6 / dthmax=1.5 / pred=8 / w_head=2.0 / w_clear=3.0 / w_speed=0.4 / w_rot=0.2`、俯仰候选 `[-0.45, 0, 0.45]`、航向候选 21 个、速度候选 5 个（5×21×3=315）。
