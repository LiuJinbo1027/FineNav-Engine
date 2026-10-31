# 地形配置（测试用）

每套配置一个 YAML，顶层键 `retap_terrain:` 与 `retap_core::TerrainConfig` 的字段一一对应。
**同一个文件有两个消费者**，所以调参只需要改一处：

```bash
# 离线：PCD → 地形属性，几秒钟
retap_map_tool analyze --pcd map.pcd --config config/<file>.yaml --output-dir /tmp/out

# 在线：rosbag 回放 → 增量建图 + 地形分析 + RViz
ros2 launch finenav_retap retap_rosbag.launch.py config:=<file>.yaml
```

## 与 `demo/params/` 的分工

参数集拆成两半，同一个数只存一处：

| 位置 | 管什么 | 字段 |
|---|---|---|
| `config/<file>.yaml` | **地形的含义**（标定） | `resolution`、四个硬阈值、四个权重、`collision_neighbor`、`clearance_voxels`、`support_voxels`、`min_voxel_points`、`frame_height`、`filling_radius`、`points_number_threshold` |
| `demo/params/<scenario>.yaml` | **机器人与数据源**（场景） | `map_path`、`scan_topic`、`scan_in_map_frame`、`origin_offset_*`、`update_free`、批处理策略、规划器、localizer |

launch 用 `merge_params()` 把 `retap_terrain:` 这一段合进场景文件，再把合并结果喂给
所有节点，所以 `config/` 里的文件本身必须是**完整的** `TerrainConfig`（离线工具就是
这样单独读它的），而 `demo/params/` 里不再重复这些数。

## 几何：横向 1+8R 环 + 纵向 z 窗口（两个方向都参数化）

`checkNeighborCells` 以当前体素为中心扫一个水平环 × 竖直 z 窗口的盒子。三个参数各管一个维度：

### 横向 `collision_neighbor`

编码是 `1 + 8R`：索引 0 是中心，之后每一层壳 8 个偏移（先 `±x`/`±y` 的四正方向，再四个对角）。
上游写死的 9 / 17 / 25 正好是 R=1/2/3，现在 R 可到 15。

| `collision_neighbor` | R | 外沿半跨 (R+0.5)·res @0.20 | @0.12 | @0.05 |
|---|---|---|---|---|
| 9 | 1 | 0.30 m | 0.18 m | 0.075 m |
| 17 | 2 | 0.50 m | 0.30 m | 0.125 m |
| 25 | 3 | 0.70 m | 0.42 m | 0.175 m |
| 49 | 6 | 1.30 m | 0.78 m | **0.325 m** |

选环的依据是机体宽度：(2R+1)·res ≥ 0.60 m。所以 `res 0.20` 该配 ring 9（3×0.2=0.60），
`res 0.05` 该配 ring 49（13×0.05=0.65）——**换分辨率必须同时换环，否则物理半跨跟着缩水**。

### 纵向 `clearance_voxels`（向上）

`for (z = clearance_voxels; z >= -support_voxels; z--)`：

| z | 判据 |
|---|---|
| `z == +clearance_voxels` | **硬阻挡**：任一水平偏移上有占据 → collision（这一层是"会不会撞到头顶"） |
| `z = 1 .. clearance_voxels-1` | 只查地形硬约束（坡度/粗糙度/稀疏度/可通行度），有占据且地形达标也算堵 |
| `z == 0` | 当前体素自己（同上，只查地形约束） |

上游写死的是 3，等价于 `clearance_voxels: 3`。现在是"允许 0.60 m 机体通过"这个意图 → 层数：

| res | 0.60 m 对应层数 | `clearance_voxels` | 硬阻挡层的物理高度 |
|---|---|---|---|
| 0.20 | 3 | 4 | 0.80 m |
| 0.12 | 5 | 6 | 0.72 m |
| 0.05 | 12 | 13 | 0.65 m |

> 硬阻挡层取 `ceil(0.60/res)+1` 而不是正好 `0.60/res`，让它始终落在 0.60 m
> 机体带**之上**（与上游 `±3` 时"0.60–0.80 m 才硬判"的关系一致）。要说清楚的是：
> 这只是"正上方有没有悬空物"的粗判据，不是扫掠体净空——地面到 `+C` 之间的东西
> 只走"地形约束"这一步，不会因为高度把走廊卡死。真正的净空判据需要实测的机体
> 高度与雷达安装高度，那部分数据目前没有，这里只记录参数化后的代码事实。

### 纵向 `support_voxels`（向下）

向下是 **4 层**（`z = 0, -1, -2, -3`，含当前体素），对应 `support_voxels: 3`，所有配置一致：

| z / 索引 | 判据 |
|---|---|
| `z = 0, -1` | 只查地形硬约束 |
| `z = -2`，`i = 1..8`（ring 1 四正方向） | 必须**有**占据，否则当悬崖 |
| `z = -3`，`i >= 8`（ring 2 及以外） | 必须**有**占据，否则当悬崖 |

`z = -3` 的 `i >= 8` 是上游的 off-by-one（ring 1 的索引到 8 为止），这里**保留**，
否则与上游结果不能逐体素对齐。

注意层数是按体素算的，物理深度随 res 变：0.20 → 0.60 m，0.05 → 0.15 m。
5 cm 下"悬崖"实际只探到地面以下 0.10–0.15 m。这是刻意的（先保持一致的行为），
如果要让支撑判据与分辨率无关，得再加一个米制的向下窗口，不在本版。

### `min_voxel_points`（N，证据阈值）

只作用在 `checkNeighborCells` 里：一个体素的点数 `< N` 时，**既不算占据也不算支撑**
（按 free/unknown 对待）。`0` = 关闭（上游语义：有点就算占据）。

它是一把双刃剑：N 大一点，头顶的灰尘不再假碰撞（好），但地面上的稀薄点也不再能支撑
（可能把真实平台判成悬崖）。左右两侧的正确 N 并不相同，属于下一步要拆的参数。

## 怎么读实测数字（可复现）

```bash
# 1) rosbag → map 系累积点云 + 位姿轨迹
/usr/bin/python3 demo/scripts/bag2pcd.py <bag_dir> /tmp/retap_exp/data
#    -> clouds=1220 points=3971000 poses=24402（/cloud_registered_body 按最近
#       /localization_pose 变换到 map；traj.txt 是 /localization_pose 的 x y z）

# 2) 分析 + 打分
ros2 run finenav_retap retap_map_tool analyze --pcd /tmp/retap_exp/data/bag_accumulated.pcd \
  --config config/<file>.yaml --output-dir /tmp/out
ros2 run finenav_retap retap_map_tool info --bt /tmp/out/bag_accumulated_terrain.bt \
  --histogram --trajectory /tmp/retap_exp/data/traj.txt
```

**轨迹可通行率**的定义：把 24402 个位姿去重成体素格，每格从 +0.2 m 向下扫 0.6 m，
第一个占据体素是"地面"；该体素 `collision == false` 记 passable，否则 blocked，
整列没有占据记 no_surface。机器人**确实开过去了**，所以 passable 的下界是 100%：
所有 blocked 都是假阳性（净空/支撑/地形约束误判）。

分母随 res 变（97 / 169 / 470 格），所以只比百分比，不比绝对数。
早期文档里的 20/90、37/90、68/160 来自另一份累积点云，已被下表取代；
按同一套方法重测后 `res 0.20 + ring 9` 仍是可通行率最高的低分辨率配置。

## 文件

同一份点云（3.97 M 点，24.5 m 轨迹）上的实测：

| 文件 | res | ring (R) | 半跨 | C | 占据体素 | 可通行体素 | 轨迹 passable | analysis |
|---|---|---|---|---|---|---|---|---|
| `terrain_res020_ring17.yaml` | 0.20 | 17 (2) | 0.50 m | 4 | 57 917 | 5 236 (9.0%) | 19/97 = 19.6% | 465 ms |
| `terrain_res020_ring09.yaml` | 0.20 | 9 (1) | 0.30 m | 4 | 57 917 | 6 525 (11.3%) | 38/97 = 39.2% | 459 ms |
| `terrain_res020_ring25.yaml` | 0.20 | 25 (3) | 0.70 m | 4 | 57 917 | 4 277 (7.4%) | 7/97 = 7.2% | 624 ms |
| `terrain_res012_ring17.yaml` | 0.12 | 17 (2) | 0.30 m | 6 | 141 990 | 12 296 (8.7%) | 62/169 = 36.7% | 1 151 ms |
| `terrain_res005_ring49.yaml` | 0.05 | 49 (6) | 0.325 m | 13 | 540 895 | 16 428 (3.0%) | 151/470 = 32.1% | 5 932 ms |

结论与上一版一致：**轨迹可通行率对水平半跨单调敏感**，分辨率本身（0.20 → 0.12，
半跨都是 0.30 m）只带来 39.2% → 36.7% 的小幅下降，代价是 2.5× 时间。
`res 0.20 + ring 9` 仍是低分辨率下的最优配置。

> **读"可通行体素"这一列时注意分层。** 这份回放的地图里有上下两层水平面：
> 行走地面在 z ≈ 0，上方板层在 z ≈ 3.9–4.5。按 z 切开后
> （`demo/scripts/retap_bands.py`），`terrain_res020_ring09` / `terrain_res010_ring25` /
> `terrain_res005_ring49` 三行的可通行体素中有 **97.7% / 98.3% / 91.6%** 落在
> z ∈ [2.5, 5.0] 的板层上；行走地面所在的 z ∈ [−0.6, 2.0) 只有 149 / 199 / 1 387 个。
> 0.12 那一行没有做分层。
> **所以"可通行体素"和"可通行面积"这两列主要描述的是板层，不是机器人脚下的地面**；
> 轨迹 passable 才是地面上的判据。分层表见
> [`doc/resolution_density_report.md`](../doc/resolution_density_report.md) §4。

## res 0.05 实验：分辨率本身值不值

点数分布随 res 塌陷（`info --histogram`）：

| res | =1 点 | ≤2 | ≤5 | ≤10 | ≤19 | >19（`points_number_threshold`=20 的达标线） |
|---|---|---|---|---|---|---|
| 0.20 | 32.8% | 45.0% | 60.4% | 70.6% | 78.2% | 21.8% |
| 0.12 | 38.6% | 52.1% | 68.0% | 77.4% | 83.8% | 16.2% |
| 0.05 | 50.8% | 65.0% | 79.1% | 86.6% | 91.7% | 8.3% |

> 这张表在 2026-10-31 修正过：此前版本把 `>19` 的那一档并进了 `≤19` 桶，
> 于是同一批计数被同时印成 "≤19 100.0%" 和 "≥20 13.4%"（自相矛盾），
> `≥20` 一栏实际是 ">10 点"。修正后的 `>19` 才是"能否达到 N_th=20"的上界。

`N` 与 `sparsity_threshold` 的扫描（只改这两个值，其余同 `terrain_res005_ring49.yaml`）：

```bash
sed 's/^  min_voxel_points: .*/  min_voxel_points: 3/' config/terrain_res005_ring49.yaml > /tmp/n3.yaml
sed 's/^  sparsity_threshold: .*/  sparsity_threshold: 1.01/'       config/terrain_res005_ring49.yaml > /tmp/no_sparse.yaml
```

| 变体 | 可通行体素 | 轨迹 passable | analysis |
|---|---|---|---|
| 基准（N=0, sparsity 0.70） | 16 428 (3.0%) | 151/470 = 32.1% | 5.9 s |
| sparsity 关（1.01） | 46 158 (8.5%) | 160/470 = 34.0% | 7.1 s |
| N=3 | 17 073 (3.2%) | 215/470 = **45.7%** | 6.2 s |
| N=5 | 15 830 (2.9%) | 240/470 = **51.1%** | 5.7 s |
| N=3 + sparsity 关 | 32 222 (6.0%) | 233/470 = 49.6% | 8.7 s |

可以读出的三件事：

1. **轨迹被堵的原因不是 sparsity**。关掉稀疏度阈值让"可通行体素"翻 2.8 倍
   （分层看：板层 ×2.97，行走地面 ×1.11），轨迹可通行率只从 32.1% 涨到 34.0% ——
   稀疏度影响的是板层/全图统计，不是地面走廊判定的那一步。
2. **是灰尘**。N=3 让全图可通行体素几乎不变（3.2%），分层看行走地面 ×1.64、板层 ×0.98，
   轨迹可通行率却 +13.6 个百分点；行走地面的单点体素（0.05 m 下占 46.2%）
   在净空层上制造了假碰撞。
3. **N 是双刃的**。N=5 的全图可通行体素（15 830）反而少于 N=3（17 073）：分层看
   行走地面仍涨（2 270 → 2 626），减少的是板层（14 803 → 13 204）——板层上的稀薄点
   失去了"支撑"资格，一部分水平面被判成悬崖。向上和向下的 N 应该分开。

`res 0.05` 仍然不是推荐配置：13× 的分析时间（5.9 s vs 0.46 s，内存 593 MB）、
`roughness` 在 5 cm 下变成噪声（实测 [−2334, 3558] 且含 NaN，来自单点体素的零协方差）、
以及 §支撑层物理深度只有 0.15 m。它现在的价值是**压力测试**：把灰尘、稀疏度、
支撑判据的弱点一次性放大给人看。

> 等输入（每行融合全部 1220 帧）的完整对比、密度门槛重标定的定量结论、
> 吞吐曲线与调参意见，见 [`doc/resolution_density_report.md`](../doc/resolution_density_report.md)。
