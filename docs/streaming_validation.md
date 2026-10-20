# 流式建图实施与验证记录

更新：2026-10-21。环境：`vslam`、Ubuntu 22.04、ROS 2 Humble，本仓库独立 install。
运行结果在 Git 忽略的 `outputs/`，日志在 `log/`。以下是实际验证范围，不代表几十公里精度保证。
操作命令见 [中文 README](../README.md#大场景流式建图) / [English README](../README.en.md#large-scale-streaming-mapping)。

## 1. 已实现与保持不变的部分

- `super_lio_mapping`：不可变本体静态帧、实际去畸变参考时刻及配套原始位姿；有界异步写盘；
  帧元数据和 XYZI 校验；持久化块/索引；离线索引恢复；外存排序归并的加权体素、瓦片和 ROI 导出。
- `super_lio_interfaces`：持久化通知/状态，以及异步导出接口。
- `super_lio_backend`：按需读帧、关键帧图、落盘描述子、Scan Context 检索、KISS-Matcher 粗配准、
  small_gicp 精配准、双向重叠/残差/退化/重力/邻帧检查、延迟未来帧复核、鲁棒 iSAM2、撤销与图重建。
- 全部有效静态处理帧仍参与最终建图；配准关键帧和预览抽样不影响保存密度。
  NCLT 最终 0.10 m，Retail 最终 0.05 m。前端估计、外参、M-detector 和 OctVox 参数保持基线。
- 在线路径限长、计时统计改为在线统计、应用输入和写盘队列限额；异常明确失败，不静默丢建图帧。
- 新统一 launch、退出自动导出、原始/优化版本分开保存、局部坐标瓦片浏览和高度配色。
  原有数据集 launch 保留 legacy 保存行为。

## 2. NCLT 完整旧模式对照

同一构建、相同参数、动态筛选开启、2 倍回放。数据长度约 1,026 秒，轨迹约 1.1 km。
对照目录：`outputs/streaming_legacy_full_control`、`outputs/streaming_nclt_full`。

| 指标 | legacy | streamed |
|---|---:|---:|
| 实际输出/建图帧 | 10,200 | 10,200 |
| 收到 LiDAR / IMU | 10,212 / 96,629 | 10,212 / 96,629 |
| 有效静态输入点 | 42,510,562 | 42,510,562 |
| 原始地图点（0.10 m） | 12,704,144 | 12,704,144 |
| 前端进程峰值 RSS，含退出保存 | 3,868.41 MiB | 823.05 MiB |
| 前端平均处理时间 | 16.89 ms | 16.36 ms |
| 前端最大处理时间 | 53.80 ms | 60.97 ms |
| 节点退出 / 回放验证 | 通过 | 通过 |

10,200 个共同时间戳逐行比较，最大 XYZ 差 **0 m**，最大四元数分量差 **0**。
帧存储 submitted=durable=10,200，校验记录完整；写盘队列峰值约 0.236 MiB。
初始化未产出帧及末尾 2 个缺少后续 IMU 覆盖的扫描不属于存储丢帧。
两个报告的监控话题匹配计数可能少于持久化帧数；完整性以节点计数、轨迹、索引与 manifest 交叉核对。

原始地图 42 个瓦片加回 double 原点后，对旧 PCD 采样检查 36,910 点，最大最近邻坐标差
**0.00003185 m**，最大强度差 **0**；预设容差 0.0001 m，通过。
该微小差异来自局部瓦片 float 坐标的重编码，不涉及降低分辨率。
报告：`outputs/streaming_nclt_full/tile_roundtrip_check.json`。

NCLT 约 120 秒片段另用旧体素滤波直接重放相同 2,838,920 个存储输入点：
920,266 个输出点逐点匹配，XYZ 和强度最大差均为 **0**。
日志：`log/backend/nclt_reference_check.log`。这验证了外排序体素归并的等价性，
不能把“地图点数相同”单独作为精度证据。

## 3. Retail_Street 与动态过滤

完整流式回放：`outputs/retail_streamed_full`。

| 指标 | 流式结果 |
|---|---:|
| 有效帧 / 持久化帧 | 1,347 / 1,347 |
| 静态输入点 / 动态筛除点 | 8,711,398 / 51,913 |
| 0.05 m 地图点 | 1,020,251 |
| 前端峰值 RSS | 523.16 MiB |
| 平均 / 最大处理 | 20.34 / 48.02 ms |
| 回放、轨迹/点云有效性、退出 | 通过 |

对相同存储输入重新执行旧滤波，1,020,251 个地图点的 XYZ 和强度差均为 **0**。
日志：`log/backend/retail_reference_check.log`。

独立 legacy 回放 `outputs/retail_legacy_control` 为 1,346 帧，峰值 964.24 MiB。
它少接收到起始的一帧 LiDAR 和部分 IMU，与流式输入不完全相同；不把这两次地图/轨迹差异
归因于保存算法，也不声称两次回放逐帧完全一致。严格的保存等价性使用上述同输入旧滤波对照。
处理耗时是并发验证时的单次结果，不能据此宣称前端加速或性能退化。

## 4. 后端功能与真实质量边界

`outputs/graph_unit_06`：合成 81 帧，包含有已知漂移的往返、反向视角和三维静态结构。

- 无回环时逐帧位姿完全保持原值。
- 全局粗配准在 180° 反向视角下通过方向与平移检查。
- 3 条接受回环均通过未用于配准的未来静态帧检查，终点误差从 0.32985 m 降到 0.13939 m。
- 明显错误边注入被拒绝；逐条撤销后全部帧恢复原始位姿。
- 合成测试不等于真实重复走廊、低重叠、动态残留负样本集已经齐备。

ROS 验证 `outputs/backend_ros_03`：异步保存、81 帧快照、17,296 地图点、964 点预览
（预算 1,000）、无可撤销边时明确返回失败、正常退出均通过。

完整 NCLT 新图版本 `outputs/streaming_nclt_full/graph_quality_checked`：
10,200 帧、704 个关键帧；13 个通过邻帧一致性的候选进入未来帧复核，接受 6 条、拒绝 7 条。
候选在当前关键帧之后至少等待 5 帧，用当前/历史锚点之后第 3 帧静态云复核，
任一方向的 1 m 截断最近邻 RMSE 变差则拒绝。审计保存在 `loops.jsonl`。

随后使用两端锚点之后第 8 帧做额外独立检查，这些帧没有参与配准与接受门槛。
6 对重访只有 1 对在最终图上双向均不变差，其余有约 0.01–0.19 mm 的轻微增加。
这种量级不构成已证实的显著精度下降，但**未满足全面不变差的严格门槛**，也不足以证明优化提高质量。
因此新入口默认 `loop_closure:=false`，自动导出原始稠密地图；优化结果仅作为独立待审核版本。
没有可靠 GT，未计算绝对 ATE/RPE，不把描述子命中或回环数量当成真实精度指标。
报告：`outputs/streaming_nclt_full/quality_checked.json`。

上述图的最终导出 `outputs/streaming_nclt_full/maps/quality_checked`：
10,200 帧、42,510,562 输入点、12,711,364 地图点、42 个瓦片，耗时 9.00 秒，
导出子进程峰值 RSS **136.92 MiB**。这是独立导出资源，不包含前后端和 RViz 并发峰值。
旧实验图 `graph/` 及 `map/optimized/` 保留作历史结果，使用当前结果请选 `graph_quality_checked`。

## 5. 故障、启动和格式验证

| 检查 | 结果与证据 |
|---|---|
| 无损帧读写、负坐标、体素边界、不同外排序预算、count 加权 | `outputs/mapping_unit_03`，通过 |
| 索引损坏后从完整块恢复、负载校验失败拒绝 | `mapping_check`，通过；未提交 `.part` 保留 |
| 队列溢出 | incomplete 与失败原因落盘，close 返回失败，锁释放后可离线恢复；通过 |
| 部分写入 | 用子进程 RLIMIT_FSIZE 模拟磁盘写失败，durable 不前移、失败地图不发布；`outputs/mapping_fault_01`，通过 |
| 无效保存模式、乱序 IMU 超过队列上限 | ROS 注入验证 `outputs/input_guards_02`，通过 |
| NCLT 新统一 launch | `outputs/large_launch_nclt_04`；238 帧、231,184 点，原始/优化输出同帧、默认无回环，通过 |
| Retail 新统一 launch | `outputs/large_launch_retail_01`；243 帧、299,138 点、0.05 m、动态筛选开启，通过 |
| RViz 配置 / 地图浏览 | YAML、launch 参数加载通过；ROS 点云浏览已通过；本次无桌面鼠标交互人工验收 |
| 构建与代码风格 | `./scripts/build_humble.sh --with-backend` 完成 7 包；C++ 修改按 clang-format、Python 语法检查、Git whitespace 检查通过 |

最终输出文件在写入/关闭成功并同步后发布 manifest；临时目录不能视作完整地图。
存储块 fsync 后再提交索引。模拟部分写入不等于已完成真实 SSD 掉电/文件系统损坏测试。

## 6. 复现命令

在容器内加载本仓库环境，测试输出目录须为新目录：

```bash
./scripts/build_humble.sh --with-backend
source install/setup.bash
ros2 run super_lio_mapping mapping_check outputs/check_store_new
ros2 run super_lio_mapping mapping_fault_check outputs/check_fault_new
ros2 run super_lio_backend graph_check outputs/check_graph_new
python3 super_lio_backend/tests/ros_check.py outputs/check_graph_new/store outputs/check_ros_new
python3 scripts/check_input_guards.py outputs/check_guards_new
python3 super_lio_backend/tests/launch_check.py data/NCLT/20130110 outputs/check_launch_new
python3 scripts/validate_nclt.py data/NCLT/20130110 \
  --output outputs/check_full_new --dynamic --rate 2 --streamed
python3 scripts/validate_retail_street.py \
  /work/code/3D/fast_ws/src/FAST-LIVO2-ROS2/data/Retail_Street \
  --output outputs/check_retail_new --dynamic --streamed --export-pcd
ros2 run super_lio_mapping mapping_reference_check \
  outputs/check_retail_new/map/store outputs/check_retail_new/map/raw/map.pcd .05
ros2 run super_lio_backend superm_optimize outputs/check_full_new/map/store \
  outputs/check_full_new/graph_review
python3 scripts/check_mapping_quality.py outputs/check_full_new/map/store \
  outputs/check_full_new/graph_review outputs/check_full_new/quality.json
```

`mapping_reference_check` 是小规模对照工具，限制输入不超过 1,000 万点；它为了执行旧算法
会累计参考点，不属于运行时流式流程。质量检查脚本额外依赖验证环境中的 NumPy/SciPy。
不同回放需核对实际输入，才能进行严格的轨迹等价比较。

## 7. 未完成的准入条件

- 真实连续 5/10/30–50 km 数据，以及前端 float 在大坐标下的专项验证；尚无移动原点。
- 整容器前端＋后端＋导出＋RViz 并发内存/CPU/磁盘预算，长期慢盘、断电和进程强杀恢复矩阵。
- 标注的真假回环集合、固定局部结构 ROI 的墙厚/点面残差、独立 GT 与重复运行波动。
- 更细粒度的局部几何优化、信息自适应节点增密、GNSS、跨 session 重定位不在已交付范围。

点云工作集已受限，索引/图/检索键仍随有效帧数增长；不能声称整个程序固定常量内存。
无重访、无外部约束的长路线仍会累积漂移。默认原始地图保留当前局部质量，
开启回环须单独审核优化结果。S8 尚未通过，不宣称 30–50 km 已经验证可用。

## 8. 独立提交与回退

| 提交 | 功能 |
|---|---|
| `37b3dd5` | 帧存储与外存地图导出 |
| `d6fcdc3` | 前端异步接入、持续运行边界和公共接口 |
| `d319033` | 真实点云旧体素滤波等价对照工具 |
| `9d836b2` | 固定版本、隔离安装后端依赖 |
| `50de8a8` | 回环、位姿图、撤销、ROS 服务及浏览器 |
| `0b332b1` | 延迟未来帧质量门槛与独立质量检查 |
| `eaa08fa` | 依赖许可证副本格式统一 |
| `f1db231` | 部分写入检测、incomplete 状态及故障注入 |
| `329287a` | 乱序输入限额与参数验证 |
| `011683b` | 大场景启动、自动导出与瓦片显示工作流 |
| `defcc3b` | 容器共享目录版本记录与日志路径修复 |

每项完成相应验证后提交。依赖性的回退应先撤销较晚的使用方，再撤销底层功能，
或在新工作树检出目标提交进行对照；不要对含个人修改的工作区使用 `reset --hard`。
未改动或提交用户预先存在的 Livox 驱动个人修改。
