# NCLT 20130110 — ROS 2 Humble

> 新运行的日志集中保存在 `log/validation/`；请查看 `report.json` 中的
> `log_dir`、`node_log`、`bag_log` 获取路径。以下旧结果的日志位置保持历史记录。
> 日志目录说明见 [README](../README.md#日志目录)。

## 数据与参考配置

- 原数据：`/media/zty/新加卷/Dataset/lightning-lm/NCLT/20130110`。
- 容器可见副本：`/work/code/3D/SuperM-LIO/data/NCLT/20130110`。
  原始数据没有修改。当前容器仅挂载 `/work/code/3D`，因此不能使用指向宿主机 `/media` 的软链接。
- 参考：`/work/code/3D/lightning-lm/config/default_nclt.yaml` 及该工程的
  `src/core/lio/pointcloud_preprocess.cc`。没有依赖或加载 Lightning 的安装环境。
- SQLite rosbag2，7,563,796,480 字节；元数据版本 8 可直接由当前 Humble 读取。
- 时长 1025.886353 秒；点云 10,212 帧、IMU 96,629 条。
  GPS 话题不参与本程序建图。

| 输入 | 包内名称 | ROS 类型 | 单位/约定 |
|---|---|---|---|
| 点云 | `points_raw` | `sensor_msgs/msg/PointCloud2` | `x,y,z` FLOAT32 米 |
| 点内时间 | 字段 `time`，offset=16 | FLOAT32 | 相对扫描开始，微秒，乘 `1e-6` 得秒 |
| 扫描线 | 字段 `ring`，offset=20 | UINT16 | 0–31，不要求用于当前 LIO |
| IMU | `imu_raw` | `sensor_msgs/msg/Imu` | 加速度 m/s²，角速度 rad/s |

点云 point_step=22，frame_id=`velodyne`，没有 intensity 字段，适配器使用固定强度 1。
节点订阅使用绝对名称 `/points_raw`、`/imu_raw`；`ros2 bag play --topics` 使用包内相对名称。

整包检查覆盖 341,228,654 个原始点：点内时间范围 0.000045–0.133266 秒，
没有越界时间、扫描内乱序、扫描重叠或非递增的点云头时间戳。
原始点云最大时间间隔 0.137142 秒。原始扫描点数范围 6,368–66,793。
扫描结束时间取全帧有效时间最大值，不受空间过滤和采样步长影响。

## 点云颜色与结构查看

这份转换包没有 RGB 或反射强度字段；保存地图中的 `intensity` 因而都是 1。
按 Intensity 着色会显示成同一种颜色。XYZ 几何仍然保留，不需要重新建图。

`nclt.launch.py` 现在使用专用 `rviz/nclt.rviz`：静态点云按世界坐标 Z 高度着色，
动态点保持红色，鼠标工具仍默认 Move Camera。高度配色只用于显示，不写入 PCD。

当前已打开的 RViz 可直接在 Displays → LiDAR 中设置：

- `Color Transformer: AxisColor`，`Axis: Z`，`Use Fixed Frame: true`。
- 关闭 `Autocompute Value Bounds`，展开它，将 `Min Value` 设为 -8、`Max Value` 设为 12。
  单位为米。固定范围能保持不同帧颜色一致；若只看局部低矮结构，可进一步缩小范围。
- `Style: Points`，`Size (Pixels): 1` 或 2；点过大时，密集立面容易互相遮挡。
- 用俯视观察道路与建筑轮廓，用斜视观察墙面、台阶和重影。

当前静态点云显示 `Decay Time=5`，只累计最近 5 秒的扫描，不代表已保存的完整地图。
要检查最终全图，使用 CloudCompare 打开 `map.pcd`，选中点云，执行
`Edit → Colors → Height Ramp`，方向选 Z。再缩放或截取局部检查立面和层间结构。
此操作改变查看器中的颜色，原有 PCD 无需重新生成。
参考：[CloudCompare 官方高度配色说明](https://www.cloudcompare.org/doc/wiki/index.php/Colors%5CHeight_Ramp)。

检查了现有 `outputs/nclt_20261019_145119_931142/map/map.pcd`：12,706,747 个点，
强度最小值与最大值均为 1。Z 坐标抽样的 1%–99% 范围约 -7.56 至 11.04 m，
因此默认显示范围选为 -8 至 12 m。其他场景可以按实际高度调整该显示范围。

## 配置选择

使用新增的 `super_lio/config/nclt_20130110.yaml` 和 `nclt.launch.py`。
旧的 `config/NCLT.yaml` 中包含 0.28 m 的平移和历史输出路径，不能代表用户提供的
Lightning 配置，因此没有用它替代本数据的专用配置。

- 雷达到 IMU：单位旋转、零平移，遵循用户提供的 Lightning 配置。
- `imu_type=0`：输入已经是 SI 单位；初始加速度范数约 9.8047 m/s²。
- 盲区 0.5 m；每 3 点取 1 点；在线扫描体素 0.30 m；在线地图体素 0.50 m。
  比参考配置的步长 6、扫描体素 0.50 m 保留更多输入细节。
- 最终 PCD 体素 0.10 m，独立于在线地图；每 200 帧写一个分块，保存最终合并地图。
- M-detector 使用水平 360°、垂直 ±40° 的覆盖范围，关闭 KITTI 特定车辆遮罩和聚类。
  动态分类仍运行；这些阈值是适配起点，未进行带标签精度评估。
- 映射节点使用有时间预算的 `spin_all(2 ms)` 排空就绪回调，避免大点云处理降低高频 IMU 的接收频率。
- `lio.ros.reliable_input=true`：离线包使用可靠接收；默认传感器接收仍可兼容 best-effort
  发布者。自定义实时发布者必须与订阅端 QoS 匹配。

## 运行与复现

宿主机数据准备和容器启动命令见 [中文 README](../README.md#回放-nclt-20130110)
及 [English README](../README.en.md#replay-nclt-20130110)。

在容器中构建并加载本仓库环境：

```bash
cd /work/code/3D/SuperM-LIO
./scripts/build_humble.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/validate_nclt.py data/NCLT/20130110 \
  --output outputs/nclt_check --dynamic
```

输出目录必须为新目录。验证工具检查完整回放至少处理 95% 的扫描、覆盖 95% 的时间跨度，
并检查有效位姿、递增时间戳、位姿最大间隔小于 1 秒、动态点数守恒、静态建图点数一致、
PCD 和 TUM 轨迹保存以及正常退出。加速回放后会等待排队扫描处理完毕。
节点退出时输出接收、入队、同步及未完成扫描数量，便于发现输入丢帧或同步问题。

## 已完成检查

- `scripts/build_humble.sh`：四个 ament 包构建通过，仅使用 Humble 与本仓库安装环境。
- NCLT 前 60 秒动态过滤：585 帧位姿，441,563 点 PCD，时间戳/位姿/点数检查通过。
- 实际 `ros2 launch super_lio nclt.launch.py filter_before_observe:=true` 短回放：
  138 帧轨迹，保存 PCD，最终 launch 正常退出。结果在 `outputs/nclt_ordered_launch/`。
- Retail_Street 12 秒动态过滤最终回归：109 帧位姿，144,265 点 PCD，全部检查通过。

## 回调与传输回归

初始 best-effort 加速全程回放仅处理约 9,100 帧，未通过完整性检查。
NCLT 离线输入改用可靠 QoS，并与 rosbag2 的可靠发布者核对匹配。
可靠接收的初次动态测试曾因 IMU 到达时间倒退而退出；直接使用 Humble 的
`rosbag2_py.SequentialReader` 遍历整包没有发现原始时间倒退。

高负载短测进一步发现原主循环 `spin_some()` 每轮至多处理一条 IMU，接收会积压。
映射节点改为 `spin_all(2 ms)`，在保持单线程数据访问的同时连续处理就绪回调。
另一轮加速回放量化到一次 0.051418 秒的 IMU 到达乱序。因此，NCLT 专用配置还启用
`lio.ros.imu_reorder_window=0.2`：缓冲区按传感器时间排序，等待覆盖扫描结束之后的
0.2 秒 IMU，再交给滤波器积分。超过窗口的倒退仍退出；已经无法参与积分的迟到样本
单独计数，完整验收要求该计数为 0。其他数据集默认窗口为 0，保留原时间倒退保护。

包结束或暂停、连续 0.5 秒没有 IMU 输入时，缓冲区自动排空，但不外推缺失的 IMU。
退出日志保留接收、重排、迟到及同步数量，不会把异常时间静默混入轨迹。

200 秒数据、8 倍速短回放的对照结果（共享宿主机负载，不是专用性能基准）：

| 检查 | 原执行循环 | 修复后 |
|---|---:|---:|
| 没有可同步 IMU 的扫描 | 70 | 0 |
| 建图点数匹配错误 | 33 | 0 |
| 最大位姿时间间隔 (s) | 0.379 | 0.124 |
| 验证结果 | FAIL | PASS |

结果分别位于 `outputs/nclt_reliable_stress/` 和 `outputs/nclt_executor_stress/`。
这不是对所有加速倍率的保证；交互使用默认 1 倍速即可。

## 有界重排与时钟重置回归

- 人为将短包中 101 条 IMU 的到达时间延迟 80 ms，保留原传感器时间戳。
  399 帧点云、3,738 条 IMU 全部接收，101 条重排，迟到丢弃 0、空 IMU 同步 0，
  输出 388 帧有效位姿，全部检查通过：`outputs/nclt_reorder_verified/report.json`。
- 将另一短包中的一条 IMU 传感器时间回拨 1 秒：节点明确报错退出，保护检查通过。
  预期结果：`outputs/nclt_clock_reset_expectation.json`；该负例的回放报告本身应为 FAIL。

复现夹具及检查（在容器中加载本仓库环境后运行，输出路径必须全新）：

```bash
python3 tests/nclt/make_order_fixtures.py data/NCLT/20130110 \
  --output outputs/order_fixtures
ROS_DOMAIN_ID=46 python3 scripts/validate_nclt.py outputs/order_fixtures/reordered \
  --output outputs/order_check --dynamic --rate 4
# 此负例应以状态 1 退出；按 report.json 的 node_log 路径检查时间倒退错误。
ROS_DOMAIN_ID=47 python3 scripts/validate_nclt.py outputs/order_fixtures/clock_reset \
  --output outputs/reset_check --rate 4
```

## 最终完整回放

在 `vslam` 内用最终代码、独立 ROS 域同时完整回放，倍率 4，线程数 4。
计时包含共享宿主机负载，不能视作专用机器性能基准。保存体素均为 0.10 m。

| 检查 | 基础 LIO | 动态过滤 |
|---|---:|---:|
| 结果 | PASS | PASS |
| 激光回调接收数 | 10,212 | 10,211 |
| IMU 回调接收数 | 96,629 | 96,629 |
| 有效位姿 / 建图帧 | 10,200 | 10,200 |
| 空 IMU 同步 / 迟到 IMU 丢弃 | 0 / 0 | 0 / 0 |
| 无效位姿 / 时间倒退 / 点数错误 | 0 / 0 / 0 | 0 / 0 / 0 |
| 最大位姿时间间隔 (s) | 0.172 | 0.172 |
| 平均每帧处理 (ms) | 5.88 | 8.98 |
| 峰值 RSS，含最终地图合并 (MiB) | 3610.5 | 3871.3 |
| 最终 PCD 点数 | 13,669,914 | 12,704,144 |
| 最终 PCD 大小 (bytes) | 218,718,818 | 203,266,498 |
| 动态 / 静态分类点数 | — | 4,527,234 / 42,510,562 |
| 退出状态 | 0 | 0 |

两种模式的位姿均覆盖 1024.649 秒。初始化消耗 10 帧；最后两帧没有足以覆盖
扫描结束的 IMU，因此不用于建图，最终均输出 10,200 帧位姿。
动态回放中另有最后一帧未在播放器退出前完成传输，它同样属于无 IMU 覆盖的末尾，
没有改变最终轨迹及地图。运行命令和验证工具已加入 `--wait-for-all-acked 5000`，
避免可靠发布者在大消息完成传输前退出；该选项已在乱序短包回归中验证。

两个最终 PCD 均校验了 POINTS、二进制字节数及全部点的有限性，没有 NaN/Inf，
没有体素索引溢出。最终动态地图 12,704,144 点，约 193.9 MiB。

本地完整输出（Git 忽略）：

- `outputs/nclt_ordered_baseline/report.json`、`map/map.pcd`、`trajectory/NCLT_20130110.txt`
- `outputs/nclt_ordered_dynamic/report.json`、`map/map.pcd`、`trajectory/NCLT_20130110.txt`
- `outputs/nclt_map_content.json`：最终地图内容检查
- `outputs/nclt_reader_check.json`：Humble 顺序读取与原始传感器时间检查

动态分类点数包含多帧重复观测，不能解释为唯一目标数量或检测准确率。

本次验证不包含真值轨迹误差或动态目标精度评估。当前程序不包含 Lightning 配置中的
回环、位姿图优化或 GPS/RTK 融合，不能据此认为二者具有相同的全局建图精度。
