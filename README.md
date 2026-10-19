# SuperM-LIO — ROS 2 Humble

**简体中文** | [English](README.en.md)

基于 Super-LIO 的激光惯性里程计与建图程序，集成 M-detector 动态目标过滤。
仅支持 Ubuntu 22.04 / ROS 2 Humble，已在 `vslam` Docker 容器中验证。
Livox ROS 2 驱动和 SDK2 源码位于本仓库 `third_party/`，随工程一起构建。

## 构建与环境加载

从宿主机进入容器后构建：

```bash
docker exec -it vslam /bin/bash
cd /work/code/3D/SuperM-LIO
./scripts/build_humble.sh
source install/setup.bash
```

构建脚本会加载 Humble。每个新的容器终端执行：

```bash
source /opt/ros/humble/setup.bash
source /work/code/3D/SuperM-LIO/install/setup.bash
```

切换工作空间时建议使用新终端。构建和运行只需本仓库的安装环境。

## 日志目录

构建和运行日志默认集中到工作区 `log/`，该目录已被 Git 忽略：

```text
log/
├── build/                  # 构建脚本自动保存的终端输出
├── colcon/                 # colcon 分包构建日志
├── ros/                    # ROS、ros2 launch、RViz 日志
├── glog/                   # 建图/重定位节点 glog 日志
└── validation/<运行名_时间戳>/
    ├── node.log            # 验证时的建图输出
    ├── bag.log             # 验证时的播放器输出
    ├── ros/
    └── glog/
```

更新后重新构建并加载环境即可生效，无需手动重定向到工作区根目录：

```bash
./scripts/build_humble.sh
source install/setup.bash
```

构建脚本同时显示并保存输出，构建失败仍返回失败状态。验证结果继续保存在
`outputs/<运行名>/report.json`，其中 `log_dir`、`node_log`、`bag_log` 指向对应日志。
地图和轨迹仍保存在该运行的 `outputs/` 目录。
可用验证参数 `--log-dir /绝对路径/运行日志目录` 自定义单次日志位置。

默认根目录可在首次加载环境前通过 `SUPERM_LOG_DIR` 指定；显式设置的
`ROS_LOG_DIR` 和 `GLOG_log_dir` 会保留。直接执行诊断命令需要重定向时，请写入
`log/` 下，例如 `mkdir -p log/manual` 后使用 `> log/manual/check.log 2>&1`。
早期验证记录中的根目录日志文件名是历史路径，新运行采用以上目录。

## 回放 NCLT 20130110

专用配置为 `super_lio/config/nclt_20130110.yaml`，对应 Lightning 的
`default_nclt.yaml` 所使用的转换版 rosbag2。保留动态目标过滤、RViz、PCD 和轨迹保存。

当前容器没有挂载宿主机 `/media`。本次已将数据复制到仓库的
`data/NCLT/20130110`（Git 忽略），无需重新创建容器。若重新准备工作区，在**宿主机**执行：

```bash
cd /work/code/3D/SuperM-LIO
mkdir -p data/NCLT/20130110
cp -n /media/zty/新加卷/Dataset/lightning-lm/NCLT/20130110/{metadata.yaml,20130110.db3} \
  data/NCLT/20130110/
```

构建后，在第一个**容器终端**加载本仓库环境并启动：

```bash
cd /work/code/3D/SuperM-LIO
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch super_lio nclt.launch.py rviz:=true
```

第二个容器终端加载相同环境后回放。话题选择使用包内实际名称，**没有开头的斜杠**：

```bash
ros2 bag play /work/code/3D/SuperM-LIO/data/NCLT/20130110 \
  --clock --wait-for-all-acked 5000 --topics points_raw imu_raw
```

回放结束后在建图终端按 Ctrl-C，等待地图保存。输出位于
`outputs/nclt_<时间戳>/map/map.pcd` 和
`outputs/nclt_<时间戳>/trajectory/NCLT_20130110.txt`。
启动参数与 Retail_Street 相同，例如 `dyn_filter:=false`、`output_dir:=...`。

点云 `time` 为 FLOAT32 微秒相对时间；IMU 使用 m/s² 和 rad/s。
外参按 Lightning 配置采用单位矩阵、零平移。在线点云体素为 0.30 m，最终 PCD
体素为 0.10 m；可在专用 YAML 中调整 `lio.map.ds_size`。
NCLT 离线输入启用 `lio.ros.reliable_input=true`，与当前包的可靠发布者匹配。
`lio.ros.imu_reorder_window=0.2` 会在积分前整理小范围乱序的 IMU，增加约 0.2 秒缓冲。
此入口适配当前 LIO 和 M-detector，不包含 Lightning 的回环、位姿图及 GPS 融合。

完整自动验证：

```bash
python3 scripts/validate_nclt.py data/NCLT/20130110 \
  --output outputs/nclt_check --dynamic
```

目录必须为新目录；`--duration 60` 可短测，`--rate 4` 可加速回放。
详见 [NCLT 验证记录](docs/nclt_validation.md)。


NCLT 默认使用 `nclt.rviz` 按 Z 高度配色（-8 至 12 m），动态点显示为红色。
该转换包没有真实 RGB/反射强度，按 Intensity 着色会同色。
当前窗口也可将 LiDAR 的 `Color Transformer` 改为 `AxisColor`、`Axis` 选 Z。
已有 PCD 无需重建，完整地图可在 CloudCompare 中按高度查看。
详见 [颜色与结构查看](docs/nclt_validation.md#点云颜色与结构查看)。

## 依赖

当前 `vslam` 已具备所需依赖。新建 Humble 环境可安装：

```bash
apt-get update
apt-get install -y python3-colcon-common-extensions python3-yaml \
  ros-humble-ament-cmake ros-humble-rclcpp-components ros-humble-pcl-conversions \
  ros-humble-tf2-ros ros-humble-rosidl-default-generators \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins ros-humble-rviz2 \
  libeigen3-dev libpcl-dev libopencv-dev libtbb-dev \
  libgoogle-glog-dev libgflags-dev
```

`basic` 提供几何与日志库，`m_detector` 导出 `dyn_obj_filter`，`super_lio` 提供建图及
重定位节点。M-detector 原仓库的数据集评估工具保留为参考源码，当前只构建集成过滤库。

## 回放 Retail_Street

第一个容器终端加载环境并启动建图：

```bash
cd /work/code/3D/SuperM-LIO
ros2 launch super_lio retail_street.launch.py rviz:=true
```

无图形界面运行时省略 `rviz:=true`。第二个容器终端加载相同环境后回放：

```bash
ros2 bag play \
  /work/code/3D/fast_ws/src/FAST-LIVO2-ROS2/data/Retail_Street \
  --clock --topics /livox/lidar /livox/imu
```

默认 `use_sim_time:=true`，输入消息分别为 `livox_ros_driver2/msg/CustomMsg` 和
`sensor_msgs/msg/Imu`，图像数据不参与处理。回放结束后，在启动建图的终端按 **Ctrl-C**，
等待保存完成。每次启动生成独立的 `outputs/<时间戳>/`：

```text
outputs/<时间戳>/
├── map/map.pcd                  # 最终地图
├── map/PCD/scans_*.pcd          # 分块点云
└── trajectory/Retail_Street.txt # TUM 格式轨迹
```

输出目录中已有地图分块或同名轨迹时会报错，请为新实验使用新目录。

常用启动参数：

```bash
# 关闭动态目标过滤，运行基础 LIO
ros2 launch super_lio retail_street.launch.py dyn_filter:=false
# 在 ESKF 观测更新前过滤动态点
ros2 launch super_lio retail_street.launch.py filter_before_observe:=true
# 指定新的输出目录
ros2 launch super_lio retail_street.launch.py output_dir:=/tmp/superm_run_01
# 使用自定义 ROS 2 参数文件
ros2 launch super_lio retail_street.launch.py params_file:=/path/to/params.yaml
```

## 地图密度

默认 `super_lio/config/retail_street.yaml` 已改为保留全部有效输入点，处理时使用
15 cm 体素，最终 PCD 使用 5 cm 体素：

| 参数 | 修改前 | 当前默认 |
|---|---:|---:|
| `lio.sensor.filter_rate`：输入点抽样步长 | 3 | 1 |
| `lio.sensor.voxel_fliter_size`：每帧建图点云体素 | 0.30 m | 0.15 m |
| `lio.map.ds_size`：最终 PCD 体素 | 0.25 m | 0.05 m |

最终地图保存采用带符号三维坐标的稀疏体素索引，避免大范围地图使用厘米级体素时
因 PCL 全局网格索引溢出而跳过降采样。

`voxel_fliter_size` 保留了原代码中的拼写。体素和抽样步长越小，保留的细节越多，计算量、
内存和文件大小也会增加。内部地图的 `lio.hash_map.vox_resolution` 用途不同，不代表
最终 PCD 的保存分辨率。

修改 YAML 后重新执行 `./scripts/build_humble.sh`，再回放数据生成新地图。
已丢弃的测量点无法从旧 PCD 中恢复。需要进一步增密时，可尝试
`voxel_fliter_size: 0.10`、`ds_size: 0.03`，并观察处理耗时；这组可选值尚未实测。

`lio.map.if_filter: false` 可关闭最终空间降采样，保留全部静态建图输入点，动态目标过滤
仍由 `dyn_obj.en` 控制。分块保存和 `lio.map.save_interval: -1` 均支持该开关；`-1`
会将整段数据累计在内存。正的保存间隔表示每 N 个处理帧写一个分块，所有处理帧都参与
累计，并非只保存每第 N 帧。

## RViz 鼠标操作

随工程提供的 `retail_street.rviz` 已显式配置交互工具，默认选择 **Move Camera**，
使用以 `world` 为参考的 Orbit 视图：

- **左键拖动**：旋转视角。
- **中键拖动**：平移视角。
- **右键拖动或滚轮**：缩放。
- 使用选择、测量等工具后，点击工具栏 **Move Camera** 或按 **Esc** 恢复视角操作。

重新构建后请重启 launch，让 RViz 加载更新后的安装配置。单独打开 RViz 时也可明确指定：

```bash
rviz2 -d /work/code/3D/SuperM-LIO/install/super_lio/share/super_lio/rviz/retail_street.rviz
```

## 数据与参数说明

Retail_Street 使用相邻 FAST-LIVO2 工程中的 Avia 标定作为起点：
`p_imu = R_lidar_imu * p_lidar + t_lidar_imu`，平移为
`[0.04165, 0.02326, -0.0284]` 米，旋转为单位矩阵。
点的 `offset_time` 单位为纳秒，相对 `header.stamp`；IMU 加速度由初始化均值归一化到
配置的重力大小。ROS 2 参数位于 `/**/ros__parameters` 下，节点参数名使用点号分隔。

`lio.sensor.max_scan_duration` 默认 0.2 秒，超过上限的点时间会被丢弃；大部分点时间
损坏的扫描整帧跳过，IMU 保留给下一帧积分。Retail_Street 中存在一帧约 3.05 秒的异常
点偏移，正常扫描约为 0.1 秒。

动态检测使用 Avia 阈值和视场、dataset 模式 3、0.01 rad 角分辨率。水平/垂直分辨率
参数名与含义一致。聚类扩展默认关闭，关闭时不分配聚类网格；启用后仍使用稠密网格，
需要根据内存设置体素大小。这些参数是可运行的起点，实际场景仍可能需要调参。

## 输出与验证

| 话题 | 内容 |
|---|---|
| `/lio/odom`、`/lio/path` | `world` 中的机体位姿与轨迹 |
| `/tf` | `world` → `body` |
| `/lio/cloud_world` | 默认发布经过动态过滤的建图点云 |
| `/lio/mdet/dyn_cloud`、`/lio/mdet/steady_cloud` | 动态点、静态点，均在世界坐标系中 |
| `/lio/mdet/counts` | 每帧 `[输入点数, 动态点数, 静态点数]` |
| `/lio/processing_time` | 毫秒，position 的 x/y/z 分别为本帧/均值/标准差 |

轨迹文件开关不影响里程计话题发布。`lio.output.dense: true` 会在可视化话题发布全部
去畸变点，但最终地图仍使用经过动态过滤的建图点云；该开关不控制 PCD 密度。

完整无界面验证：

```bash
python3 scripts/validate_retail_street.py \
  /work/code/3D/fast_ws/src/FAST-LIVO2-ROS2/data/Retail_Street \
  --output outputs/check_baseline
python3 scripts/validate_retail_street.py \
  /work/code/3D/fast_ws/src/FAST-LIVO2-ROS2/data/Retail_Street \
  --output outputs/check_dynamic --dynamic
```

可加 `--duration 12` 做短回放，`--before-observe` 测试观测前过滤，`--params-file` 使用
其他配置，`--param 名称:=值` 添加参数覆盖。验证脚本检查位姿、时间戳、点数守恒、
点云尺寸和输出文件，记录耗时、RSS、PCD 点数及 JSON 报告。输出目录必须是新目录。
每次回放请重启建图节点，避免混用不同回放的滤波历史。

重定位入口：`ros2 launch super_lio relocation.launch.py params_file:=/path/to/relocation.yaml`。
需提供已有地图和匹配的传感器参数。现有其他传感器 YAML 已迁移到 ROS 2 格式，完整
回放验收覆盖 Retail_Street 和 NCLT 20130110。历史结果见[Humble 验证记录](docs/humble_validation.md)，
本次增密与交互检查见[地图密度及 RViz 验证](docs/density_rviz_validation.md)。

## 上游项目

- [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)
- [M-detector](https://github.com/hku-mars/M-detector)
- [Livox 依赖及本地修改说明](third_party/README.md)
