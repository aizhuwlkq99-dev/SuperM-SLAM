# 地图密度与 RViz 交互验证 / Map density and RViz checks

环境：`vslam` / Ubuntu 22.04 / ROS 2 Humble。输入为同一段约 135 秒的 Retail_Street，
仅回放 LiDAR 和 IMU，启用动态过滤。

Environment: `vslam`, Ubuntu 22.04, ROS 2 Humble. Both density profiles replay the
same full Retail_Street bag with LiDAR/IMU topics and the dynamic filter enabled.

## 修改 / Changes

| 参数 / Parameter | 旧值 / Previous | 新值 / Current |
|---|---:|---:|
| `lio.sensor.filter_rate` | 3 | 1 |
| `lio.sensor.voxel_fliter_size` | 0.30 m | 0.15 m |
| `lio.map.ds_size` | 0.25 m | 0.05 m |

最终 PCD 的体素降采样使用三维整数键的稀疏哈希表，并计算每个体素的位置、强度均值。
这修复了厘米级体素与大地图范围组合导致 PCL 扁平网格索引溢出、退回保存全部点的问题。
降采样只作用于已判定为静态的建图输入，动态点不会因为增密而被加回。

Final PCD downsampling uses sparse signed 3D voxel keys and position/intensity
centroids. It avoids the flattened PCL grid's integer overflow at centimetre
resolution over wide extents. Saving still consumes the filtered static mapping
input; increasing density does not reintroduce classified moving points.

`save_interval=-1` 的保存分支现在也遵守 `lio.map.if_filter`。关闭该参数仅关闭空间降采样。
The `save_interval=-1` branch now respects `lio.map.if_filter` too; disabling it
only disables spatial downsampling.

## 全量回放结果 / Full replay results

| 项目 / Check | 旧密度 / Previous | 新密度 / Current |
|---|---:|---:|
| 验收 / Result | PASS | PASS |
| 有效位姿 / Valid poses | 1,347 | 1,346 |
| PCD 点数 / Saved points | 42,329 | 1,018,816 |
| PCD 大小 / PCD bytes | 677,452 | 16,301,248 |
| 平均处理耗时 / Mean processing (ms/scan) | 9.34 | 19.26 |
| 无效位姿 / Invalid poses | 0 | 0 |
| 时间戳异常 / Non-increasing timestamps | 0 | 0 |
| 静态云与建图输入点数不符 / Mapping count mismatch | 0 | 0 |

新地图点数约为旧配置的 **24.1 倍**，最终文件约 15.5 MiB，
5 cm 空间降采样实际生效，没有 PCL 索引溢出提示。两次独立启动的初始化帧数略有差异。
新配置记录的峰值 RSS 约 965 MiB，包含最终地图合并和保存阶段；
回放末段 RSS 约 567 MiB。耗时受宿主机同时运行的任务影响。

The new map contains **24.1 times** as many points and occupies about
15.5 MiB. The final 5 cm voxel filter is effective, with no grid-index
overflow warning. Independent startup timing accounts for the small difference
in initialization frame counts. New-profile peak RSS is about
965 MiB including final map merging/saving; late-replay RSS is about
567 MiB. Timing depends on other host workloads.
No ground truth was provided, so these checks do not establish localization or
dynamic-detection accuracy.

最终产物 / Final artifacts:

- `outputs/density_dense_verified/map/map.pcd`
- `outputs/density_dense_verified/report.json`
- `outputs/density_sparse/report.json`

复现 / Reproduce the final profile:

```bash
source /opt/ros/humble/setup.bash
source /work/code/3D/SuperM-LIO/install/setup.bash
cd /work/code/3D/SuperM-LIO
python3 scripts/validate_retail_street.py \
  /work/code/3D/fast_ws/src/FAST-LIVO2-ROS2/data/Retail_Street \
  --dynamic --output outputs/density_check_new
```

## 保存逻辑测试 / Save-path checks

- 体素单元测试覆盖 ±100 m 范围、5 cm 体素、负坐标、位置/强度均值、NaN 点和无效体素尺寸。
  Unit checks cover +/-100 m extent, 5 cm cells, negative coordinates,
  position/intensity centroids, NaN points and invalid leaf sizes.
- 对 8,704,680 个累积静态点的独立检查，5 cm 合并得到 1,018,816 点。
  A standalone check reduces 8,704,680 accumulated static points to 1,018,816
  points at 5 cm. This diagnostic preceded the final integration replay.
- 关闭最终降采样的短回放：单文件保存 791,078 点；分块保存 761,961 点。
  两者均与各自回放的静态输入点累计值严格相等。
  Short replays with final downsampling disabled retain exactly their respective
  accumulated static inputs: 791,078 points for single-file saving and 761,961
  for chunked saving. Their wall-clock cutoffs produce different frame counts.

记录 / Artifacts:
`outputs/density_unfiltered_single/report.json`,
`outputs/density_unfiltered_chunks/report.json`, `map_voxel.log`.

复现单元测试 / Reproduce the voxel test (inside the container):

```bash
source /opt/ros/humble/setup.bash
cd /work/code/3D/SuperM-LIO
cmake -S tests/map_voxel -B build/map_voxel
cmake --build build/map_voxel -j2
build/map_voxel/map_voxel_check
```

## RViz 交互 / RViz interaction

原配置没有 `Visualization Manager/Tools`，鼠标事件缺少相应操作工具。
现在配置了 MoveCamera、Interact、Select、FocusCamera 和 Measure，并默认激活 MoveCamera。

The configuration now explicitly loads MoveCamera, Interact, Select,
FocusCamera and Measure and selects MoveCamera by default. The previous file
omitted the tools list.

在容器的 X11 显示中打开独立 RViz 窗口，使用 Qt 鼠标事件检查相机属性：

- 左键拖动改变 Yaw（旋转）。Left drag changes yaw.
- 中键拖动改变 Focal Point（平移）。Middle drag changes focal point.
- 右键拖动改变 Distance（缩放）。Right drag changes distance.

全部通过。检查结果位于 `rviz_camera.log`，窗口截图位于 `outputs/rviz_camera.png`。
测试窗口不接收 bag 数据，只验证交互工具和相机响应。
All checks pass. The isolated window tests interaction/camera response without
bag data; it is not a screenshot of the completed map.

复现 / Reproduce (requires the container's graphical display):

```bash
source /opt/ros/humble/setup.bash
cd /work/code/3D/SuperM-LIO
cmake -S tests/rviz_camera -B build/rviz_camera
cmake --build build/rviz_camera -j2
build/rviz_camera/rviz_camera_check super_lio/rviz/retail_street.rviz
```

上述测试目录通过 `COLCON_IGNORE` 排除，不增加正常构建所需依赖。
The test projects are excluded from normal workspace discovery by `COLCON_IGNORE`.
