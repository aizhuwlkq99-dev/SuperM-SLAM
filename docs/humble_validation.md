# Humble migration validation — 2026-10-19

> 后续地图增密与 RViz 鼠标交互修复及实测结果见 [最新验证记录](density_rviz_validation.md)。下表为初次迁移时的参数与结果。

> 新增 NCLT 20130110 的数据约定、启动方法和验证结果见 [NCLT 验证记录](nclt_validation.md)。

Validated in Docker container `vslam`, Ubuntu 22.04, ROS 2 Humble, GCC 11.4,
PCL 1.12.1, Eigen 3.4, TBB 2021.5. Build and execution load only
`/opt/ros/humble/setup.bash` and this repository's `install/setup.bash`.
All four packages (`basic`, `livox_ros_driver2`, `m_detector`, `super_lio`) build
successfully using `scripts/build_humble.sh`.

## Full Retail_Street replay

Input: `/work/code/3D/fast_ws/src/FAST-LIVO2-ROS2/data/Retail_Street`.
Recorded duration: 135.470252209 seconds; 1,355 LiDAR and 27,447 IMU messages.
Camera messages are excluded. Replay rate: 1.0; processing threads: 4 per node.
The two modes ran concurrently in distinct ROS domains, so timing includes
shared host load and is not a dedicated-machine benchmark.

| Check | Plain LIO | Dynamic filter |
|---|---:|---:|
| Result | PASS | PASS |
| Published poses / cloud frames | 1347 | 1347 |
| Invalid poses | 0 | 0 |
| Non-increasing pose timestamps | 0 | 0 |
| Largest timestamp interval (s) | 0.200 | 0.200 |
| Mean synchronized-scan processing (ms) | 5.59 | 9.17 |
| Peak process RSS (MiB) | 82.4 | 523.0 |
| Moving point classifications | — | 35,503 |
| Static point classifications | — | 2,118,979 |
| Filter point conservation failures | — | 0 |
| Invalid point-cloud dimensions | 0 | 0 |
| Saved PCD size (bytes) | 683,996 | 677,452 |
| Saved TUM poses | 1347 | 1347 |
| Node exit status | 0 | 0 |

During dynamic replay, RSS was about 509.0 MiB after
warm-up and 521.6 MiB near the end. Map growth accounts for part
of this increase; this finite replay does not establish long-duration bounds.
Moving classifications count repeated observations, not unique objects or
validated ground-truth dynamic points. No ground-truth trajectory or labels
were provided, so accuracy and detector precision/recall were not evaluated.

Raw results and maps are preserved locally (ignored by Git):

- `outputs/verified_baseline/report.json`, `map/map.pcd`, `trajectory/Retail_Street.txt`
- `outputs/verified_dynamic/report.json`, `map/map.pcd`, `trajectory/Retail_Street.txt`
- Historical node and bag logs were beside each report. New runs store logs under
  `log/validation/`; use the report's `log_dir`, `node_log`, and `bag_log` fields.

## Additional checks

- Actual `ros2 launch super_lio retail_street.launch.py` starts and exits cleanly;
  a short replay with `filter_before_observe:=true` saves a map and trajectory
  (`outputs/launch_before_observe/`).
- Final Avia configuration with observation-before-filter ordering is covered
  by the full dynamic replay. Alternate filter-before-observation ordering is
  covered by a 12-second replay: 109 valid poses, 109
  matched mapping/static-cloud frames, and 0 point-count mismatches
  (`outputs/verified_before_observe/report.json`).
- Invalid filter rate and malformed extrinsic array fail with explicit errors
  and exit status 1 (`outputs/environment_checks.json`).
- A fresh shell resolves `livox_ros_driver2` and `liblivox_lidar_sdk_shared.so`
  inside this workspace. No FAST-LIVO2 overlay or external SDK is linked.
- `relocation_node` loads a saved map and exits cleanly
  (`outputs/relocation_startup/report.json`). Full relocation tracking and real
  Livox hardware operation were not tested.
- Shell/Python syntax checks, all 13 sensor YAML schemas/extrinsic arrays, and
  `git diff --check` pass. RViz2 configuration is supplied; GUI rendering was
  not exercised in the headless tests.

## Data and integration fixes

The bag contains one scan at header time 946685480.999355 with 23,808 of its
24,000 point offsets beyond 0.2 seconds (maximum about 3.05 seconds). Normal
scans last about 0.1 seconds. This scan is skipped while its IMU samples remain
available for the next scan, preventing skipped subsequent scans and repeated
pose timestamps. The limit is `lio.sensor.max_scan_duration`.

The embedded filter now maintains PCL cloud dimensions, including empty clouds;
this fixes an arithmetic exception in `pcl::transformPointCloud`. Clustering
storage is allocated only when enabled, queue and frame capacities are checked
at runtime, angular parameter names match their variables, and point pools are
released on shutdown. Output folders are never cleared with shell commands;
existing map fragments or trajectory files require a new output directory.
