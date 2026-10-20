# SuperM-LIO — ROS 2 Humble

[简体中文](README.md) | **English**

Super-LIO LiDAR–inertial odometry with an embedded M-detector moving-object
filter. This repository targets Ubuntu 22.04 and ROS 2 Humble, including the
`vslam` Docker container. Livox ROS 2 driver and SDK2 sources are included under
`third_party/` and built locally. All commands below run inside the container.

Bounded streaming frame storage, external-memory dense tile export, and an
independent loop backend are available. Raw mapping poses remain the default;
loop optimization requires explicit enablement and separate quality review.
See the usage below and [validation record (Chinese)](docs/streaming_validation.md).

```bash
docker exec -it vslam /bin/bash
cd /work/code/3D/SuperM-LIO
./scripts/build_humble.sh
source install/setup.bash
```

The build script loads `/opt/ros/humble/setup.bash`. Each new terminal needs:

```bash
source /opt/ros/humble/setup.bash
source /work/code/3D/SuperM-LIO/install/setup.bash
```

No FAST-LIVO2 installation environment is needed. Build artifacts are local to
this repository. Use a fresh terminal when changing workspaces.

## Large-scale streaming mapping

Streaming mode stores every effective static processing frame, retaining the
existing ESKF, OctVox, dynamic filtering, and map resolutions. External sorting
exports dense tiles without loading the complete point cloud into RAM at shutdown.
Existing dataset launch files retain the `legacy` saving mode.

Build the optional backend; checked, pinned dependencies install inside this repository:

```bash
./scripts/build_humble.sh --with-backend
source install/setup.bash
ros2 launch super_lio_backend large_scale.launch.py dataset:=nclt \
  output_dir:=/work/code/3D/SuperM-LIO/outputs/my_nclt rviz:=true
```

Replay from another terminal with this workspace sourced:

```bash
ros2 bag play data/NCLT/20130110 --clock --delay 2 \
  --wait-for-all-acked 5000 --topics points_raw imu_raw
```

For Retail, select `dataset:=retail` and play
`/work/code/3D/fast_ws/src/FAST-LIVO2-ROS2/data/Retail_Street`.
Use a new or empty output directory. After playback, press Ctrl-C in the mapping
terminal and wait for export to finish. Map voxels remain **0.10 m for NCLT** and
**0.05 m for Retail**.

```text
outputs/my_nclt/
├── parameters.yaml         # Resolved frontend configuration
├── provenance.json         # Configuration hash, source revision, dirty-tree flag
├── map/store/              # Immutable static body frames, raw poses, index and manifest
├── graph/                  # Per-frame pose snapshot, keyframes, loop audit and report
├── trajectory/             # Complete raw TUM trajectory
└── maps/raw/               # Full-density tiles and manifest.json
```

**Loop closure defaults to disabled** to preserve existing mapping results.
`loop_closure:=true shutdown_export:=both` enables the experimental backend and
also exports `maps/optimized/`, retaining the raw map. The backend implements
Scan Context retrieval, KISS-Matcher/small_gicp registration, delayed independent
frame checks, robust iSAM2, and constraint revocation. Independent real NCLT
checks do not yet establish improvement across every evaluated revisit.

Re-export an ROI, view tiles, or evaluate an offline optimized version:

```bash
ros2 run super_lio_mapping superm_export outputs/my_nclt/map/store \
  outputs/my_nclt/exports/roi --leaf 0.1 --memory-mib 256 --pcd \
  --roi -20 -20 -10 80 80 20
ros2 launch super_lio_backend map_view.launch.py \
  manifest:=/work/code/3D/SuperM-LIO/outputs/my_nclt/maps/raw/manifest.json \
  center_x:=0.0 center_y:=0.0 radius:=100.0 max_points:=200000
ros2 run super_lio_backend superm_optimize outputs/my_nclt/map/store \
  outputs/my_nclt/graph_review
ros2 run super_lio_mapping superm_export outputs/my_nclt/map/store \
  outputs/my_nclt/maps/review --leaf 0.1 --poses outputs/my_nclt/graph_review/poses.txt
```

Each tile uses local coordinates: add its double-precision `origin` from the
manifest when assembling it. The supplied viewer applies origins, colors by
height, and limits preview points without changing dense files. `--pcd` also
writes a conventional world-coordinate PCD for small regions. Choose
`shutdown_export:=none` to store frames and graph now and export later.
`/slam/save_map` exports asynchronously; `/slam/reject_last_loop` revokes the last
accepted constraint. See the [backend interface guide](super_lio_backend/README.md).

The full NCLT control produced identical 10,200-frame trajectories; frontend peak
RSS fell from 3,868 MiB to 823 MiB. Export resources are measured separately.
Continuous tens-of-kilometers accuracy, frontend large-coordinate precision,
and total concurrent container memory remain unvalidated. See the
[validation record and limits (Chinese)](docs/streaming_validation.md) and
[implementation stages (Chinese)](docs/large_scale_mapping_plan.md).

## Log directories

Build and runtime logs default to the workspace's Git-ignored `log/` directory:

```text
log/
├── build/                  # Build script console output
├── colcon/                 # Per-package colcon logs
├── ros/                    # ROS, ros2 launch and RViz logs
├── glog/                   # Mapping / relocation glog files
└── validation/<run_timestamp>/
    ├── node.log
    ├── bag.log
    ├── ros/
    └── glog/
```

Rebuild and source the workspace to activate the logging environment hook:

```bash
./scripts/build_humble.sh
source install/setup.bash
```

The build script displays and saves output automatically and preserves build
failure status. Validation results remain in `outputs/<run>/report.json`;
its `log_dir`, `node_log`, and `bag_log` fields locate the logs. Maps and
trajectories remain in the run's `outputs/` directory. Use validator option
`--log-dir /absolute/path/to/run_logs` to override a single run's log directory.

Set `SUPERM_LOG_DIR` before first sourcing to choose a different log root.
Explicit `ROS_LOG_DIR` and `GLOG_log_dir` settings are preserved. If manually
redirecting diagnostics, use a subdirectory, for example `mkdir -p log/manual`
then `> log/manual/check.log 2>&1`. Root-level log names in older validation
records refer to their historical locations; new runs use this layout.

## Replay NCLT 20130110

`super_lio/config/nclt_20130110.yaml` supports the converted rosbag2 used by
Lightning's `default_nclt.yaml`, including dynamic filtering, RViz, PCD output,
and trajectory output.

The current container does not mount the host's `/media`. A copy is available
under `data/NCLT/20130110` in this workspace (ignored by Git). To prepare a fresh
workspace, run on the **host**:

```bash
cd /work/code/3D/SuperM-LIO
mkdir -p data/NCLT/20130110
cp -n /media/zty/新加卷/Dataset/lightning-lm/NCLT/20130110/{metadata.yaml,20130110.db3} \
  data/NCLT/20130110/
```

After building, start mapping in the first **container terminal**:

```bash
cd /work/code/3D/SuperM-LIO
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch super_lio nclt.launch.py rviz:=true
```

Source the same environment in another container terminal and replay. Topic
selection uses the bag's stored names, without a leading slash:

```bash
ros2 bag play /work/code/3D/SuperM-LIO/data/NCLT/20130110 \
  --clock --wait-for-all-acked 5000 --topics points_raw imu_raw
```

After playback, press Ctrl-C in the mapping terminal and wait for saving.
Outputs are `outputs/nclt_<timestamp>/map/map.pcd` and
`outputs/nclt_<timestamp>/trajectory/NCLT_20130110.txt`.
Launch arguments match the Retail_Street launch, including `dyn_filter:=false`
and `output_dir:=...`.

The FLOAT32 `time` field stores relative microseconds. IMU units are m/s² and
rad/s. Identity rotation and zero translation follow the supplied Lightning
configuration. Online scan voxels are 0.30 m; saved-map voxels are 0.10 m,
controlled independently by `lio.map.ds_size` in the dedicated YAML.
NCLT enables `lio.ros.reliable_input=true` to match this bag's reliable publishers.
`lio.ros.imu_reorder_window=0.2` sorts mildly reordered IMUs before integration,
adding approximately 0.2 seconds of buffering.
This integration uses the current LIO and M-detector; Lightning's loop closure,
pose graph, and GPS fusion are outside this launch.

Run a full automated check in the container:

```bash
python3 scripts/validate_nclt.py data/NCLT/20130110 \
  --output outputs/nclt_check --dynamic
```

Use a new output directory. Add `--duration 60` for a smoke test or `--rate 4`
for faster playback. See [NCLT validation](docs/nclt_validation.md).


NCLT uses `nclt.rviz` with Z-height coloring (-8 to 12 m); dynamic points stay red.
This converted bag has no RGB or reflectivity, so Intensity coloring is uniform.
In an existing RViz window, set LiDAR's `Color Transformer` to `AxisColor` and
`Axis` to Z. Existing PCD files need no rebuilding; use height coloring in
CloudCompare to inspect the full saved map.
See [viewing colors and structure](docs/nclt_validation.md#点云颜色与结构查看).

## Dependencies

The checked `vslam` container already has the required dependencies. For a new
Humble environment:

```bash
apt-get update
apt-get install -y python3-colcon-common-extensions python3-yaml \
  ros-humble-ament-cmake ros-humble-rclcpp-components ros-humble-pcl-conversions \
  ros-humble-tf2-ros ros-humble-rosidl-default-generators \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins ros-humble-rviz2 \
  libeigen3-dev libpcl-dev libopencv-dev libtbb-dev \
  libgoogle-glog-dev libgflags-dev
```

`basic` exports the shared geometry/logging library, `m_detector` exports
`dyn_obj_filter`, and `super_lio` builds mapping and relocation executables.
The old M-detector dataset evaluation utilities are upstream reference sources;
they are not built or installed. The supported detector entry point is the
embedded filter in `super_lio_node`.

## Retail_Street playback

Start the mapping node first:

```bash
cd /work/code/3D/SuperM-LIO
ros2 launch super_lio retail_street.launch.py rviz:=true
```

RViz is optional; omit `rviz:=true` for headless operation. In another terminal
load Humble and this workspace, then:

```bash
ros2 bag play \
  /work/code/3D/fast_ws/src/FAST-LIVO2-ROS2/data/Retail_Street \
  --clock --topics /livox/lidar /livox/imu
```

The launch defaults to `use_sim_time:=true`. It consumes
`livox_ros_driver2/msg/CustomMsg` and `sensor_msgs/msg/Imu`; camera data is unused.
At bag completion, press Ctrl-C in the launch terminal to save the final map.
Each launch creates an output directory under `outputs/<timestamp>/` containing
`map/map.pcd`, intermediate `map/PCD/scans_*.pcd`, and
`trajectory/Retail_Street.txt` (TUM poses). Existing map fragments or trajectory files are rejected; use a fresh directory for each run.

Useful launch arguments:

```bash
# Plain LIO baseline
ros2 launch super_lio retail_street.launch.py dyn_filter:=false
# Filter dynamic points before the ESKF observation update
ros2 launch super_lio retail_street.launch.py filter_before_observe:=true
# Choose a new output directory, or a different ROS 2 parameter file
ros2 launch super_lio retail_street.launch.py output_dir:=/tmp/superm_run_01
```

The Retail_Street config starts from the adjacent FAST-LIVO2 Avia calibration:
`p_imu = R_lidar_imu * p_lidar + t_lidar_imu`, with
`t = [0.04165, 0.02326, -0.0284]` metres and identity rotation. Point
`offset_time` is nanoseconds relative to `header.stamp`; the scan endpoint uses
the maximum valid offset before spatial filtering. The configurable
`lio.sensor.max_scan_duration` defaults to 0.2 s and discards invalid point
offsets; scans with a majority of invalid offsets are skipped as a whole.
Retail_Street contains one such scan, with offsets reaching 3.05 s. IMU acceleration is scaled from
its initialization mean to the configured gravity norm. Parameters are stored
under `/**/ros__parameters`, with dot-separated node parameter names.

The detector configuration uses the upstream Avia thresholds and FOV, dataset
mode 3, and 0.01 rad
angular cells. Horizontal/vertical resolution keys now match their names.
Cluster extension is disabled by default and allocates nothing while disabled.
Enabled clustering still uses a dense voxel grid and should be configured with
appropriate voxel size for available memory. These parameters are a working
starting point, not an accuracy calibration for every scene.

## Map density

The default Retail_Street profile keeps every valid source point, downsamples
the per-scan mapping cloud at 15 cm, and saves the final PCD with 5 cm voxels:

| Parameter in `super_lio/config/retail_street.yaml` | Previous | Current |
|---|---:|---:|
| `lio.sensor.filter_rate` | 3 | 1 |
| `lio.sensor.voxel_fliter_size` | 0.30 m | 0.15 m |
| `lio.map.ds_size` | 0.25 m | 0.05 m |

Final map saving uses sparse signed 3D voxel keys, so a large bounding box
with centimetre voxels does not overflow a flattened PCL grid index.

The spelling `voxel_fliter_size` is retained for compatibility. Smaller voxels
and a smaller filter rate retain more detail and increase processing time,
memory use and file size. The online map's `lio.hash_map.vox_resolution` has a
separate purpose and does not specify the saved PCD resolution.

Edit these values, run `./scripts/build_humble.sh`, and replay into a new output
directory. Existing PCD files cannot recover discarded measurements. For even
more detail, try `voxel_fliter_size: 0.10` and `ds_size: 0.03` and check processing
time; these optional settings have not been benchmarked.

The `if_filter` and `save_interval` settings below apply to legacy saving.
Streaming stores static frames losslessly; the backend `map_leaf` or exporter
`--leaf` controls final voxel resolution.

To retain all points in the static mapping input, set `lio.map.if_filter: false`.
This disables the
final spatial voxel filter; it does not disable M-detector. It works both with
chunked saving and with `lio.map.save_interval: -1`. The latter accumulates the
whole sequence in memory. A positive save interval writes chunks every N
processed frames; every processed frame is accumulated, not just every Nth one.

## RViz mouse controls

The supplied `retail_street.rviz` explicitly installs the interaction tools and
selects **Move Camera** by default, using an Orbit view in the `world` frame.

- Left-button drag: rotate the view.
- Middle-button drag: pan the view.
- Right-button drag or mouse wheel: zoom.
- If a selection/measurement tool is active, click **Move Camera** in the toolbar
  or press **Esc** to return to camera navigation.

After rebuilding, restart the launch to load the updated installed RViz file.
For a separate RViz window, explicitly load it with:

```bash
rviz2 -d /work/code/3D/SuperM-LIO/install/super_lio/share/super_lio/rviz/retail_street.rviz
```

## Outputs and validation

- `/lio/odom`, `/lio/path`: IMU/body pose in `world`; odometry publishes even if
  trajectory file output is disabled.
- `/tf`: `world` → `body` at the odometry timestamp.
- `/lio/cloud_world`: static mapping cloud with this dataset's default
  `lio.output.dense=false`. Setting dense output true publishes all deskewed
  points for visualization; saved maps still use the filtered mapping cloud.
- `/lio/mdet/dyn_cloud`, `/lio/mdet/steady_cloud`: classified clouds in `world`.
- `/lio/mdet/counts`: `[input, dynamic, static]` per filtered frame.
- `/lio/processing_time`: per synchronized scan, milliseconds in position
  `x=current`, `y=mean`, `z=standard deviation`.

Run a complete headless replay with validation and JSON results:

```bash
python3 scripts/validate_retail_street.py \
  /work/code/3D/fast_ws/src/FAST-LIVO2-ROS2/data/Retail_Street \
  --output outputs/check_baseline
python3 scripts/validate_retail_street.py \
  /work/code/3D/fast_ws/src/FAST-LIVO2-ROS2/data/Retail_Street \
  --output outputs/check_dynamic --dynamic
```

Output directories must be new. `--duration 12` performs a smoke replay;
`--before-observe` tests the alternate filtering order. Use `--params-file` to
choose a configuration and repeated `--param NAME:=VALUE` arguments for ROS
parameter overrides. The checker isolates
its ROS domain, stops both processes, checks odometry timestamps and finite
poses, checks filter count conservation, records process RSS, and validates
saved PCD/trajectory files. Restart the mapping node for each bag playback;
backwards sensor timestamps are rejected instead of mixing filter histories.

Relocation remains available through `ros2 launch super_lio relocation.launch.py
params_file:=/path/to/relocation.yaml`; provide a saved map and matching sensor
calibration. The existing sensor YAMLs use the ROS 2 schema, but Retail_Street and NCLT 20130110
are covered by the replay validation.

See [validation records](docs/humble_validation.md) for measured results and
[map density / RViz checks](docs/density_rviz_validation.md) for this update.

## Upstream projects

- [Super-LIO](https://github.com/Liansheng-Wang/Super-LIO)
- [M-detector](https://github.com/hku-mars/M-detector)
- [Livox dependencies and local changes](third_party/README.md)
