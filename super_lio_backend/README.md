# Disk-backed ROS 2 mapping backend

This package leaves Super-LIO's ESKF and local map unchanged. It tails committed
frame records, builds bounded-context registration submaps, stores full Scan
Context descriptors on disk, and retains only retrieval keys and graph metadata.
Every processed mapping frame remains available for final dense reconstruction.

Candidates use global descriptor retrieval and spatial neighbors. Geometry is
verified with small_gicp and optional KISS-Matcher coarse registration, followed
by bidirectional residual, overlap, degeneracy, gravity, and neighboring-keyframe
checks. A candidate waits for future static frames unused by registration;
constraints that worsen either directional capped residual are rejected. A robust
SE(3) iSAM2 graph stages each constraint before accepting it.
These checks reduce false closures; they do not establish absolute accuracy on
unlabelled data. Default noise values require dataset-specific evaluation.

```bash
./scripts/build_backend_deps.sh
source /opt/ros/humble/setup.bash
colcon --log-base log/colcon build --packages-up-to super_lio super_lio_backend \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
ros2 run super_lio_backend superm_optimize STORE NEW_GRAPH_DIRECTORY
ros2 run super_lio_mapping superm_export STORE NEW_MAP_DIRECTORY --leaf 0.1 \
  --poses NEW_GRAPH_DIRECTORY/poses.txt
```

`superm_optimize --no-loops` preserves raw poses exactly. `--no-global` disables
coarse registration; `--context N` changes registration context, never map density.
The graph directory contains every frame's optimized pose, a TUM trajectory,
keyframe-to-frame lookup, a report, and loop acceptance/rejection/revocation audit.
All source frames stay immutable. Do not overwrite an existing graph directory.

For online use, set `store_path` and `graph_directory` on `superm_backend_node`.
It publishes `map -> world`, `/slam/path_optimized`, and a bounded recent-cloud
preview `/slam/map_optimized`. That preview is not the complete dense map.
`/slam/save_map` accepts an asynchronous export request; completion is reported
on `/slam/export_status` and by the final output manifest. `/slam/reject_last_loop`
rebuilds the graph without the last accepted constraint, including subsequent
odometry. Earlier constraints can be reconsidered by reprocessing the journal.

`superm_map_viewer` reads selected exported tiles with a fixed preview point
budget. Set `manifest`, `center_x/y/z`, `radius`, and `max_points`; view topic
`/slam/map_tiles` in RViz using `map_view`. Tile origins are applied in double
precision before creating local display points. Dense files remain unchanged.

Validation executables are `graph_check` and `tests/ros_check.py`; see repository
validation records for measured results. These checks cover known reverse-view
revisits, global registration direction, unchanged no-loop poses, gross false
constraints, revocation, asynchronous export, and bounded ROS tile visualization.
Real continuous 30–50 km accuracy has not yet been validated.

The repository's `large_scale.launch.py` defaults to `loop_closure:=false`.
`shutdown_export:=raw` exports raw dense tiles on orderly shutdown; `both` also
exports the optimized version, and `none` leaves export to the standalone tool.
The launch records resolved frontend parameters and Git/configuration provenance.
It supports `dataset:=nclt` and `dataset:=retail` without loading other workspaces.
Use `map_view.launch.py manifest:=/absolute/map/manifest.json` to inspect tiles.
See [measured results and quality limits](../docs/streaming_validation.md).
