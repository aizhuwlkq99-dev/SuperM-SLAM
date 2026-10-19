#!/usr/bin/env bash
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
log_root="${SUPERM_LOG_DIR:-$repo_dir/log}"
mkdir -p "$log_root/build"
log_root="$(cd -- "$log_root" && pwd)"
build_log="$log_root/build/$(date -u +%Y%m%dT%H%M%S_%N)_$$.log"
source /opt/ros/humble/setup.bash
cd "$repo_dir"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
echo "Build log: $build_log"
colcon --log-base "$log_root/colcon" build --packages-up-to super_lio --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "$@" 2>&1 | tee "$build_log"
