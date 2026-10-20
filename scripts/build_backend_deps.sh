#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
dep_root="$repo_dir/third_party/backend"
mkdir -p "$repo_dir/log/backend"
python3 "$repo_dir/scripts/bootstrap_backend_sources.py"
cmake -S "$dep_root/sources/gtsam" -B "$dep_root/build/gtsam" \
 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$dep_root/install" \
 -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
 -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
 '-DCMAKE_INSTALL_RPATH=$ORIGIN' -DGTSAM_WITH_TBB=OFF -DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF -DGTSAM_USE_BOOST_FEATURES=OFF \
 > "$repo_dir/log/backend/gtsam_configure.log" 2>&1
cmake --build "$dep_root/build/gtsam" -j "${SUPERM_DEPS_JOBS:-4}" > "$repo_dir/log/backend/gtsam_build.log" 2>&1
cmake --install "$dep_root/build/gtsam" > "$repo_dir/log/backend/gtsam_install.log" 2>&1

cmake -S "$dep_root/sources/small_gicp" -B "$dep_root/build/small_gicp" \
 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$dep_root/install" \
 -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_WITH_TBB=OFF \
 > "$repo_dir/log/backend/small_gicp_configure.log" 2>&1
cmake --build "$dep_root/build/small_gicp" -j "${SUPERM_DEPS_JOBS:-4}" > "$repo_dir/log/backend/small_gicp_build.log" 2>&1
cmake --install "$dep_root/build/small_gicp" > "$repo_dir/log/backend/small_gicp_install.log" 2>&1
cmake -S "$dep_root/sources/kiss_matcher/cpp/kiss_matcher" -B "$dep_root/build/kiss_matcher" \
 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$dep_root/install" \
 -DUSE_SYSTEM_TBB=ON -DUSE_SYSTEM_EIGEN3=ON -DUSE_SYSTEM_ROBIN=OFF \
 -DFETCHCONTENT_SOURCE_DIR_ROBIN="$dep_root/sources/robin" \
 -DSUPERM_DEP_SOURCES="$dep_root/sources" -DBUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
 > "$repo_dir/log/backend/kiss_matcher_configure.log" 2>&1
cmake --build "$dep_root/build/kiss_matcher" -j "${SUPERM_DEPS_JOBS:-4}" > "$repo_dir/log/backend/kiss_matcher_build.log" 2>&1
cmake --install "$dep_root/build/kiss_matcher" > "$repo_dir/log/backend/kiss_matcher_install.log" 2>&1
