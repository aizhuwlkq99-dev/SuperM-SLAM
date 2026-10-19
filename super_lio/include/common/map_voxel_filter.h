#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <pcl/point_cloud.h>

namespace LI2Sup {
// A global map can span more than INT32_MAX grid cells at centimetre resolution.
// Keep signed 3D keys rather than flattening the entire bounding box into an int.
template<typename Point>
pcl::PointCloud<Point> downsampleSavedMap(const pcl::PointCloud<Point>& input, float leaf) {
  if (!std::isfinite(leaf) || leaf <= 0)
    throw std::invalid_argument("Saved map voxel size must be finite and positive");
  using Key = std::array<int64_t, 3>;
  struct Hash {
    std::size_t operator()(const Key& key) const {
      std::size_t seed = 0;
      for (auto coordinate : key)
        seed ^= std::hash<int64_t>{}(coordinate) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
  };
  struct Cell {
    double x = 0, y = 0, z = 0, intensity = 0;
    std::size_t count = 0;
  };
  std::unordered_map<Key, Cell, Hash> cells;
  cells.reserve(std::min<std::size_t>(input.size(), 1000000));
  for (const auto& point : input) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    Key key;
    const double coordinates[] = {point.x, point.y, point.z};
    for (int axis = 0; axis < 3; ++axis) {
      const double coordinate = std::floor(coordinates[axis] / leaf);
      if (coordinate < static_cast<double>(std::numeric_limits<int64_t>::min()) ||
          coordinate >= static_cast<double>(std::numeric_limits<int64_t>::max()))
        throw std::overflow_error("Saved map coordinate exceeds voxel key range");
      key[axis] = static_cast<int64_t>(coordinate);
    }
    auto& cell = cells[key];
    cell.x += point.x; cell.y += point.y; cell.z += point.z;
    cell.intensity += point.intensity;
    ++cell.count;
  }
  pcl::PointCloud<Point> output;
  output.header = input.header;
  output.sensor_origin_ = input.sensor_origin_;
  output.sensor_orientation_ = input.sensor_orientation_;
  output.reserve(cells.size());
  for (const auto& entry : cells) {
    const auto& cell = entry.second;
    Point point{};
    point.x = cell.x / cell.count;
    point.y = cell.y / cell.count;
    point.z = cell.z / cell.count;
    point.intensity = cell.intensity / cell.count;
    output.push_back(point);
  }
  output.width = static_cast<uint32_t>(output.size());
  output.height = 1;
  output.is_dense = true;
  return output;
}
}  // namespace LI2Sup
