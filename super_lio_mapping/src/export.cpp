#include <pcl/common/transforms.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <queue>
#include <sstream>
#include <stdexcept>

#include "super_lio_mapping/store.hpp"
namespace superm {
namespace {
using Key = std::array<int64_t, 3>;
struct Cell {
  Key key;
  double sums[4];
  uint64_t count;
};
struct DiskPoint {
  double xyz[3];
  float intensity;
};
bool less(const Cell& a, const Cell& b) {
  return a.key < b.key;
}
void add(Cell& a, const Cell& b) {
  for (int i = 0; i < 4; ++i)
    a.sums[i] += b.sums[i];
  a.count += b.count;
}
Key key_of(const double* p, double leaf) {
  Key key;
  for (int i = 0; i < 3; ++i) {
    double x = std::floor(p[i] / leaf);
    if (!std::isfinite(x) || x <= double(INT64_MIN) || x >= double(INT64_MAX))
      throw std::runtime_error("voxel coordinate outside int64 range");
    key[i] = int64_t(x);
  }
  return key;
}
int64_t floor_div(int64_t a, int64_t b) {
  int64_t q = a / b;
  return q - ((a % b) < 0);
}
std::string name(const Key& k) {
  return std::to_string(k[0]) + "_" + std::to_string(k[1]) + "_" + std::to_string(k[2]);
}
void checked(std::ostream& out) {
  if (!out)
    throw std::runtime_error("map export write failed");
}
void header(std::ostream& out, uint64_t count,
            const Eigen::Vector3d& origin = Eigen::Vector3d::Zero()) {
  out << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 "
         "1 1\nWIDTH "
      << count << "\nHEIGHT 1\nVIEWPOINT " << std::setprecision(17) << origin.x() << ' '
      << origin.y() << ' ' << origin.z() << " 1 0 0 0\nPOINTS " << count << "\nDATA binary\n";
}
void point(std::ostream& out, const Cell& cell, const Eigen::Vector3d& origin) {
  float p[4];
  for (int i = 0; i < 3; ++i)
    p[i] = float(cell.sums[i] / cell.count - origin[i]);
  p[3] = float(cell.sums[3] / cell.count);
  out.write(reinterpret_cast<const char*>(p), sizeof(p));
}
uint64_t merge(const std::vector<fs::path>& files, const fs::path& output) {
  struct Item {
    Cell cell;
    size_t index;
  };
  struct Compare {
    bool operator()(const Item& a, const Item& b) const {
      return a.cell.key > b.cell.key;
    }
  };
  std::vector<std::ifstream> inputs;
  for (auto& p : files) {
    inputs.emplace_back(p, std::ios::binary);
    if (!inputs.back())
      throw std::runtime_error("cannot read export run");
  }
  std::priority_queue<Item, std::vector<Item>, Compare> heap;
  for (size_t i = 0; i < inputs.size(); ++i) {
    Cell c;
    if (inputs[i].read(reinterpret_cast<char*>(&c), sizeof(c)))
      heap.push({c, i});
  }
  std::ofstream out(output, std::ios::binary);
  Cell current{};
  bool have = false;
  uint64_t n = 0;
  while (!heap.empty()) {
    auto item = heap.top();
    heap.pop();
    if (have && item.cell.key != current.key) {
      out.write(reinterpret_cast<char*>(&current), sizeof(current));
      ++n;
      have = false;
    }
    if (!have) {
      current = item.cell;
      have = true;
    } else
      add(current, item.cell);
    Cell next;
    if (inputs[item.index].read(reinterpret_cast<char*>(&next), sizeof(next)))
      heap.push({next, item.index});
    else if (!inputs[item.index].eof())
      throw std::runtime_error("corrupt export run");
  }
  if (have) {
    out.write(reinterpret_cast<char*>(&current), sizeof(current));
    ++n;
  }
  checked(out);
  out.close();
  return n;
}
fs::path reduce_tile(const fs::path& bucket, const fs::path& work, size_t budget, double leaf) {
  std::ifstream input(bucket, std::ios::binary);
  if (!input)
    throw std::runtime_error("cannot read tile bucket");
  size_t maximum = std::max<size_t>(1, budget / sizeof(Cell));
  std::vector<Cell> cells;
  cells.reserve(maximum);
  std::vector<fs::path> runs;
  DiskPoint p;
  while (true) {
    cells.clear();
    while (cells.size() < maximum && input.read(reinterpret_cast<char*>(&p), sizeof(p))) {
      Cell c{};
      c.key = key_of(p.xyz, leaf);
      for (int i = 0; i < 3; ++i)
        c.sums[i] = p.xyz[i];
      c.sums[3] = p.intensity;
      c.count = 1;
      cells.push_back(c);
    }
    if (cells.empty())
      break;
    std::sort(cells.begin(), cells.end(), less);
    auto path = work / ("run_" + std::to_string(runs.size()));
    std::ofstream out(path, std::ios::binary);
    Cell current = cells[0];
    for (size_t i = 1; i < cells.size(); ++i) {
      if (cells[i].key == current.key)
        add(current, cells[i]);
      else {
        out.write(reinterpret_cast<char*>(&current), sizeof(current));
        current = cells[i];
      }
    }
    out.write(reinterpret_cast<char*>(&current), sizeof(current));
    checked(out);
    runs.push_back(path);
  }
  if (!input.eof())
    throw std::runtime_error("cannot read tile bucket");
  if (runs.empty())
    throw std::runtime_error("empty tile");
  size_t level = 0;
  while (runs.size() > 1) {
    std::vector<fs::path> next;
    for (size_t i = 0; i < runs.size(); i += 16) {
      auto e = std::min(i + 16, runs.size());
      std::vector<fs::path> group(runs.begin() + i, runs.begin() + e);
      auto p = work / ("merge_" + std::to_string(level) + "_" + std::to_string(i));
      merge(group, p);
      for (auto& old : group)
        fs::remove(old);
      next.push_back(p);
    }
    runs.swap(next);
    ++level;
  }
  return runs.front();
}
}  // namespace
ExportResult export_map(const fs::path& root, const fs::path& output,
                        const ExportOptions& options) {
  const double leaf = static_cast<float>(options.leaf);
  if (!std::isfinite(leaf) || leaf <= 0 || !std::isfinite(options.tile_size) ||
      options.tile_size <= 0 || options.workspace_bytes < 4096)
    throw std::invalid_argument("invalid export resolution/budget");
  const int64_t tile_voxels = std::max<int64_t>(1, std::llround(options.tile_size / leaf));
  const double edge = tile_voxels * leaf;
  auto frames = read_index(root);
  if (!options.poses.empty()) {
    if (options.poses.size() > frames.size())
      throw std::invalid_argument("snapshot exceeds committed frames");
    frames.resize(options.poses.size());
  }
  if (frames.empty())
    throw std::runtime_error("no committed frames");
  if (fs::exists(output))
    throw std::runtime_error("export directory exists; use a new version");
  auto staging = output;
  staging += ".part";
  if (fs::exists(staging))
    throw std::runtime_error("incomplete export exists; use a new version");
  fs::create_directories(staging / "work");
  fs::create_directories(staging / "tiles");
  ExportResult result;
  std::map<Key, fs::path> buckets;
  struct Open {
    std::ofstream stream;
    uint64_t age;
  };
  std::map<Key, Open> open;
  uint64_t age = 0;
  for (size_t i = 0; i < frames.size(); ++i) {
    auto f = read_frame(root, frames[i]);
    Eigen::Matrix4d T = options.poses.empty() ? f.pose : options.poses[i];
    if (!T.allFinite())
      throw std::runtime_error("nonfinite snapshot pose");
    Cloud transformed;
    if (options.raw_float_transform && (options.poses.empty() || T == f.pose))
      pcl::transformPointCloud(f.cloud, transformed, T.cast<float>());
    for (size_t j = 0; j < f.cloud.size(); ++j) {
      auto& pt = f.cloud[j];
      DiskPoint p{};
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;
      if (!transformed.empty()) {
        auto& q = transformed[j];
        p.xyz[0] = q.x;
        p.xyz[1] = q.y;
        p.xyz[2] = q.z;
      } else {
        Eigen::Vector3d q =
            T.block<3, 3>(0, 0) * Eigen::Vector3d(pt.x, pt.y, pt.z) + T.block<3, 1>(0, 3);
        for (int a = 0; a < 3; ++a)
          p.xyz[a] = q[a];
      }
      p.intensity = pt.intensity;
      if (options.roi) {
        bool inside = true;
        for (int a = 0; a < 3; ++a)
          inside &= p.xyz[a] >= options.minimum[a] && p.xyz[a] < options.maximum[a];
        if (!inside)
          continue;
      }
      auto k = key_of(p.xyz, leaf);
      Key tile;
      for (int a = 0; a < 3; ++a)
        tile[a] = floor_div(k[a], tile_voxels);
      auto it = open.find(tile);
      if (it == open.end()) {
        if (open.size() >= 16) {
          auto old = std::min_element(open.begin(), open.end(),
                                      [](auto& a, auto& b) { return a.second.age < b.second.age; });
          checked(old->second.stream);
          old->second.stream.close();
          open.erase(old);
        }
        auto path = staging / "work" / ("bucket_" + name(tile));
        buckets[tile] = path;
        it = open.emplace(tile, Open{std::ofstream(path, std::ios::binary | std::ios::app), ++age})
                 .first;
      }
      it->second.age = ++age;
      it->second.stream.write(reinterpret_cast<char*>(&p), sizeof(p));
      checked(it->second.stream);
      ++result.input_points;
    }
    ++result.frames;
  }
  for (auto& item : open) {
    checked(item.second.stream);
    item.second.stream.close();
  }
  open.clear();
  std::ostringstream manifest;
  manifest << std::setprecision(17)
           << "{\"schema\":\"superm-map-v1\",\"frame_count\":" << result.frames
           << ",\"graph_version\":" << json_string(options.graph_version) << ",\"leaf\":" << leaf
           << ",\"tile_size\":" << edge
           << ",\"tile_coordinates\":\"local; add origin to obtain map coordinates\",\"tiles\":[";
  std::vector<fs::path> final_runs;
  for (auto& item : buckets) {
    auto work = staging / "work" / ("sort_" + name(item.first));
    fs::create_directory(work);
    auto sorted = reduce_tile(item.second, work, options.workspace_bytes / 2, leaf);
    uint64_t count = fs::file_size(sorted) / sizeof(Cell);
    Eigen::Vector3d origin;
    for (int a = 0; a < 3; ++a)
      origin[a] = item.first[a] * edge;
    auto filename = "tile_" + name(item.first) + ".pcd";
    std::ofstream out(staging / "tiles" / filename, std::ios::binary);
    header(out, count, origin);
    std::ifstream in(sorted, std::ios::binary);
    Cell c;
    while (in.read(reinterpret_cast<char*>(&c), sizeof(c)))
      point(out, c, origin);
    checked(out);
    if (result.tiles++)
      manifest << ',';
    manifest << "{\"file\":" << json_string("tiles/" + filename) << ",\"points\":" << count
             << ",\"origin\":[" << origin.x() << ',' << origin.y() << ',' << origin.z() << "]}";
    result.points += count;
    final_runs.push_back(sorted);
    fs::remove(item.second);
  }
  if (options.single_pcd) {
    std::ofstream out(staging / "map.pcd", std::ios::binary);
    header(out, result.points);
    for (auto& p : final_runs) {
      std::ifstream in(p, std::ios::binary);
      Cell c;
      while (in.read(reinterpret_cast<char*>(&c), sizeof(c)))
        point(out, c, Eigen::Vector3d::Zero());
    }
    checked(out);
  }
  manifest << "],\"points\":" << result.points << ",\"input_points\":" << result.input_points
           << ",\"complete\":true}\n";
  atomic_text(staging / "manifest.json", manifest.str());
  fs::remove_all(staging / "work");
  fs::rename(staging, output);
  return result;
}
}  // namespace superm
