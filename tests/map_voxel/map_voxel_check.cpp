#include <common/map_voxel_filter.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <algorithm>

int main(int argc, char** argv) {
  using Point = pcl::PointXYZI;
  pcl::PointCloud<Point> input;
  auto add = [&](float x, float intensity) {
    Point point;
    point.x = point.y = point.z = x;
    point.intensity = intensity;
    input.push_back(point);
  };
  // The +/-100 m extent overflows PCL's flattened grid at a 5 cm leaf size.
  add(-100, 1); add(100, 2);
  add(0.01f, 10); add(0.02f, 30);
  add(-0.01f, 10); add(-0.02f, 30);
  add(0.06f, 50);
  Point invalid;
  invalid.x = std::numeric_limits<float>::quiet_NaN();
  invalid.y = invalid.z = 0;
  input.push_back(invalid);
  auto output = LI2Sup::downsampleSavedMap(input, 0.05f);
  std::sort(output.begin(), output.end(), [](const auto& a, const auto& b) { return a.x < b.x; });
  if (output.size() != 5 || output.height != 1 || output.width != 5 ||
      std::abs(output[1].x + 0.015f) > 1e-6 || std::abs(output[2].x - 0.015f) > 1e-6 ||
      output[1].intensity != 20 || output[2].intensity != 20) return 1;
  try { LI2Sup::downsampleSavedMap(input, 0.0f); return 2; }
  catch (const std::invalid_argument&) {}
  std::cout << "PASS: wide extent, negative cells, centroids, intensity and invalid input" << std::endl;
  if (argc == 4) {
    pcl::PointCloud<Point> map;
    if (pcl::io::loadPCDFile(argv[1], map) != 0) return 3;
    auto filtered = LI2Sup::downsampleSavedMap(map, std::stof(argv[3]));
    if (pcl::io::savePCDFileBinary(argv[2], filtered) != 0) return 4;
    std::cout << "Saved-map check: " << map.size() << " -> " << filtered.size() << " points" << std::endl;
  }
}
