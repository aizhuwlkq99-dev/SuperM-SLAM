#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <iostream>
#include <tuple>

#include "../../super_lio/include/common/map_voxel_filter.h"
#include "super_lio_mapping/store.hpp"

// Small-data regression oracle only. Production export never accumulates the map.
int main(int argc, char** argv) {
  try {
    if (argc != 4)
      throw std::invalid_argument("Usage: mapping_reference_check STORE MAP_PCD LEAF");
    auto index = superm::read_index(argv[1]);
    uint64_t total = 0;
    for (const auto& info : index)
      total += info.points;
    if (total > 10000000)
      throw std::runtime_error("Reference check is limited to ten million input points");
    superm::Cloud accumulated;
    for (const auto& info : index) {
      auto frame = superm::read_frame(argv[1], info);
      superm::Cloud world;
      pcl::transformPointCloud(frame.cloud, world, frame.pose.cast<float>());
      accumulated += world;
    }
    auto expected = LI2Sup::downsampleSavedMap(accumulated, std::stof(argv[3]));
    superm::Cloud actual;
    if (pcl::io::loadPCDFile(argv[2], actual))
      throw std::runtime_error("Cannot read exported PCD");
    if (expected.size() != actual.size())
      throw std::runtime_error("Voxel coverage changed");
    auto order = [](const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
      return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
    };
    std::sort(expected.begin(), expected.end(), order);
    std::sort(actual.begin(), actual.end(), order);
    double maximum = 0, intensity = 0;
    for (size_t i = 0; i < actual.size(); ++i) {
      maximum = std::max(
          maximum, double((expected[i].getVector3fMap() - actual[i].getVector3fMap()).norm()));
      intensity =
          std::max(intensity, double(std::abs(expected[i].intensity - actual[i].intensity)));
    }
    std::cout << "{\"frames\":" << index.size() << ",\"input_points\":" << total
              << ",\"map_points\":" << actual.size() << ",\"max_xyz_difference\":" << maximum
              << ",\"max_intensity_difference\":" << intensity << "}\n";
    return maximum <= 1e-6 && intensity <= 1e-6 ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
