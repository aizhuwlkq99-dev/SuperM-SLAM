#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <stdexcept>

#include "../../super_lio/include/common/map_voxel_filter.h"
#include "super_lio_mapping/store.hpp"
void require(bool ok, const char* s) {
  if (!ok)
    throw std::runtime_error(s);
}
int main(int argc, char** argv) {
  try {
    superm::fs::path root = argc > 1 ? argv[1] : "/tmp/superm_mapping_check";
    require(!superm::fs::exists(root), "test directory exists");
    superm::StoreOptions o;
    o.chunk_bytes = 900;
    o.queue_bytes = 1 << 20;
    o.commit_seconds = 0.01;
    superm::FrameStore store(root / "store", o);
    for (size_t i = 0; i < 19; ++i) {
      superm::Frame f;
      f.id = i;
      f.start = f.end = f.reference = i * 0.1;
      f.pose(0, 3) = i % 3;
      for (int j = -70; j < 70; ++j) {
        pcl::PointXYZI p;
        p.x = j * .03f;
        p.y = -.1f;
        p.z = (j % 7) * .04f;
        p.intensity = j + 70;
        f.cloud.push_back(p);
      }
      store.submit(std::move(f));
    }
    store.close();
    auto index = superm::read_index(root / "store");
    require(index.size() == 19, "frame count");
    require(superm::read_frame(root / "store", index[7]).cloud.size() == 140, "point round trip");
    superm::ExportOptions e;
    e.leaf = .1;
    e.tile_size = .7;
    e.workspace_bytes = 4096;
    e.single_pcd = true;
    auto a = superm::export_map(root / "store", root / "small", e);
    e.workspace_bytes = 1024 * 1024;
    auto b = superm::export_map(root / "store", root / "large", e);
    require(a.points == b.points && a.input_points == 2660, "external merge lost points");
    superm::Cloud x, y;
    pcl::io::loadPCDFile((root / "small/map.pcd").string(), x);
    pcl::io::loadPCDFile((root / "large/map.pcd").string(), y);
    require(x.size() == y.size(), "merge sizes");
    for (size_t i = 0; i < x.size(); ++i)
      require((x[i].getVector3fMap() - y[i].getVector3fMap()).norm() < 1e-6 &&
                  std::abs(x[i].intensity - y[i].intensity) < 1e-5,
              "weighted external merge differs");
    superm::Cloud reference;
    for (auto& info : index) {
      auto f = superm::read_frame(root / "store", info);
      superm::Cloud transformed;
      pcl::transformPointCloud(f.cloud, transformed, f.pose.cast<float>());
      reference += transformed;
    }
    auto expected = LI2Sup::downsampleSavedMap(reference, 0.1f);
    auto order = [](const pcl::PointXYZI& a, const pcl::PointXYZI& b) {
      return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
    };
    std::sort(x.begin(), x.end(), order);
    std::sort(expected.begin(), expected.end(), order);
    require(x.size() == expected.size(), "legacy voxel coverage differs");
    for (size_t i = 0; i < x.size(); ++i)
      require((x[i].getVector3fMap() - expected[i].getVector3fMap()).norm() < 1e-6 &&
                  std::abs(x[i].intensity - expected[i].intensity) < 1e-5,
              "legacy centroid/attribute differs");
    {
      std::ofstream broken(root / "store/index.txt");
      broken << "partial index";
    }
    require(superm::recover_store(root / "store") == 19, "durable index recovery failed");
    auto corrupt = root / "store/frames" / index[0].chunk;
    {
      std::fstream f(corrupt, std::ios::in | std::ios::out | std::ios::binary);
      f.seekp(index[0].offset + 190);
      char bad = 42;
      f.write(&bad, 1);
    }
    bool rejected = false;
    try {
      superm::read_frame(root / "store", index[0]);
    } catch (...) {
      rejected = true;
    }
    require(rejected, "corruption not detected");
    std::cout
        << "storage, weighted external merge, negative boundaries, bounded runs, checksum: PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
