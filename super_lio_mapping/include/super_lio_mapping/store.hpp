#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>
namespace superm {
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
namespace fs = std::filesystem;
struct Frame {
  uint64_t id = 0;
  double start = 0, end = 0, reference = 0;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  Cloud cloud;
};
struct FrameInfo {
  uint64_t id = 0, offset = 0, points = 0;
  std::string chunk;
  double start = 0, end = 0, reference = 0;
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
};
struct StoreOptions {
  size_t queue_bytes = 128 * 1024 * 1024, chunk_bytes = 32 * 1024 * 1024;
  double commit_seconds = 1.0;
};
struct StoreStats {
  uint64_t queued = 0, durable = 0, points = 0, queue_bytes = 0, peak_queue_bytes = 0;
  std::string error;
};
std::string json_string(const std::string& s);
void atomic_text(const fs::path& path, const std::string& text);
std::vector<FrameInfo> read_index(const fs::path& root);
Frame read_frame(const fs::path& root, const FrameInfo& info);
// Offline only: rebuild the durable index from complete, checksummed chunks.
size_t recover_store(const fs::path& root);
class FrameStore {
 public:
  FrameStore(const fs::path& root, StoreOptions options = {},
             std::function<void(const FrameInfo&)> committed = {});
  ~FrameStore();
  void submit(Frame frame);
  void close();
  StoreStats stats() const;

 private:
  void run();
  void commit();
  void manifest(bool complete);
  fs::path root_;
  StoreOptions options_;
  std::function<void(const FrameInfo&)> committed_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Frame> queue_;
  size_t bytes_ = 0, peak_ = 0;
  uint64_t submitted_ = 0, durable_ = 0, points_ = 0;
  bool stopping_ = false, closed_ = false;
  std::string error_;
  std::thread worker_;
  int lock_fd_ = -1;
  int fd_ = -1;
  size_t chunk_id_ = 0, chunk_size_ = 0;
  std::string chunk_name_;
  std::vector<FrameInfo> pending_;
};
struct ExportOptions {
  double leaf = 0.1, tile_size = 100;
  size_t workspace_bytes = 256 * 1024 * 1024;
  bool single_pcd = false, raw_float_transform = true;
  std::vector<Eigen::Matrix4d> poses;
  bool roi = false;
  Eigen::Vector3d minimum = Eigen::Vector3d::Zero(), maximum = Eigen::Vector3d::Zero();
  std::string graph_version = "raw";
};
struct ExportResult {
  uint64_t frames = 0, input_points = 0, points = 0, tiles = 0;
};
ExportResult export_map(const fs::path& root, const fs::path& output,
                        const ExportOptions& options = {});
}  // namespace superm
