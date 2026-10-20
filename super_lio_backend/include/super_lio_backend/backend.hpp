#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <tbb/global_control.h>

#include <array>
#include <fstream>
#include <memory>
#include <super_lio_mapping/store.hpp>
namespace superm {
struct BackendOptions {
  double keyframe_distance = 2, keyframe_angle = 10, keyframe_seconds = 2;
  double exclusion_seconds = 60, scan_context_threshold = 0.18;
  double registration_leaf = 0.4, minimum_overlap = 0.55, maximum_rmse = 0.25;
  size_t context_frames = 10, maximum_registration_points = 60000, candidates = 5;
  bool loop_closure = true, global_registration = true;
  int threads = 2;
};
class Backend {
  friend struct BackendTestAccess;

 public:
  Backend(fs::path store, fs::path output, BackendOptions options = {});
  void poll();
  void save();
  bool reject_last_loop();
  std::vector<Eigen::Matrix4d> frame_poses() const;
  Cloud preview(size_t frames = 100, size_t limit = 100000) const;
  const std::vector<FrameInfo>& frames() const {
    return frames_;
  }
  size_t keyframes() const {
    return keys_.size();
  }
  size_t loops() const {
    return loops_;
  }
  Eigen::Matrix4d correction() const;

 private:
  struct Keyframe {
    size_t frame;
    std::array<float, 20> ring;
    uint64_t descriptor_offset;
  };
  using Descriptor = std::array<float, 1200>;
  struct Verified {
    size_t old_key = 0, current_key = 0;
    Eigen::Matrix4d relative = Eigen::Matrix4d::Identity(),
                    correction = Eigen::Matrix4d::Identity();
    double score = 0, overlap = 0, rmse = 0;
  };
  Cloud submap(size_t frame) const;
  Descriptor descriptor(const Cloud&) const;
  std::pair<double, int> similarity(const Descriptor&, const Descriptor&) const;
  Descriptor read_descriptor(size_t key) const;
  void insert(size_t frame);
  bool verify(size_t old_key, size_t current_key, int yaw_shift, Verified& result);
  bool accept(const Verified& loop);
  void audit(const Verified&, const std::string& event, const std::string& reason = "");
  fs::path store_, output_;
  BackendOptions options_;
  uint64_t cursor_ = 0;
  std::vector<FrameInfo> frames_;
  std::vector<Keyframe> keys_;
  std::unique_ptr<tbb::global_control> parallelism_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  std::fstream descriptors_;
  std::ofstream audit_;
  std::unique_ptr<Verified> pending_;
  std::vector<Verified> accepted_;
  size_t loops_ = 0;
  double last_loop_time_ = -1e30;
  gtsam::Values estimate_;
};
}  // namespace superm
