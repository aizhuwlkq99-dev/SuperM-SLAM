#include <cmath>
#include <iostream>
#include <kiss_matcher/KISSMatcher.hpp>
#include <random>
#include <stdexcept>

#include "super_lio_backend/backend.hpp"

namespace superm {
struct BackendTestAccess {
  static bool holdout_checks_passed(const Backend& backend) {
    for (const auto& loop : backend.accepted_)
      if (loop.holdout_after > loop.holdout_before + 1e-5)
        return false;
    return !backend.accepted_.empty();
  }
  static bool bad_constraint(Backend& backend) {
    Backend::Verified loop;
    loop.old_key = 0;
    loop.current_key = backend.keys_.size() - 1;
    loop.relative(0, 3) = 500;
    return backend.accept(loop);
  }
};
}  // namespace superm
void require(bool value, const char* message) {
  if (!value)
    throw std::runtime_error(message);
}
int main(int argc, char** argv) {
  try {
    if (argc != 2)
      throw std::invalid_argument("Provide a new test output directory");
    superm::fs::path root = argv[1];
    std::mt19937 random(713);
    std::uniform_real_distribution<float> xy(-20, 40), height(-2, 5);
    std::vector<Eigen::Vector3f> landmarks;
    for (int i = 0; i < 7000; ++i)
      landmarks.emplace_back(xy(random), xy(random), height(random));
    std::vector<Eigen::Matrix4d> truth;
    superm::FrameStore store(root / "store");
    for (size_t i = 0; i <= 80; ++i) {
      superm::Frame frame;
      frame.id = i;
      frame.start = frame.end = frame.reference = i;
      double x = (i <= 40 ? i : 80 - i) * .5;
      double yaw = i <= 40 ? 0 : M_PI;
      Eigen::Matrix3d rotation =
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      Eigen::Vector3d position(x, 0, 0);
      Eigen::Matrix4d real = Eigen::Matrix4d::Identity();
      real.block<3, 3>(0, 0) = rotation;
      real.block<3, 1>(0, 3) = position;
      truth.push_back(real);
      frame.pose = real;
      frame.pose(0, 3) += .004 * i;
      frame.pose(1, 3) += .001 * i;
      for (const auto& landmark : landmarks) {
        Eigen::Vector3d point = rotation.transpose() * (landmark.cast<double>() - position);
        if (point.head<2>().norm() > 22)
          continue;
        pcl::PointXYZI p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        p.intensity = 1;
        frame.cloud.push_back(p);
      }
      store.submit(std::move(frame));
    }
    store.close();
    auto infos = superm::read_index(root / "store");
    auto first = superm::read_frame(root / "store", infos.front());
    auto last = superm::read_frame(root / "store", infos.back());
    std::vector<Eigen::Vector3f> source, target;
    for (const auto& p : last.cloud)
      source.emplace_back(p.x, p.y, p.z);
    for (const auto& p : first.cloud)
      target.emplace_back(p.x, p.y, p.z);
    kiss_matcher::KISSMatcher matcher{kiss_matcher::KISSMatcherConfig(0.4f)};
    auto coarse = matcher.estimate(source, target);
    require(coarse.valid && matcher.getNumFinalInliers() >= 30,
            "Global coarse registration failed");
    require(coarse.translation.norm() < .1, "Global registration translation is incorrect");
    require(
        Eigen::AngleAxisd(coarse.rotation.transpose() * truth.back().block<3, 3>(0, 0)).angle() <
            .05,
        "Global registration direction/rotation is incorrect");
    superm::BackendOptions options;
    options.context_frames = 2;
    options.exclusion_seconds = 30;
    options.global_registration = false;
    options.loop_closure = false;
    superm::Backend odometry(root / "store", root / "odometry", options);
    odometry.poll();
    odometry.save();
    auto unchanged = odometry.frame_poses();
    for (size_t i = 0; i < unchanged.size(); ++i)
      require(unchanged[i] == odometry.frames()[i].pose, "Odometry-only graph changed raw poses");
    options.loop_closure = true;
    superm::Backend optimized(root / "store", root / "optimized", options);
    optimized.poll();
    optimized.save();
    require(optimized.loops() > 0, "True reverse-view revisit did not produce a verified loop");
    require(superm::BackendTestAccess::holdout_checks_passed(optimized),
            "Accepted loop worsened independent future geometry");
    auto poses = optimized.frame_poses();
    double raw =
        (odometry.frames().back().pose.block<3, 1>(0, 3) - truth.back().block<3, 1>(0, 3)).norm();
    double improved = (poses.back().block<3, 1>(0, 3) - truth.back().block<3, 1>(0, 3)).norm();
    require(improved < raw * .8, "Loop optimization did not improve known endpoint error");
    require(!superm::BackendTestAccess::bad_constraint(optimized), "Gross false loop was accepted");
    const size_t count = optimized.loops();
    while (optimized.loops())
      require(optimized.reject_last_loop(), "Loop revocation failed");
    auto restored = optimized.frame_poses();
    for (size_t i = 0; i < restored.size(); ++i)
      require(restored[i] == odometry.frames()[i].pose, "Revocation failed to restore all frames");
    std::cout << "loops=" << count << " raw_endpoint_error=" << raw
              << " optimized_endpoint_error=" << improved
              << " odometry_equivalence/false_edge/revocation=PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
