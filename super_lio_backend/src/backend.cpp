#include "super_lio_backend/backend.hpp"

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <kiss_matcher/KISSMatcher.hpp>
#include <small_gicp/registration/registration_helper.hpp>
#include <sstream>
#include <stdexcept>
namespace superm {
namespace {
gtsam::Pose3 pose(const Eigen::Matrix4d& T) {
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  q.normalize();
  return gtsam::Pose3(gtsam::Rot3(q.toRotationMatrix()), gtsam::Point3(T.block<3, 1>(0, 3)));
}
double angle(const Eigen::Matrix4d& T) {
  return Eigen::AngleAxisd(Eigen::Quaterniond(T.block<3, 3>(0, 0)).normalized()).angle();
}
Cloud voxel(const Cloud& cloud, float leaf) {
  Cloud result;
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setLeafSize(leaf, leaf, leaf);
  filter.setInputCloud(cloud.makeShared());
  filter.filter(result);
  return result;
}
std::vector<Eigen::Vector3d> vectors(const Cloud& c) {
  std::vector<Eigen::Vector3d> p;
  p.reserve(c.size());
  for (auto& q : c)
    p.emplace_back(q.x, q.y, q.z);
  return p;
}
struct Metrics {
  double overlap = 0, rmse = 1e30, p95 = 1e30, capped_rmse = 1e30;
};
Metrics metrics(const Cloud& source, const Cloud& target, const Eigen::Matrix4d& T,
                double distance) {
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(target.makeShared());
  std::vector<int> ids(1);
  std::vector<float> ds(1);
  std::vector<double> residuals;
  double sum = 0, capped_sum = 0;
  for (auto& p : source) {
    Eigen::Vector3d v = T.block<3, 3>(0, 0) * Eigen::Vector3d(p.x, p.y, p.z) + T.block<3, 1>(0, 3);
    pcl::PointXYZI q;
    q.x = v.x();
    q.y = v.y();
    q.z = v.z();
    const bool found = tree.nearestKSearch(q, 1, ids, ds) > 0;
    capped_sum += found ? std::min<double>(ds[0], 1.0) : 1.0;
    if (found && ds[0] < distance * distance) {
      sum += ds[0];
      residuals.push_back(std::sqrt(ds[0]));
    }
  }
  Metrics m;
  m.capped_rmse = std::sqrt(capped_sum / std::max<size_t>(source.size(), 1));
  m.overlap = double(residuals.size()) / std::max<size_t>(source.size(), 1);
  if (!residuals.empty()) {
    m.rmse = std::sqrt(sum / residuals.size());
    std::sort(residuals.begin(), residuals.end());
    m.p95 = residuals[std::min(residuals.size() - 1, size_t(residuals.size() * .95))];
  }
  return m;
}
}  // namespace
Backend::Backend(fs::path store, fs::path out, BackendOptions opt)
    : store_(std::move(store)), output_(std::move(out)), options_(opt) {
  if (opt.keyframe_distance <= 0 || opt.registration_leaf <= 0 || opt.context_frames < 1 ||
      opt.context_frames > 100 || opt.maximum_registration_points < 200 ||
      opt.maximum_registration_points > 1000000 || !std::isfinite(opt.keyframe_distance) ||
      !std::isfinite(opt.registration_leaf) || opt.threads < 1 || opt.threads > 64 ||
      opt.candidates < 1 || opt.candidates > 100 || opt.minimum_overlap <= 0 ||
      opt.minimum_overlap > 1)
    throw std::invalid_argument("invalid backend options");
  if (fs::exists(output_))
    throw std::runtime_error("backend output exists; choose a new graph directory");
  parallelism_ = std::make_unique<tbb::global_control>(tbb::global_control::max_allowed_parallelism,
                                                       options_.threads);
  omp_set_num_threads(options_.threads);
  omp_set_max_active_levels(1);
  fs::create_directories(output_);
  descriptors_.open(output_ / "descriptors.bin",
                    std::ios::in | std::ios::out | std::ios::binary | std::ios::trunc);
  audit_.open(output_ / "loops.jsonl");
  if (!descriptors_ || !audit_)
    throw std::runtime_error("cannot create backend descriptor/audit files");
  gtsam::ISAM2Params p;
  p.relinearizeThreshold = 0.01;
  p.relinearizeSkip = 1;
  isam_ = std::make_unique<gtsam::ISAM2>(p);
}
void Backend::poll() {
  if (!fs::exists(store_ / "index.txt"))
    return;
  std::ifstream in(store_ / "index.txt");
  in.seekg(cursor_);
  std::string line;
  while (std::getline(in, line)) {
    if (in.eof())
      break;
    FrameInfo f;
    std::istringstream row(line);
    if (!(row >> f.id >> f.chunk >> f.offset >> f.points >> f.start >> f.end >> f.reference))
      throw std::runtime_error("invalid committed index");
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        if (!(row >> f.pose(r, c)))
          throw std::runtime_error("invalid index pose");
    if (f.id != frames_.size() || !f.pose.allFinite())
      throw std::runtime_error("frame index gap");
    frames_.push_back(f);
    cursor_ = uint64_t(in.tellg());
    insert(frames_.size() - 1);
    if (ready_ && f.id >= keys_[ready_->current_key].frame + 5) {
      if (accept(*ready_)) {
        audit(*ready_, "accepted");
        last_loop_time_ = frames_[keys_[ready_->current_key].frame].reference;
      }
      ready_.reset();
    }
  }
}
Cloud Backend::submap(size_t index) const {
  Cloud points;
  Eigen::Matrix4d inv = frames_[index].pose.inverse();
  size_t first = index >= options_.context_frames - 1 ? index - options_.context_frames + 1 : 0;
  for (size_t i = first; i <= index; ++i) {
    auto f = read_frame(store_, frames_[i]);
    Eigen::Matrix4d T = inv * f.pose;
    for (auto& p : f.cloud) {
      Eigen::Vector3d q =
          T.block<3, 3>(0, 0) * Eigen::Vector3d(p.x, p.y, p.z) + T.block<3, 1>(0, 3);
      pcl::PointXYZI v = p;
      v.x = q.x();
      v.y = q.y();
      v.z = q.z();
      if (std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z))
        points.push_back(v);
    }
  }
  auto result = voxel(points, options_.registration_leaf);
  if (result.size() > options_.maximum_registration_points) {
    Cloud bounded;
    const double step = double(result.size()) / options_.maximum_registration_points;
    for (size_t i = 0; i < options_.maximum_registration_points; ++i)
      bounded.push_back(result[size_t(i * step)]);
    return bounded;
  }
  return result;
}
Backend::Descriptor Backend::descriptor(const Cloud& points) const {
  Descriptor d{};
  for (auto& p : points) {
    double radius = std::hypot(p.x, p.y);
    if (radius < 1 || radius >= 80)
      continue;
    int ring = std::min(19, int(radius / 4));
    double a = std::atan2(p.y, p.x);
    if (a < 0)
      a += 2 * M_PI;
    int sector = std::min(59, int(a * 60 / (2 * M_PI)));
    d[ring * 60 + sector] = std::max(d[ring * 60 + sector], p.z + 3.0f);
  }
  return d;
}
Backend::Descriptor Backend::read_descriptor(size_t k) const {
  Descriptor d;
  std::ifstream in(output_ / "descriptors.bin", std::ios::binary);
  in.seekg(keys_[k].descriptor_offset);
  if (!in.read(reinterpret_cast<char*>(d.data()), sizeof(d)))
    throw std::runtime_error("cannot read descriptor");
  return d;
}
std::pair<double, int> Backend::similarity(const Descriptor& current, const Descriptor& old) const {
  double best = 1;
  int shift = 0;
  for (int s = 0; s < 60; ++s) {
    double sum = 0;
    int columns = 0;
    for (int c = 0; c < 60; ++c) {
      double dot = 0, a = 0, b = 0;
      for (int r = 0; r < 20; ++r) {
        double x = current[r * 60 + c], y = old[r * 60 + (c + s) % 60];
        dot += x * y;
        a += x * x;
        b += y * y;
      }
      if (a > 0 && b > 0) {
        sum += dot / std::sqrt(a * b);
        ++columns;
      }
    }
    double score = columns < 10 ? 1 : 1 - sum / columns;
    if (score < best) {
      best = score;
      shift = s;
    }
  }
  return {best, shift};
}
void Backend::insert(size_t index) {
  auto& f = frames_[index];
  if (!keys_.empty()) {
    auto& previous = frames_[keys_.back().frame];
    auto relative = previous.pose.inverse() * f.pose;
    double dist = relative.block<3, 1>(0, 3).norm();
    if (dist < options_.keyframe_distance &&
        angle(relative) < options_.keyframe_angle * M_PI / 180 &&
        (f.reference - previous.reference < options_.keyframe_seconds || dist < .2))
      return;
  }
  Cloud cloud = submap(index);
  auto d = descriptor(cloud);
  Keyframe key;
  key.frame = index;
  key.descriptor_offset = uint64_t(descriptors_.tellp());
  for (int r = 0; r < 20; ++r) {
    key.ring[r] = 0;
    for (int c = 0; c < 60; ++c)
      key.ring[r] += d[r * 60 + c] / 60;
  }
  descriptors_.write(reinterpret_cast<char*>(d.data()), sizeof(d));
  descriptors_.flush();
  keys_.push_back(key);
  size_t current = keys_.size() - 1;
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values values;
  auto raw = pose(f.pose);
  values.insert(current, raw);
  if (!current) {
    factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        0, raw, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
  } else {
    auto prior = pose(frames_[keys_[current - 1].frame].pose);
    gtsam::Vector6 sigma;
    sigma << .01, .01, .01, .05, .05, .05;
    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        current - 1, current, prior.between(raw), gtsam::noiseModel::Diagonal::Sigmas(sigma));
    values.update(current,
                  isam_->calculateEstimate<gtsam::Pose3>(current - 1).compose(prior.between(raw)));
  }
  isam_->update(factors, values);
  estimate_ = isam_->calculateEstimate();
  if (!options_.loop_closure || ready_ || f.reference - last_loop_time_ < 10)
    return;
  std::vector<std::pair<double, size_t>> candidates;
  for (size_t k = 0; k < current; ++k) {
    if (f.reference - frames_[keys_[k].frame].reference < options_.exclusion_seconds)
      continue;
    double delta = 0;
    for (int r = 0; r < 20; ++r) {
      double e = key.ring[r] - keys_[k].ring[r];
      delta += e * e;
    }
    candidates.emplace_back(delta, k);
  }
  std::sort(candidates.begin(), candidates.end());
  if (candidates.size() > options_.candidates)
    candidates.resize(options_.candidates);
  std::vector<std::pair<double, size_t>> nearby;
  for (size_t k = 0; k < current; ++k) {
    const auto& old = frames_[keys_[k].frame];
    if (f.reference - old.reference < options_.exclusion_seconds)
      continue;
    double distance = (f.pose.block<3, 1>(0, 3) - old.pose.block<3, 1>(0, 3)).norm();
    if (distance < 20)
      nearby.emplace_back(distance, k);
  }
  std::sort(nearby.begin(), nearby.end());
  for (size_t i = 0; i < std::min(nearby.size(), options_.candidates); ++i) {
    const auto key = nearby[i].second;
    if (std::none_of(candidates.begin(), candidates.end(),
                     [key](const auto& c) { return c.second == key; }))
      candidates.emplace_back(0, key);
  }
  double best = options_.scan_context_threshold;
  size_t match = 0;
  int yaw = 0;
  bool found = false;
  for (auto& c : candidates) {
    auto s = similarity(d, read_descriptor(c.second));
    if (s.first < best) {
      best = s.first;
      match = c.second;
      yaw = s.second;
      found = true;
    }
  }
  if (!found)
    return;
  Verified result;
  result.old_key = match;
  result.current_key = current;
  result.score = best;
  if (!verify(match, current, yaw, result))
    return;
  result.correction = frames_[keys_[match].frame].pose * result.relative * f.pose.inverse();
  if (pending_) {
    double dt = f.reference - frames_[keys_[pending_->current_key].frame].reference;
    auto diff = pending_->correction.inverse() * result.correction;
    double old_dt = std::abs(frames_[keys_[match].frame].reference -
                             frames_[keys_[pending_->old_key].frame].reference);
    if (dt < 10 && old_dt < 15 && diff.block<3, 1>(0, 3).norm() < .5 &&
        angle(diff) < 3 * M_PI / 180) {
      ready_ = std::make_unique<Verified>(result);
      audit(result, "awaiting_holdout", "neighbor consensus passed; wait for unused future frames");
      pending_.reset();
      return;
    }
  }
  pending_ = std::make_unique<Verified>(result);
  audit(result, "pending", "requires independent neighboring keyframe verification");
}
bool Backend::verify(size_t old, size_t current, int shift, Verified& v) {
  auto target = submap(keys_[old].frame);
  auto source = submap(keys_[current].frame);
  if (target.size() < 200 || source.size() < 200) {
    audit(v, "rejected", "too few static points");
    return false;
  }
  auto target_v = vectors(target), source_v = vectors(source);
  Eigen::Isometry3d initial = Eigen::Isometry3d::Identity();
  initial.matrix() = frames_[keys_[old].frame].pose.inverse() * frames_[keys_[current].frame].pose;
  double target_yaw = shift * 2 * M_PI / 60;
  double initial_yaw = std::atan2(initial.linear()(1, 0), initial.linear()(0, 0));
  initial.linear() =
      Eigen::AngleAxisd(target_yaw - initial_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      initial.linear();
  small_gicp::RegistrationSetting settings;
  settings.num_threads = options_.threads;
  settings.downsampling_resolution = options_.registration_leaf;
  settings.max_correspondence_distance = 2.0;
  settings.max_iterations = 40;
  auto aligned = small_gicp::align(target_v, source_v, initial, settings);
  auto evaluate = [&](const Eigen::Matrix4d& T) {
    auto forward = metrics(source, target, T, .5);
    auto reverse = metrics(target, source, T.inverse(), .5);
    v.overlap = std::min(forward.overlap, reverse.overlap);
    v.rmse = std::max(forward.rmse, reverse.rmse);
    return v.overlap >= options_.minimum_overlap && v.rmse <= options_.maximum_rmse &&
           std::max(forward.p95, reverse.p95) < .45;
  };
  if (!aligned.converged || !evaluate(aligned.T_target_source.matrix())) {
    if (!options_.global_registration) {
      audit(v, "rejected", "local registration failed");
      return false;
    }
    std::vector<Eigen::Vector3f> src, tgt;
    for (auto& p : source)
      src.emplace_back(p.x, p.y, p.z);
    for (auto& p : target)
      tgt.emplace_back(p.x, p.y, p.z);
    kiss_matcher::KISSMatcher matcher{
        kiss_matcher::KISSMatcherConfig(float(options_.registration_leaf))};
    auto coarse = matcher.estimate(src, tgt);
    if (!coarse.valid || matcher.getNumFinalInliers() < 30) {
      audit(v, "rejected", "global registration insufficient inliers");
      return false;
    }
    initial.linear() = coarse.rotation;
    initial.translation() = coarse.translation;
    aligned = small_gicp::align(target_v, source_v, initial, settings);
  }
  if (!aligned.converged || !evaluate(aligned.T_target_source.matrix())) {
    audit(v, "rejected", "bidirectional overlap/residual failed");
    return false;
  }
  Eigen::Matrix<double, 6, 1> scale =
      aligned.H.diagonal().cwiseMax(1e-12).cwiseSqrt().cwiseInverse();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(scale.asDiagonal() * aligned.H *
                                                                 scale.asDiagonal());
  if (eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() < 1e-5 ||
      aligned.num_inliers < 100) {
    audit(v, "rejected", "degenerate registration geometry");
    return false;
  }
  auto up = aligned.T_target_source.linear() * Eigen::Vector3d::UnitZ();
  if (up.z() < std::cos(15 * M_PI / 180)) {
    audit(v, "rejected", "inconsistent gravity");
    return false;
  }
  v.relative = aligned.T_target_source.matrix();
  return true;
}
bool Backend::accept(Verified& v) {
  auto trial = std::make_unique<gtsam::ISAM2>(*isam_);
  gtsam::NonlinearFactorGraph factors;
  gtsam::Vector6 sigma;
  sigma << .03, .03, .03, .15, .15, .15;
  auto noise =
      gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                        gtsam::noiseModel::Diagonal::Sigmas(sigma));
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(v.old_key, v.current_key,
                                                             pose(v.relative), noise);
  trial->update(factors, gtsam::Values());
  trial->update();
  auto values = trial->calculateEstimate();
  auto error = pose(v.relative)
                   .between(values.at<gtsam::Pose3>(v.old_key).between(
                       values.at<gtsam::Pose3>(v.current_key)));
  if (error.translation().norm() > .5 || gtsam::Rot3::Logmap(error.rotation()).norm() > .1) {
    audit(v, "rejected", "pose graph did not satisfy verified constraint");
    return false;
  }
  for (size_t k = 1; k < keys_.size(); ++k) {
    auto raw = pose(frames_[keys_[k - 1].frame].pose).between(pose(frames_[keys_[k].frame].pose));
    auto err = raw.between(values.at<gtsam::Pose3>(k - 1).between(values.at<gtsam::Pose3>(k)));
    if (err.translation().norm() > .5 || gtsam::Rot3::Logmap(err.rotation()).norm() > .15) {
      audit(v, "rejected", "pose graph causes excessive local deformation");
      return false;
    }
  }
  const size_t old_frame = keys_[v.old_key].frame + 3;
  const size_t current_frame = keys_[v.current_key].frame + 3;
  if (current_frame >= frames_.size()) {
    audit(v, "rejected", "independent future observations unavailable");
    return false;
  }
  auto pose_at = [&](size_t index) {
    size_t k = 0;
    while (k + 1 < keys_.size() && keys_[k + 1].frame <= index)
      ++k;
    auto correction =
        values.at<gtsam::Pose3>(k).compose(pose(frames_[keys_[k].frame].pose).inverse());
    if (k + 1 < keys_.size()) {
      auto next =
          values.at<gtsam::Pose3>(k + 1).compose(pose(frames_[keys_[k + 1].frame].pose).inverse());
      double duration = frames_[keys_[k + 1].frame].reference - frames_[keys_[k].frame].reference;
      double alpha = std::clamp(
          (frames_[index].reference - frames_[keys_[k].frame].reference) / std::max(duration, 1e-9),
          0.0, 1.0);
      correction = correction.compose(
          gtsam::Pose3::Expmap(alpha * gtsam::Pose3::Logmap(correction.between(next))));
    }
    return Eigen::Matrix4d(correction.matrix() * frames_[index].pose);
  };
  auto heldout_cloud = [&](size_t index) {
    auto frame = read_frame(store_, frames_[index]);
    Cloud result;
    for (const auto& point : frame.cloud)
      if (point.getVector3fMap().allFinite() && point.getVector3fMap().head<2>().norm() < 30)
        result.push_back(point);
    return result;
  };
  auto target = heldout_cloud(old_frame);
  auto source = heldout_cloud(current_frame);
  if (source.size() < 200 || target.size() < 200) {
    audit(v, "rejected", "insufficient independent static observations");
    return false;
  }
  Eigen::Matrix4d raw_relative = frames_[old_frame].pose.inverse() * frames_[current_frame].pose;
  Eigen::Matrix4d optimized_relative = pose_at(old_frame).inverse() * pose_at(current_frame);
  const auto raw_forward = metrics(source, target, raw_relative, .5);
  const auto raw_reverse = metrics(target, source, raw_relative.inverse(), .5);
  const auto opt_forward = metrics(source, target, optimized_relative, .5);
  const auto opt_reverse = metrics(target, source, optimized_relative.inverse(), .5);
  v.holdout_before = std::max(raw_forward.capped_rmse, raw_reverse.capped_rmse);
  v.holdout_after = std::max(opt_forward.capped_rmse, opt_reverse.capped_rmse);
  if (opt_forward.capped_rmse > raw_forward.capped_rmse + 1e-5 ||
      opt_reverse.capped_rmse > raw_reverse.capped_rmse + 1e-5) {
    audit(v, "rejected", "independent future static geometry becomes worse");
    return false;
  }
  isam_ = std::move(trial);
  estimate_ = values;
  accepted_.push_back(v);
  ++loops_;
  return true;
}
bool Backend::reject_last_loop() {
  if (accepted_.empty())
    return false;
  accepted_.pop_back();
  loops_ = accepted_.size();
  auto rebuilt = std::make_unique<gtsam::ISAM2>();
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  for (size_t k = 0; k < keys_.size(); ++k) {
    auto raw = pose(frames_[keys_[k].frame].pose);
    values.insert(k, raw);
    if (!k)
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          0, raw, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
    else {
      gtsam::Vector6 sigma;
      sigma << .01, .01, .01, .05, .05, .05;
      auto previous = pose(frames_[keys_[k - 1].frame].pose);
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          k - 1, k, previous.between(raw), gtsam::noiseModel::Diagonal::Sigmas(sigma));
    }
  }
  for (auto& edge : accepted_) {
    gtsam::Vector6 sigma;
    sigma << .03, .03, .03, .15, .15, .15;
    auto noise =
        gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                          gtsam::noiseModel::Diagonal::Sigmas(sigma));
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(edge.old_key, edge.current_key,
                                                             pose(edge.relative), noise);
  }
  rebuilt->update(graph, values);
  for (int i = 0; i < 5; ++i)
    rebuilt->update();
  isam_ = std::move(rebuilt);
  estimate_ = isam_->calculateEstimate();
  pending_.reset();
  ready_.reset();
  Verified v;
  audit(v, "revoked", "last accepted loop removed; graph rebuilt including subsequent odometry");
  save();
  return true;
}
std::vector<Eigen::Matrix4d> Backend::frame_poses() const {
  std::vector<Eigen::Matrix4d> result;
  result.reserve(frames_.size());
  size_t k = 0;
  for (size_t j = 0; j < frames_.size(); ++j) {
    if (!loops_ || keys_.empty()) {
      result.push_back(frames_[j].pose);
      continue;
    }
    while (k + 1 < keys_.size() && keys_[k + 1].frame <= j)
      ++k;
    auto correction =
        estimate_.at<gtsam::Pose3>(k).compose(pose(frames_[keys_[k].frame].pose).inverse());
    if (k + 1 < keys_.size()) {
      auto next = estimate_.at<gtsam::Pose3>(k + 1).compose(
          pose(frames_[keys_[k + 1].frame].pose).inverse());
      double dt = frames_[keys_[k + 1].frame].reference - frames_[keys_[k].frame].reference;
      double alpha = std::clamp(
          (frames_[j].reference - frames_[keys_[k].frame].reference) / std::max(dt, 1e-9), 0., 1.);
      correction = correction.compose(
          gtsam::Pose3::Expmap(alpha * gtsam::Pose3::Logmap(correction.between(next))));
    }
    result.push_back(correction.matrix() * frames_[j].pose);
  }
  return result;
}
Eigen::Matrix4d Backend::correction() const {
  if (!loops_ || keys_.empty())
    return Eigen::Matrix4d::Identity();
  return estimate_.at<gtsam::Pose3>(keys_.size() - 1).matrix() *
         frames_[keys_.back().frame].pose.inverse();
}
Cloud Backend::preview(size_t count, size_t limit) const {
  Cloud points;
  if (frames_.empty())
    return points;
  auto poses = frame_poses();
  size_t first = frames_.size() > count ? frames_.size() - count : 0;
  for (size_t i = frames_.size(); i-- > first;) {
    auto f = read_frame(store_, frames_[i]);
    for (auto& p : f.cloud) {
      if (points.size() >= limit)
        return voxel(points, 1.0);
      Eigen::Vector3d q =
          poses[i].block<3, 3>(0, 0) * Eigen::Vector3d(p.x, p.y, p.z) + poses[i].block<3, 1>(0, 3);
      pcl::PointXYZI v = p;
      v.x = q.x();
      v.y = q.y();
      v.z = q.z();
      points.push_back(v);
    }
  }
  return voxel(points, 1.0);
}
void Backend::audit(const Verified& v, const std::string& event, const std::string& reason) {
  audit_ << std::setprecision(17) << "{\"event\":" << json_string(event)
         << ",\"reason\":" << json_string(reason) << ",\"old\":" << v.old_key
         << ",\"current\":" << v.current_key << ",\"score\":" << v.score
         << ",\"overlap\":" << v.overlap << ",\"rmse\":" << v.rmse
         << ",\"holdout_before\":" << v.holdout_before << ",\"holdout_after\":" << v.holdout_after
         << ",\"relative\":[";
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) {
      if (r || c)
        audit_ << ',';
      audit_ << v.relative(r, c);
    }
  audit_ << "]}\n";
  audit_.flush();
  if (!audit_)
    throw std::runtime_error("loop audit write failed");
}
void Backend::save() {
  auto poses = frame_poses();
  std::ostringstream matrix, tum;
  matrix << std::setprecision(17);
  tum << std::setprecision(17);
  for (size_t i = 0; i < poses.size(); ++i) {
    matrix << i;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        matrix << ' ' << poses[i](r, c);
    matrix << '\n';
    Eigen::Quaterniond q(poses[i].block<3, 3>(0, 0));
    q.normalize();
    auto p = poses[i].block<3, 1>(0, 3);
    tum << frames_[i].reference << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' ' << q.x()
        << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
  }
  std::ostringstream keyframes;
  keyframes << std::setprecision(17);
  for (size_t k = 0; k < keys_.size(); ++k)
    keyframes << k << ' ' << keys_[k].frame << ' ' << frames_[keys_[k].frame].reference << '\n';
  atomic_text(output_ / "keyframes.txt", keyframes.str());
  atomic_text(output_ / "poses.txt", matrix.str());
  atomic_text(output_ / "trajectory_optimized.txt", tum.str());
  std::ostringstream report;
  report << "{\"frames\":" << frames_.size() << ",\"keyframes\":" << keys_.size()
         << ",\"accepted_loops\":" << loops_
         << ",\"reference\":\"body/IMU, actual deskew reference time\",\"graph\":\"SE3 prior + "
            "odometry + robust verified loops\"}\n";
  atomic_text(output_ / "report.json", report.str());
}
}  // namespace superm
