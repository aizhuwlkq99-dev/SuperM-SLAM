
#include "ros/ROSWrapper.h"
#include "basic/ros_parameters.h"
#include <algorithm>
#include <stdexcept>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include "super_lio/msg/cloud_pose.hpp"
#include "super_lio/msg/cloud_pose2.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using namespace BASIC;

namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node){
  read_parameter(node, "lio.map.save_map", g_save_map, g_save_map);
  LOG(INFO) << GREEN << " ---> [Param] map/save_map: " << (g_save_map ? "true" : "false") << RESET;
  read_parameter(node, "lio.map.if_filter", g_if_filter, g_if_filter);
  read_parameter(node, "lio.map.save_map_dir", g_save_map_dir, g_save_map_dir);
  g_save_map_dir = std::filesystem::absolute(g_save_map_dir).string();
  read_parameter(node, "lio.map.map_name", g_map_name, g_map_name);
  read_parameter(node, "lio.map.ds_size", g_map_ds_size, g_map_ds_size);
  read_parameter(node, "lio.map.save_interval", g_pcd_save_interval, g_pcd_save_interval);

  read_parameter(node, "lio.evo.timer", g_time_evo, g_time_evo);

  // pose output
  read_parameter(node, "lio.evo.pose_output_en", g_pose_output_en, g_pose_output_en);
  LOG(INFO) << GREEN << " ---> [Param] evo/pose_output_en: " << (g_pose_output_en ? "true" : "false") << RESET;

  read_parameter(node, "lio.evo.seq_name", g_seq_name, g_seq_name);
  LOG(INFO) << GREEN << " ---> [Param] evo/seq_name: " << g_seq_name << RESET;

  read_parameter(node, "lio.evo.pose_output_dir", g_pose_output_dir, g_pose_output_dir);
  LOG(INFO) << GREEN << " ---> [Param] evo/pose_output_dir: " << g_pose_output_dir << RESET;

  // ROS Topic input
  read_parameter(node, "lio.ros.lidar_topic", g_lidar_topic, g_lidar_topic);
  read_parameter(node, "lio.ros.imu_topic", g_imu_topic, g_imu_topic);

  // sensor cfg
  read_parameter(node, "lio.sensor.lidar_type", g_lidar_type, g_lidar_type);
  double temp_range_dis = 0.8;
  read_parameter(node, "lio.sensor.blind", temp_range_dis, temp_range_dis);
  g_blind2 = temp_range_dis * temp_range_dis;
  temp_range_dis = 60.0;
  read_parameter(node, "lio.sensor.maxrange", temp_range_dis, temp_range_dis);
  g_maxrange2 = temp_range_dis * temp_range_dis;
  read_parameter(node, "lio.sensor.filter_rate", g_filter_rate, g_filter_rate);
  read_parameter(node, "lio.sensor.max_scan_duration", g_max_scan_duration, g_max_scan_duration);
  read_parameter(node, "lio.sensor.enable_downsample", g_enable_downsample, g_enable_downsample);
  read_parameter(node, "lio.sensor.voxel_fliter_size", g_voxel_fliter_size, g_voxel_fliter_size);

  read_parameter(node, "lio.sensor.gravity_norm", g_gravity_norm, g_gravity_norm);
  read_parameter(node, "lio.sensor.imu_type", g_imu_type, g_imu_type);
  read_parameter(node, "lio.sensor.imu_na", g_imu_na, g_imu_na);
  read_parameter(node, "lio.sensor.imu_ng", g_imu_ng, g_imu_ng);
  read_parameter(node, "lio.sensor.imu_nba", g_imu_nba, g_imu_nba);
  read_parameter(node, "lio.sensor.imu_nbg", g_imu_nbg, g_imu_nbg);

  // extrinsic
  std::vector<double> extrinsic_lidar_imu{0,0,0,1,0,0,0,1,0,0,0,1};
  std::vector<double> extrinsic_odom_robo(6, 0.0);
  read_parameter(node, "lio.extrinsic.lidar_imu", extrinsic_lidar_imu, extrinsic_lidar_imu);      // 3 + 9
  if(extrinsic_lidar_imu.size() != 12) throw std::invalid_argument("lio.extrinsic.lidar_imu must contain 12 doubles");
  V3 __t = V3(extrinsic_lidar_imu[0],
              extrinsic_lidar_imu[1],
              extrinsic_lidar_imu[2]);
  M3 __R = Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(extrinsic_lidar_imu.data() + 3).cast<scalar>();
  g_lidar_imu = SE3(__R, __t);  // lidar in imu frame·
  read_parameter(node, "lio.extrinsic.odom_robo", extrinsic_odom_robo, extrinsic_odom_robo);     // 3 + 3 x,y,z,r,p,y
  if(extrinsic_odom_robo.size() != 6) throw std::invalid_argument("lio.extrinsic.odom_robo must contain 6 doubles");
  __t = V3(extrinsic_odom_robo[0],
           extrinsic_odom_robo[1],
           extrinsic_odom_robo[2]);
  auto temp_R = Eigen::AngleAxisd(extrinsic_odom_robo[5]/180 * M_PI, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(extrinsic_odom_robo[4]/180 * M_PI, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(extrinsic_odom_robo[3]/180 * M_PI, Eigen::Vector3d::UnitX());
  g_odom_robo.R_ = temp_R.cast<scalar>();

  /// ATTENTION:
  /// The transpose is intentionally applied here and represents the inverse of R.
  /// Misinterpreting this will lead to incorrect transformations.
  M3 _R = g_odom_robo.R_.transpose();
  g_odom_robo.R_  = _R;
  g_odom_robo = SE3(_R, __t);  // lidar in robot frame

  auto temp_R_yaw = Eigen::AngleAxisd(extrinsic_odom_robo[5]/180 * M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  g_lidar_robo_yaw = temp_R_yaw.cast<scalar>();

  // hash_map
  int hash_capacity = static_cast<int>(g_ivox_capacity);
  read_parameter(node, "lio.hash_map.hash_capacity", hash_capacity, hash_capacity);
  g_ivox_capacity = hash_capacity;
  read_parameter(node, "lio.hash_map.vox_resolution", g_ivox_resolution, g_ivox_resolution);

  // kf
  read_parameter(node, "lio.kf.kf_type", g_kf_type, g_kf_type);
  read_parameter(node, "lio.kf.kf_max_iterations", g_kf_max_iterations, g_kf_max_iterations);
  read_parameter(node, "lio.kf.kf_align_gravity", g_kf_align_gravity, g_kf_align_gravity);
  read_parameter(node, "lio.kf.kf_quit_eps", g_kf_quit_eps, g_kf_quit_eps);
  read_parameter(node, "lio.kf.imu_int_frame", g_imu_int_frame, g_imu_int_frame);
  read_parameter(node, "lio.kf.dyn_filter_before_observe", g_dyn_filter_before_observe, g_dyn_filter_before_observe);

  // submaps
  read_parameter(node, "lio.submap.submap_resolution", g_submap_resolution, g_submap_resolution);
  read_parameter(node, "lio.submap.submap_capacity", g_submap_capacity, g_submap_capacity);

  // visual
  read_parameter(node, "lio.output.robot", g_2_robot, g_2_robot);
  read_parameter(node, "lio.output.planner", g_planner_enable, g_planner_enable);
  read_parameter(node, "lio.output.plan_env_world", g_2_plan_env_world, g_2_plan_env_world);
  read_parameter(node, "lio.output.plan_env_body", g_2_plan_env_body, g_2_plan_env_body);
  read_parameter(node, "lio.output.ml_map", g_2_ml_map, g_2_ml_map);
  read_parameter(node, "lio.output.map", g_visual_map, g_visual_map);
  read_parameter(node, "lio.output.dense", g_visual_dense, g_visual_dense);
  read_parameter(node, "lio.output.pub_step", g_pub_step, g_pub_step);

  g_update_map = false;
  read_parameter(node, "lio.relocation.update_map", g_update_map, g_update_map);
  std::vector<double> init_pose;
  read_parameter(node, "lio.relocation.init_pose", init_pose, init_pose);
  if(init_pose.size() == 6){
    g_init_px    = init_pose[0];
    g_init_py    = init_pose[1];
    g_init_pz    = init_pose[2];
    g_init_roll  = init_pose[3];
    g_init_pitch = init_pose[4];
    g_init_yaw   = init_pose[5];
  }else{
    g_init_px = 0.0f;
    g_init_py = 0.0f;
    g_init_pz = 0.0f;
    g_init_roll = 0.0f;
    g_init_pitch = 0.0f;
    g_init_yaw = 0.0f;
  }
  if(g_filter_rate < 1 || g_voxel_fliter_size <= 0 || g_map_ds_size <= 0 ||
     g_pub_step < 1 || g_imu_int_frame < 1 || g_pcd_save_interval == 0 ||
     hash_capacity <= 0 || g_ivox_resolution <= 0 || g_kf_max_iterations < 1 ||
     !std::isfinite(g_max_scan_duration) || g_max_scan_duration <= 0)
    throw std::invalid_argument("Invalid LIO sampling, initialization, map or iteration parameters");
  if(g_save_map) {
    const auto fragments = std::filesystem::path(g_save_map_dir) / "PCD";
    if(std::filesystem::exists(std::filesystem::path(g_save_map_dir) / g_map_name) ||
       (std::filesystem::exists(fragments) && !std::filesystem::is_empty(fragments)))
      throw std::invalid_argument("Map output already exists; choose a new lio.map.save_map_dir");
    std::filesystem::create_directories(fragments);
  }
}

std::tuple<float, float, float> getColorFromVelocity(float velocity, float max_velocity = 2.0f) {
  float ratio = std::clamp(velocity / max_velocity, 0.0f, 1.0f);
  float r, g, b;
  if (ratio < 0.5f) {
    float t = ratio / 0.5f;
    r = 0.0f;
    g = t;
    b = 1.0f;
  } else {
    float t = (ratio - 0.5f) / 0.5f;
    r = t;
    g = 1.0f - t;
    b = 1.0f - t;
  }
  return {r, g, b};
}


void livox2pcl(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg, CloudPtr& point_cloud){
  point_cloud->clear();
  if(msg->points.size() < 2 || msg->point_num != msg->points.size()) return;
  CloudPtr cloud_full(new PointCloudType());
  int plsize = msg->point_num;
  cloud_full->resize(plsize);
  point_cloud->reserve(plsize);
  std::vector<uint8_t> is_valid_pt(plsize, false);
  std::vector<std::size_t> index(plsize - 1);
  std::iota(std::begin(index), std::end(index), 1);

  std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
    if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)
    {
      // if (i % g_filter_rate == 0)
      {
        cloud_full->at(i).x = msg->points[i].x;
        cloud_full->at(i).y = msg->points[i].y;
        cloud_full->at(i).z = msg->points[i].z;
        cloud_full->at(i).intensity = msg->points[i].reflectivity;

        if ((abs(cloud_full->at(i).x - cloud_full->at(i - 1).x) > 1e-7) ||
            (abs(cloud_full->at(i).y - cloud_full->at(i - 1).y) > 1e-7) ||
            (abs(cloud_full->at(i).z - cloud_full->at(i - 1).z) > 1e-7))
        {
          double normal_dis = cloud_full->at(i).x * cloud_full->at(i).x +
                              cloud_full->at(i).y * cloud_full->at(i).y +
                              cloud_full->at(i).z * cloud_full->at(i).z;
          if(normal_dis > g_blind2 and normal_dis < g_maxrange2){
            is_valid_pt[i] = true;
          }
        }
      }
    }
  });

  for (int i = 1; i < plsize; i++) {
    if (is_valid_pt[i]) {
      point_cloud->points.push_back(cloud_full->at(i));
    }
  }
}


std::string lidarTypeToString(int type) {
  if (type <= 0 || type >= static_cast<int>(LID_TYPE_NAMES.size())) return "UNKNOWN";
  return LID_TYPE_NAMES[type];
}

ROSWrapper::ROSWrapper(const rclcpp::Node::SharedPtr& node) : node_(node) {
  bool reliable_input = false;
  read_parameter(*node_, "lio.ros.reliable_input", reliable_input, false);
  read_parameter(*node_, "lio.ros.imu_reorder_window", imu_reorder_window_, 0.0);
  if (!std::isfinite(imu_reorder_window_) || imu_reorder_window_ < 0 || imu_reorder_window_ > 1.0)
    throw std::invalid_argument("lio.ros.imu_reorder_window must be between 0 and 1 seconds");
  auto lidar_qos = rclcpp::SensorDataQoS().keep_last(1000);
  auto imu_qos = rclcpp::SensorDataQoS().keep_last(10000);
  if (reliable_input) {
    lidar_qos.reliable();
    imu_qos.reliable();
  }
  if (g_lidar_type == LID_TYPE::LIVOX) {
    subLidar_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      g_lidar_topic, lidar_qos, [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) { livoxHandler(msg); });
  } else {
    subLidar_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      g_lidar_topic, lidar_qos, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) { stdMsgHandler(msg); });
  }
  subIMU_ = node_->create_subscription<sensor_msgs::msg::Imu>(g_imu_topic, imu_qos,
    [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { imuHandler(msg); });
  pub_odom_ = publisher<nav_msgs::msg::Odometry>("/lio/odom", 100);
  const int path_limit = node_->declare_parameter<int>("lio.output.path_max_points", 5000);
  path_publish_period_ = node_->declare_parameter<double>("lio.output.path_publish_period", 1.0);
  const int lidar_limit = node_->declare_parameter<int>("lio.ros.max_lidar_queue", 128);
  const int imu_limit = node_->declare_parameter<int>("lio.ros.max_imu_queue", 20000);
  if (path_limit < 2 || path_publish_period_ <= 0 || lidar_limit < 1 || imu_limit < 1)
    throw std::invalid_argument("invalid path/input queue limits");
  path_max_points_ = path_limit;
  max_lidar_queue_ = lidar_limit;
  max_imu_queue_ = imu_limit;
  pub_path_ = publisher<nav_msgs::msg::Path>("/lio/path", 1);
  pub_path_robot_ = publisher<sensor_msgs::msg::PointCloud2>("/lio/path_robot", 1);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  msg_path_point_.header.frame_id = "world";
  msg2uav_.header.frame_id = "world";
  path_.header.frame_id = "world";
  initPoseOutput();
}


void ROSWrapper::livoxHandler(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg){
  ++received_lidar_;
  if(msg->point_num < 10 || msg->point_num != msg->points.size()) return;
  LidarData lidar_data;
  std::size_t ptsize = msg->point_num;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(ptsize / g_filter_rate + 1);

  double offset_time = 0.0;
  double max_offset = 0.0;
  std::size_t invalid_times = 0;
  for (const auto& point : msg->points) {
    const double offset = point.offset_time * 1e-9;
    if (offset > g_max_scan_duration) { ++invalid_times; continue; }
    const auto tag = point.tag & 0x30;
    if (tag == 0x10 || tag == 0x00) max_offset = std::max(max_offset, offset);
  }
  // A scan with mostly corrupt timestamps has too little valid time coverage
  // to constrain an observation. Skip it and integrate its IMUs with the next scan.
  if (invalid_times > msg->point_num / 2) {
    RCLCPP_WARN(node_->get_logger(), "Skipping corrupt LiDAR scan: %zu/%u point offsets exceed %.3f s",
                invalid_times, msg->point_num, g_max_scan_duration);
    return;
  }
  if (invalid_times)
    RCLCPP_WARN(node_->get_logger(), "Discarding %zu LiDAR points with offsets above %.3f s", invalid_times, g_max_scan_duration);
  for(std::size_t _i = 0; _i < ptsize; _i += g_filter_rate){
    auto& pt = msg->points[_i];
    if (pt.offset_time * 1e-9 > g_max_scan_duration) continue;
    auto tag = pt.tag & 0x30;
    if (tag == 0x10 || tag == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2 && dis < g_maxrange2){
        offset_time = pt.offset_time * 1e-9;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, offset_time);
      }
    }
  }
  lidar_data.start_time = rclcpp::Time(msg->header.stamp).seconds();
  lidar_data.end_time = lidar_data.start_time + max_offset;
  if (lidar_data.pc->empty()) return;
  if (lidar_data.start_time < last_received_lidar_) {
    throw std::runtime_error("LiDAR time moved backwards; restart node for a new bag playback");
  }
  last_received_lidar_ = lidar_data.start_time;
  if (lidar_buffer_.size() >= max_lidar_queue_)
    throw std::runtime_error("LiDAR queue limit exceeded; reduce playback rate");
  lidar_buffer_.push_back(lidar_data);
  ++queued_lidar_;
}


inline bool validPoint(double x, double y, double z)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return false;

  double d2 = x * x + y * y + z * z;
  return (d2 > g_blind2 && d2 < g_maxrange2);
}

void ROSWrapper::stdMsgHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
  ++received_lidar_;
  if(msg->data.size() < 10) return;

  LidarData lidar_data;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());

  double offset_time = 0.0;
  double dis = 0.0;

  switch (g_lidar_type) {

  case LID_TYPE::HESAI16:
  {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    const double time_begin = pl_orig.points[0].timestamp;
    lidar_data.start_time = time_begin;
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate)
    {
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.timestamp - time_begin;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = time_begin + offset_time;
    break;
  }
  case LID_TYPE::VEL_NCLT:
  {
    // Converted NCLT bags store relative firing times in microseconds and
    // have no intensity field. The header marks the beginning of the scan.
    const auto time_field = std::find_if(msg->fields.begin(), msg->fields.end(),
        [](const auto& field) { return field.name == "time"; });
    if (time_field == msg->fields.end() ||
        time_field->datatype != sensor_msgs::msg::PointField::FLOAT32) {
      RCLCPP_ERROR(node_->get_logger(), "NCLT requires a FLOAT32 relative 'time' field in microseconds");
      return;
    }
    pcl::PointCloud<NCLT::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = rclcpp::Time(msg->header.stamp).seconds();

    double scan_duration = 0.0;
    for(std::size_t i = 0; i < pl_orig.size(); ++i){
      auto& pt = pl_orig.points[i];
      offset_time = pt.time * 1e-6;
      if (!std::isfinite(offset_time) || offset_time < 0.0 ||
          offset_time > g_max_scan_duration) continue;
      // Scan end must not depend on spatial filtering or subsampling.
      scan_duration = std::max(scan_duration, offset_time);
      if (i % g_filter_rate != 0) continue;
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, 1.0, offset_time);
    }
    if (lidar_data.pc->empty() || scan_duration <= 0.0) return;
    std::stable_sort(lidar_data.pc->begin(), lidar_data.pc->end(),
        [](const auto& a, const auto& b) { return a.offset_time < b.offset_time; });
    lidar_data.end_time = lidar_data.start_time + scan_duration;
    break;
  }
  case LID_TYPE::VELO16:
  case LID_TYPE::VELO32:
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = rclcpp::Time(msg->header.stamp).seconds();

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      // Velodyne drivers stamp the sweep at its END, so per-point time can be
      // negative (down to -period). Normalise into [0, period]: negative times
      // would index M-detector's temporal depth map out of bounds (SIGSEGV).
      const float t = pt.time < 0.0f ? pt.time + 0.1f : pt.time;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, t);
    }
    if (!lidar_data.pc->empty())
      lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time;
    else
      lidar_data.end_time = lidar_data.start_time;
    break;
  }
  case LID_TYPE::VELO_KITTI:
  {
    // KITTI raw -> bag clouds carry only x/y/z/intensity (no per-point time,
    // no ring) and their header.stamp is 0, so the scan is stamped from the
    // sim clock; play the bag with `rosbag play --clock` + use_sim_time.
    pcl::PointCloud<pcl::PointXYZI> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = rclcpp::Time(msg->header.stamp).seconds();
    if (lidar_data.start_time <= 0.0) lidar_data.start_time = node_->now().seconds();

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, 0.0);
    }
    lidar_data.end_time = lidar_data.start_time;  // no per-point time: no deskew
    break;
  }
  case LID_TYPE::RSLIDAR:
  {
    // M2DGR-plus RoboSense bags declare only x/y/z/intensity. The undeclared
    // float at byte offset 16 looks like a noise metric (non-monotonic,
    // 0-57), NOT a time ramp, so no deskew is possible: all points are
    // registered at the scan stamp.
    const std::size_t n_pts = std::size_t(msg->width) * std::size_t(msg->height);
    lidar_data.pc->reserve(n_pts / g_filter_rate + 1);
    lidar_data.start_time = rclcpp::Time(msg->header.stamp).seconds();

    for (std::size_t i = 0; i < n_pts; i += g_filter_rate) {
      const float* p = reinterpret_cast<const float*>(msg->data.data() + i * msg->point_step);
      if (!validPoint(p[0], p[1], p[2])) continue;
      lidar_data.pc->emplace_back(p[0], p[1], p[2], p[3], 0.0);
    }
    lidar_data.end_time = lidar_data.start_time;  // no per-point time: no deskew
    break;
  }
  case OUSTER:
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = rclcpp::Time(msg->header.stamp).seconds();

    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.t * 1e-9;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  default:
    return;
  }

  if (lidar_buffer_.size() >= max_lidar_queue_)
    throw std::runtime_error("LiDAR queue limit exceeded; reduce playback rate");
  lidar_buffer_.push_back(lidar_data);
  ++queued_lidar_;
}



void ROSWrapper::imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr& msg){
  ++received_imu_;
  IMUData data;
  data.secs = rclcpp::Time(msg->header.stamp).seconds();
  data.acc  = V3(msg->linear_acceleration.x,
                 msg->linear_acceleration.y,
                 msg->linear_acceleration.z);
  data.gyr  = V3(msg->angular_velocity.x,
                 msg->angular_velocity.y,
                 msg->angular_velocity.z);

  if (data.secs < last_timestamp_imu_) {
    if (last_timestamp_imu_ - data.secs <= imu_reorder_window_) {
      ++reordered_imu_;
      // Hold scans behind an IMU watermark, so late samples normally still
      // belong to an unprocessed scan. Never integrate an IMU twice.
      if (data.secs <= last_timestamp_lidar_) {
        ++late_imu_;
        return;
      }
      const auto pos = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), data.secs,
          [](const auto& imu, double stamp) { return imu.secs < stamp; });
      if (pos == imu_buffer_.end() || pos->secs != data.secs) imu_buffer_.insert(pos, data);
      last_imu_arrival_ = std::chrono::steady_clock::now();
      // The optional high-rate predictor must also retain increasing time.
      return;
    }
    RCLCPP_ERROR(node_->get_logger(), "IMU timestamp %.9f arrived after %.9f (delta %.6f s)",
                 data.secs, last_timestamp_imu_, data.secs - last_timestamp_imu_);
    throw std::runtime_error("IMU time moved backwards; restart node for a new bag playback");
  }

  if (imu_buffer_.size() >= max_imu_queue_)
    throw std::runtime_error("IMU queue limit exceeded; input cannot keep up");
  imu_buffer_.push_back(data);
  last_timestamp_imu_ = data.secs;
  last_imu_arrival_ = std::chrono::steady_clock::now();

  auto pub_imu_odom = publisher<nav_msgs::msg::Odometry>("/lio/imu/odom", 10);    /// imu frame -> imu freq
  auto pub_robo_odom = publisher<nav_msgs::msg::Odometry>("/lio/robo/odom", 10);   /// robot frame -> imu freq

  DynamicState imu_state, robo_state;
  if(eskf_->Predict(data, imu_state, robo_state)){
    nav_msgs::msg::Odometry odom_imu, odom_robo;

    {
      odom_imu.pose.pose.position.x = imu_state.p(0);
      odom_imu.pose.pose.position.y = imu_state.p(1);
      odom_imu.pose.pose.position.z = imu_state.p(2);

      Quat q(imu_state.R);
      q.normalize();

      odom_imu.pose.pose.orientation.x = q.x();
      odom_imu.pose.pose.orientation.y = q.y();
      odom_imu.pose.pose.orientation.z = q.z();
      odom_imu.pose.pose.orientation.w = q.w();

      odom_imu.twist.twist.linear.x = imu_state.v(0);
      odom_imu.twist.twist.linear.y = imu_state.v(1);
      odom_imu.twist.twist.linear.z = imu_state.v(2);

      odom_imu.twist.twist.angular.x = imu_state.w(0);
      odom_imu.twist.twist.angular.y = imu_state.w(1);
      odom_imu.twist.twist.angular.z = imu_state.w(2);
    }

    {
      odom_robo.pose.pose.position.x = robo_state.p(0);
      odom_robo.pose.pose.position.y = robo_state.p(1);
      odom_robo.pose.pose.position.z = robo_state.p(2);

      Quat q(robo_state.R);
      q.normalize();

      odom_robo.pose.pose.orientation.x = q.x();
      odom_robo.pose.pose.orientation.y = q.y();
      odom_robo.pose.pose.orientation.z = q.z();
      odom_robo.pose.pose.orientation.w = q.w();
    }

    odom_imu.header.stamp = msg->header.stamp;
    odom_robo.header.stamp = msg->header.stamp;
    odom_imu.header.frame_id = "world";
    odom_robo.header.frame_id = "world";
    pub_imu_odom->publish(odom_imu);
    pub_robo_odom->publish(odom_robo);
  }
}


bool ROSWrapper::sync_measure(MeasureGroup& meas){
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }else{
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed_) {
    meas.lidar = lidar_buffer_.front();
    lidar_pushed_ = true;
  }

  if(last_timestamp_lidar_ > meas.lidar.end_time){
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }

  if (last_timestamp_imu_ < meas.lidar.end_time) {
    return false;
  }
  // Account for bounded delivery reordering. At bag end/pause, drain after
  // 0.5 wall seconds without input, while still requiring actual IMU coverage.
  if (imu_reorder_window_ > 0 &&
      last_timestamp_imu_ < meas.lidar.end_time + imu_reorder_window_ &&
      std::chrono::steady_clock::now() - last_imu_arrival_ < std::chrono::milliseconds(500)) {
    return false;
  }

  /*** push imu_ data, and pop from imu_ buffer ***/
  double imu_time = imu_buffer_.front().secs;
  meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < meas.lidar.end_time)) {
    imu_time = imu_buffer_.front().secs;
    if (imu_time > meas.lidar.end_time) break;
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  last_timestamp_lidar_ = meas.lidar.end_time;
  lidar_buffer_.pop_front();
  lidar_pushed_ = false;
  if (meas.imu.empty()) {
    ++empty_imu_scans_;
    return false;
  }
  ++synced_scans_;
  return true;
}


void ROSWrapper::pub_odom(const NavState& state){
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp = stamp_from_seconds(state.timestamp);
  odom.pose.pose.position.x = state.p[0];
  odom.pose.pose.position.y = state.p[1];
  odom.pose.pose.position.z = state.p[2];

  V4 temp_q = state.R.coeffs();
  odom.pose.pose.orientation.x = temp_q[0];
  odom.pose.pose.orientation.y = temp_q[1];
  odom.pose.pose.orientation.z = temp_q[2];
  odom.pose.pose.orientation.w = temp_q[3];

  odom.twist.twist.linear.x = state.v[0];
  odom.twist.twist.linear.y = state.v[1];
  odom.twist.twist.linear.z = state.v[2];

  odom.child_frame_id = "body";
  pub_odom_->publish(odom);
  if(g_pose_output_en){    // imu frame -> lidar frequency

    // Write to file (TUM format)
    if(pose_file_initialized_ && pose_file_.is_open()){
      pose_file_ << state.timestamp << " "
                 << state.p[0] << " "
                 << state.p[1] << " "
                 << state.p[2] << " "
                 << temp_q[0] << " "
                 << temp_q[1] << " "
                 << temp_q[2] << " "
                 << temp_q[3] << std::endl;
    }
  }

  V3 robo_position = state.R.R_ * ( - g_odom_robo.R_ * g_odom_robo.t_) + state.p;

  if(g_2_robot){
    auto pub_msg2uav_ = publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
    M3 robo_rotation = state.R.R_ * g_odom_robo.R_;
    msg2uav_.header.stamp = odom.header.stamp;
    msg2uav_.pose.position.x = robo_position[0];
    msg2uav_.pose.position.y = robo_position[1];
    msg2uav_.pose.position.z = robo_position[2];
    Quat robo_quat(robo_rotation);
    msg2uav_.pose.orientation.w = robo_quat.w();
    msg2uav_.pose.orientation.x = robo_quat.x();
    msg2uav_.pose.orientation.y = robo_quat.y();
    msg2uav_.pose.orientation.z = robo_quat.z();
    pub_msg2uav_->publish(msg2uav_);
  }

  // if(1)
  if((last_path_point_ - robo_position).norm() > 0.1)
  {
    /// nav_msgs::msg::Path
    path_.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped point;
    point.header = odom.header;
    point.pose = odom.pose.pose;
    path_.poses.push_back(point);
    if (path_.poses.size() > path_max_points_)
      path_.poses.erase(path_.poses.begin());
    if (state.timestamp - last_path_publish_ >= path_publish_period_) {
      pub_path_->publish(path_);
      last_path_publish_ = state.timestamp;
    }

    /// sensor_msgs::msg::PointCloud2
    // pcl::PointXYZRGB point_robot;
    // point_robot.x = robo_position[0];
    // point_robot.y = robo_position[1];
    // point_robot.z = robo_position[2];
    // auto [r, g, b] = getColorFromVelocity(state.v.norm());
    // point_robot.r = static_cast<uint8_t>(r * 255);
    // point_robot.g = static_cast<uint8_t>(g * 255);
    // point_robot.b = static_cast<uint8_t>(b * 255);
    // point_robot.a = 255;
    // pcl::PointCloud<pcl::PointXYZRGB> path_point_;
    // path_point_.push_back(point_robot);
    // pcl::toROSMsg(path_point_, msg_path_point_);
    // msg_path_point_.header.stamp = odom.header.stamp;
    // msg_path_point_.header.frame_id = "world";
    // pub_path_robot_->publish(msg_path_point_);

    last_path_point_ = robo_position;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom.header;
  transform.child_frame_id = "body";
  transform.transform.translation.x = state.p[0];
  transform.transform.translation.y = state.p[1];
  transform.transform.translation.z = state.p[2];
  transform.transform.rotation = odom.pose.pose.orientation;
  br_->sendTransform(transform);
}



void ROSWrapper::pub_cloud_world(const CloudPtr& pc,double time){
  auto pub_cloud_world_ = publisher<sensor_msgs::msg::PointCloud2>("/lio/cloud_world", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = stamp_from_seconds(time);
  pub_cloud_world_->publish(cloud);
}


/// M-detector output: moving points vs. the static points used for mapping.
void ROSWrapper::pub_mdet_result(const CloudPtr& dyn_cloud,
                                 const CloudPtr& steady_cloud,
                                 double time, std::size_t input_count){
  if (dyn_cloud->size() + steady_cloud->size() != input_count)
    throw std::runtime_error("Dynamic filter did not conserve point count");
  std_msgs::msg::UInt64MultiArray counts;
  counts.data = {input_count, dyn_cloud->size(), steady_cloud->size()};
  publisher<std_msgs::msg::UInt64MultiArray>("/lio/mdet/counts")->publish(counts);
  auto pub_dyn_cloud_ = publisher<sensor_msgs::msg::PointCloud2>("/lio/mdet/dyn_cloud", 10);
  auto pub_steady_cloud_ = publisher<sensor_msgs::msg::PointCloud2>("/lio/mdet/steady_cloud", 10);

  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*dyn_cloud, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = stamp_from_seconds(time);
  pub_dyn_cloud_->publish(cloud);

  pcl::toROSMsg(*steady_cloud, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = stamp_from_seconds(time);
  pub_steady_cloud_->publish(cloud);
}


void ROSWrapper::pub_cloud2planner(const CloudPtr& pc, double time){
  auto pub_cloud2robot_ = publisher<sensor_msgs::msg::PointCloud2>("/lio/robo/cloud_world", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = stamp_from_seconds(time);
  pub_cloud2robot_->publish(cloud);
}


void ROSWrapper::pub_cloud_body_pose(const CloudPtr& pc,
  const NavState& state)
{
  auto pub_output2robot_ = publisher<super_lio::msg::CloudPose>("/lio/body/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = stamp_from_seconds(state.timestamp);
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];

  pub_output2robot_->publish(cloud_pose);
}


void ROSWrapper::pub_cloud_world_pose(const CloudPtr& pc,
   const NavState& state)
{
  auto pub_output2robot_ = publisher<super_lio::msg::CloudPose>("/lio/world/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = stamp_from_seconds(state.timestamp);
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];

  pub_output2robot_->publish(cloud_pose);
}


void ROSWrapper::pub_cloud_body_pose(
      const BASIC::VV3& pc_body,
      const NavState& state)
{
  auto pub_msg_ = publisher<super_lio::msg::CloudPose2>("/lio/dense/cloud_pose", 10);

  super_lio::msg::CloudPose2 cloud_pose;
  cloud_pose.header.stamp = stamp_from_seconds(state.timestamp);

  cloud_pose.pose.reserve(12);
  cloud_pose.pose.push_back(state.p[0]);
  cloud_pose.pose.push_back(state.p[1]);
  cloud_pose.pose.push_back(state.p[2]);

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      cloud_pose.pose.push_back(state.R.R_(r, c));

  cloud_pose.cloud_lidar.reserve(pc_body.size() * 3);
  for (const auto& pt : pc_body) {
    cloud_pose.cloud_lidar.push_back(pt[0]);
    cloud_pose.cloud_lidar.push_back(pt[1]);
    cloud_pose.cloud_lidar.push_back(pt[2]);
  }

  pub_msg_->publish(cloud_pose);
}


void ROSWrapper::pub_processing_time(double time, double current_time, double mean_time, double std_time)
{
  auto pub_process_time_ = publisher<geometry_msgs::msg::PoseStamped>("/lio/processing_time", 10);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp_from_seconds(time);
  msg.pose.position.x = current_time;
  msg.pose.position.y = mean_time;
  msg.pose.position.z = std_time;
  pub_process_time_->publish(msg);
}


void ROSWrapper::set_global_map(const BASIC::CloudPtr& global_map){
  pcl::toROSMsg(*global_map, global_map_msg_);
  global_map_msg_.header.frame_id = "world";

  auto global_map_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lio/global_map", rclcpp::QoS(1).transient_local());
  publishers_["/lio/global_map"] = global_map_pub;
  global_map_timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this, global_map_pub]() {
    global_map_msg_.header.stamp = node_->now();
    global_map_pub->publish(global_map_msg_);
  });
}

void ROSWrapper::set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init)
{
  if (flg_finish_init) { init_pose_sub_.reset(); return; }
  if (init_pose_sub_) return;
  init_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1,
      [this, &init_pose, &flg_get_init_guess]
      (const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
      {
        V3 init_translation;
        init_translation <<
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            0.2;

        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double w = msg->pose.pose.orientation.w;

        Quat init_rotation(w, x, y, z);

        init_pose = SE3(SO3(init_rotation.toRotationMatrix()), init_translation);

        flg_get_init_guess = true;

        LOG(INFO) << YELLOW
                  << " ---> GET Initial guess: "
                  << init_translation.transpose()
                  << " yaw: "
                  << init_rotation.toRotationMatrix()
                          .eulerAngles(0, 1, 2)
                          .transpose()
                  << RESET;
      });


}


void ROSWrapper::initPoseOutput(){
  if(!g_pose_output_en){
    return;
  }

  // Create output directory if it doesn't exist
  std::filesystem::create_directories(g_pose_output_dir);

  // Open file for writing
  std::string filename = g_pose_output_dir + "/" + g_seq_name + ".txt";
  if(std::filesystem::exists(filename))
    throw std::invalid_argument("Trajectory output already exists; choose a new lio.evo.pose_output_dir");
  pose_file_.open(filename, std::ios::out);

  if(!pose_file_.is_open()){
    LOG(ERROR) << RED << " ---> Failed to open pose output file: " << filename << RESET;
    pose_file_initialized_ = false;
    return;
  }

  // Write TUM format header
  pose_file_ << "# timestamp tx ty tz qx qy qz qw" << std::endl;
  pose_file_.setf(std::ios::fixed, std::ios::floatfield);
  pose_file_.precision(9);

  pose_file_initialized_ = true;
  LOG(INFO) << GREEN << " ---> Pose output file opened: " << filename << RESET;
}


void ROSWrapper::closePoseOutput(){
  if(pose_file_.is_open()){
    pose_file_.close();
    LOG(INFO) << GREEN << " ---> Pose output file closed" << RESET;
  }
  pose_file_initialized_ = false;
}


} // namespace END.
