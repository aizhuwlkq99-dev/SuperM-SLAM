

#ifndef ROSWRAPPER_HPP_
#define ROSWRAPPER_HPP_

#include <map>
#include <tuple>
#include <deque>
#include <vector>
#include <execution>
#include <fstream>

 #include <pcl/point_types.h>
 #include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "basic/alias.h"
#include "basic/logs.h"
#include "basic/Manifold.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "common/ds.h"

#include "lio/params.h"
#include "lio/ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"


// #define SIM_GAZEBO


namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node);

std::tuple<float, float, float> getColorFromVelocity(float velocity, float max_velocity);

void livox2pcl(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg, BASIC::CloudPtr& point_cloud);

class ROSWrapper{
public:
  explicit ROSWrapper(const rclcpp::Node::SharedPtr& node);
  rclcpp::Node::SharedPtr node() const { return node_; }
  ~ROSWrapper(){
    LOG(INFO) << "Sensor summary: received_lidar=" << received_lidar_
              << " queued_lidar=" << queued_lidar_ << " synced_scans=" << synced_scans_
              << " empty_imu_scans=" << empty_imu_scans_
              << " pending_lidar=" << lidar_buffer_.size()
              << " received_imu=" << received_imu_
              << " reordered_imu=" << reordered_imu_ << " late_imu=" << late_imu_;
    closePoseOutput();
  };
  using Ptr = std::shared_ptr<ROSWrapper>;
  bool sync_measure(MeasureGroup&);


  void setESKF(ESKF::Ptr& eskf) {
    eskf_ = eskf;
  }

  void setMap(OctVoxMap<BASIC::V3, BASIC::scalar>::Ptr& ivox) {
    ivox_ = ivox;
  }

  void clear(){
    lidar_buffer_.clear();
    imu_buffer_.clear();
    lidar_pushed_ = false;
    last_timestamp_imu_ = -1.0;
    last_timestamp_lidar_ = -1.0;
  }

  void pub_odom(const NavState&);
  void pub_cloud_world(const BASIC::CloudPtr& pc, double time);
  void pub_mdet_result(const BASIC::CloudPtr& dyn_cloud,
                       const BASIC::CloudPtr& steady_cloud,
                       double time, std::size_t input_count);
  void pub_cloud2planner(const BASIC::CloudPtr& pc, double time);
  void pub_cloud_body_pose(const BASIC::CloudPtr& pc,
                           const NavState& state);
  void pub_cloud_world_pose(const BASIC::CloudPtr& pc,
                            const NavState& state);
  void pub_cloud_body_pose( const BASIC::VV3& pc_body,
                            const NavState& state);
  void pub_processing_time(double time, double current_time, double mean_time, double std_time);

  void set_global_map(const BASIC::CloudPtr& global_map);

  void set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init = false);

  void initPoseOutput();
  void closePoseOutput();


private:
  void imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr&);
  void livoxHandler(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr&);
  void stdMsgHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&);


  rclcpp::Node::SharedPtr node_;
  rclcpp::SubscriptionBase::SharedPtr subLidar_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subIMU_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::TimerBase::SharedPtr global_map_timer_;
  std::map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
  template<typename Msg>
  typename rclcpp::Publisher<Msg>::SharedPtr publisher(const std::string& topic, int depth = 10) {
    auto it = publishers_.find(topic);
    if (it == publishers_.end()) {
      auto pub = node_->create_publisher<Msg>(topic, depth);
      publishers_[topic] = pub;
      return pub;
    }
    return std::static_pointer_cast<rclcpp::Publisher<Msg>>(it->second);
  }
  std::deque<IMUData>   imu_buffer_;
  std::deque<LidarData> lidar_buffer_;
  bool lidar_pushed_ = false;
  double last_timestamp_imu_ = -1.0;
  double last_timestamp_lidar_ = -1.0;
  double last_received_lidar_ = -1.0;
  std::size_t received_lidar_ = 0, queued_lidar_ = 0;
  std::size_t synced_scans_ = 0, empty_imu_scans_ = 0;
  std::size_t received_imu_ = 0;
  std::size_t reordered_imu_ = 0, late_imu_ = 0;
  double imu_reorder_window_ = 0.0;
  std::chrono::steady_clock::time_point last_imu_arrival_;

  ESKF::Ptr eskf_ = nullptr;
  OctVoxMap<BASIC::V3, BASIC::scalar>::Ptr ivox_ = nullptr;

  /// output
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;        // imu frame -> lidar frequency
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;        // robo path
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_path_robot_;

  size_t path_max_points_ = 5000;
  double path_publish_period_ = 1.0;
  double last_path_publish_ = -1.0;
  size_t max_lidar_queue_ = 128, max_imu_queue_ = 20000;
  nav_msgs::msg::Path path_;
  sensor_msgs::msg::PointCloud2 msg_path_point_;
  sensor_msgs::msg::PointCloud2 global_map_msg_;
  geometry_msgs::msg::PoseStamped msg2uav_;
  BASIC::V3 last_path_point_ = BASIC::V3(0, 0, -100);
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

  // Pose output to file
  std::ofstream pose_file_;
  bool pose_file_initialized_ = false;
};

} // namespace END.

#endif
