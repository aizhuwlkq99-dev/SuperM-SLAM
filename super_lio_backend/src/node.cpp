#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <boost/property_tree/json_parser.hpp>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <super_lio_interfaces/srv/export_map.hpp>

#include "super_lio_backend/backend.hpp"
class MappingNode : public rclcpp::Node {
 public:
  MappingNode() : Node("superm_backend") {
    store_ = declare_parameter<std::string>("store_path", "");
    output_ = declare_parameter<std::string>("graph_directory", "");
    leaf_ = declare_parameter<double>("map_leaf", .1);
    shutdown_export_ = declare_parameter<std::string>("shutdown_export", "none");
    shutdown_maps_ = declare_parameter<std::string>("shutdown_map_directory", "");
    if (shutdown_export_ != "none" && shutdown_export_ != "raw" && shutdown_export_ != "both")
      throw std::invalid_argument("shutdown_export must be none, raw, or both");
    if (shutdown_export_ != "none" && shutdown_maps_.empty())
      throw std::invalid_argument("shutdown_map_directory is required for shutdown export");
    if (store_.empty() || output_.empty())
      throw std::invalid_argument("store_path and graph_directory are required");
    superm::BackendOptions options;
    options.loop_closure = declare_parameter<bool>("loop_closure", true);
    options.global_registration = declare_parameter<bool>("global_registration", true);
    options.context_frames = declare_parameter<int>("context_frames", 10);
    options.threads = declare_parameter<int>("registration_threads", 2);
    graph_ = std::make_unique<superm::Backend>(store_, output_, options);
    path_ = create_publisher<nav_msgs::msg::Path>("/slam/path_optimized", 1);
    map_ = create_publisher<sensor_msgs::msg::PointCloud2>("/slam/map_optimized", 1);
    status_ = create_publisher<std_msgs::msg::String>("/slam/export_status", 1);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      graph_->poll();
      publish();
    });
    save_ = create_service<super_lio_interfaces::srv::ExportMap>(
        "/slam/save_map",
        [this](const std::shared_ptr<super_lio_interfaces::srv::ExportMap::Request> request,
               std::shared_ptr<super_lio_interfaces::srv::ExportMap::Response> response) {
          if (exporting_) {
            response->accepted = false;
            response->message = "An export is already running";
            return;
          }
          if (export_thread_.joinable())
            export_thread_.join();
          try {
            graph_->poll();
            graph_->save();
            superm::ExportOptions opt;
            opt.leaf = request->leaf_size > 0 ? request->leaf_size : leaf_;
            opt.single_pcd = request->single_pcd;
            // Freeze every pose, even for raw output, so newly committed frames cannot leak into
            // this version.
            if (request->raw_poses) {
              for (auto& f : graph_->frames())
                opt.poses.push_back(f.pose);
              opt.graph_version = "raw";
            } else {
              opt.poses = graph_->frame_poses();
              opt.graph_version = "graph-loops-" + std::to_string(graph_->loops());
            }
            if (!request->pose_snapshot.empty()) {
              std::ifstream in(request->pose_snapshot);
              if (!in)
                throw std::runtime_error("cannot open pose snapshot");
              opt.poses.clear();
              uint64_t id;
              while (in >> id) {
                if (id != opt.poses.size())
                  throw std::runtime_error("noncontiguous pose snapshot");
                Eigen::Matrix4d T;
                for (int r = 0; r < 4; ++r)
                  for (int c = 0; c < 4; ++c)
                    if (!(in >> T(r, c)))
                      throw std::runtime_error("invalid snapshot");
                opt.poses.push_back(T);
              }
              opt.graph_version = request->pose_snapshot;
            }
            if (opt.poses.empty())
              throw std::runtime_error("No committed frames to export");
            auto destination = superm::fs::absolute(request->output_directory);
            if (request->output_directory.empty() || superm::fs::exists(destination))
              throw std::runtime_error("Use a new output directory");
            exporting_ = true;
            export_thread_ = std::thread([this, opt, destination] {
              std_msgs::msg::String status;
              try {
                auto result = superm::export_map(store_, destination, opt);
                status.data = "complete: " + destination.string() +
                              " points=" + std::to_string(result.points);
              } catch (const std::exception& e) {
                status.data = std::string("failed: ") + e.what();
              }
              if (rclcpp::ok()) {
                try {
                  status_->publish(status);
                } catch (const rclcpp::exceptions::RCLError&) {
                }
              }
              std::cout << status.data << std::endl;
              exporting_ = false;
            });
            response->accepted = true;
            response->message = "Export started; monitor /slam/export_status and output manifest";
          } catch (const std::exception& e) {
            response->accepted = false;
            response->message = e.what();
          }
        });
    undo_ = create_service<std_srvs::srv::Trigger>(
        "/slam/reject_last_loop", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> r) {
          r->success = graph_->reject_last_loop();
          r->message =
              r->success ? "Last accepted loop removed; graph rebuilt" : "No accepted loop";
        });
  }
  ~MappingNode() override {
    if (export_thread_.joinable())
      export_thread_.join();
  }
  void finish() {
    if (export_thread_.joinable())
      export_thread_.join();
    // The frontend flushes asynchronously on the same shutdown signal.
    for (int i = 0; i < 100; ++i) {
      try {
        boost::property_tree::ptree p;
        boost::property_tree::read_json((superm::fs::path(store_) / "manifest.json").string(), p);
        if (p.get<bool>("complete", false))
          break;
      } catch (const std::exception&) {
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    graph_->poll();
    graph_->save();
    std::cout << "Backend saved: frames=" << graph_->frames().size()
              << " keyframes=" << graph_->keyframes() << " loops=" << graph_->loops() << std::endl;
    if (shutdown_export_ != "none" && !graph_->frames().empty()) {
      boost::property_tree::ptree manifest;
      boost::property_tree::read_json((superm::fs::path(store_) / "manifest.json").string(),
                                      manifest);
      if (!manifest.get<bool>("complete", false))
        throw std::runtime_error("Frontend store is incomplete; automatic export skipped");
      superm::ExportOptions options;
      options.leaf = leaf_;
      for (const auto& frame : graph_->frames())
        options.poses.push_back(frame.pose);
      auto maps = superm::fs::path(shutdown_maps_);
      superm::export_map(store_, maps / "raw", options);
      if (shutdown_export_ == "both") {
        options.poses = graph_->frame_poses();
        options.graph_version = "graph-loops-" + std::to_string(graph_->loops());
        superm::export_map(store_, maps / "optimized", options);
      }
      std::cout << "Shutdown map export complete: " << maps << std::endl;
    }
  }

 private:
  void publish() {
    if (graph_->frames().empty())
      return;
    const auto T = graph_->correction();
    Eigen::Quaterniond q(T.block<3, 3>(0, 0));
    q.normalize();
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "world";
    tf.transform.translation.x = T(0, 3);
    tf.transform.translation.y = T(1, 3);
    tf.transform.translation.z = T(2, 3);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_->sendTransform(tf);
    auto poses = graph_->frame_poses();
    nav_msgs::msg::Path path;
    path.header = tf.header;
    path.header.frame_id = "map";
    size_t stride = std::max<size_t>(1, (poses.size() + 4999) / 5000);
    for (size_t i = 0; i < poses.size(); i += stride) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = poses[i](0, 3);
      p.pose.position.y = poses[i](1, 3);
      p.pose.position.z = poses[i](2, 3);
      Eigen::Quaterniond r(poses[i].block<3, 3>(0, 0));
      r.normalize();
      p.pose.orientation.x = r.x();
      p.pose.orientation.y = r.y();
      p.pose.orientation.z = r.z();
      p.pose.orientation.w = r.w();
      path.poses.push_back(p);
    }
    path_->publish(path);
    if (++ticks_ % 5 == 0 && map_->get_subscription_count()) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(graph_->preview(), msg);
      msg.header = path.header;
      map_->publish(msg);
    }
  }
  std::string store_, output_, shutdown_export_, shutdown_maps_;
  double leaf_;
  size_t ticks_ = 0;
  std::unique_ptr<superm::Backend> graph_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<super_lio_interfaces::srv::ExportMap>::SharedPtr save_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_;
  std::thread export_thread_;
  std::atomic<bool> exporting_{false};
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MappingNode>();
    rclcpp::spin(node);
    node->finish();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Backend error: " << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
