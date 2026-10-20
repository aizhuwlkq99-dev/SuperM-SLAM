#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/property_tree/json_parser.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "super_lio_mapping/store.hpp"

// Read only sampled records from selected tiles. Dense files stay unchanged.
class TileViewer : public rclcpp::Node {
 public:
  TileViewer() : Node("superm_map_viewer") {
    const auto manifest = declare_parameter<std::string>("manifest", "");
    center_ = Eigen::Vector3d(declare_parameter<double>("center_x", 0),
                              declare_parameter<double>("center_y", 0),
                              declare_parameter<double>("center_z", 0));
    const double radius = declare_parameter<double>("radius", 100);
    const int maximum = declare_parameter<int>("max_points", 200000);
    if (manifest.empty() || radius <= 0 || maximum < 1)
      throw std::invalid_argument("manifest, positive radius and max_points are required");
    boost::property_tree::ptree metadata;
    boost::property_tree::read_json(manifest, metadata);
    if (metadata.get<std::string>("schema") != "superm-map-v1")
      throw std::runtime_error("Unsupported map manifest");
    struct Tile {
      std::string file;
      uint64_t points;
      Eigen::Vector3d origin;
    };
    std::vector<Tile> tiles;
    uint64_t total = 0;
    const double edge = metadata.get<double>("tile_size");
    for (const auto& entry : metadata.get_child("tiles")) {
      Tile tile;
      tile.file = entry.second.get<std::string>("file");
      const auto relative = superm::fs::path(tile.file);
      if (relative.is_absolute())
        throw std::runtime_error("Absolute tile path");
      for (const auto& part : relative)
        if (part == "..")
          throw std::runtime_error("Invalid tile path");
      tile.points = entry.second.get<uint64_t>("points");
      int axis = 0;
      for (const auto& value : entry.second.get_child("origin")) {
        if (axis >= 3)
          throw std::runtime_error("Invalid tile origin");
        tile.origin[axis++] = value.second.get_value<double>();
      }
      if (axis != 3 || !tile.origin.allFinite())
        throw std::runtime_error("Invalid tile origin");
      bool selected = true;
      for (int a = 0; a < 3; ++a)
        selected &=
            tile.origin[a] < center_[a] + radius && tile.origin[a] + edge > center_[a] - radius;
      if (selected) {
        total += tile.points;
        tiles.push_back(tile);
      }
    }
    const uint64_t stride = std::max<uint64_t>(1, (total + maximum - 1) / maximum);
    superm::Cloud cloud;
    cloud.reserve(maximum);
    for (const auto& tile : tiles) {
      std::ifstream input(superm::fs::path(manifest).parent_path() / tile.file, std::ios::binary);
      if (!input)
        throw std::runtime_error("Cannot read tile");
      std::string line;
      bool binary = false;
      while (std::getline(input, line)) {
        if (line == "DATA binary") {
          binary = true;
          break;
        }
        if (line.rfind("DATA ", 0) == 0)
          break;
      }
      if (!binary)
        throw std::runtime_error("Viewer expects the exported binary XYZI tiles");
      const auto start = input.tellg();
      for (uint64_t i = 0; i < tile.points && cloud.size() < size_t(maximum); i += stride) {
        float point[4];
        input.seekg(start + std::streamoff(i * sizeof(point)));
        if (!input.read(reinterpret_cast<char*>(point), sizeof(point)))
          throw std::runtime_error("Truncated tile");
        Eigen::Vector3d local =
            Eigen::Vector3d(point[0], point[1], point[2]) + tile.origin - center_;
        if (!local.allFinite() || local.head<2>().norm() > radius)
          continue;
        pcl::PointXYZI p;
        p.x = local.x();
        p.y = local.y();
        p.z = local.z();
        p.intensity = point[3];
        cloud.push_back(p);
      }
    }
    pcl::toROSMsg(cloud, message_);
    message_.header.frame_id = "map_view";
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/slam/map_tiles",
                                                                 rclcpp::QoS(1).transient_local());
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      message_.header.stamp = now();
      geometry_msgs::msg::TransformStamped transform;
      transform.header = message_.header;
      transform.header.frame_id = "map";
      transform.child_frame_id = "map_view";
      transform.transform.translation.x = center_.x();
      transform.transform.translation.y = center_.y();
      transform.transform.translation.z = center_.z();
      transform.transform.rotation.w = 1;
      broadcaster_->sendTransform(transform);
      publisher_->publish(message_);
    });
    RCLCPP_INFO(get_logger(), "Loaded %zu preview points from %zu tiles; dense files unchanged",
                cloud.size(), tiles.size());
  }

 private:
  Eigen::Vector3d center_;
  sensor_msgs::msg::PointCloud2 message_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TileViewer>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
