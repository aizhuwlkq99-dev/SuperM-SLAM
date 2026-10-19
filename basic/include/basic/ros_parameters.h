#pragma once

#include <rclcpp/rclcpp.hpp>
#include <type_traits>
#include <cmath>

namespace BASIC {
inline builtin_interfaces::msg::Time stamp_from_seconds(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}

// ROS parameters store floating point values as doubles and integers as int64.
template <typename T>
void read_parameter(rclcpp::Node& node, const std::string& key, T& value,
                    const T& fallback) {
  if constexpr (std::is_floating_point_v<T>) {
    if (!node.has_parameter(key)) node.declare_parameter<double>(key, fallback);
    value = static_cast<T>(node.get_parameter(key).as_double());
  } else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
    if (!node.has_parameter(key)) node.declare_parameter<int64_t>(key, fallback);
    value = static_cast<T>(node.get_parameter(key).as_int());
  } else {
    if (!node.has_parameter(key)) node.declare_parameter<T>(key, fallback);
    value = node.get_parameter(key).get_value<T>();
  }
}
}  // namespace BASIC
