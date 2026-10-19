cmake_minimum_required(VERSION 3.16)
project(livox_ros_driver2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg" "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs)
add_subdirectory(../Livox-SDK2/sdk_core ${CMAKE_CURRENT_BINARY_DIR}/livox_sdk2)
add_library(livox_driver_component SHARED
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp
  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp
  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp
  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)
target_compile_definitions(livox_driver_component PRIVATE BUILDING_ROS2)
target_include_directories(livox_driver_component PRIVATE src 3rdparty ${PCL_INCLUDE_DIRS})
ament_target_dependencies(livox_driver_component rclcpp rclcpp_components rcutils sensor_msgs std_msgs pcl_conversions)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(livox_driver_component ${typesupport_target} livox_lidar_sdk_shared ${PCL_LIBRARIES} pthread)
rclcpp_components_register_node(livox_driver_component
  PLUGIN "livox_ros::DriverNode" EXECUTABLE livox_ros_driver2_node)
install(TARGETS livox_driver_component LIBRARY DESTINATION lib)
install(DIRECTORY config launch_ROS2 DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
