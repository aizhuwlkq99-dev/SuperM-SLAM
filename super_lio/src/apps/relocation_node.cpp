#include <rclcpp/rclcpp.hpp>
#include <tbb/global_control.h>
#include "lio/super_lio_reloc.h"
#include "basic/logs.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  google::InitGoogleLogging(argv[0]);
  int result = 0;
  try {
    BASIC::init_log();
    auto node = std::make_shared<rclcpp::Node>("relocation_node");
    const int threads = node->declare_parameter<int>("processing_threads", 4);
    if (threads < 1) throw std::invalid_argument("processing_threads must be positive");
    tbb::global_control parallelism(tbb::global_control::max_allowed_parallelism, threads);
    LI2Sup::LoadParamFromRos(*node);
    auto wrapper = std::make_shared<LI2Sup::ROSWrapper>(node);
    auto lio = std::make_shared<LI2Sup::SuperLIOReLoc>();
    lio->setROSWrapper(wrapper);
    lio->init();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    rclcpp::WallRate rate(500);
    while (rclcpp::ok() && LI2Sup::g_flag_run) {
      executor.spin_some();
      lio->process();
      rate.sleep();
    }
    lio->saveMap();
    lio->printTimeRecord();
  } catch (const std::exception& e) {
    LOG(ERROR) << e.what();
    result = 1;
  }
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return result;
}
