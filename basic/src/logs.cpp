#include "basic/logs.h"
#include <cstdlib>

namespace BASIC {

void init_log(){
  const auto* glog_dir = std::getenv("GLOG_log_dir");
  const auto* ros_dir = std::getenv("ROS_LOG_DIR");
  if (glog_dir && *glog_dir) {
    init_log_dir(glog_dir);
  } else if (ros_dir && *ros_dir) {
    init_log_dir((std::filesystem::path(ros_dir) / "glog").string());
  } else {
    init_log_dir((std::filesystem::current_path() / "log" / "glog").string());
  }
}


void init_log_dir(std::string log_dir){
  FLAGS_logbufsecs = 0;
  FLAGS_minloglevel = google::INFO;
  FLAGS_alsologtostderr = true;
  const auto directory = std::filesystem::absolute(log_dir);
  std::filesystem::create_directories(directory);
  FLAGS_log_dir = directory.string();
}

}
