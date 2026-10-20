#include <signal.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "super_lio_mapping/store.hpp"

namespace {
void require(bool condition, const char* message) {
  if (!condition)
    throw std::runtime_error(message);
}
superm::Frame frame() {
  superm::Frame result;
  for (size_t i = 0; i < 10000; ++i) {
    pcl::PointXYZI point;
    point.x = i * .01f;
    point.y = point.z = 0;
    point.intensity = 1;
    result.cloud.push_back(point);
  }
  return result;
}
void limit_file_size() {
  ::signal(SIGXFSZ, SIG_IGN);
  const rlimit limit{1024, 1024};
  require(::setrlimit(RLIMIT_FSIZE, &limit) == 0, "Cannot inject write failure");
}
template <class Operation>
void child_check(Operation operation) {
  const pid_t child = ::fork();
  require(child >= 0, "fork failed");
  if (child == 0) {
    try {
      limit_file_size();
      operation();
      ::_exit(0);
    } catch (const std::exception& error) {
      std::cerr << error.what() << std::endl;
      ::_exit(1);
    }
  }
  int status = 0;
  require(::waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
          "Injected fault was not handled correctly");
}
}  // namespace
int main(int argc, char** argv) {
  try {
    require(argc == 2, "Provide a new test directory");
    const superm::fs::path root = argv[1];
    require(!superm::fs::exists(root), "Test output exists");
    superm::StoreOptions options;
    options.queue_bytes = 1024;
    superm::FrameStore overflow(root / "overflow", options);
    bool failed = false;
    try {
      overflow.submit(frame());
    } catch (const std::exception&) {
      failed = true;
    }
    require(failed, "Oversized frame was accepted");
    failed = false;
    try {
      overflow.close();
    } catch (const std::exception&) {
      failed = true;
    }
    require(failed, "Queue failure was hidden by close");
    std::ifstream input(root / "overflow/manifest.json");
    std::string manifest((std::istreambuf_iterator<char>(input)), {});
    require(manifest.find("\"complete\":false") != std::string::npos &&
                manifest.find("queue budget exceeded") != std::string::npos,
            "Queue failure was not persisted");
    // Recovery must be available even while the failed writer object still exists.
    require(superm::recover_store(root / "overflow") == 0, "Failure retained the writer lock");
    child_check([&] {
      superm::FrameStore store(root / "write_failure");
      store.submit(frame());
      bool rejected = false;
      try {
        store.close();
      } catch (const std::exception&) {
        rejected = true;
      }
      require(rejected, "Partial frame write reported success");
      require(store.stats().durable == 0, "Partial chunk was marked durable");
    });
    {
      superm::FrameStore store(root / "valid");
      store.submit(frame());
      store.close();
    }
    child_check([&] {
      bool rejected = false;
      try {
        superm::export_map(root / "valid", root / "failed_export");
      } catch (const std::exception&) {
        rejected = true;
      }
      require(rejected, "Partial export reported success");
      require(!superm::fs::exists(root / "failed_export"), "Failed map was published");
    });
    std::cout << "queue overflow, writer unlock, partial writes, unpublished failed export: PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
