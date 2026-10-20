#ifndef TIMER_H_
#define TIMER_H_
#include <glog/logging.h>

#include <chrono>
#include <map>
#include <string>
#include <utility>
// Constant storage per measured function; long runs no longer retain every sample.
class Timer {
 public:
  struct TimerRecord {
    size_t count = 0;
    double mean = 0, m2 = 0, maximum = 0;
  };
  template <class F>
  void Evaluate(F&& fn, const std::string& name) {
    auto start = std::chrono::steady_clock::now();
    std::forward<F>(fn)();
    double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    auto& r = records_[name];
    double d = ms - r.mean;
    r.mean += d / ++r.count;
    r.m2 += d * (ms - r.mean);
    r.maximum = std::max(r.maximum, ms);
  }
  void PrintAll() {
    double total = 0;
    for (const auto& item : records_) {
      const auto& r = item.second;
      LOG(INFO) << item.first << " mean_ms=" << r.mean << " max_ms=" << r.maximum
                << " count=" << r.count;
      total += r.mean;
    }
    LOG(INFO) << "Algorithm mean_ms=" << total;
  }
  void Clear() {
    records_.clear();
  }

 private:
  std::map<std::string, TimerRecord> records_;
};
#endif
