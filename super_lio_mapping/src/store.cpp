#include "super_lio_mapping/store.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace superm {
namespace {
constexpr uint64_t magic = 0x31454d4152464d53ULL;
constexpr size_t header_size = 8 + 8 + 24 + 128 + 8 + 8;
void write_all(int fd, const void* ptr, size_t n) {
  auto p = static_cast<const char*>(ptr);
  while (n) {
    auto done = ::write(fd, p, n);
    if (done < 0 && errno == EINTR)
      continue;
    if (done <= 0)
      throw std::runtime_error("frame write failed: " + std::string(strerror(errno)));
    p += done;
    n -= done;
  }
}
void sync_dir(const fs::path& p) {
  int fd = ::open(p.c_str(), O_RDONLY | O_DIRECTORY);
  if (fd < 0)
    throw std::runtime_error("cannot open directory for sync");
  int r = ::fsync(fd);
  ::close(fd);
  if (r)
    throw std::runtime_error("directory sync failed");
}
uint64_t hash(const char* p, size_t n, uint64_t h = 1469598103934665603ULL) {
  for (size_t i = 0; i < n; ++i) {
    h ^= static_cast<unsigned char>(p[i]);
    h *= 1099511628211ULL;
  }
  return h;
}
template <class T>
void put(std::vector<char>& b, const T& v) {
  auto p = reinterpret_cast<const char*>(&v);
  b.insert(b.end(), p, p + sizeof(T));
}
template <class T>
T get(const std::vector<char>& b, size_t& k) {
  if (k + sizeof(T) > b.size())
    throw std::runtime_error("truncated frame");
  T v;
  memcpy(&v, b.data() + k, sizeof(T));
  k += sizeof(T);
  return v;
}
size_t bytes(const Frame& f) {
  return sizeof(Frame) + sizeof(pcl::PointXYZI) * f.cloud.points.capacity();
}
size_t wire_bytes(const Frame& f) {
  return header_size + 16 * f.cloud.size();
}
}  // namespace
std::string json_string(const std::string& s) {
  std::ostringstream o;
  o << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\')
      o << '\\' << char(c);
    else if (c < 32)
      o << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << int(c) << std::dec;
    else
      o << char(c);
  }
  o << '"';
  return o.str();
}
void atomic_text(const fs::path& p, const std::string& s) {
  fs::create_directories(p.parent_path());
  auto tmp = p;
  tmp += ".part";
  int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0)
    throw std::runtime_error("cannot create " + tmp.string());
  try {
    write_all(fd, s.data(), s.size());
    if (::fsync(fd))
      throw std::runtime_error("sync failed");
    ::close(fd);
    fd = -1;
    fs::rename(tmp, p);
    sync_dir(p.parent_path());
  } catch (...) {
    if (fd >= 0)
      ::close(fd);
    throw;
  }
}
FrameStore::FrameStore(const fs::path& root, StoreOptions opt,
                       std::function<void(const FrameInfo&)> cb)
    : root_(root), options_(opt), committed_(std::move(cb)) {
  uint16_t endian = 1;
  if (*reinterpret_cast<char*>(&endian) != 1)
    throw std::runtime_error("SMF v1 requires little endian");
  if (!opt.queue_bytes || !opt.chunk_bytes || !std::isfinite(opt.commit_seconds) ||
      opt.commit_seconds <= 0)
    throw std::invalid_argument("invalid store budget");
  fs::create_directories(root_ / "frames");
  if (fs::exists(root_ / "index.txt") || fs::exists(root_ / "manifest.json"))
    throw std::runtime_error("store already exists; use a new run directory");
  lock_fd_ = ::open((root_ / "writer.lock").c_str(), O_RDWR | O_CREAT, 0644);
  if (lock_fd_ < 0 || ::flock(lock_fd_, LOCK_EX | LOCK_NB)) {
    if (lock_fd_ >= 0)
      ::close(lock_fd_);
    lock_fd_ = -1;
    throw std::runtime_error("store is locked");
  }
  try {
    atomic_text(root_ / "index.txt", "");
    manifest(false);
    worker_ = std::thread(&FrameStore::run, this);
  } catch (...) {
    ::close(lock_fd_);
    lock_fd_ = -1;
    throw;
  }
}
FrameStore::~FrameStore() {
  try {
    close();
  } catch (...) {
  }
  if (fd_ >= 0)
    ::close(fd_);
  if (lock_fd_ >= 0)
    ::close(lock_fd_);
}
StoreStats FrameStore::stats() const {
  std::lock_guard<std::mutex> l(mutex_);
  return {submitted_, durable_, points_, bytes_, peak_, error_};
}
void FrameStore::submit(Frame f) {
  if (!f.pose.allFinite() || !std::isfinite(f.reference) || !std::isfinite(f.start) ||
      !std::isfinite(f.end) || f.start > f.end)
    throw std::invalid_argument("invalid frame pose/time");
  size_t n = bytes(f);
  std::lock_guard<std::mutex> l(mutex_);
  if (stopping_ || !error_.empty())
    throw std::runtime_error("frame store unavailable: " + error_);
  if (f.id != submitted_)
    throw std::runtime_error("noncontiguous frame id");
  if (n > options_.queue_bytes || bytes_ > options_.queue_bytes - n) {
    error_ = "frame queue budget exceeded; run incomplete (reduce offline playback rate)";
    stopping_ = true;
    condition_.notify_all();
    throw std::runtime_error(error_);
  }
  bytes_ += n;
  peak_ = std::max(peak_, bytes_);
  ++submitted_;
  queue_.push_back(std::move(f));
  condition_.notify_one();
}
void FrameStore::manifest(bool complete) {
  auto s = stats();
  std::ostringstream o;
  o << "{\n\"schema\":\"superm-frames-v1\",\"coordinate_frame\":\"body\",\"pose_frame\":\"world\","
       "\"complete\":"
    << (complete ? "true" : "false") << ",\"submitted_frames\":" << s.queued
    << ",\"durable_frames\":" << s.durable << ",\"points\":" << s.points
    << ",\"peak_queue_bytes\":" << s.peak_queue_bytes << ",\"error\":" << json_string(s.error)
    << "}\n";
  atomic_text(root_ / "manifest.json", o.str());
}
void FrameStore::close() {
  {
    std::lock_guard<std::mutex> l(mutex_);
    stopping_ = true;
    condition_.notify_all();
  }
  if (worker_.joinable())
    worker_.join();
  auto s = stats();
  if (!closed_) {
    try {
      manifest(s.error.empty() && s.queued == s.durable);
      closed_ = true;
    } catch (...) {
      if (lock_fd_ >= 0) {
        ::close(lock_fd_);
        lock_fd_ = -1;
      }
      throw;
    }
    if (lock_fd_ >= 0) {
      ::close(lock_fd_);
      lock_fd_ = -1;
    }
  }
  if (!s.error.empty())
    throw std::runtime_error("frame store: " + s.error);
}
void FrameStore::commit() {
  if (fd_ < 0)
    return;
  if (::fsync(fd_))
    throw std::runtime_error("chunk sync failed");
  if (::close(fd_))
    throw std::runtime_error("chunk close failed");
  fd_ = -1;
  fs::rename(root_ / "frames" / (chunk_name_ + ".part"), root_ / "frames" / chunk_name_);
  sync_dir(root_ / "frames");
  std::ostringstream rows;
  rows << std::setprecision(17);
  for (auto& i : pending_) {
    rows << i.id << ' ' << i.chunk << ' ' << i.offset << ' ' << i.points << ' ' << i.start << ' '
         << i.end << ' ' << i.reference;
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        rows << ' ' << i.pose(r, c);
    rows << '\n';
  }
  int fd = ::open((root_ / "index.txt").c_str(), O_WRONLY | O_APPEND);
  if (fd < 0)
    throw std::runtime_error("cannot append index");
  auto s = rows.str();
  try {
    write_all(fd, s.data(), s.size());
    if (::fsync(fd))
      throw std::runtime_error("index sync failed");
    ::close(fd);
  } catch (...) {
    ::close(fd);
    throw;
  }
  {
    std::lock_guard<std::mutex> l(mutex_);
    for (auto& i : pending_) {
      ++durable_;
      points_ += i.points;
    }
  }
  manifest(false);
  if (committed_)
    for (auto& i : pending_)
      committed_(i);
  pending_.clear();
  chunk_size_ = 0;
  ++chunk_id_;
}
void FrameStore::run() {
  try {
    auto last = std::chrono::steady_clock::now();
    while (true) {
      Frame f;
      bool have = false;
      {
        std::unique_lock<std::mutex> l(mutex_);
        condition_.wait_for(l, std::chrono::milliseconds(50),
                            [&] { return stopping_ || !queue_.empty(); });
        if (!queue_.empty()) {
          f = std::move(queue_.front());
          queue_.pop_front();
          have = true;
        } else if (stopping_)
          break;
      }
      if (have) {
        if (fd_ < 0) {
          std::ostringstream n;
          n << "chunk_" << std::setw(8) << std::setfill('0') << chunk_id_ << ".smf";
          chunk_name_ = n.str();
          fd_ = ::open((root_ / "frames" / (chunk_name_ + ".part")).c_str(),
                       O_WRONLY | O_CREAT | O_EXCL, 0644);
          if (fd_ < 0)
            throw std::runtime_error("cannot create frame chunk");
        }
        std::vector<char> b;
        b.reserve(wire_bytes(f));
        put(b, magic);
        put(b, f.id);
        put(b, f.start);
        put(b, f.end);
        put(b, f.reference);
        for (int r = 0; r < 4; ++r)
          for (int c = 0; c < 4; ++c)
            put(b, f.pose(r, c));
        uint64_t count = f.cloud.size();
        put(b, count);
        size_t hash_offset = b.size();
        put(b, uint64_t(0));
        for (auto& p : f.cloud) {
          put(b, p.x);
          put(b, p.y);
          put(b, p.z);
          put(b, p.intensity);
        }
        uint64_t checksum = hash(b.data(), hash_offset);
        checksum = hash(b.data() + header_size, b.size() - header_size, checksum);
        memcpy(b.data() + hash_offset, &checksum, 8);
        write_all(fd_, b.data(), b.size());
        pending_.push_back(
            {f.id, chunk_size_, count, chunk_name_, f.start, f.end, f.reference, f.pose});
        chunk_size_ += b.size();
        {
          std::lock_guard<std::mutex> l(mutex_);
          bytes_ -= bytes(f);
        }
      }
      if (fd_ >= 0 &&
          (chunk_size_ >= options_.chunk_bytes ||
           std::chrono::duration<double>(std::chrono::steady_clock::now() - last).count() >=
               options_.commit_seconds)) {
        commit();
        last = std::chrono::steady_clock::now();
      }
    }
    commit();
  } catch (const std::exception& e) {
    {
      std::lock_guard<std::mutex> l(mutex_);
      error_ = e.what();
      stopping_ = true;
    }
    try {
      manifest(false);
    } catch (...) {
    }
  }
}
std::vector<FrameInfo> read_index(const fs::path& root) {
  std::ifstream in(root / "index.txt");
  if (!in)
    throw std::runtime_error("cannot read frame index");
  std::vector<FrameInfo> out;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty())
      continue;
    FrameInfo f;
    std::istringstream row(line);
    if (!(row >> f.id >> f.chunk >> f.offset >> f.points >> f.start >> f.end >> f.reference))
      throw std::runtime_error("incomplete index; run superm_export recover first");
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        if (!(row >> f.pose(r, c)))
          throw std::runtime_error("invalid index pose");
    std::string extra;
    if (row >> extra)
      throw std::runtime_error("extra index fields");
    if (f.id != out.size() || fs::path(f.chunk).filename() != f.chunk || !f.pose.allFinite() ||
        f.points > 10000000)
      throw std::runtime_error("invalid frame index");
    out.push_back(std::move(f));
  }
  return out;
}
Frame read_frame(const fs::path& root, const FrameInfo& info) {
  if (fs::path(info.chunk).filename() != info.chunk)
    throw std::runtime_error("invalid chunk path");
  if (info.points > 10000000)
    throw std::runtime_error("frame size exceeds reader bound");
  std::ifstream in(root / "frames" / info.chunk, std::ios::binary);
  if (!in)
    throw std::runtime_error("missing frame chunk");
  in.seekg(info.offset);
  std::vector<char> b(header_size + 16 * info.points);
  if (!in.read(b.data(), b.size()))
    throw std::runtime_error("truncated chunk");
  size_t k = 0;
  if (get<uint64_t>(b, k) != magic)
    throw std::runtime_error("invalid chunk magic");
  Frame f;
  f.id = get<uint64_t>(b, k);
  f.start = get<double>(b, k);
  f.end = get<double>(b, k);
  f.reference = get<double>(b, k);
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      f.pose(r, c) = get<double>(b, k);
  auto count = get<uint64_t>(b, k);
  size_t ho = k;
  auto expected = get<uint64_t>(b, k);
  auto sum = hash(b.data(), ho);
  sum = hash(b.data() + header_size, b.size() - header_size, sum);
  if (count != info.points || f.id != info.id || sum != expected || f.reference != info.reference ||
      f.start != info.start || f.end != info.end || f.pose != info.pose)
    throw std::runtime_error("frame/index checksum or metadata mismatch");
  f.cloud.resize(count);
  for (auto& p : f.cloud) {
    p.x = get<float>(b, k);
    p.y = get<float>(b, k);
    p.z = get<float>(b, k);
    p.intensity = get<float>(b, k);
  }
  return f;
}
size_t recover_store(const fs::path& root) {
  int lock = ::open((root / "writer.lock").c_str(), O_RDWR);
  if (lock < 0 || ::flock(lock, LOCK_EX | LOCK_NB)) {
    if (lock >= 0)
      ::close(lock);
    throw std::runtime_error("stop the writer before recovery");
  }
  try {
    std::vector<fs::path> chunks;
    for (const auto& item : fs::directory_iterator(root / "frames"))
      if (item.path().extension() == ".smf")
        chunks.push_back(item.path());
    std::sort(chunks.begin(), chunks.end());
    uint64_t id = 0, points = 0;
    std::ostringstream index;
    index << std::setprecision(17);
    for (const auto& file : chunks) {
      uint64_t offset = 0, total = fs::file_size(file);
      std::ifstream in(file, std::ios::binary);
      while (offset < total) {
        std::vector<char> b(header_size);
        in.seekg(offset);
        if (!in.read(b.data(), b.size()))
          throw std::runtime_error("truncated committed chunk; refuse to discard data");
        size_t k = 0;
        if (get<uint64_t>(b, k) != magic)
          throw std::runtime_error("bad committed chunk magic");
        FrameInfo f;
        f.id = get<uint64_t>(b, k);
        f.start = get<double>(b, k);
        f.end = get<double>(b, k);
        f.reference = get<double>(b, k);
        for (int r = 0; r < 4; ++r)
          for (int c = 0; c < 4; ++c)
            f.pose(r, c) = get<double>(b, k);
        f.points = get<uint64_t>(b, k);
        f.chunk = file.filename().string();
        f.offset = offset;
        if (f.id != id || f.points > 10000000 || header_size + 16 * f.points > total - offset)
          throw std::runtime_error("invalid committed frame during recovery");
        read_frame(root, f);
        index << f.id << ' ' << f.chunk << ' ' << f.offset << ' ' << f.points << ' ' << f.start
              << ' ' << f.end << ' ' << f.reference;
        for (int r = 0; r < 4; ++r)
          for (int c = 0; c < 4; ++c)
            index << ' ' << f.pose(r, c);
        index << '\n';
        ++id;
        points += f.points;
        offset += header_size + 16 * f.points;
      }
    }
    atomic_text(root / "index.txt", index.str());
    atomic_text(root / "recovery.json", "{\"recovered_frames\":" + std::to_string(id) +
                                            ",\"points\":" + std::to_string(points) +
                                            ",\"uncommitted_part_files_ignored\":true}\n");
    ::close(lock);
    return id;
  } catch (...) {
    ::close(lock);
    throw;
  }
}

}  // namespace superm
