#include <iostream>
#include <sstream>

#include "super_lio_mapping/store.hpp"
int main(int argc, char** argv) {
  try {
    if (argc == 3 && std::string(argv[1]) == "recover") {
      std::cout << "recovered_frames=" << superm::recover_store(argv[2]) << '\n';
      return 0;
    }
    if (argc < 3) {
      std::cerr << "Usage: superm_export STORE OUTPUT [--leaf m] [--tile m] [--memory-mib N] "
                   "[--pcd] [--poses file] [--roi xmin ymin zmin xmax ymax zmax]\n";
      return 2;
    }
    superm::ExportOptions opt;
    for (int i = 3; i < argc; ++i) {
      std::string a = argv[i];
      auto value = [&]() {
        if (++i >= argc)
          throw std::invalid_argument("missing option value");
        return std::string(argv[i]);
      };
      if (a == "--leaf")
        opt.leaf = std::stod(value());
      else if (a == "--tile")
        opt.tile_size = std::stod(value());
      else if (a == "--memory-mib")
        opt.workspace_bytes = size_t(std::stoull(value())) * 1024 * 1024;
      else if (a == "--pcd")
        opt.single_pcd = true;
      else if (a == "--poses") {
        std::ifstream in(value());
        if (!in)
          throw std::runtime_error("cannot open pose snapshot");
        uint64_t id;
        while (in >> id) {
          if (id != opt.poses.size())
            throw std::runtime_error("noncontiguous snapshot");
          Eigen::Matrix4d T;
          for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
              if (!(in >> T(r, c)))
                throw std::runtime_error("truncated snapshot");
          opt.poses.push_back(T);
        }
        opt.graph_version = "optimized";
      } else if (a == "--roi") {
        opt.roi = true;
        for (int j = 0; j < 3; ++j)
          opt.minimum[j] = std::stod(value());
        for (int j = 0; j < 3; ++j)
          opt.maximum[j] = std::stod(value());
        if ((opt.minimum.array() >= opt.maximum.array()).any())
          throw std::invalid_argument("invalid ROI");
      } else
        throw std::invalid_argument("unknown option: " + a);
    }
    auto r = superm::export_map(argv[1], argv[2], opt);
    std::cout << "{\"frames\":" << r.frames << ",\"input_points\":" << r.input_points
              << ",\"points\":" << r.points << ",\"tiles\":" << r.tiles << "}\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
