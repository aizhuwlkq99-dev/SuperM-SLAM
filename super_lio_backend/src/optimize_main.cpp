#include <iostream>

#include "super_lio_backend/backend.hpp"
int main(int argc, char** argv) {
  try {
    if (argc < 3) {
      std::cerr
          << "Usage: superm_optimize STORE GRAPH_OUTPUT [--no-loops] [--no-global] [--context N]\n";
      return 2;
    }
    superm::BackendOptions options;
    for (int i = 3; i < argc; ++i) {
      std::string a = argv[i];
      if (a == "--no-loops")
        options.loop_closure = false;
      else if (a == "--no-global")
        options.global_registration = false;
      else if (a == "--context" && i + 1 < argc)
        options.context_frames = std::stoul(argv[++i]);
      else
        throw std::invalid_argument("unknown option");
    }
    superm::Backend graph(argv[1], argv[2], options);
    graph.poll();
    graph.save();
    std::cout << "frames=" << graph.frames().size() << " keyframes=" << graph.keyframes()
              << " loops=" << graph.loops() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
