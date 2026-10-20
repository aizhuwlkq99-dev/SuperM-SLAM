# Isolated backend dependencies

`versions.json` locks full revisions and archive SHA-256 checksums for GTSAM,
small_gicp, KISS-Matcher, ROBIN, xenium, and PMC. Source archives, build trees,
and installed binaries are ignored by Git. Upstream license texts are retained
under `licenses/`.

Inside `vslam`, run:

```bash
./scripts/build_backend_deps.sh
```

The script uses system Eigen, PCL dependencies, TBB, OpenMP, FLANN, and LZ4.
It installs only into this directory's `install/` prefix. GTSAM's local RPATH
allows its METIS dependency to resolve without another workspace's environment.
ROBIN's nested downloads are redirected to the checksummed local PMC/xenium
sources; no floating xenium branch is used. ROS 1 and GPU dependencies are not
required. Configure/build/install output is stored in `log/backend/`.

To change a version, update its revision and archive checksum together and
rerun compatibility and graph tests. A source marker allows verified cached
archives to be reused; delete that source directory to force a fresh fetch.
