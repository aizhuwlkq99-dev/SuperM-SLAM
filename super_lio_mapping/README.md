# Bounded map storage

The library stores every submitted static body-frame cloud with its paired raw
pose and actual deskew reference time. Queue accounting uses allocated PCL point
storage; the in-flight serialization buffer is additional bounded working memory.

SMF v1 uses little-endian scalar fields, a fixed frame header, packed XYZI float32
payloads, and a checksum over metadata and points. Chunk files are synced and
renamed before their index rows become durable. A writer lock prevents recovery
while a writer is active. Recovery rebuilds the index from complete chunks and
refuses checksum failures; uncommitted `.part` files are preserved.

`superm_export` creates a new version directory. It uses bounded sorted runs and
16-way external merges, preserving counts when combining voxel centroids. Tiles
contain **local** coordinates; add each tile's `origin` from `manifest.json` to
obtain map coordinates. `--pcd` additionally writes a world-coordinate PCD for
small maps. Existing output directories are never overwritten.

```bash
ros2 run super_lio_mapping superm_export STORE NEW_OUTPUT --leaf 0.1 --memory-mib 256
ros2 run super_lio_mapping superm_export recover STORE
ros2 run super_lio_mapping mapping_check outputs/unique_storage_test
```

The check covers lossless frame reads, bounded external merging against the
previous voxel filter, negative coordinates, weighted attributes, index recovery,
and corrupted payload rejection. Test paths must be new directories.
