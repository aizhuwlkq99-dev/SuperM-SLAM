#!/usr/bin/env python3
"""Create short NCLT fixtures with bounded delivery reordering and a clock reset.

Requires this workspace's Humble environment. Original sensor data is read-only.
"""
import argparse
from pathlib import Path
import struct

import rosbag2_py


def write_bag(path, rows):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for name, kind in [('points_raw', 'sensor_msgs/msg/PointCloud2'),
                       ('imu_raw', 'sensor_msgs/msg/Imu')]:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=kind, serialization_format='cdr'))
    for stamp, topic, data in sorted(rows, key=lambda row: row[0]):
        writer.write(topic, data, stamp)
    del writer  # Flush metadata before replay.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    rows, base = [], None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if base is None:
            base = stamp
        if stamp - base > 40_000_000_000:
            break
        if topic in ('points_raw', 'imu_raw'):
            rows.append((stamp, topic, data))
    if not rows:
        parser.error('No converted NCLT sensor messages found')
    reordered, reset, imu_index, reset_done = [], [], 0, False
    for stamp, topic, data in rows:
        if topic == 'imu_raw':
            imu_index += 1
        delay = 80_000_000 if topic == 'imu_raw' and imu_index % 37 == 0 else 0
        reordered.append((stamp + delay, topic, data))
        if stamp - base <= 3_000_000_000:
            if topic == 'imu_raw' and stamp - base >= 2_000_000_000 and not reset_done:
                # One sensor stamp moves back by a full second; arrival order stays intact.
                assert data[:4] == b'\x00\x01\x00\x00', 'Expected little-endian CDR'
                data = bytearray(data)
                seconds, nanos = struct.unpack_from('<iI', data, 4)
                struct.pack_into('<iI', data, 4, seconds - 1, nanos)
                data, reset_done = bytes(data), True
            reset.append((stamp, topic, data))
    assert reset_done
    write_bag(output / 'reordered', reordered)
    write_bag(output / 'clock_reset', reset)
    print(f'Created {imu_index // 37} delayed IMUs and one clock reset under {output}')


if __name__ == '__main__':
    main()
