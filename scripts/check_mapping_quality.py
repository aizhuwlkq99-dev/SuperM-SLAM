#!/usr/bin/env python3
"""Compare trajectories and revisit geometry using frames unused by loop verification.

This is a relative geometry diagnostic, not a substitute for ground-truth accuracy.
Requires NumPy and SciPy in the validation environment only.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def read_index(store):
    records = []
    for line in (store / 'index.txt').read_text().splitlines():
        fields = line.split()
        records.append((fields[1], int(fields[2]), int(fields[3]),
                        np.asarray(fields[7:], dtype=float).reshape(4, 4)))
    return records


def cloud(store, record):
    filename, offset, count, _ = record
    with (store / 'frames' / filename).open('rb') as stream:
        stream.seek(offset + 184)  # Explicit little-endian SMF v1 header.
        points = np.fromfile(stream, dtype='<f4', count=count * 4).reshape(-1, 4)[:, :3]
    return points[np.isfinite(points).all(axis=1) & (np.linalg.norm(points[:, :2], axis=1) < 30)]


def residuals(source, target, transform):
    placed = source @ transform[:3, :3].T + transform[:3, 3]
    forward = cKDTree(target).query(placed, workers=1)[0]
    reverse = cKDTree(placed).query(target, workers=1)[0]
    def describe(values):
        return {'capped_rmse_m': float(np.sqrt(np.mean(np.minimum(values, 1) ** 2))),
                'median_m': float(np.median(values)), 'p95_m': float(np.percentile(values, 95)),
                'overlap_0_5m': float(np.mean(values < .5))}
    return {'forward': describe(forward), 'reverse': describe(reverse)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('store', type=Path)
    parser.add_argument('graph', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--legacy-trajectory', type=Path)
    parser.add_argument('--streamed-trajectory', type=Path)
    parser.add_argument('--offset', type=int, default=8)
    args = parser.parse_args()
    if args.offset <= 5:
        parser.error('Use offset > 5 to exclude the delayed acceptance observations')
    records = read_index(args.store)
    keys = np.loadtxt(args.graph / 'keyframes.txt', ndmin=2)[:, 1].astype(int)
    poses = np.loadtxt(args.graph / 'poses.txt', ndmin=2)[:, 1:].reshape(-1, 4, 4)
    report = {'schema': 'superm-quality-v1', 'holdout_offset': args.offset,
              'absolute_ground_truth': False, 'revisits': []}
    for line in (args.graph / 'loops.jsonl').read_text().splitlines():
        loop = json.loads(line)
        if loop['event'] != 'accepted':
            continue
        old, current = (int(keys[loop[name]]) + args.offset for name in ('old', 'current'))
        if current >= len(records):
            continue
        target, source = cloud(args.store, records[old]), cloud(args.store, records[current])
        raw = np.linalg.inv(records[old][3]) @ records[current][3]
        optimized = np.linalg.inv(poses[old]) @ poses[current]
        before, after = residuals(source, target, raw), residuals(source, target, optimized)
        improved = all(after[direction]['capped_rmse_m'] <=
                       before[direction]['capped_rmse_m'] + 1e-5
                       for direction in ('forward', 'reverse'))
        report['revisits'].append({'old_frame': old, 'current_frame': current,
                                   'raw': before, 'optimized': after,
                                   'both_directions_nonworsening': improved})
    report['all_revisits_nonworsening'] = all(
        pair['both_directions_nonworsening'] for pair in report['revisits'])
    if args.legacy_trajectory and args.streamed_trajectory:
        old = np.loadtxt(args.legacy_trajectory)
        new = np.loadtxt(args.streamed_trajectory)
        _, old_ids, new_ids = np.intersect1d(old[:, 0], new[:, 0], return_indices=True)
        report['trajectory'] = {
            'legacy_frames': len(old), 'streamed_frames': len(new), 'common_frames': len(old_ids),
            'max_position_difference_m': float(np.linalg.norm(
                old[old_ids, 1:4] - new[new_ids, 1:4], axis=1).max()),
            'max_quaternion_component_difference': float(np.abs(
                old[old_ids, 4:8] - new[new_ids, 4:8]).max()),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
