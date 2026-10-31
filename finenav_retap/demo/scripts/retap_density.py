#!/usr/bin/env python3
"""Point counts per voxel, split by height, straight from the accumulated cloud.

`retap_map_tool info --histogram` reports the point-count distribution over the
whole map.  The map holds two horizontal surfaces (see `retap_bands.py`), so
the aggregate mixes them; this voxelizes the raw cloud at each resolution and
splits the same distribution by the z band of each voxel's centre.

Counts are capped at 30 to match `IgTree::insertPoint()`, which stops counting
past 30 points per voxel.  The `ALL` rows therefore reproduce
`info --histogram` and are the check that the split is trustworthy.

The second table counts how many occupied z-layers each xy column has in the
band.  The downward support test asks for an occupied voxel 2 and 3 layers
below, so a one-voxel-thick surface is judged a cliff regardless of how many
points it holds; the layer count is what that test sees.

Usage:
    demo/scripts/retap_density.py /tmp/retap_exp/data/bag_accumulated.pcd
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from retap_plot import read_pcd  # noqa: E402


def pct(mask, count):
    return 100.0 * mask.sum() / count if count else 0.0


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("pcd", help="accumulated cloud in map frame")
    parser.add_argument("--resolutions", default="0.20,0.10,0.05")
    parser.add_argument("--floor", default="-0.6,2.0",
                        help="write --floor=-0.6,2.0 when negative")
    parser.add_argument("--upper", default="2.5,5.0")
    args = parser.parse_args()

    floor = tuple(float(v) for v in args.floor.split(","))
    upper = tuple(float(v) for v in args.upper.split(","))
    bands = (("floor", floor), ("upper", upper), ("ALL", None))

    cloud = read_pcd(args.pcd)
    points = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1).astype(np.float64)

    print("points per voxel (capped at 30)")
    print("%-6s %-6s %9s %9s %7s %7s %7s %7s" %
          ("res", "band", "voxels", "mean", "=1", "<=5", "<=19", ">19"))
    for res in (float(v) for v in args.resolutions.split(",")):
        keys = np.floor(points / res).astype(np.int64)
        _, inverse, counts = np.unique(keys, axis=0, return_inverse=True,
                                       return_counts=True)
        capped = np.minimum(counts, 30)
        # z of each voxel's centre, looked up through the point -> voxel map
        voxel_z = np.empty(len(counts))
        voxel_z[inverse] = keys[:, 2] * res + res / 2.0
        for label, band in bands:
            if band is None:
                mask = np.ones(len(counts), dtype=bool)
            else:
                mask = (voxel_z >= band[0]) & (voxel_z < band[1])
            c = capped[mask]
            if not len(c):
                continue
            print("%-6.2f %-6s %9d %9.2f %6.1f%% %6.1f%% %6.1f%% %6.1f%%" %
                  (res, label, len(c), c.mean(), pct(c == 1, len(c)),
                   pct(c <= 5, len(c)), pct(c <= 19, len(c)),
                   pct(c > 19, len(c))))

    print()
    print("occupied z-layers per xy column")
    print("%-6s %-6s %9s %8s %7s %7s %7s" %
          ("res", "band", "columns", "median", ">=2", ">=3", ">=4"))
    for res in (float(v) for v in args.resolutions.split(",")):
        keys = np.floor(points / res).astype(np.int64)
        z_center = keys[:, 2] * res + res / 2.0
        for label, (lo, hi) in bands[:2]:
            mask = (z_center >= lo) & (z_center < hi)
            if not mask.any():
                continue
            column_z = np.unique(keys[mask][:, :3], axis=0)[:, :2]
            _, column_counts = np.unique(column_z, axis=0, return_counts=True)
            print("%-6.2f %-6s %9d %8.1f %6.1f%% %6.1f%% %6.1f%%" %
                  (res, label, len(column_counts), np.median(column_counts),
                   pct(column_counts >= 2, len(column_counts)),
                   pct(column_counts >= 3, len(column_counts)),
                   pct(column_counts >= 4, len(column_counts))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
