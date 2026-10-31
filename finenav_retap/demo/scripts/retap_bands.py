#!/usr/bin/env python3
"""Split one or more analysed maps by height: which layer are the green voxels on?

A recorded indoor scan holds two horizontal surfaces at the same xy -- the
walking floor near z = 0 and the slab above it at z ~ 4.3 -- and a single
top-down count of "drivable voxels" adds them together.  This reads the
`*_drivable.pcd` that `retap_map_tool analyze` writes and reports, per map,
how many drivable voxels fall into each band.

Usage:
    demo/scripts/retap_bands.py /tmp/retap_exp/cmp_res020_ring09 \
        /tmp/retap_exp/cmp_res005_ring49

Bands are half-open [lo, hi).  The defaults were chosen from the z profile
(`retap_plot.py --z-profile`): everything drivable sits in [0.0, 2.0) or
[2.5, 5.0), with a gap in between.
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from retap_plot import read_pcd  # noqa: E402


def find_drivable(directory):
    hits = glob.glob(os.path.join(directory, "*_drivable.pcd"))
    if not hits:
        raise SystemExit("{}: no *_drivable.pcd -- run retap_map_tool analyze first"
                         .format(directory))
    return hits[0]


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("directories", nargs="+", help="analyze --output-dir")
    parser.add_argument("--floor", default="-0.6,2.0",
                        help="lower band lo,hi (default -0.6,2.0); "
                             "write --floor=-0.6,2.0 when negative")
    parser.add_argument("--upper", default="2.5,5.0", help="upper band lo,hi")
    args = parser.parse_args()

    floor = tuple(float(v) for v in args.floor.split(","))
    upper = tuple(float(v) for v in args.upper.split(","))
    bands = (("floor", floor), ("upper", upper))

    print("%-30s %9s %9s %9s %8s %8s" %
          ("config", "drivable", "floor", "upper", "other", "med z"))
    for directory in args.directories:
        cloud = read_pcd(find_drivable(directory))
        keep = cloud["intensity"] > 0.5
        z = cloud["z"][keep]
        counts = []
        for _, (lo, hi) in bands:
            counts.append(int(((z >= lo) & (z < hi)).sum()))
        name = os.path.basename(directory.rstrip("/"))
        median = float(np.median(z)) if len(z) else float("nan")
        print("%-30s %9d %9d %9d %8d %8.2f" %
              (name, len(z), counts[0], counts[1],
               len(z) - counts[0] - counts[1], median))
    return 0


if __name__ == "__main__":
    sys.exit(main())
