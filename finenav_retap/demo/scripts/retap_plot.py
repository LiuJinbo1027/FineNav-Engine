#!/usr/bin/env python3
"""Top-down picture of one or more terrain analyses.

The table in config/README.md measures the terrain with text only; this turns
the same artifacts into a picture so "where did the drivable area go" is
visible instead of argued about.  It reads what `retap_map_tool analyze`
already wrote (no ROS, no bag replay):

    <dir>/<name>_drivable.pcd    intensity 1 = collision == false
    <dir>/<name>_collision.pcd   intensity 1 = collision
    traj.txt                     `x y z` per pose (bag2pcd.py output)

Usage:
    demo/scripts/retap_plot.py --input-dir /tmp/retap_out --trajectory traj.txt \\
        --out /tmp/retap_out.png --title "res 0.20 ring 9"

    # several runs side by side, same extent:
    demo/scripts/retap_plot.py \\
        --input-dir /tmp/o_020 --label "0.20 m" \\
        --input-dir /tmp/o_005 --label "0.05 m" \\
        --trajectory traj.txt --out /tmp/compare.png

Only the xy projection is drawn: the question this answers is coverage on the
floor plane, not height.  A recorded indoor scan holds two horizontal surfaces
at the same xy -- the walking floor at z ~ 0 and the slab above it at z ~ 4.3
-- so a single top-down picture superimposes them and you cannot tell which
layer a green voxel belongs to.  `--z-min/--z-max` keep one horizontal slice;
`--z-profile` draws the drivable count vs z instead of a map, which is how the
slice boundaries were chosen.  `--extent x0,x1,y0,y1` pins the axes so panels
are comparable; without it every panel uses its own data bounds.  Write both
it and a negative `--z-min` with an equals sign (`--extent=-20,20,-6,30`,
`--z-min=-0.6`), or argparse reads the value as the next option.
"""
import argparse
import glob
import os
import struct
import sys

import numpy as np


def read_pcd(path):
    """Minimal binary PCD reader for FIELDS/TYPE F/SIZE 4 records."""
    with open(path, "rb") as handle:
        header = {}
        while True:
            line = handle.readline().decode("ascii", "replace").strip()
            if not line or line.startswith("#"):
                continue
            key, _, value = line.partition(" ")
            header[key] = value
            if key == "DATA":
                break
        if header.get("DATA") != "binary":
            raise RuntimeError("{}: only binary PCD is supported".format(path))
        fields = header["FIELDS"].split()
        counts = [int(c) for c in header.get("COUNT", " ".join("1" * len(fields))).split()]
        sizes = [int(s) for s in header["SIZE"].split()]
        types = header["TYPE"].split()
        if any(t != "F" or s != 4 for t, s in zip(types, sizes)):
            raise RuntimeError("{}: only 32-bit floats are supported".format(path))
        n = int(header["POINTS"])
        record = sum(c * 4 for c in counts)
        data = np.frombuffer(handle.read(n * record), dtype=np.uint8)
        if data.size != n * record:
            raise RuntimeError("{}: truncated ({} of {} points)".format(
                path, data.size // record, n))
        # PCD binary is ROW-major: x0 y0 z0 i0 x1 y1 ... -- one record per
        # point, so the fields are columns of an (n, floats-per-record) table.
        # (Reading each field as one contiguous block instead silently yields
        # x,y,z,intensity interleaved and draws a plausible-looking, wrong map.)
        table = data.view("<f4").reshape(n, record // 4)
        columns = {}
        offset = 0
        for name, count in zip(fields, counts):
            columns[name] = (table[:, offset] if count == 1
                             else table[:, offset:offset + count])
            offset += count
        return columns


def find(directory, suffix):
    hits = glob.glob(os.path.join(directory, "*" + suffix))
    if not hits:
        raise RuntimeError("{}: no *{}".format(directory, suffix))
    return hits[0]


def load_trajectory(path):
    if not path:
        return None
    rows = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.split()
            if len(parts) >= 2:
                rows.append((float(parts[0]), float(parts[1])))
    return np.array(rows) if rows else None


def draw(ax, directory, trajectory, title, z_min=None, z_max=None):
    collision = read_pcd(find(directory, "_collision.pcd"))
    drivable = read_pcd(find(directory, "_drivable.pcd"))

    def pick(cloud):
        """Return (x, y, mask) for the voxels this panel keeps.

        A horizontal slice is a filter, not a new map: the same PCD holds both
        the walking floor (z ~ 0) and the slab above it (z ~ 4.3), and a plain
        top-down projection of both is unreadable because they overlap.
        """
        mask = cloud["intensity"] > 0.5
        if z_min is not None:
            mask = mask & (cloud["z"] >= z_min)
        if z_max is not None:
            mask = mask & (cloud["z"] <= z_max)
        return cloud["x"], cloud["y"], mask

    drivable_xy = pick(drivable)
    collision_xy = pick(collision)

    # Collision first and faint, drivable on top: every occupied voxel is one
    # or the other, and the collision set is ~10x larger, so drawing it last
    # would bury the answer under a solid red field.
    for (xs, ys, mask), colour, label, alpha, size in (
            (collision_xy, "#d73027", "collision", 0.18, 0.5),
            (drivable_xy, "#1a9850", "drivable", 0.9, 1.0)):
        ax.scatter(xs[mask], ys[mask], s=size, c=colour, linewidths=0,
                   alpha=alpha,
                   label="{} ({} voxels)".format(label, int(mask.sum())),
                   rasterized=True)
    if trajectory is not None:
        ax.plot(trajectory[:, 0], trajectory[:, 1], "-", color="#333333",
                linewidth=0.8, label="robot path (z ~ 0)")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x [m]", fontsize=8)
    ax.set_ylabel("y [m]", fontsize=8)
    ax.tick_params(labelsize=7)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(fontsize=7, markerscale=8, loc="upper right")


def draw_z_profile(ax, directory, label, bin_width):
    """Drivable voxel count vs z -- where the two layers actually are.

    The log scale is not decoration: the slab at z ~ 4.3 holds ~10x more
    drivable voxels than the walking floor, and a linear axis hides the
    floor band entirely.
    """
    drivable = read_pcd(find(directory, "_drivable.pcd"))
    mask = drivable["intensity"] > 0.5
    z = drivable["z"][mask]
    lo = np.floor(z.min() / bin_width) * bin_width
    hi = np.ceil(z.max() / bin_width) * bin_width
    edges = np.arange(lo, hi + bin_width, bin_width)
    counts, edges = np.histogram(z, edges)
    centers = 0.5 * (edges[:-1] + edges[1:])
    ax.plot(centers, counts, drawstyle="steps-mid", linewidth=1.3,
            label="{} ({} drivable voxels)".format(label, len(z)))
    ax.set_yscale("log")
    ax.set_xlabel("z [m]", fontsize=9)
    ax.set_ylabel("drivable voxels per {:.2f} m".format(bin_width), fontsize=9)
    ax.tick_params(labelsize=7)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input-dir", action="append", required=True,
                        help="analyze --output-dir (repeat for a grid)")
    parser.add_argument("--label", action="append", default=[],
                        help="panel title (repeat, aligned with --input-dir)")
    parser.add_argument("--trajectory", default="", help="traj.txt from bag2pcd.py")
    parser.add_argument("--out", required=True, help="output PNG")
    parser.add_argument("--title", default="", help="overall title")
    parser.add_argument("--extent", default="",
                        help="x0,x1,y0,y1 to pin all panels")
    parser.add_argument("--z-min", type=float, default=None,
                        help="keep voxels with z >= this (horizontal slice); "
                             "write --z-min=-0.6 when negative")
    parser.add_argument("--z-max", type=float, default=None,
                        help="keep voxels with z <= this")
    parser.add_argument("--z-profile", action="store_true",
                        help="instead of a map, draw drivable voxels vs z "
                             "(log scale) -- shows where the layers are")
    parser.add_argument("--z-bin", type=float, default=0.2,
                        help="z histogram bin width for --z-profile")
    args = parser.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = list(args.label) + [""] * (len(args.input_dir) - len(args.label))

    if args.z_profile:
        fig, ax = plt.subplots(figsize=(8.0, 5.0))
        for index, directory in enumerate(args.input_dir):
            draw_z_profile(ax, directory,
                           labels[index] or os.path.basename(
                               directory.rstrip("/")),
                           args.z_bin)
        if args.title:
            ax.set_title(args.title, fontsize=11)
        fig.tight_layout()
        fig.savefig(args.out, dpi=140)
        print(args.out)
        return 0

    trajectory = load_trajectory(args.trajectory)
    panels = max(1, len(args.input_dir))
    cols = min(2, panels)
    rows = (panels + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(6.2 * cols, 5.6 * rows),
                             squeeze=False)

    band = ""
    if args.z_min is not None or args.z_max is not None:
        band = " [z {}..{} m]".format(
            "-inf" if args.z_min is None else "{:.1f}".format(args.z_min),
            "+inf" if args.z_max is None else "{:.1f}".format(args.z_max))

    for index, directory in enumerate(args.input_dir):
        ax = axes[index // cols][index % cols]
        draw(ax, directory, trajectory,
             (labels[index] or os.path.basename(directory.rstrip("/"))) + band,
             z_min=args.z_min, z_max=args.z_max)
        if args.extent:
            x0, x1, y0, y1 = (float(v) for v in args.extent.split(","))
            ax.set_xlim(x0, x1)
            ax.set_ylim(y0, y1)
    for index in range(panels, rows * cols):
        axes[index // cols][index % cols].axis("off")

    if args.title:
        fig.suptitle(args.title, fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    print(args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
