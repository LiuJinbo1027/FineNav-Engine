"""Shared parameter-file plumbing for the retap_demo launch files.

The parameter set is split across two files on purpose:

  demo/params/<scenario>.yaml   what the robot and the data source are:
                                map_path, scan_topic, scan_in_map_frame,
                                origin_offset_*, update_free, the online batch
                                policy, planner, world view, navigator,
                                localizer.
  config/<terrain>.yaml         what the terrain means: resolution, the four
                                hard thresholds, the four weights,
                                collision_neighbor, frame_height,
                                filling_radius, points_number_threshold.

config/<terrain>.yaml is also a complete standalone retap_core::TerrainConfig
(loadTerrainConfigFile() applies it over the struct defaults), so the same file
feeds `retap_map_tool analyze --config`; a calibration done offline therefore
transfers to the demo verbatim, field for field.

merge_params() writes one temporary YAML with the config's `retap_terrain:`
block merged into the params file's terrain-map node, and every node gets that
file.  No number is stored twice -- changing collision_neighbor in config/ is
the only edit needed to change it everywhere.
"""

import copy
import os
import tempfile

import yaml

# Where the terrain-map node lives inside a parameter file.
TERRAIN_PATH = ("/retap_demo/retap_terrain_map", "ros__parameters",
                "retap_terrain_map")

# The fields worth printing back, in the order the geometry reads them.
SUMMARY_KEYS = ("resolution", "collision_neighbor", "clearance_voxels",
                "support_voxels", "min_voxel_points", "update_free",
                "slope_threshold", "roughness_threshold", "sparsity_threshold",
                "traversability_threshold")


def ring_radius(ring):
    """collision_neighbor = 1 + 8*R -> R (the Chebyshev radius in voxels)."""
    ring = int(ring)
    return max(1, (ring - 1) // 8) if ring >= 9 else 1


def half_span_m(res, ring):
    """Reach of the horizontal ring: the outer edge of its last shell."""
    return (ring_radius(ring) + 0.5) * float(res)


def resolve_config(share, name):
    """Accept an absolute path, a bare file name, or a name without .yaml."""
    if not name:
        raise RuntimeError(
            "config:= is required -- it names the terrain calibration "
            "(see {}/config/README.md); the scenario parameter file does not "
            "carry resolution or the thresholds".format(share))
    candidates = [name] if os.path.isabs(name) else [
        os.path.join(share, "config", name),
        os.path.join(share, "config", name + ".yaml"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise RuntimeError(
        "config '{}' not found; looked in {}".format(name, candidates))


def read_params(params_file):
    with open(params_file, encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def read_param(params_file, *path):
    """Follow `path` into the parameter file; None when any step is missing."""
    node = read_params(params_file)
    for key in path:
        if not isinstance(node, dict) or key not in node:
            return None
        node = node[key]
    return node


def merge_params(params_file, config_path):
    """Merge config_path['retap_terrain'] over params_file's terrain node.

    Returns (merged_file_path, terrain_dict).  The temporary directory is left
    behind on purpose: its path is printed at launch time, and unlinking a file
    a node may still be reading is a race not worth having.
    """
    base = read_params(params_file)
    override = read_params(config_path)
    terrain = override.get("retap_terrain", override.get("terrain", {}))
    if not isinstance(terrain, dict):
        raise RuntimeError(
            "{}: 'retap_terrain' must be a mapping".format(config_path))

    merged = copy.deepcopy(base)
    node = merged
    for key in TERRAIN_PATH:
        node = node.setdefault(key, {})
    node.update(terrain)

    out_dir = tempfile.mkdtemp(prefix="retap_cfg_")
    out_path = os.path.join(out_dir, "merged.yaml")
    with open(out_path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(merged, handle, sort_keys=False, allow_unicode=True)
    return out_path, terrain


def summary(config_path, terrain, merged_path):
    """The one log block that says what this run actually uses.

    The vertical numbers describe the z-window the neighbour test sweeps, not a
    clearance in metres: only the voxel at exactly +clearance_voxels is a hard
    obstruction, the layers below it only have to pass the terrain constraints
    (slope / roughness / sparsity / traversability).  Whether that maps onto
    "the robot is 0.60 m tall" is a calibration question, so the window is
    reported as voxel counts next to the metres they happen to span.
    """
    res = float(terrain.get("resolution", 0.2))
    ring = int(terrain.get("collision_neighbor", 17))
    up = int(terrain.get("clearance_voxels", 3))
    down = int(terrain.get("support_voxels", 3))
    n = int(terrain.get("min_voxel_points", 0))
    text = "[retap] config {}  res={} m  collision_neighbor={} (ring R={})  ".format(
        os.path.basename(config_path), res, ring, ring_radius(ring))
    text += "horizontal half-span={:.3f} m  ".format(half_span_m(res, ring))
    text += "z-scan=-{down}..+{up} voxels ({:.2f} m down / {:.2f} m up, " \
            "hard layer at +{up})  ".format(down * res, up * res,
                                            down=down, up=up)
    text += "min_voxel_points={}".format(n if n > 0 else "0 (off)")
    text += "\n[retap] effective: " + "  ".join(
        "{}={}".format(k, terrain[k]) for k in SUMMARY_KEYS if k in terrain)
    text += "\n[retap] merged params: {}".format(merged_path)
    return text
