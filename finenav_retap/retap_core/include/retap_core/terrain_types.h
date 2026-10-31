/*
 * Ported from the member defaults of mapping_module::TerrainEvaluation
 * (ReTAP-Planner, HITSZ-NRSL, Apache-2.0).
 *
 * This struct is the single semantic source of the terrain parameters:
 * the ROS 2 parameter files and the FineNav plugin both fill it in.
 */
#pragma once

namespace retap_core {

struct TerrainConfig {
  float roughness_threshold = 1;
  float sparsity_threshold = 0.2;
  float slope_threshold = 0.785;
  float roughness_weight = 1.391;
  float sparsity_weight = 1;
  float slope_weight = 0.686;
  float traversability_weight = 2.3;
  int points_number_threshold = 20;
  float traversability_threshold = 0.7;
  // Horizontal ring of the neighbour test: 1 + 8 * R offsets, i.e. a
  // (2R+1) x (2R+1) block centred on the voxel.  9 = 3x3 (R=1), 17 = 5x5
  // (R=2, upstream default), 25 = 7x7 (R=3), 49 = 13x13 (R=6, what a 0.60 m
  // footprint needs at 0.05 m).  See TerrainEvaluation::checkNeighborCells().
  float collision_neighbor = 17;
  // Vertical scan of the neighbour test: z = +1 .. +clearance_voxels upward and
  // z = 0 .. -support_voxels downward.
  //   z == +clearance_voxels  : occupied => hard obstruction (净空)
  //   z == 1 .. clearance-1   : occupied => must pass checkNodeConstraints
  //   z == -2 / -3            : must be occupied (support / cliff test)
  // 3 / 3 reproduces upstream exactly.  At 0.2 m a 60 cm robot needs 4.
  int clearance_voxels = 3;
  int support_voxels = 3;
  // Evidence threshold N: a voxel counts as occupied for the neighbour test
  // only once it holds at least N points.  0 = disabled (any point occupies,
  // upstream semantics).
  int min_voxel_points = 0;
  float frame_height = 0.5;
  float filling_radius = 0.5;

  // ── static / online split (retap_core only; upstream had no switch) ──
  // Static PCD: update_free=0 and unknown space is treated as free, which is
  // what the 2.5D column model expects.
  // Online lidar: update_free=2, matching WorldRepresentation's global map.
  int update_free = 0;
  double terrain_update_period_sec = 0.5;  // upstream terrainUpdate(): 0.5 s
  int terrain_update_batch = 3;            // upstream: list_size >= 3
};

}  // namespace retap_core
