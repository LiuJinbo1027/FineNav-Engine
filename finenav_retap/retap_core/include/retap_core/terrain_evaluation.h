/*
 * Ported from ReTAP-Planner (HITSZ-NRSL, Apache-2.0).
 * Copyright Year: 2026
 * Copyright Owner: Networked Robotics and Systems Lab
 * Authors: Yuxiang Li, Kun Chen, Haoyao Chen
 *
 * retap_core: ROS 1 dependencies removed.  The class, member names, field
 * defaults and algorithm order are unchanged; only the following were cut:
 *   - ROS parameter reads (replaced by configure(TerrainConfig))
 *   - publishers, services, tf broadcaster, nav_msgs costmap
 *   - the terrainUpdate() queue-consumer thread (replaced by FineNav's
 *     MapRuntime post-update hook; see terrain_model.h)
 *   - the local (0.025 m) octree, which was only used by terrainInit()
 */
#pragma once

#include <pcl/point_types.h>

#include <Eigen/Geometry>
#include <pcl/point_types.h>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "retap_core/ikd-Tree/ikd_Tree_impl.h"
#include "retap_core/terrain_types.h"
#include "retap_core/utils/disjoint_set.h"
#include "retap_core/utils/octomap_utils.h"
#include "retap_core/world_representation/ig_tree.h"

namespace mapping_module {
using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;

class TerrainEvaluation {
 public:
  explicit TerrainEvaluation(std::shared_ptr<octomap::IgTree> tree);
  ~TerrainEvaluation();
  void configure(const retap_core::TerrainConfig &cfg);
  void updateMapTerrainAttribute(octomap::point3d update_position,
                                 octomap::Boundingbox update_bbox);
  void updateMapTerrainAttribute(octomap::KeySet &update_voxels,  // NOLINT
                                 octomap::point3d *update_position = nullptr);
  void updateVoxelTerrainAttribute(octomap::OcTreeKey &iter_key,  // NOLINT
                                   octomap::IgTreeNode *iter_node,
                                   octomap::KeySet *update_voxels = nullptr,
                                   octomap::point3d *update_position = nullptr,
                                   octomap::KeySet *neigh_voxels = nullptr);
  int updateVoxelTerrainCollision(octomap::OcTreeKey iter_key,
                                  octomap::IgTreeNode *iter_node);
  bool checkDrivable(octomap::OcTreeKey key);
  bool checkNodeConstraints(octomap::IgTreeNode *node, bool debug_msg = false);
  bool checkNeighborCells(octomap::OcTreeKey key, bool debug_msg = false);
  void updateDrivableDisjointSet(
      std::vector<octomap::OcTreeKey> &update_voxels,    // NOLINT
      std::vector<octomap::IgTreeNode *> &update_nodes,  // NOLINT
      std::vector<octomap::OcTreeKey> &revoke_voxels,    // NOLINT
      std::vector<octomap::IgTreeNode *> &revoke_nodes,  // NOLINT
      octomap::point3d update_position);

  // retap_core: body of the old query_nearest_drivable_point service callback
  // with the ROS request/response replaced by plain arguments.
  bool queryNearestDrivable(const octomap::point3d &query_point,
                            octomap::point3d *result_point,
                            bool *in_disjoint_set = nullptr);

  // retap_core: readiness of the nearest-drivable index (disjoint set root +
  // ikd-tree).  queryNearestDrivable() needs both; a caller that rebuilds the
  // index after loading a .bt uses this to skip a second build per call.
  bool hasDrivableIndex() const {
    return kdtree_initialized_ && drivable_disjoint_root_valid_;
  }

  // retap_core: geometry of the neighbour test, for logging and validation.
  // Radius of the horizontal ring in voxels; the ring reaches (R+0.5)*res.
  int neighborRadius() const {
    const int ring = static_cast<int>(collision_neighbor_);
    return ring >= 9 ? (ring - 1) / 8 : 1;
  }
  int clearanceVoxels() const { return clearance_voxels_; }
  int supportVoxels() const { return support_voxels_; }
  int minVoxelPoints() const { return min_voxel_points_; }

  void resetTerrainRepresentation(std::shared_ptr<octomap::IgTree> tree);
  void terrainInit(Eigen::Isometry3d &current_pose);

  std::shared_ptr<octomap::IgTree> octree() const { return octree_ptr_; }

 private:
  float roughness_threshold_ = 1;
  float sparsity_threshold_ = 0.2;
  float slope_threshold_ = 0.785;
  float roughness_weight_ = 1.391;
  float sparsity_weight_ = 1;
  float slope_weight_ = 0.686;
  float traversability_weight_ = 2.3;
  int points_number_threshold_ = 20;
  float traversability_threshold_ = 0.7;
  float neighbor_radius_;
  std::shared_ptr<octomap::IgTree> octree_ptr_;
  std::string map_frame_;  // NOLINT
  float collision_neighbor_ = 17;
  // retap_core: upstream hard-coded the vertical scan to z in [-3, +3] and the
  // horizontal ring to the 25-entry index_x/index_y tables.  Both are
  // parameters now; see TerrainConfig for the semantics.
  int clearance_voxels_ = 3;
  int support_voxels_ = 3;
  int min_voxel_points_ = 0;
  // Offsets of the neighbour test in voxel units: [0] = the voxel itself, then
  // 8 per Chebyshev shell.  Rebuilt by rebuildNeighborOffsets().
  std::vector<int> neighbor_dx_;
  std::vector<int> neighbor_dy_;
  float frame_height_ = 0.5;
  float filling_radius_ = 0.5;
  std::shared_ptr<DisjointSet<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash>>
      drivable_disjoint_set_;
  bool drivable_disjoint_root_valid_ = false;
  bool kdtree_initialized_ = false;

  octomap::OcTreeKey drivable_disjoint_root_;
  KD_TREE<PointType>::Ptr kdtree_ptr_;

  // retap_core: builds neighbor_dx_/neighbor_dy_ from collision_neighbor_.
  void rebuildNeighborOffsets();
  // retap_core: occupied AND has at least min_voxel_points_ points (N=0 means
  // the point count is not consulted).  Applies to checkNeighborCells only.
  bool isVoxelOccupied(octomap::IgTreeNode *node) const;

  std::mutex kdtree_mutex_;
};
}  // namespace mapping_module
