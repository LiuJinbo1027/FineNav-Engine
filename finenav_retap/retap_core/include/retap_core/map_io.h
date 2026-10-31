/*
 * retap_core: file I/O used by the ROS 2 tools and by the FineNav Map plugin.
 * New code (not ported); the ReTAP originals live in MappingModule /
 * WorldRepresentation and depend on ROS.
 */
#pragma once

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

#include "retap_core/terrain_model.h"
#include "retap_core/world_representation/ig_tree.h"

namespace retap_core {

enum class VoxelField {
  kTraversability,
  kSlope,
  kRoughness,
  kSparsity,
  kCollision,
  kDrivable,   // 1 - collision: passes checkNodeConstraints and checkNeighborCells
  kConnected,
};

struct TerrainStats {
  size_t occupied_voxels = 0;
  size_t drivable_voxels = 0;
  size_t connected_voxels = 0;
  float slope_min = 0, slope_mean = 0, slope_max = 0;
  float roughness_min = 0, roughness_mean = 0, roughness_max = 0;
  float sparsity_min = 0, sparsity_mean = 0, sparsity_max = 0;
  float traversability_min = 0, traversability_mean = 0,
        traversability_max = 0;
};

bool loadPcd(const std::string &path, std::vector<Eigen::Vector3f> *points,
             std::string *error);

bool saveTerrainBt(octomap::IgTree &tree, const std::string &path,
                   std::string *error);
std::unique_ptr<octomap::IgTree> loadTerrainBt(const std::string &path,
                                               std::string *error);

// Writes one PCD whose intensity is the requested field.  Rows with unknown
// nodes are skipped.
bool writeVoxelCloud(const octomap::IgTree &tree, const std::string &path,
                     VoxelField field, std::string *error);

TerrainStats computeStats(octomap::IgTree &tree);

struct TerrainBuildResult {
  size_t input_points = 0;
  size_t occupied_voxels = 0;
  size_t drivable_voxels = 0;
  size_t connected_voxels = 0;
  double load_ms = 0;
  double insert_ms = 0;
  double analysis_ms = 0;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();  // connectivity root seed
};

// PCD -> IgTree -> terrain attributes, the offline equivalent of
// MappingModule::loopProcessMaps().  This is the path S1 (retap_map_tool) and
// S3 (RetapTerrainMap) both use; LoadOctomapFromPCD from upstream is not used
// because it never calls IgTreeNode::insertPoint() and therefore has no NDT
// statistics to analyse.
//
// `center` is the position used to pick the connectivity root (the robot pose
// in ReTAP).  When null the centroid of the input points is used.
std::unique_ptr<TerrainModel> buildTerrainFromPcd(
    const std::string &pcd_path, double resolution, const TerrainConfig &cfg,
    TerrainBuildResult *result, std::string *error,
    const Eigen::Vector3f *center = nullptr);

// Nearest occupied, collision-free voxel to `query` within `radius` (metres).
// Used to snap A* start/goal, the role of ReTAP's
// query_nearest_drivable_point service.  Unlike
// TerrainEvaluation::queryNearestDrivable() this needs no disjoint-set index,
// so it also works on a tree loaded from a .bt file.
bool nearestDrivableVoxel(octomap::IgTree &tree, const Eigen::Vector3f &query,
                          float radius, Eigen::Vector3f *result);

}  // namespace retap_core
