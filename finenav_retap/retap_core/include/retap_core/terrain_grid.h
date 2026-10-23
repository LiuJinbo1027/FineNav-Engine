/*
 * retap_core: read-only facade over IgTree.  New code (not ported); the
 * Planner and WorldView only depend on this type, never on the Map plugin.
 */
#pragma once

#include <Eigen/Core>

#include "retap_core/world_representation/ig_tree.h"

namespace retap_core {

struct VoxelSample {
  bool occupied = false;
  bool collision = true;  // same default as IgTreeNode (collision_ == 0)
  bool connected = false;
  float traversability = 0.f;
  float slope = 0.f;
  float roughness = 0.f;
  float sparsity = 0.f;
  float height_diff = 0.f;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
};

class TerrainGrid {
 public:
  explicit TerrainGrid(octomap::IgTree &tree) : tree_(&tree) {}

  double resolution() const { return tree_->getResolution(); }

  bool coordToKey(const Eigen::Vector3f &p, octomap::OcTreeKey *key) const;
  Eigen::Vector3f keyToCoord(const octomap::OcTreeKey &key) const;

  bool sample(const octomap::OcTreeKey &key, VoxelSample *out) const;
  VoxelSample sample(const octomap::OcTreeKey &key) const;
  bool isOccupied(const octomap::OcTreeKey &key) const;
  bool isDrivable(const octomap::OcTreeKey &key) const;

  octomap::IgTree &tree() { return *tree_; }
  const octomap::IgTree &tree() const { return *tree_; }

 private:
  octomap::IgTree *tree_ = nullptr;
};

}  // namespace retap_core
