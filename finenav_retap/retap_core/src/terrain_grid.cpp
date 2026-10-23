#include "retap_core/terrain_grid.h"

namespace retap_core {

bool TerrainGrid::coordToKey(const Eigen::Vector3f &p,
                             octomap::OcTreeKey *key) const {
  return tree_->coordToKeyChecked(octomap::point3d(p.x(), p.y(), p.z()), *key);
}

Eigen::Vector3f TerrainGrid::keyToCoord(const octomap::OcTreeKey &key) const {
  octomap::point3d p = tree_->keyToCoord(key);
  return Eigen::Vector3f(p.x(), p.y(), p.z());
}

bool TerrainGrid::sample(const octomap::OcTreeKey &key,
                         VoxelSample *out) const {
  octomap::IgTreeNode *node = tree_->search(key);
  if (!node) return false;
  out->occupied = tree_->isNodeOccupied(node);
  out->collision = node->getCollision();
  out->connected = node->getConnected();
  out->traversability = node->getTraversability();
  out->slope = node->getSlope();
  out->roughness = node->getRoughness();
  out->sparsity = node->getSparsity();
  out->height_diff = node->getHeightDiff();
  out->center = keyToCoord(key);
  return true;
}

VoxelSample TerrainGrid::sample(const octomap::OcTreeKey &key) const {
  VoxelSample s;
  sample(key, &s);
  return s;
}

bool TerrainGrid::isOccupied(const octomap::OcTreeKey &key) const {
  octomap::IgTreeNode *node = tree_->search(key);
  return node && tree_->isNodeOccupied(node);
}

bool TerrainGrid::isDrivable(const octomap::OcTreeKey &key) const {
  octomap::IgTreeNode *node = tree_->search(key);
  if (!node || !tree_->isNodeOccupied(node)) return false;
  return !node->getCollision();
}

}  // namespace retap_core
