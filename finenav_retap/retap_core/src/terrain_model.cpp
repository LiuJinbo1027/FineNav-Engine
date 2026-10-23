#include "retap_core/terrain_model.h"

#include <algorithm>
#include <limits>

namespace retap_core {

TerrainModel::TerrainModel(double resolution, const TerrainConfig &cfg)
    : resolution_(resolution), cfg_(cfg) {
  tree_.reset(new octomap::IgTree(resolution_));
  evaluator_.reset(
      new mapping_module::TerrainEvaluation(tree_));
  evaluator_->configure(cfg_);
}

TerrainModel::TerrainModel(std::shared_ptr<octomap::IgTree> tree,
                           const TerrainConfig &cfg)
    : cfg_(cfg), tree_(std::move(tree)) {
  if (tree_ == nullptr) tree_.reset(new octomap::IgTree(0.2));
  resolution_ = tree_->getResolution();
  evaluator_.reset(new mapping_module::TerrainEvaluation(tree_));
  evaluator_->configure(cfg_);
}

octomap::Boundingbox TerrainModel::boundingBox(
    const std::vector<Eigen::Vector3f> &points,
    const Eigen::Vector3f &origin) const {
  octomap::Boundingbox bbx;
  if (points.empty()) return bbx;
  bbx.reset();
  for (const auto &p : points) {
    bbx.insertPoint(octomap::point3d(p.x(), p.y(), p.z()));
  }
  // Include the sensor origin: ray casting starts there and a bbox that
  // excludes it would silently drop the first free cells.
  bbx.insertPoint(octomap::point3d(origin.x(), origin.y(), origin.z()));
  bbx.enlarge(static_cast<float>(resolution_));
  return bbx;
}

ScanResult TerrainModel::insertCloud(
    const std::vector<Eigen::Vector3f> &points, const Eigen::Vector3f &origin,
    const ScanOptions &opts) {
  octomap::Pointcloud pc;
  pc.reserve(points.size());
  for (const auto &p : points) {
    pc.push_back(octomap::point3d(p.x(), p.y(), p.z()));
  }
  return insertCloud(pc, octomap::point3d(origin.x(), origin.y(), origin.z()),
                     opts);
}

ScanResult TerrainModel::insertCloud(const octomap::Pointcloud &pc,
                                     const octomap::point3d &origin,
                                     const ScanOptions &opts) {
  ScanResult result;
  result.point_count = pc.size();
  if (pc.size() == 0) return result;

  std::vector<Eigen::Vector3f> points;
  points.reserve(pc.size());
  for (size_t i = 0; i < pc.size(); ++i) {
    const auto &p = pc.getPoint(i);
    points.emplace_back(p.x(), p.y(), p.z());
  }
  octomap::Boundingbox bbx =
      boundingBox(points, Eigen::Vector3f(origin.x(), origin.y(), origin.z()));

  auto inserted = tree_->insertPointCloud(pc, origin, bbx, opts.update_free,
                                          opts.max_range);
  result.occupied_keys.swap(inserted.first);
  return result;
}

ScanResult TerrainModel::insertScan(const std::vector<Eigen::Vector3f> &points,
                                    const Eigen::Vector3f &origin,
                                    const ScanOptions &opts) {
  return insertCloud(points, origin, opts);
}

void TerrainModel::evaluateAll(const Eigen::Vector3f &center) {
  octomap::KeySet occupied;
  for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
    if (!tree_->isNodeOccupied(&*it)) continue;
    occupied.insert(it.getKey());
  }
  if (occupied.empty()) return;
  octomap::point3d c(center.x(), center.y(), center.z());
  evaluator_->updateMapTerrainAttribute(occupied, &c);
  ++generation_;
}

void TerrainModel::evaluateDirty(const octomap::KeySet &dirty_keys,
                                 const Eigen::Vector3f &center) {
  if (dirty_keys.empty()) return;
  octomap::KeySet dirty = dirty_keys;  // upstream API takes a non-const ref
  octomap::point3d c(center.x(), center.y(), center.z());
  evaluator_->updateMapTerrainAttribute(dirty, &c);
  ++generation_;
}

bool TerrainModel::queryNearestDrivable(const octomap::point3d &query,
                                        octomap::point3d *result,
                                        bool *in_disjoint_set) {
  return evaluator_->queryNearestDrivable(query, result, in_disjoint_set);
}

size_t TerrainModel::occupiedVoxelCount() const {
  size_t n = 0;
  for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
    if (tree_->isNodeOccupied(&*it)) ++n;
  }
  return n;
}

size_t TerrainModel::drivableVoxelCount() const {
  size_t n = 0;
  for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
    if (!tree_->isNodeOccupied(&*it)) continue;
    if (!it->getCollision()) ++n;
  }
  return n;
}

}  // namespace retap_core
