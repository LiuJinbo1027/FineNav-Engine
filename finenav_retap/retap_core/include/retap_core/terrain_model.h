/*
 * retap_core: glue between the ported IgTree / TerrainEvaluation and the
 * rest of the system.  New code (not ported).
 *
 * Static path (S0-S4): insertCloud() with update_free = 0, then evaluateAll().
 * Online path (S5):     insertScan() returns the occupied keys of one scan,
 *                       evaluateDirty() re-runs the same maths on just those
 *                       keys plus the neighbours found in the first pass.
 * Both paths call exactly the upstream TerrainEvaluation methods.
 */
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "retap_core/terrain_evaluation.h"
#include "retap_core/terrain_types.h"
#include "retap_core/world_representation/ig_tree.h"

namespace retap_core {

struct ScanOptions {
  int update_free = 0;          // 0 = no free, 1 = all free, 2 = 1 m behind hit
  float max_range = 1e6f;       // upstream static: 1e6, online global: 10 m
};

struct ScanResult {
  octomap::KeySet occupied_keys;  // exactly what IgTree::insertPointCloud returns
  size_t point_count = 0;
};

class TerrainModel {
 public:
  explicit TerrainModel(double resolution,
                        const TerrainConfig &cfg = TerrainConfig());
  // Adopt an existing tree, e.g. one loaded by loadTerrainBt().  The terrain
  // attributes stored in the file travel with it; the evaluator is rebuilt on
  // top of the same tree so online insertion can continue incrementally.
  explicit TerrainModel(std::shared_ptr<octomap::IgTree> tree,
                        const TerrainConfig &cfg = TerrainConfig());

  // ── insertion ───────────────────────────────────────────────────────────
  ScanResult insertCloud(const std::vector<Eigen::Vector3f> &points,
                         const Eigen::Vector3f &origin,
                         const ScanOptions &opts = ScanOptions{});
  ScanResult insertCloud(const octomap::Pointcloud &pc,
                         const octomap::point3d &origin,
                         const ScanOptions &opts = ScanOptions{});
  // Alias used by the online path; same as insertCloud() with opts.update_free
  // set by the caller (2 for a lidar scan).
  ScanResult insertScan(const std::vector<Eigen::Vector3f> &points,
                        const Eigen::Vector3f &origin,
                        const ScanOptions &opts = ScanOptions{});

  // ── terrain ─────────────────────────────────────────────────────────────
  // Every occupied leaf, exactly like MappingModule::loopProcessMaps().
  void evaluateAll(const Eigen::Vector3f &center);
  // Only the dirty keys; identical maths to evaluateAll.
  void evaluateDirty(const octomap::KeySet &dirty_keys,
                     const Eigen::Vector3f &center);

  // ── queries ─────────────────────────────────────────────────────────────
  bool queryNearestDrivable(const octomap::point3d &query,
                            octomap::point3d *result,
                            bool *in_disjoint_set = nullptr);
  size_t occupiedVoxelCount() const;
  size_t drivableVoxelCount() const;

  octomap::IgTree &tree() { return *tree_; }
  const octomap::IgTree &tree() const { return *tree_; }
  mapping_module::TerrainEvaluation &evaluator() { return *evaluator_; }
  const TerrainConfig &config() const { return cfg_; }
  double resolution() const { return tree_->getResolution(); }
  uint64_t generation() const { return generation_; }

 private:
  octomap::Boundingbox boundingBox(const std::vector<Eigen::Vector3f> &points,
                                   const Eigen::Vector3f &origin) const;

  double resolution_ = 0.2;
  TerrainConfig cfg_;
  std::shared_ptr<octomap::IgTree> tree_;
  std::shared_ptr<mapping_module::TerrainEvaluation> evaluator_;
  uint64_t generation_ = 0;
};

}  // namespace retap_core
