// Incremental (online) semantics: dirty-key evaluation, free-space carving and
// robot-relative connectivity.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "retap_core/terrain_model.h"

namespace {

std::vector<Eigen::Vector3f> makePlane(float x0, float x1, float y0, float y1,
                                       float spacing, float z) {
  std::vector<Eigen::Vector3f> points;
  for (float x = x0; x <= x1 + 1e-4f; x += spacing) {
    for (float y = y0; y <= y1 + 1e-4f; y += spacing) {
      points.emplace_back(x, y, z);
    }
  }
  return points;
}

void appendPatch(std::vector<Eigen::Vector3f> *points, float cx, float cy,
                 float half, float z) {
  for (float x = cx - half; x <= cx + half + 1e-4f; x += 0.05f) {
    for (float y = cy - half; y <= cy + half + 1e-4f; y += 0.05f) {
      points->emplace_back(x, y, z);
    }
  }
}

// ReTAP's slugcat settings: the header defaults (sparsity 0.2) are tuned for
// dense lidar frames and reject a 16-points-per-voxel synthetic plane.
retap_core::TerrainConfig testConfig() {
  retap_core::TerrainConfig cfg;
  cfg.slope_threshold = 0.65;
  cfg.sparsity_threshold = 0.7;
  cfg.roughness_threshold = 0.5;
  cfg.traversability_threshold = 0.805;
  return cfg;
}

}  // namespace

TEST(Incremental, DirtyUpdateOnAnalyzedMapMatchesRebuild) {
  const float spacing = 0.05f;
  const auto plane = makePlane(-2.5f, 2.5f, -2.5f, 2.5f, spacing, 0.0f);
  std::vector<Eigen::Vector3f> bump;
  appendPatch(&bump, 0.6f, -0.4f, 0.2f, 0.06f);  // small raised patch
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();

  // Incremental: analyze the plane, then fold in the patch with evaluateDirty.
  retap_core::TerrainModel incremental(0.2);
  incremental.insertCloud(plane, origin);
  incremental.evaluateAll(origin);
  const auto scan = incremental.insertScan(
      bump, origin, retap_core::ScanOptions{0, 1e6f});
  ASSERT_GT(scan.occupied_keys.size(), 0u);
  incremental.evaluateDirty(scan.occupied_keys, origin);

  // Batch reference: same final point set, one analysis pass.
  retap_core::TerrainModel batch(0.2);
  std::vector<Eigen::Vector3f> all = plane;
  all.insert(all.end(), bump.begin(), bump.end());
  batch.insertCloud(all, origin);
  batch.evaluateAll(origin);

  size_t compared = 0;
  for (auto it = batch.tree().begin_leafs(); it != batch.tree().end_leafs();
       ++it) {
    if (!batch.tree().isNodeOccupied(&(*it))) continue;
    octomap::IgTreeNode *ref = &(*it);
    octomap::IgTreeNode *got = incremental.tree().search(it.getKey());
    ASSERT_NE(got, nullptr) << "incremental map is missing an occupied voxel";
    EXPECT_NEAR(ref->getSlope(), got->getSlope(), 1e-4f);
    EXPECT_NEAR(ref->getRoughness(), got->getRoughness(), 1e-4f);
    EXPECT_NEAR(ref->getSparsity(), got->getSparsity(), 1e-4f);
    EXPECT_NEAR(ref->getTraversability(), got->getTraversability(), 1e-4f);
    EXPECT_EQ(ref->getCollision(), got->getCollision());
    ++compared;
  }
  EXPECT_GT(compared, 400u);
}

TEST(Incremental, UpdateFreePolicy) {
  std::vector<Eigen::Vector3f> wall;
  for (float y = -0.5f; y <= 0.5f + 1e-4f; y += 0.05f) {
    for (float z = -0.5f; z <= 0.5f + 1e-4f; z += 0.05f) {
      wall.emplace_back(5.0f, y, z);
    }
  }
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();

  auto count_free = [](retap_core::TerrainModel &model) {
    size_t free_count = 0;
    for (auto it = model.tree().begin_leafs(); it != model.tree().end_leafs();
         ++it) {
      if (!model.tree().isNodeOccupied(&(*it))) ++free_count;
    }
    return free_count;
  };

  retap_core::TerrainModel carved(0.2);
  carved.insertScan(wall, origin, retap_core::ScanOptions{2, 50.f});
  EXPECT_GT(count_free(carved), 0u)
      << "update_free = 2 must carve the 1 m behind each endpoint";

  retap_core::TerrainModel bare(0.2);
  bare.insertScan(wall, origin, retap_core::ScanOptions{0, 50.f});
  EXPECT_EQ(count_free(bare), 0u) << "update_free = 0 must not create free cells";
}

TEST(Incremental, ConnectivityRootFollowsRobot) {
  // Two 3 m x 3 m patches 9 m apart: each is wide enough to have a drivable
  // interior, and they are far enough apart that no 6-neighbour link exists.
  std::vector<Eigen::Vector3f> points = makePlane(-7.5f, -4.5f, -1.5f, 1.5f,
                                                  0.05f, 0.0f);
  appendPatch(&points, 6.0f, 0.0f, 1.5f, 0.0f);

  auto connected_on = [](retap_core::TerrainModel &model, bool left) {
    size_t count = 0;
    octomap::IgTree &tree = model.tree();
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
      if (!tree.isNodeOccupied(&(*it)) || !it->getConnected()) continue;
      const double x = tree.keyToCoord(it.getKey()).x();
      if ((x < 0.0) == left) ++count;
    }
    return count;
  };

  retap_core::TerrainModel near_left(0.2, testConfig());
  near_left.insertCloud(points, Eigen::Vector3f::Zero());
  const Eigen::Vector3f left_center(-6.f, 0.f, 0.f);
  near_left.evaluateAll(left_center);
  EXPECT_GT(connected_on(near_left, true), 0u);
  EXPECT_EQ(connected_on(near_left, false), 0u)
      << "the disconnected patch is not reachable from the robot";

  retap_core::TerrainModel near_right(0.2, testConfig());
  near_right.insertCloud(points, Eigen::Vector3f::Zero());
  const Eigen::Vector3f right_center(6.f, 0.f, 0.f);
  near_right.evaluateAll(right_center);
  EXPECT_EQ(connected_on(near_right, true), 0u);
  EXPECT_GT(connected_on(near_right, false), 0u);
}
