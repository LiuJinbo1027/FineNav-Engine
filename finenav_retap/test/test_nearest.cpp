// ReTAP's nearest-drivable semantics, as RetapTerrainMap::nearestDrivable now
// uses them: an exact voxel hit when the query lies in the robot's connected
// component, otherwise the 3-D nearest voxel of that component.  Distance is
// never a failure condition -- only an empty map is.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "retap_core/map_io.h"
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

// ReTAP's slugcat settings (see test_incremental.cpp): the header defaults
// reject a 16-points-per-voxel synthetic plane.
retap_core::TerrainConfig testConfig() {
  retap_core::TerrainConfig cfg;
  cfg.slope_threshold = 0.65;
  cfg.sparsity_threshold = 0.7;
  cfg.roughness_threshold = 0.5;
  cfg.traversability_threshold = 0.805;
  return cfg;
}

bool query(retap_core::TerrainModel *model, float x, float y, float z,
           octomap::point3d *out, bool *exact) {
  *exact = false;
  return model->evaluator().queryNearestDrivable(octomap::point3d(x, y, z), out,
                                                exact);
}

}  // namespace

// A 2-D goal arrives with z = 0 while the terrain is at z = 2: no radius is
// allowed to reject the snap, which is exactly the case that used to fail with
// "no drivable voxel within 1.50 m of the goal".
TEST(NearestDrivable, SnapsFromTwoMetresBelowWithoutRadius) {
  retap_core::TerrainModel model(0.2, testConfig());
  model.insertCloud(makePlane(-2.f, 2.f, -2.f, 2.f, 0.05f, 2.f),
                    Eigen::Vector3f(0.f, 0.f, 2.f));
  model.evaluateAll(Eigen::Vector3f(0.f, 0.f, 2.f));
  ASSERT_GT(model.drivableVoxelCount(), 0u);

  octomap::point3d snapped;
  bool exact = false;
  ASSERT_TRUE(query(&model, 0.5f, 0.5f, 0.f, &snapped, &exact));
  EXPECT_FALSE(exact);  // z = 0 is not a drivable voxel
  EXPECT_NEAR(snapped.z(), 2.0, 0.25);  // one voxel below the 2.0 surface
  EXPECT_NEAR(snapped.x(), 0.5, 0.2);
  EXPECT_NEAR(snapped.y(), 0.5, 0.2);
}

// A query inside the robot's component returns that voxel's centre, not a
// neighbour, and reports the exact hit.
TEST(NearestDrivable, ExactVoxelReturnsItsCentre) {
  retap_core::TerrainModel model(0.2, testConfig());
  model.insertCloud(makePlane(-2.f, 2.f, -2.f, 2.f, 0.05f, 0.f),
                    Eigen::Vector3f::Zero());
  model.evaluateAll(Eigen::Vector3f::Zero());

  // Pick an interior drivable voxel, away from the collision ring the upstream
  // checkNeighborCells() paints around the edge of an isolated plane.
  octomap::OcTreeKey key;
  bool found = false;
  for (auto it = model.tree().begin_leafs(); it != model.tree().end_leafs();
       ++it) {
    const octomap::point3d p = model.tree().keyToCoord(it.getKey());
    if (!model.tree().isNodeOccupied(&(*it)) || it->getCollision()) continue;
    if (std::fabs(p.x()) > 1.0 || std::fabs(p.y()) > 1.0) continue;
    key = it.getKey();
    found = true;
    break;
  }
  ASSERT_TRUE(found);
  ASSERT_TRUE(model.evaluator().checkDrivable(key));
  const octomap::point3d centre = model.tree().keyToCoord(key);

  octomap::point3d snapped;
  bool exact = false;
  ASSERT_TRUE(query(&model, static_cast<float>(centre.x()),
                    static_cast<float>(centre.y()),
                    static_cast<float>(centre.z()), &snapped, &exact));
  EXPECT_TRUE(exact);
  EXPECT_NEAR(snapped.x(), centre.x(), 1e-4);
  EXPECT_NEAR(snapped.y(), centre.y(), 1e-4);
  EXPECT_NEAR(snapped.z(), centre.z(), 1e-4);
}

// An isolated platform that is not 6-connected to the robot must not be
// snapped to, even when it is the geometrically nearest drivable voxel.  This
// is why the index is rooted at the robot instead of at the query.
TEST(NearestDrivable, DisconnectedIslandIsNotSnappedTo) {
  retap_core::TerrainModel model(0.2, testConfig());
  std::vector<Eigen::Vector3f> points;
  appendPatch(&points, -2.f, 0.f, 1.0f, 0.f);  // robot's platform
  appendPatch(&points, 2.f, 0.f, 1.0f, 0.f);   // 2 m gap: separate component
  model.insertCloud(points, Eigen::Vector3f::Zero());
  // Root = drivable voxel nearest the robot, which stands on the left patch.
  model.evaluateAll(Eigen::Vector3f(-2.f, 0.f, 0.f));

  octomap::point3d snapped;
  bool exact = false;
  // The query sits on the island's own surface.
  ASSERT_TRUE(query(&model, 2.f, 0.f, 0.f, &snapped, &exact));
  EXPECT_FALSE(exact) << "a voxel outside the root component is not an exact hit";
  EXPECT_LT(snapped.x(), -0.9f) << "snapped onto the island at x = " << snapped.x();
}

// After loading a .bt the disjoint set and ikd-tree are empty (connectivity is
// not serialized), so the query must fail until the index is rebuilt -- the
// rebuild is what RetapTerrainMap::ensureNearestIndex() performs.
TEST(NearestDrivable, BtLoadNeedsIndexRebuild) {
  const retap_core::TerrainConfig cfg = testConfig();
  const std::string path =
      (std::filesystem::temp_directory_path() / "retap_test_nearest.bt").string();
  {
    retap_core::TerrainModel built(0.2, cfg);
    built.insertCloud(makePlane(-2.f, 2.f, -2.f, 2.f, 0.05f, 2.f),
                      Eigen::Vector3f(0.f, 0.f, 2.f));
    built.evaluateAll(Eigen::Vector3f(0.f, 0.f, 2.f));
    std::string error;
    ASSERT_GT(built.drivableVoxelCount(), 0u);
    ASSERT_TRUE(retap_core::saveTerrainBt(built.tree(), path, &error)) << error;
  }

  std::string error;
  std::unique_ptr<octomap::IgTree> loaded = retap_core::loadTerrainBt(path, &error);
  ASSERT_NE(loaded, nullptr) << error;
  std::filesystem::remove(path);

  retap_core::TerrainModel adopted(std::move(loaded), cfg);
  EXPECT_FALSE(adopted.evaluator().hasDrivableIndex());

  octomap::point3d snapped;
  bool exact = false;
  EXPECT_FALSE(query(&adopted, 0.5f, 0.5f, 0.f, &snapped, &exact));

  // What ensureNearestIndex() does: seed the index from every occupied,
  // collision-free leaf, rooted at the drivable voxel nearest the robot.
  std::vector<octomap::OcTreeKey> keys;
  std::vector<octomap::IgTreeNode *> nodes;
  for (auto it = adopted.tree().begin_leafs(); it != adopted.tree().end_leafs();
       ++it) {
    if (!adopted.tree().isNodeOccupied(&(*it)) || it->getCollision()) continue;
    keys.push_back(it.getKey());
    nodes.push_back(&(*it));
  }
  ASSERT_FALSE(keys.empty());
  std::vector<octomap::OcTreeKey> no_revoke_keys;
  std::vector<octomap::IgTreeNode *> no_revoke_nodes;
  adopted.evaluator().updateDrivableDisjointSet(
      keys, nodes, no_revoke_keys, no_revoke_nodes,
      octomap::point3d(0.0, 0.0, 2.0));
  EXPECT_TRUE(adopted.evaluator().hasDrivableIndex());

  size_t connected = 0;
  for (octomap::IgTreeNode *node : nodes) {
    if (node->getConnected()) ++connected;
  }
  EXPECT_EQ(connected, keys.size()) << "a single plane must form one component";

  ASSERT_TRUE(query(&adopted, 0.5f, 0.5f, 0.f, &snapped, &exact));
  EXPECT_NEAR(snapped.z(), 2.0, 0.25);
}

// An empty map (or one whose terrain analysis produced no drivable voxel) has
// nothing to snap to; that is the only remaining failure mode.
TEST(NearestDrivable, EmptyMapReportsFailure) {
  retap_core::TerrainModel empty(0.2, testConfig());
  octomap::point3d snapped;
  bool exact = false;
  EXPECT_FALSE(query(&empty, 0.f, 0.f, 0.f, &snapped, &exact));
}
