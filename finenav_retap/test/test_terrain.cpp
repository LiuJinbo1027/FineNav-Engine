// Terrain attributes on synthetic geometry: flat plane, 30 deg ramp, 0.5 m
// step, degenerate clouds and the .bt round trip.

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "retap_core/map_io.h"
#include "retap_core/terrain_model.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

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

struct TerrainStats {
  size_t occupied = 0;
  size_t collision_free = 0;
  double mean_slope = 0.0;
  double mean_roughness = 0.0;
};

TerrainStats collect(retap_core::TerrainModel &model) {
  TerrainStats stats;
  double slope_sum = 0.0;
  double roughness_sum = 0.0;
  octomap::IgTree &tree = model.tree();
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (!tree.isNodeOccupied(&(*it))) continue;
    ++stats.occupied;
    slope_sum += it->getSlope();
    roughness_sum += it->getRoughness();
    if (!it->getCollision()) ++stats.collision_free;
  }
  if (stats.occupied > 0) {
    stats.mean_slope = slope_sum / static_cast<double>(stats.occupied);
    stats.mean_roughness =
        roughness_sum / static_cast<double>(stats.occupied);
  }
  return stats;
}

}  // namespace

TEST(Terrain, FlatPlaneIsDrivable) {
  retap_core::TerrainConfig cfg;
  cfg.slope_threshold = 0.65;
  cfg.sparsity_threshold = 0.7;
  cfg.roughness_threshold = 0.5;
  cfg.traversability_threshold = 0.805;
  retap_core::TerrainModel model(0.2, cfg);

  const auto points = makePlane(-2.5f, 2.5f, -2.5f, 2.5f, 0.05f, 0.0f);
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  model.insertCloud(points, origin);
  model.evaluateAll(origin);

  const TerrainStats stats = collect(model);
  ASSERT_GT(stats.occupied, 400u);
  EXPECT_LT(stats.mean_slope, 0.05);
  EXPECT_LT(stats.mean_roughness, 0.2);
  EXPECT_GT(model.drivableVoxelCount(), 400u);

  // The terrain passes only if the upstream ±2 neighbour ring (80 voxels) is
  // populated.  At the rim of a synthetic, isolated plane those columns are
  // empty, so ReTAP marks the outer ~3-cell ring as collision.  The interior
  // must be completely drivable.
  const float half = 2.5f;
  const float margin = 0.8f;
  size_t interior = 0, interior_free = 0;
  for (auto it = model.tree().begin_leafs(); it != model.tree().end_leafs();
       ++it) {
    if (!model.tree().isNodeOccupied(&(*it))) continue;
    const auto p = model.tree().keyToCoord(it.getKey());
    // The plane edge voxel centres sit exactly on half - margin; the rim
    // collision band reaches the next cell inward, so exclude that column.
    if (std::max(std::fabs(p.x()), std::fabs(p.y())) >= half - margin) continue;
    ++interior;
    if (!it->getCollision()) ++interior_free;
  }
  EXPECT_GT(interior, 200u);
  EXPECT_EQ(interior, interior_free) << "the flat interior must be drivable";
}

TEST(Terrain, RampSlopeIsRecovered) {
  retap_core::TerrainModel model(0.2);
  std::vector<Eigen::Vector3f> points;
  const float tan30 = std::tan(static_cast<float>(30.0 * kPi / 180.0));
  for (float x = -2.0f; x <= 2.0f + 1e-4f; x += 0.05f) {
    for (float y = -1.0f; y <= 1.0f + 1e-4f; y += 0.05f) {
      points.emplace_back(x, y, x * tan30);
    }
  }
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  model.insertCloud(points, origin);
  model.evaluateAll(origin);

  const TerrainStats stats = collect(model);
  ASSERT_GT(stats.occupied, 100u);
  EXPECT_NEAR(stats.mean_slope, 30.0 * kPi / 180.0, 0.08);
}

TEST(Terrain, StepEdgeIsBlocked) {
  retap_core::TerrainModel model(0.2);
  std::vector<Eigen::Vector3f> points;
  for (float x = -2.0f; x <= 2.0f + 1e-4f; x += 0.05f) {
    const float z = x < 0.0f ? 0.0f : 0.5f;
    for (float y = -1.0f; y <= 1.0f + 1e-4f; y += 0.05f) {
      points.emplace_back(x, y, z);
    }
  }
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  model.insertCloud(points, origin);
  model.evaluateAll(origin);

  const TerrainStats stats = collect(model);
  ASSERT_GT(stats.occupied, 100u);
  EXPECT_LT(stats.collision_free, stats.occupied)
      << "a 0.5 m step must produce blocked voxels";
  EXPECT_GT(model.drivableVoxelCount(), 0u)
      << "the flat parts away from the step stay drivable";
}

TEST(Terrain, DegenerateCloudsDoNotCrash) {
  retap_core::TerrainModel empty_model(0.2);
  empty_model.evaluateAll(Eigen::Vector3f::Zero());
  EXPECT_EQ(empty_model.drivableVoxelCount(), 0u);

  retap_core::TerrainModel single_model(0.2);
  std::vector<Eigen::Vector3f> single = {Eigen::Vector3f(3.f, 3.f, 0.f)};
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  single_model.insertCloud(single, origin);
  single_model.evaluateAll(origin);
  // An isolated voxel is rejected as an outlier by getUpperVoxels().
  EXPECT_EQ(single_model.drivableVoxelCount(), 0u);
}

TEST(Terrain, BtRoundTripKeepsTerrainAttributes) {
  retap_core::TerrainConfig cfg;
  retap_core::TerrainModel model(0.2, cfg);
  const auto points = makePlane(-1.5f, 1.5f, -1.5f, 1.5f, 0.05f, 0.0f);
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  model.insertCloud(points, origin);
  model.evaluateAll(origin);
  ASSERT_GT(model.occupiedVoxelCount(), 0u);

  const std::string path =
      (std::filesystem::temp_directory_path() / "retap_test_terrain.bt")
          .string();
  std::string error;
  ASSERT_TRUE(retap_core::saveTerrainBt(model.tree(), path, &error)) << error;

  auto loaded = retap_core::loadTerrainBt(path, &error);
  ASSERT_NE(loaded, nullptr) << error;
  EXPECT_EQ(loaded->getTreeType(), "IgTree");

  size_t compared = 0;
  for (auto it = model.tree().begin_leafs(); it != model.tree().end_leafs();
       ++it) {
    if (!model.tree().isNodeOccupied(&(*it))) continue;
    octomap::IgTreeNode *a = &(*it);
    octomap::IgTreeNode *b = loaded->search(it.getKey());
    ASSERT_NE(b, nullptr);
    EXPECT_FLOAT_EQ(a->getSlope(), b->getSlope());
    EXPECT_FLOAT_EQ(a->getRoughness(), b->getRoughness());
    EXPECT_FLOAT_EQ(a->getSparsity(), b->getSparsity());
    EXPECT_FLOAT_EQ(a->getTraversability(), b->getTraversability());
    EXPECT_EQ(a->getCollision(), b->getCollision());
    // Documented loss: the NDT covariance and the connectivity flag are not
    // part of readData/writeData.
    EXPECT_LT(b->cov().norm(), 1e-9f);
    EXPECT_FALSE(b->getConnected());
    ++compared;
  }
  EXPECT_GT(compared, 0u);
  std::filesystem::remove(path);
}
