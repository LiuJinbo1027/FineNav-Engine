// A* on a terrain-annotated IgTree: path continuity, legality and failure.
// Uses the upstream Astar class (header-only, same signature as ReTAP).

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "retap_core/astar_3d/astar_3d.h"
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

// ReTAP slugcat thresholds (see map_io.h / TerrainConfig comments).
retap_core::TerrainConfig testConfig() {
  retap_core::TerrainConfig cfg;
  cfg.slope_threshold = 0.65;
  cfg.sparsity_threshold = 0.7;
  cfg.roughness_threshold = 0.5;
  cfg.traversability_threshold = 0.805;
  return cfg;
}

}  // namespace

TEST(AStar, FindsContinuousLegalPath) {
  retap_core::TerrainModel model(0.2, testConfig());
  const auto points = makePlane(-2.5f, 2.5f, -1.5f, 1.5f, 0.05f, 0.0f);
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  model.insertCloud(points, origin);
  model.evaluateAll(origin);

  const Eigen::Vector3f start(-1.5f, 0.f, 0.f);
  const Eigen::Vector3f goal(1.5f, 0.f, 0.f);
  octomap::IgTreeNode *start_node = model.tree().search(
      model.tree().coordToKey(octomap::point3d(start.x(), start.y(), start.z())));
  ASSERT_NE(start_node, nullptr);
  ASSERT_FALSE(start_node->getCollision()) << "start must be drivable";
  octomap::IgTreeNode *goal_node = model.tree().search(
      model.tree().coordToKey(octomap::point3d(goal.x(), goal.y(), goal.z())));
  ASSERT_NE(goal_node, nullptr);
  ASSERT_FALSE(goal_node->getCollision()) << "goal must be drivable";

  Astar astar(&model.tree(), 0.5f, 0.5f, 1.0f);
  astar.SetStartAndGoal(start, goal);
  const auto path = astar.startSearch();
  ASSERT_FALSE(path.empty());

  // A* works on voxel centres, so the endpoints are the centres of the voxels
  // that contain start/goal (within one voxel horizontally, 2 voxels in z).
  const double res = model.resolution();
  EXPECT_LT(std::hypot(path.front().x() - start.x(), path.front().y() - start.y()),
            res);
  EXPECT_LT(std::hypot(path.back().x() - goal.x(), path.back().y() - goal.y()),
            2.0 * res + 1e-4);
  EXPECT_LT(std::fabs(path.front().z() - start.z()), 2.0 * res + 1e-4);
  EXPECT_LT(std::fabs(path.back().z() - goal.z()), 2.0 * res + 1e-4);

  const double max_step = std::sqrt(3.0) * res + 1e-4;
  for (size_t i = 1; i < path.size(); ++i) {
    EXPECT_LE(static_cast<double>((path[i] - path[i - 1]).norm()), max_step);
  }
  for (const auto &p : path) {
    octomap::OcTreeKey key;
    ASSERT_TRUE(model.tree().coordToKeyChecked(
        octomap::point3d(p.x(), p.y(), p.z()), key));
    octomap::IgTreeNode *node = model.tree().search(key);
    ASSERT_NE(node, nullptr);
    EXPECT_FALSE(node->getCollision()) << "path enters a blocked voxel";
  }
}

TEST(AStar, GoalWithoutOccupiedVoxelFails) {
  retap_core::TerrainModel model(0.2, testConfig());
  const auto points = makePlane(-1.f, 1.f, -1.f, 1.f, 0.05f, 0.0f);
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  model.insertCloud(points, origin);
  model.evaluateAll(origin);

  Astar astar(&model.tree(), 0.5f, 0.5f, 1.0f);
  astar.SetStartAndGoal(Eigen::Vector3f(-0.5f, 0.f, 0.f),
                        Eigen::Vector3f(8.f, 0.f, 0.f));
  EXPECT_TRUE(astar.startSearch().empty());
}

// S4: the search box is now a constructor argument (the WorldView shrinks it
// to the start/goal neighbourhood).  A path inside a box smaller than the map
// must stay inside that box.
TEST(AStar, BoundedRegionFindsPathInsideBox) {
  retap_core::TerrainModel model(0.2, testConfig());
  const auto points = makePlane(-2.5f, 2.5f, -1.5f, 1.5f, 0.05f, 0.0f);
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  model.insertCloud(points, origin);
  model.evaluateAll(origin);

  const octomap::point3d bbx_min(-2.0, -1.0, -0.6);
  const octomap::point3d bbx_max(2.0, 1.0, 0.6);
  Astar astar(&model.tree(), 0.5f, 0.5f, 1.0f, bbx_min, bbx_max);
  astar.SetStartAndGoal(Eigen::Vector3f(-1.5f, 0.f, 0.f),
                        Eigen::Vector3f(1.5f, 0.f, 0.f));
  const auto path = astar.startSearch();
  ASSERT_FALSE(path.empty());
  for (const auto &p : path) {
    EXPECT_GE(p.x(), static_cast<float>(bbx_min.x()));
    EXPECT_LE(p.x(), static_cast<float>(bbx_max.x()));
    EXPECT_GE(p.y(), static_cast<float>(bbx_min.y()));
    EXPECT_LE(p.y(), static_cast<float>(bbx_max.y()));
    EXPECT_GE(p.z(), static_cast<float>(bbx_min.z()));
    EXPECT_LE(p.z(), static_cast<float>(bbx_max.z()));
  }
}

// S4: upstream's bounds check was commented out because its box was fixed at
// +/-300 m.  With a parameterised box an out-of-range start/goal would index
// open[]/openMinimums[] out of bounds, so CellOnGrid must reject it cleanly.
TEST(AStar, StartOrGoalOutsideRegionFails) {
  retap_core::TerrainModel model(0.2, testConfig());
  const auto points = makePlane(-2.5f, 2.5f, -1.5f, 1.5f, 0.05f, 0.0f);
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  model.insertCloud(points, origin);
  model.evaluateAll(origin);

  const octomap::point3d bbx_min(-1.0, -1.0, -0.6);
  const octomap::point3d bbx_max(1.0, 1.0, 0.6);

  Astar goal_outside(&model.tree(), 0.5f, 0.5f, 1.0f, bbx_min, bbx_max);
  goal_outside.SetStartAndGoal(Eigen::Vector3f(-0.5f, 0.f, 0.f),
                               Eigen::Vector3f(1.8f, 0.f, 0.f));
  EXPECT_TRUE(goal_outside.startSearch().empty());

  Astar start_outside(&model.tree(), 0.5f, 0.5f, 1.0f, bbx_min, bbx_max);
  start_outside.SetStartAndGoal(Eigen::Vector3f(-1.8f, 0.f, 0.f),
                                Eigen::Vector3f(0.5f, 0.f, 0.f));
  EXPECT_TRUE(start_outside.startSearch().empty());
}

// S4: the search runs inside the map read lock, so it must honour a hard
// expansion cap and report why it stopped.
TEST(AStar, ExpansionBudgetStopsSearch) {
  retap_core::TerrainModel model(0.2, testConfig());
  const auto points = makePlane(-2.5f, 2.5f, -1.5f, 1.5f, 0.05f, 0.0f);
  const Eigen::Vector3f origin = Eigen::Vector3f::Zero();
  model.insertCloud(points, origin);
  model.evaluateAll(origin);

  Astar astar(&model.tree(), 0.5f, 0.5f, 1.0f,
              octomap::point3d(-3.0, -2.0, -0.6),
              octomap::point3d(3.0, 2.0, 0.6), /*max_expansions=*/1,
              /*time_budget_ms=*/0.0, /*verbose=*/false);
  astar.SetStartAndGoal(Eigen::Vector3f(-1.5f, 0.f, 0.f),
                        Eigen::Vector3f(1.5f, 0.f, 0.f));
  EXPECT_TRUE(astar.startSearch().empty());
  EXPECT_TRUE(astar.budgetExceeded());
}
