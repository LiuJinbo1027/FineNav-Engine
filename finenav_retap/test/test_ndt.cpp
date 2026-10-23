// NDT statistics stored in IgTreeNode must match the closed-form mean and
// (biased) covariance of the inserted points.

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <vector>

#include "retap_core/world_representation/ig_tree.h"

namespace {

Eigen::Vector3f mean(const std::vector<Eigen::Vector3f> &points) {
  Eigen::Vector3f mu = Eigen::Vector3f::Zero();
  for (const auto &p : points) mu += p;
  return mu / static_cast<float>(points.size());
}

Eigen::Matrix3f covariance(const std::vector<Eigen::Vector3f> &points) {
  const Eigen::Vector3f mu = mean(points);
  Eigen::Matrix3f sigma = Eigen::Matrix3f::Zero();
  for (const auto &p : points) sigma += (p - mu) * (p - mu).transpose();
  return sigma / static_cast<float>(points.size());
}

std::vector<Eigen::Vector3f> samplePoints() {
  // All points must fall inside the same 0.2 m voxel: octomap maps a
  // coordinate to floor(coord / resolution) + 32768, so keep x, y and z in
  // (0, 0.2).
  return {{0.01f, 0.02f, 0.03f},  {0.05f, 0.04f, 0.03f},
          {0.09f, 0.02f, 0.05f},  {0.07f, 0.08f, 0.04f},
          {0.03f, 0.06f, 0.07f},  {0.05f, 0.03f, 0.01f}};
}

}  // namespace

TEST(Ndt, MatchesClosedFormStatistics) {
  octomap::IgTree tree(0.2);
  const std::vector<Eigen::Vector3f> points = samplePoints();
  octomap::Pointcloud pc;
  for (const auto &p : points) pc.push_back(octomap::point3d(p.x(), p.y(), p.z()));

  octomap::Boundingbox bbox;
  const auto inserted =
      tree.insertPointCloud(pc, octomap::point3d(0, 0, 5), bbox, 0, 1e6f);
  ASSERT_EQ(inserted.first.size(), 1u) << "all points must fall in one voxel";

  octomap::IgTreeNode *node = tree.search(*inserted.first.begin());
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->pointCount(), static_cast<int>(points.size()));

  const Eigen::Vector3f mu = mean(points);
  const Eigen::Matrix3f sigma = covariance(points);
  EXPECT_LT((node->mu() - mu).norm(), 1e-5f);
  EXPECT_LT((node->cov() - sigma).norm(), 1e-5f);
}

TEST(Ndt, PointCountIsCapped) {
  octomap::IgTree tree(0.2);
  octomap::Pointcloud pc;
  for (int i = 0; i < 50; ++i) {
    const float x = 0.001f * static_cast<float>(i);
    pc.push_back(octomap::point3d(x, 0.0, 0.0));
  }
  octomap::Boundingbox bbox;
  const auto inserted =
      tree.insertPointCloud(pc, octomap::point3d(0, 0, 5), bbox, 0, 1e6f);
  ASSERT_EQ(inserted.first.size(), 1u);
  octomap::IgTreeNode *node = tree.search(*inserted.first.begin());
  ASSERT_NE(node, nullptr);
  // insertPoint() stops once pt_cnt_ > 30, so at most 31 points are fused.
  EXPECT_EQ(node->pointCount(), 31);
}

TEST(Ndt, CollisionDefaultsToBlocked) {
  octomap::IgTree tree(0.2);
  octomap::Pointcloud pc;
  pc.push_back(octomap::point3d(0, 0, 0));
  octomap::Boundingbox bbox;
  const auto inserted =
      tree.insertPointCloud(pc, octomap::point3d(0, 0, 5), bbox, 0, 1e6f);
  octomap::IgTreeNode *node = tree.search(*inserted.first.begin());
  ASSERT_NE(node, nullptr);
  // A voxel that never went through terrain analysis reports a collision.
  EXPECT_TRUE(node->getCollision());
}
