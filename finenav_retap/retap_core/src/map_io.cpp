#include "retap_core/map_io.h"

#include <octomap/AbstractOcTree.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>

namespace retap_core {
namespace {

float fieldValue(octomap::IgTreeNode *node, VoxelField field) {
  switch (field) {
    case VoxelField::kTraversability:
      return node->getTraversability();
    case VoxelField::kSlope:
      return node->getSlope();
    case VoxelField::kRoughness:
      return node->getRoughness();
    case VoxelField::kSparsity:
      return node->getSparsity();
    case VoxelField::kCollision:
      return node->getCollision() ? 1.f : 0.f;
    case VoxelField::kDrivable:
      // getCollision() is the "is this voxel navigable" flag; its default is
      // 1 (collision), so an unanalysed node is never mistaken for drivable.
      return node->getCollision() ? 0.f : 1.f;
    case VoxelField::kConnected:
      return node->getConnected() ? 1.f : 0.f;
  }
  return 0.f;
}

// One field's min/mean/max over the finite values only.  Upstream's NDT
// fusion can produce non-finite slope/roughness/traversability for a few
// nodes (degenerate eigendecomposition); letting one of those into the sum
// poisoned the mean with NaN, and the stats file then was not valid JSON.
struct FieldStats {
  float mn = std::numeric_limits<float>::infinity();
  float mx = -std::numeric_limits<float>::infinity();
  double sum = 0.0;
  std::size_t count = 0;
  void add(float value) {
    if (!std::isfinite(value)) return;
    mn = std::min(mn, value);
    mx = std::max(mx, value);
    sum += value;
    ++count;
  }
  void write(float *out_min, float *out_mean, float *out_max) const {
    *out_min = count > 0 ? mn : 0.f;
    *out_max = count > 0 ? mx : 0.f;
    *out_mean = count > 0 ? static_cast<float>(sum / static_cast<double>(count))
                          : 0.f;
  }
};

}  // namespace

bool loadPcd(const std::string &path, std::vector<Eigen::Vector3f> *points,
             std::string *error) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) != 0) {
    if (error) *error = "failed to read PCD: " + path;
    return false;
  }
  points->clear();
  points->reserve(cloud->size());
  for (const auto &p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    points->emplace_back(p.x, p.y, p.z);
  }
  return true;
}

bool saveTerrainBt(octomap::IgTree &tree, const std::string &path,
                   std::string *error) {
  // The binary .bt format only encodes occupancy bits and never calls
  // IgTreeNode::writeData(), so slope/roughness/sparsity/traversability/
  // collision would all be lost.  ReTAP therefore saves with write(), the
  // octomap ASCII container (despite the .bt suffix): a text header followed
  // by IgTreeNode::writeData() payloads.  loadTerrainBt() reads it back.
  if (!tree.write(path)) {
    if (error) *error = "failed to write " + path;
    return false;
  }
  return true;
}

std::unique_ptr<octomap::IgTree> loadTerrainBt(const std::string &path,
                                               std::string *error) {
  // Two containers are accepted:
  //   "# Octomap OcTree file"        - ASCII header + per-node payloads, the
  //                                    only format that keeps terrain fields.
  //   "# Octomap OcTree binary file" - occupancy only (octomap never calls
  //                                    IgTreeNode::writeData() here).
  // AbstractOcTree::read(filename) is not used because it only handles the
  // ASCII form; the concrete class is needed for the binary one.
  std::ifstream file(path, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    if (error) *error = "failed to open " + path;
    return nullptr;
  }
  std::string line;
  std::getline(file, line);

  if (line.rfind("# Octomap OcTree binary file", 0) == 0) {
    std::string tree_id;
    while (std::getline(file, line)) {
      if (line.rfind("id ", 0) == 0) {
        tree_id = line.substr(3);
        break;
      }
    }
    if (tree_id != "IgTree") {
      if (error) *error = "tree type is '" + tree_id + "', expected 'IgTree'";
      return nullptr;
    }
    file.close();
    std::unique_ptr<octomap::IgTree> tree(new octomap::IgTree(0.1));
    if (!tree->readBinary(path)) {
      if (error) *error = "failed to read binary octree: " + path;
      return nullptr;
    }
    return tree;
  }

  if (line.rfind("# Octomap OcTree file", 0) != 0) {
    if (error) *error = "not an octomap file (first line: '" + line + "')";
    return nullptr;
  }

  std::string tree_id;
  double resolution = -1.0;
  bool data_found = false;
  while (std::getline(file, line)) {
    if (line.rfind("id ", 0) == 0) {
      tree_id = line.substr(3);
    } else if (line.rfind("res ", 0) == 0) {
      resolution = std::stod(line.substr(4));
    } else if (line.rfind("size ", 0) == 0) {
      // informational: readData() rebuilds the tree and its size.
    } else if (line.rfind("data", 0) == 0) {
      data_found = true;
      break;
    } else if (line.empty() || line[0] != '#') {
      if (error) *error = "malformed octomap header in " + path;
      return nullptr;
    }
  }
  if (!data_found || resolution <= 0.0) {
    if (error) *error = "missing data/res section in " + path;
    return nullptr;
  }
  if (tree_id != "IgTree") {
    if (error) *error = "tree type is '" + tree_id + "', expected 'IgTree'";
    return nullptr;
  }

  std::unique_ptr<octomap::IgTree> tree(new octomap::IgTree(resolution));
  tree->readData(file);
  if (!file) {
    if (error) *error = "failed to read octree payload: " + path;
    return nullptr;
  }
  return tree;
}

bool writeVoxelCloud(const octomap::IgTree &tree, const std::string &path,
                     VoxelField field, std::string *error) {
  pcl::PointCloud<pcl::PointXYZI> out;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    // begin_leafs() on a const tree yields const nodes; the getters were not
    // const upstream, so drop constness here (read-only use).
    auto *node = const_cast<octomap::IgTreeNode *>(&*it);
    if (!tree.isNodeOccupied(node)) continue;
    octomap::point3d p = tree.keyToCoord(it.getKey());
    pcl::PointXYZI q;
    q.x = p.x();
    q.y = p.y();
    q.z = p.z();
    q.intensity = fieldValue(node, field);
    out.push_back(q);
  }
  out.height = 1;
  out.width = static_cast<uint32_t>(out.size());
  out.is_dense = true;
  if (pcl::io::savePCDFileBinary(path, out) != 0) {
    if (error) *error = "failed to write " + path;
    return false;
  }
  return true;
}

TerrainStats computeStats(octomap::IgTree &tree) {
  TerrainStats s;
  FieldStats slope, roughness, sparsity, traversability;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    auto *node = const_cast<octomap::IgTreeNode *>(&*it);
    if (!tree.isNodeOccupied(node)) continue;
    ++s.occupied_voxels;
    if (!node->getCollision()) ++s.drivable_voxels;
    if (node->getConnected()) ++s.connected_voxels;
    slope.add(node->getSlope());
    roughness.add(node->getRoughness());
    sparsity.add(node->getSparsity());
    traversability.add(node->getTraversability());
  }
  // Means are over the finite samples, not over occupied_voxels: a node whose
  // NDT eigendecomposition degenerated contributes no slope/roughness value at
  // all, and counting it as zero would bias the mean downwards.
  slope.write(&s.slope_min, &s.slope_mean, &s.slope_max);
  roughness.write(&s.roughness_min, &s.roughness_mean, &s.roughness_max);
  sparsity.write(&s.sparsity_min, &s.sparsity_mean, &s.sparsity_max);
  traversability.write(&s.traversability_min, &s.traversability_mean,
                       &s.traversability_max);
  return s;
}

std::unique_ptr<TerrainModel> buildTerrainFromPcd(
    const std::string &pcd_path, double resolution, const TerrainConfig &cfg,
    TerrainBuildResult *result, std::string *error,
    const Eigen::Vector3f *center) {
  using Clock = std::chrono::steady_clock;
  auto elapsed_ms = [](Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };

  std::vector<Eigen::Vector3f> points;
  const auto t0 = Clock::now();
  if (!loadPcd(pcd_path, &points, error)) return nullptr;
  const auto t1 = Clock::now();
  if (points.empty()) {
    if (error) *error = "no finite points in " + pcd_path;
    return nullptr;
  }

  // The connectivity root is the drivable voxel nearest this point, so it
  // stands in for the robot pose.  Without one, assume the robot is at the
  // centre of the cloud.
  Eigen::Vector3f c = Eigen::Vector3f::Zero();
  if (center != nullptr) {
    c = *center;
  } else {
    for (const auto &p : points) c += p;
    c /= static_cast<float>(points.size());
  }

  std::unique_ptr<TerrainModel> model(new TerrainModel(resolution, cfg));
  ScanOptions opts;
  opts.update_free = cfg.update_free;
  opts.max_range = 1e6f;  // offline: the whole cloud is one scan
  model->insertCloud(points, c, opts);
  const auto t2 = Clock::now();
  model->evaluateAll(c);
  const auto t3 = Clock::now();

  if (result != nullptr) {
    result->input_points = points.size();
    result->occupied_voxels = model->occupiedVoxelCount();
    result->drivable_voxels = model->drivableVoxelCount();
    result->connected_voxels = computeStats(model->tree()).connected_voxels;
    result->load_ms = elapsed_ms(t0, t1);
    result->insert_ms = elapsed_ms(t1, t2);
    result->analysis_ms = elapsed_ms(t2, t3);
    result->center = c;
  }
  return model;
}

bool nearestDrivableVoxel(octomap::IgTree &tree, const Eigen::Vector3f &query,
                          float radius, Eigen::Vector3f *result) {
  octomap::OcTreeKey key_min;
  octomap::OcTreeKey key_max;
  if (!tree.coordToKeyChecked(octomap::point3d(query.x() - radius,
                                               query.y() - radius,
                                               query.z() - radius),
                              key_min) ||
      !tree.coordToKeyChecked(octomap::point3d(query.x() + radius,
                                               query.y() + radius,
                                               query.z() + radius),
                              key_max)) {
    return false;
  }

  float best = radius * radius;
  bool found = false;
  for (auto it = tree.begin_leafs_bbx(key_min, key_max);
       it != tree.end_leafs_bbx(); ++it) {
    if (!tree.isNodeOccupied(&*it) || it->getCollision()) continue;
    const octomap::point3d p = tree.keyToCoord(it.getKey());
    const float dx = static_cast<float>(p.x()) - query.x();
    const float dy = static_cast<float>(p.y()) - query.y();
    const float dz = static_cast<float>(p.z()) - query.z();
    const float d = dx * dx + dy * dy + dz * dz;
    if (d <= best) {
      best = d;
      *result = Eigen::Vector3f(static_cast<float>(p.x()),
                                static_cast<float>(p.y()),
                                static_cast<float>(p.z()));
      found = true;
    }
  }
  return found;
}

}  // namespace retap_core
