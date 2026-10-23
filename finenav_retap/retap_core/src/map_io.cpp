#include "retap_core/map_io.h"

#include <octomap/AbstractOcTree.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
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
    case VoxelField::kConnected:
      return node->getConnected() ? 1.f : 0.f;
  }
  return 0.f;
}

void updateStats(float value, float *mn, double *sum, float *mx) {
  *mn = std::min(*mn, value);
  *mx = std::max(*mx, value);
  *sum += value;
}

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
  const float inf = std::numeric_limits<float>::infinity();
  s.slope_min = s.roughness_min = s.sparsity_min = s.traversability_min = inf;
  s.slope_max = s.roughness_max = s.sparsity_max = s.traversability_max = -inf;
  double slope_sum = 0, rough_sum = 0, sparse_sum = 0, trav_sum = 0;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    auto *node = const_cast<octomap::IgTreeNode *>(&*it);
    if (!tree.isNodeOccupied(node)) continue;
    ++s.occupied_voxels;
    if (!node->getCollision()) ++s.drivable_voxels;
    if (node->getConnected()) ++s.connected_voxels;
    updateStats(node->getSlope(), &s.slope_min, &slope_sum, &s.slope_max);
    updateStats(node->getRoughness(), &s.roughness_min, &rough_sum,
                &s.roughness_max);
    updateStats(node->getSparsity(), &s.sparsity_min, &sparse_sum,
                &s.sparsity_max);
    updateStats(node->getTraversability(), &s.traversability_min, &trav_sum,
                &s.traversability_max);
  }
  if (s.occupied_voxels > 0) {
    const double n = static_cast<double>(s.occupied_voxels);
    s.slope_mean = slope_sum / n;
    s.roughness_mean = rough_sum / n;
    s.sparsity_mean = sparse_sum / n;
    s.traversability_mean = trav_sum / n;
  }
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
