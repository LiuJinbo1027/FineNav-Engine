/* Copyright Year: 2026
 * Copyright Owner: Networked Robotics and Systems Lab
 * Authors: Yuxiang Li, Kun Chen, Haoyao Chen
 */
 
#include "retap_core/terrain_evaluation.h"

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <chrono>
#include <iostream>
#include <unordered_set>

namespace {
// retap_core: upper bound of the horizontal ring, so checkNeighborCells can
// keep its visited-offset flags on the stack.  128 -> R <= 15 (3.1 m half-span
// at 0.20 m).  Upstream's bool flag[64] capped R at 7.
constexpr int kMaxNeighborCells = 128;
constexpr int kMaxNeighborRadius = (kMaxNeighborCells - 1) / 8;
}  // namespace

namespace mapping_module {

TerrainEvaluation::TerrainEvaluation(std::shared_ptr<octomap::IgTree> tree) {
  resetTerrainRepresentation(tree);
  // The defaults are upstream's (ring 17 -> radius 2, z in [-3, +3]), so the
  // evaluator is usable before configure() is called.
  rebuildNeighborOffsets();
}

TerrainEvaluation::~TerrainEvaluation() {}

// retap_core: replaces the ros::NodeHandle::getParam() block of the upstream
// constructor.  Field names and defaults are unchanged.
void TerrainEvaluation::configure(const retap_core::TerrainConfig &cfg) {
  roughness_threshold_ = cfg.roughness_threshold;
  sparsity_threshold_ = cfg.sparsity_threshold;
  slope_threshold_ = cfg.slope_threshold;
  traversability_threshold_ = cfg.traversability_threshold;
  collision_neighbor_ = cfg.collision_neighbor;
  std::cout << "<TerrainEvaluation>: slope_threshold_=" << slope_threshold_
            << ", sparsity_threshold_=" << sparsity_threshold_
            << ", traversability_threshold_=" << traversability_threshold_
            << std::endl;
  roughness_weight_ = cfg.roughness_weight;
  sparsity_weight_ = cfg.sparsity_weight;
  slope_weight_ = cfg.slope_weight;
  traversability_weight_ = cfg.traversability_weight;
  points_number_threshold_ = cfg.points_number_threshold;
  frame_height_ = cfg.frame_height;
  filling_radius_ = cfg.filling_radius;
  clearance_voxels_ = cfg.clearance_voxels < 1 ? 1 : cfg.clearance_voxels;
  support_voxels_ = cfg.support_voxels < 0 ? 0 : cfg.support_voxels;
  min_voxel_points_ = cfg.min_voxel_points < 0 ? 0 : cfg.min_voxel_points;
  rebuildNeighborOffsets();
}

// retap_core: upstream kept the neighbour offsets in the static index_x /
// index_y tables inside checkNeighborCells (25 entries = center + 3 shells).
// The tables are generated here instead so the radius can follow the
// resolution: a 0.60 m footprint needs ring 1 at 0.20 m, ring 6 at 0.05 m.
// Entry 0 is the voxel itself; shell r contributes
//   (0,r), (0,-r), (r,0), (-r,0), (r,r), (r,-r), (-r,r), (-r,-r)
// which for r = 1..3 is exactly the upstream table.
void TerrainEvaluation::rebuildNeighborOffsets() {
  int radius = neighborRadius();
  if (radius > kMaxNeighborRadius) {
    std::cerr << "<checkNeighborCells>: collision_neighbor_="
              << collision_neighbor_ << " is above the supported ring "
              << (1 + 8 * kMaxNeighborRadius) << "; clamping." << std::endl;
    radius = kMaxNeighborRadius;
  }
  neighbor_dx_.clear();
  neighbor_dy_.clear();
  neighbor_dx_.reserve(1 + 8 * radius);
  neighbor_dy_.reserve(1 + 8 * radius);

  neighbor_dx_.push_back(0);  // 0: center
  neighbor_dy_.push_back(0);
  for (int r = 1; r <= radius; r++) {
    neighbor_dx_.insert(neighbor_dx_.end(), {0, 0, r, -r, r, r, -r, -r});
    neighbor_dy_.insert(neighbor_dy_.end(), {r, -r, 0, 0, r, -r, r, -r});
  }
}

// retap_core: evidence threshold N.  min_voxel_points_ == 0 keeps upstream
// semantics (a single point makes the voxel occupied); N > 0 treats a voxel
// with fewer points like free/unknown *inside checkNeighborCells only*, so a
// dust return can neither block a corridor nor support a cliff edge.
bool TerrainEvaluation::isVoxelOccupied(octomap::IgTreeNode *node) const {
  if (!node || !octree_ptr_->isNodeOccupied(node)) return false;
  if (min_voxel_points_ <= 0) return true;
  return node->pointCount() >= min_voxel_points_;
}

void TerrainEvaluation::resetTerrainRepresentation(
    std::shared_ptr<octomap::IgTree> tree) {
  octree_ptr_ = tree;

  // reset DisjointSet and Kdtree
  std::lock_guard<std::mutex> lock(kdtree_mutex_);
  drivable_disjoint_root_valid_ = false;
  kdtree_initialized_ = false;
  drivable_disjoint_set_.reset(
      new DisjointSet<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash>);
  kdtree_ptr_.reset(new KD_TREE<PointType>(0.3, 0.6, 0.2));
}

void TerrainEvaluation::terrainInit(Eigen::Isometry3d &current_pose) {
  std::cout << "<terrainInit>: init position:"
            << current_pose.translation().transpose() << std::endl;
  // 根据lidar_frame高度，生成邻域高密度的点云，然后插入
  float res = octree_ptr_->getResolution();
  if (filling_radius_ < res) {
    std::cerr << "<terrainInit>: filling_radius(" << filling_radius_
              << ") < res(" << res << ")" << std::endl;
    return;
  }

  float z = current_pose.translation().z() + frame_height_;
  if (fabs(current_pose.translation().x()) > 0.3 ||
      fabs(current_pose.translation().y()) > 0.3 || fabs(z) > 0.3) {
    std::cerr << "<terrainInit>: dis=("
              << fabs(current_pose.translation().x()) << ","
              << fabs(current_pose.translation().y()) << "," << fabs(z)
              << "), far away from origin. jump filling terrain" << std::endl;

    return;  // 不在原点就不填充
  }

  // 每个格子里要填充10点以上
  int times = 10;  // 0.2/10 = 0.02
  octomap::KeySet update_all_voxels;
  octomap::point3d update_center(0, 0, 0);
  int ri = filling_radius_ / res * times + 1;
  for (int i = -ri; i < ri; i++) {
    for (int j = -ri; j < ri; j++) {
      float x = i * res / times;
      float y = j * res / times;

      octomap::point3d p3d(x, y, z);
      // 先更新全局
      auto key = octree_ptr_->coordToKey(p3d);
      octree_ptr_->updateNode(key, true);
      auto node = octree_ptr_->search(key);
      node->insertPoint(p3d);
      if (update_all_voxels.count(key) == 0) {  // 更新的全局地图栅格去重
        update_all_voxels.insert(key);
      }
      // retap_core: the local (0.025 m) tree is not ported.
    }
  }

  updateMapTerrainAttribute(update_all_voxels, &update_center);
}


bool getUpperVoxels(const octomap::IgTree &tree,
                    std::vector<octomap::IgTreeNode *> &nodes_in,   // NOLINT
                    std::vector<octomap::OcTreeKey> &keys_in,       // NOLINT
                    std::vector<octomap::IgTreeNode *> &nodes_out,  // NOLINT
                    std::vector<octomap::OcTreeKey> &keys_out,      // NOLINT
                    std::vector<octomap::IgTreeNode *> &nodes_occ,  // NOLINT
                    std::vector<octomap::OcTreeKey> &keys_occ) {    // NOLINT
  std::unordered_map<octomap::OcTreeKey, octomap::key_type,
                     octomap::OcTreeKey::KeyHash>
      elevation_keys;  // [key]=z
  std::unordered_map<octomap::OcTreeKey, octomap::IgTreeNode *,
                     octomap::OcTreeKey::KeyHash>
      elevation_nodes;  // [key]=node

  keys_occ.clear();
  nodes_occ.clear();
  for (int i = 0; i < keys_in.size(); i++) {
    if (!nodes_in[i] || !tree.isNodeOccupied(nodes_in[i])) continue;
    // if (i > 8 && elevation_keys.size() <= 2) return false;
    if (i > 26 && elevation_keys.size() == 0) return false;  // 离群点
    keys_occ.push_back(keys_in[i]);
    nodes_occ.push_back(nodes_in[i]);
    auto key = keys_in[i];
    // retap_core: upstream read key.k[3]; OcTreeKey only has k[0..2], so the
    // intended z coordinate is k[2].
    octomap::key_type z = key.k[2];
    key.k[2] = 0;                     // 转成2D
    if (elevation_keys.count(key)) {  // 存在
      if (z > elevation_keys[key]) {
        elevation_keys[key] = z;
        elevation_nodes[key] = nodes_in[i];
      }
    } else {
      elevation_keys[key] = z;
      elevation_nodes[key] = nodes_in[i];
    }
  }

  nodes_out.clear();
  keys_out.clear();
  for (auto p : elevation_keys) {
    auto key = p.first;
    key.k[2] = p.second;
    keys_out.push_back(key);
    nodes_out.push_back(elevation_nodes[p.first]);
  }
  return true;
}

void TerrainEvaluation::updateVoxelTerrainAttribute(
    octomap::OcTreeKey &iter_key, octomap::IgTreeNode *iter_node,
    octomap::KeySet *update_voxels, octomap::point3d *update_position,
    octomap::KeySet *neigh_voxels) {
  // 对新bbox中的栅格，更新粗糙度、密度、斜率，以及可通行性
  if (!iter_node || !octree_ptr_->isNodeOccupied(iter_node))  // 只处理占据的
    return;  // iter_node->pointCount() == 0  // BUG:
             // 刚观测到的占据栅格，其中点数为0

  // 搜索邻域栅格
  std::vector<octomap::OcTreeKey> neigh_keys, neigh_keys_all, neigh_keys_occ;
  std::vector<octomap::IgTreeNode *> neigh_nodes, neigh_nodes_all,
      neigh_nodes_occ;
  octomap::getNeighbor125(*octree_ptr_, iter_key, &neigh_nodes_all,
                          &neigh_keys_all);
  if (!getUpperVoxels(*octree_ptr_, neigh_nodes_all, neigh_keys_all,
                      neigh_nodes, neigh_keys, neigh_nodes_occ,
                      neigh_keys_occ)) {  // 提取top层，用于地形计算
    octree_ptr_->updateNode(iter_key, false);  // 离群点滤掉
    return;
  }
  if (neigh_voxels && update_voxels) {  // 传入了neigh_voxels和update_voxels
    for (int i = 0; i < neigh_nodes_occ.size(); i++) {
      auto key = neigh_keys_occ[i];
      auto node = neigh_nodes_occ[i];
      if (neigh_voxels->find(key) == neigh_voxels->end() &&
          update_voxels->find(key) == update_voxels->end()) {
        neigh_voxels->insert(key);
      }
    }
  }

  // 用于融合NDT协方差
  Eigen::Matrix3f sigma_fused = iter_node->cov();
  Eigen::Vector3f mu_fused = iter_node->mu();
  int pt_cnt_fused = iter_node->pointCount() == 0 ? 1 : iter_node->pointCount();
  // 用于计算均值协方差
  Eigen::Matrix3f sigma_avg = Eigen::Matrix3f::Zero();
  Eigen::Vector3f mu_avg = Eigen::Vector3f::Zero();
  int pt_cnt_avg = 0;
  octomap::point3d voxel_center(0, 0, 0);
  Eigen::Vector3f center = mu_fused;
  int occ_count = 0;

  std::vector<Eigen::Vector3f> occ_points;
  std::vector<octomap::IgTreeNode *> occ_nodes;
  Eigen::MatrixXf A(neigh_nodes.size(), 3);
  float max_height = -1e3;
  float min_height = 1e3;
  for (int k = 0; k < neigh_nodes.size(); k++) {  // 邻域内计算均值、协方差
    // if (!neigh_nodes[k] || !octree_ptr_->isNodeOccupied(neigh_nodes[k]))
    //   continue;
    // 除了不存在、非占据的，不能忽略掉其他任何栅格！！！
    octomap::fuseTwoDistributions(neigh_nodes[k], mu_fused, sigma_fused,
                                  pt_cnt_fused);  // 融合NDT
    auto pt = neigh_nodes[k]->mu();
    // 用于计算均值协方差
    octomap::point3d voxel_pt = octree_ptr_->keyToCoord(neigh_keys[k]);
    voxel_center += voxel_pt;
    A.row(pt_cnt_avg++) =
        Eigen::Vector3f(voxel_pt.x(), voxel_pt.y(), voxel_pt.z());
    // float factor = 1 / static_cast<float>(pt_cnt_avg + 1);
    // sigma_avg = pt_cnt_avg * factor * sigma_avg +
    //             pt_cnt_avg * factor * factor * (voxel - mu_avg) *
    //                 (voxel - mu_avg).transpose();
    // mu_avg = mu_avg + factor * (voxel - mu_avg);
    max_height = std::max(max_height, pt.z());
    min_height = std::min(min_height, pt.z());

    occ_points.push_back(pt);
    occ_nodes.push_back(neigh_nodes[k]);

    center += pt;
    occ_count++;
  }

  center /= occ_count;
  voxel_center /= pt_cnt_avg;

  Eigen::Vector3f normal_vector(0, 0, 0);
  octomap::EigenInfo ei;
  // 点数比较少时（少于栅格数），用栅格中心计算法向量
  if (pt_cnt_fused <= pt_cnt_avg) {
    if (pt_cnt_avg > 4) {  // 栅格数量太少，栅格中心计算也不准
      Eigen::MatrixXf A_ = A.block(0, 0, pt_cnt_avg, 3);
      for (int i = 0; i < A_.rows(); i++)
        A_.row(i) -= Eigen::Vector3f(voxel_center.x(), voxel_center.y(),
                                     voxel_center.z());
      Eigen::MatrixXf ATA = A_.transpose() * A_;
      Eigen::JacobiSVD<Eigen::MatrixXf> svd(ATA, Eigen::ComputeFullV);
      Eigen::Vector3f n = svd.matrixV().col(2);  // descreasing order
      ei.min_eigen_val = svd.singularValues()(2);
      ei.mid_eigen_val = svd.singularValues()(1);
      ei.max_eigen_val = svd.singularValues()(0);
      ei.min_eigen_vec = svd.matrixV().col(2);
      ei.mid_eigen_vec = svd.matrixV().col(1);
      ei.max_eigen_vec = svd.matrixV().col(0);
      iter_node->setFuseEigen(ei);
    } else if (pt_cnt_avg < 2) {
      octree_ptr_->updateNode(iter_key, false);
      return;
    }
    // 数量太少则不计算，其法向量为(0,0,0)
  } else {  // 计算法向量
    iter_node->setFuseMu(mu_fused);
    iter_node->setFuseSigma(sigma_fused);
    iter_node->setFuseCount(pt_cnt_fused);
    iter_node->setFuseEigen();
    ei = iter_node->getFuseEigen();
  }

  if (ei.min_eigen_vec.norm() == 0) {
    octree_ptr_->updateNode(iter_key, false);
    return;
  }

  normal_vector = ei.min_eigen_vec / ei.min_eigen_vec.norm();
  // 根据法向量计算斜率：法向量与z(0,0,1)夹角
  float angle = acos(fabs(normal_vector.dot(Eigen::Vector3f::UnitZ())));
  iter_node->setSlope(angle);
  iter_node->setHeightDiff(max_height - min_height);

  // 1.1 计算旋转矩阵：法向量至z(0,0,1)
  Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(
      normal_vector, Eigen::Vector3f::UnitZ());

  // 1.1). 旋转点云，使法向量旋转至z(0,0,1)
  for (auto &p : occ_points) {
    p = rotation * (p - iter_node->mu());
  }

  // 1.2). 2D投影，选择最高点
  std::unordered_map<octomap::OcTreeKey, std::pair<float, int>,
                     octomap::OcTreeKey::KeyHash>
      height_map;  // [key]: (height, index)
  for (int i = 0; i < occ_points.size(); i++) {
    auto p = occ_points.at(i);
    auto key = octree_ptr_->coordToKey(
        octomap::point3d(p.x(), p.y(), 0));          // 用2D key去重
    if (height_map.find(key) == height_map.end()) {  // 不存在时：
      height_map[key] = std::make_pair(p.z(), i);    // 记录高度及索引
    } else {                                         // 存在时：
      if (height_map[key].first < p.z()) {           // 更新高度及索引
        height_map[key].first = p.z();
        height_map[key].second = i;
      }
    }
  }

  // 1.3). 累计投影后的栅格数、栅格内点数
  int points_projected = 0, voxels_projected = 0;
  height_map.size();
  for (auto p : height_map) {
    int cnt = std::min(occ_nodes.at(p.second.second)->pointCount(),
                       points_number_threshold_);  // 低密度点数阈值20
    points_projected += cnt;
    voxels_projected++;
  }

  // 1.4). 计算投影后的点数/栅格比
  float density_projected =  // 归一化，栅格中点数超过阈值则不再更新
      voxels_projected == 0 ? 0
                            : points_projected * 1.0 /
                                  (voxels_projected * points_number_threshold_);
  iter_node->setSparsity(1 - density_projected);

  // 2. 根据协方差计算粗糙度
  float sum = ei.min_eigen_val + ei.max_eigen_val + ei.mid_eigen_val;
  // p1 > p2 > p3
  // float P1 = ei.max_eigen_val / sum;
  float P2 = ei.mid_eigen_val / sum;
  float P3 = ei.min_eigen_val / sum;
  // float roughness = ei.min_eigen_val / ei.max_eigen_val;
  float roughness = 1 - (P2 - P3) / (P2 + P3);
  iter_node->setRoughness(roughness);
  float traverse_cost =
      traversability_weight_ * (sparsity_weight_ * iter_node->getSparsity() +
                                slope_weight_ * iter_node->getSlope() +
                                roughness_weight_ * iter_node->getRoughness());
  iter_node->setTraversability(traverse_cost);
  if (iter_node->getSlope() == 0 && iter_node->getSparsity() == 1) {
    octree_ptr_->updateNode(iter_key, false);
  }
}

// 更新包围框内的体素, unused due to low efficiency
void TerrainEvaluation::updateMapTerrainAttribute(
    octomap::point3d update_position, octomap::Boundingbox update_bbox) {
  std::cout << "<updateTerrainAttribute>: original update_bbox = "
            << update_bbox << std::endl;
  update_bbox.enlarge(neighbor_radius_);  // 对input_pc_bbox_扩大r邻域
  std::cout << "<updateTerrainAttribute>: enlarged update_bbox = "
            << update_bbox << std::endl;

  octomap::IgTree::leaf_bbx_iterator iter_begin = octree_ptr_->begin_leafs_bbx(
      update_bbox.minPoint(), update_bbox.maxPoint());
  octomap::IgTree::leaf_bbx_iterator iter_end = octree_ptr_->end_leafs_bbx();
  for (auto it = iter_begin; it != iter_end; ++it) {  // 提取包围框中的格子
    // 对新bbox中的栅格，更新粗糙度、密度、斜率，以及可通行性
    octomap::OcTreeKey iter_key = it.getKey();
    octomap::IgTreeNode *iter_node = octree_ptr_->search(iter_key);
    updateVoxelTerrainAttribute(iter_key, iter_node, nullptr, &update_position);
  }
}

// 更新给定体素集合，入口!!!
void TerrainEvaluation::updateMapTerrainAttribute(
    octomap::KeySet &update_voxels, octomap::point3d *update_position) {
  auto t1 = std::chrono::steady_clock::now();
  int update_voxels_num = update_voxels.size();
  if (update_voxels_num <= 0) {
    std::cerr << "<updateMapTerrainAttribute>: no update_voxels !!!"
              << std::endl;
    return;
  }
  octomap::KeySet neigh_voxels;

  // ROS_WARN("<updateMapTerrainAttribute>: leaf num = %ld",
  //          octree_ptr_->getNumLeafNodes());
  // 先对当前帧的占据格子(update_voxels)，更新地形属性、提取邻居占据栅格
  for (octomap::KeySet::iterator it = update_voxels.begin();
       it != update_voxels.end(); ++it) {
    octomap::OcTreeKey iter_key = *it;
    octomap::IgTreeNode *iter_node = octree_ptr_->search(iter_key);
    updateVoxelTerrainAttribute(iter_key, iter_node, &update_voxels,
                                update_position, &neigh_voxels);
  }

  // 对当前帧占据栅格的邻居栅格(neigh_voxels)，更新地形属性
  for (octomap::KeySet::iterator it = neigh_voxels.begin();
       it != neigh_voxels.end(); ++it) {
    octomap::OcTreeKey iter_key = *it;
    octomap::IgTreeNode *iter_node = octree_ptr_->search(iter_key);
    updateVoxelTerrainAttribute(iter_key, iter_node, (octomap::KeySet *)nullptr,
                                update_position, (octomap::KeySet *)nullptr);
    update_voxels.insert(iter_key);  // 将neigh_voxels合并到update_voxels
  }

  // 根据最新的地形属性，更新栅格的碰撞，保存新出现的无碰撞栅格
  std::vector<octomap::OcTreeKey> new_drivable_set;
  std::vector<octomap::IgTreeNode *> new_drivable_nodes;
  std::vector<octomap::OcTreeKey> new_indrivable_set;
  std::vector<octomap::IgTreeNode *> new_indrivable_nodes;
  for (octomap::KeySet::iterator it = update_voxels.begin();
       it != update_voxels.end(); ++it) {
    octomap::OcTreeKey iter_key = *it;
    octomap::IgTreeNode *iter_node = octree_ptr_->search(iter_key);
    int ret = updateVoxelTerrainCollision(iter_key, iter_node);
    if (ret == 1) {
      new_drivable_set.push_back(iter_key);
      new_drivable_nodes.push_back(iter_node);
    } else if (ret == -1) {
      new_indrivable_set.push_back(iter_key);
      new_indrivable_nodes.push_back(iter_node);
    }
  }

  // 将无碰撞栅格更新到disjoint set
  updateDrivableDisjointSet(new_drivable_set, new_drivable_nodes,
                            new_indrivable_set, new_indrivable_nodes,
                            *update_position);
  printf(
      "<updateMapTerrainAttribute>: update+neigh: %d+%d=%d voxels, center: "
      "[%.1f, %.1f, %.1f], timecost = %.3fs\n",
      update_voxels_num, static_cast<int>(neigh_voxels.size()),
      static_cast<int>(update_voxels.size()), update_position->x(),
      update_position->y(), update_position->z(),
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t1)
          .count());
}

// 满足约束返回true
bool TerrainEvaluation::checkNodeConstraints(octomap::IgTreeNode *node,
                                             bool debug_msg) {
  if (debug_msg) {
    printf(
        "<checkNodeConstraints>: slope:%.3f/%.3f(<), densty:%.3f/%.3f(<), "
        "roughness:%.3f/%.3f(<), traverseCost:%.3f/%.3f(<)\n",
        node->getSlope(), slope_threshold_, node->getSparsity(),
        sparsity_threshold_, node->getRoughness(), roughness_threshold_,
        node->getTraversability(), traversability_threshold_);
  }
  if (node->getSlope() > slope_threshold_ ||
      node->getSparsity() > sparsity_threshold_ ||
      node->getRoughness() > roughness_threshold_ ||
      node->getTraversability() > traversability_threshold_)  // 硬约束
    return false;
  return true;
}

// 无碰撞返回true
bool TerrainEvaluation::checkNeighborCells(octomap::OcTreeKey key,
                                           bool debug_msg) {
  // retap_core: the horizontal ring and the vertical extent are parameters.
  // With the defaults (collision_neighbor 17, clearance 3, support 3) the
  // offsets, the visiting order and every rule below are identical to
  // upstream, so existing calibrations keep reproducing bit for bit.
  //
  // Horizontal: neighbor_dx_/neighbor_dy_, 1 + 8R entries, Chebyshev radius R.
  //   9 -> R=1 (3x3, 0.30 m at 0.20 m), 17 -> R=2 (5x5), 25 -> R=3,
  //   49 -> R=6 (13x13, 0.325 m at 0.05 m).
  // Vertical: z = +clearance_voxels .. -support_voxels.
  //   z == +clearance_voxels : occupied => hard obstruction (stop)
  //   z == 1 .. clearance-1  : occupied => must pass checkNodeConstraints
  //   z == 0                 : the voxel itself, same constraint test
  //   z == -2, 1 <= i <= 8   : ring 1 must be occupied (support)
  //   z == -3, i >= 8        : ring 2+ must be occupied (cliff)
  // The downward reach is 4 layers counted from the voxel itself
  // (z = 0, -1, -2, -3), which is what support_voxels = 3 means.
  const int neighbor_count = static_cast<int>(neighbor_dx_.size());
  std::array<uint8_t, kMaxNeighborCells> flag{};

  bool stop = false;
  pcl::PointCloud<pcl::PointXYZI> neigh_points;
  for (int z = clearance_voxels_; z >= -support_voxels_ && !stop; z--) {
    for (int i = 0; i < neighbor_count && !stop; i++) {
      if (flag[i]) continue;  // 该位置向下投射有了占据点则跳过
      octomap::OcTreeKey tmp = key;
      tmp.k[0] += neighbor_dx_[i];
      tmp.k[1] += neighbor_dy_[i];
      tmp.k[2] += z;

      pcl::PointXYZI pc;
      pc.intensity = -4;  // 正常为紫色，其他为高度渐变颜色
      octomap::IgTreeNode *node = octree_ptr_->search(tmp);
      if (isVoxelOccupied(node)) {
        flag[i] = true;
        if (z == clearance_voxels_) {  // 最高层不能有占据
          pc.intensity = z;
          stop = true;
        } else if (!checkNodeConstraints(node)) {  // 其它层的占据必须为可通行
          pc.intensity = 4;                        // 不满足约束为红色
          stop = true;
        }
      } else {  // free or unknown（N > 0 时也包括点数不足 N 的体素）
        if (z == -2 && 1 <= i && i <= 8) {  // 1邻域，-2层不能有free
          pc.intensity = z;
          stop = true;
        }
        if (z == -3 && 8 <= i) {  // 2邻域，-3层不能有free
          // retap_core: upstream wrote `i >= 8` while ring 1 ends at index 8,
          // so one ring-1 corner is included.  Kept as-is on purpose: the
          // default configuration must stay bit-identical to upstream.
          pc.intensity = z;
          stop = true;
        }
      }
      // retap_core: upstream built this cloud unconditionally and then
      // dropped it; it is only read on the (debug) path.
      if (debug_msg) {
        auto p3d = octree_ptr_->keyToCoord(tmp);
        pc.x = p3d.x();
        pc.y = p3d.y();
        pc.z = p3d.z();
        neigh_points.push_back(pc);
      }
    }
  }
  if (debug_msg) {
    // retap_core: upstream published neigh_points on a ROS topic here.
    (void)neigh_points;
  }
  return !stop;
}

bool TerrainEvaluation::checkDrivable(octomap::OcTreeKey key) {
  if (!drivable_disjoint_root_valid_) return false;
  if (!drivable_disjoint_set_->count(key) ||
      drivable_disjoint_set_->find(key) != drivable_disjoint_root_) {
    return false;
  }
  return true;
}

void TerrainEvaluation::updateDrivableDisjointSet(
    std::vector<octomap::OcTreeKey> &update_voxels,
    std::vector<octomap::IgTreeNode *> &update_nodes,
    std::vector<octomap::OcTreeKey> &revoke_voxels,
    std::vector<octomap::IgTreeNode *> &revoke_nodes,
    octomap::point3d update_position) {
  std::lock_guard<std::mutex> lock(kdtree_mutex_);
  // 找到最近邻的key作为root
  if (!drivable_disjoint_root_valid_) {
    float min_dis = 1e9;
    for (auto iter_key : update_voxels) {
      octomap::point3d dis =
          octree_ptr_->keyToCoord(iter_key) - update_position;
      if (dis.norm() < min_dis) {
        min_dis = dis.norm();
        drivable_disjoint_root_ = iter_key;
        drivable_disjoint_root_valid_ = true;
      }
    }
  }

  // 先插入disjoint set
  for (auto iter_key : update_voxels) {
    if (!drivable_disjoint_set_->count(iter_key))
      drivable_disjoint_set_->insert(iter_key);
  }

  // 再与邻域合并
  for (auto iter_key : update_voxels) {
    std::vector<octomap::OcTreeKey> neigh_keys;
    octomap::getNeighborSix(*octree_ptr_, iter_key,
                            (std::vector<octomap::IgTreeNode *> *)nullptr,
                            &neigh_keys);
    // octomap::getNeighbor27(*octree_ptr_, iter_key,
    //                         (std::vector<octomap::IgTreeNode *> *)nullptr,
    //                         &neigh_keys);
    for (auto &key : neigh_keys) {
      if (drivable_disjoint_set_->count(key))
        drivable_disjoint_set_->merge(iter_key, key);
    }
  }

  // 更新root
  if (drivable_disjoint_root_valid_)
    drivable_disjoint_root_ =
        drivable_disjoint_set_->find(drivable_disjoint_root_);

  // 再遍历一遍，将连通的体素，setConnected
  // 通过连通条件筛选可通行体素，避免房顶等孤立的可通行区域
  PointVector add_points, del_points;
  int iter_key_cnt = 0;
  for (auto iter_key : update_voxels) {
    if (drivable_disjoint_set_->find(iter_key) == drivable_disjoint_root_) {
      update_nodes.at(iter_key_cnt)->setConnected(true);  // 可通行
      auto p = octree_ptr_->keyToCoord(iter_key);
      add_points.push_back(PointType(p.x(), p.y(), p.z()));
    }
    iter_key_cnt++;
  }
  iter_key_cnt = 0;
  for (auto iter_key : revoke_voxels) {
    revoke_nodes.at(iter_key_cnt)->setConnected(false);  // 不可通行
    auto p = octree_ptr_->keyToCoord(iter_key);
    del_points.push_back(PointType(p.x(), p.y(), p.z()));
    iter_key_cnt++;
  }

  // 更新ikdtree
  if (!kdtree_initialized_) {
    if (add_points.size() > 0) {
      kdtree_ptr_->Build(add_points);  // 第一次必须build，后面才能add
      kdtree_initialized_ = true;
    }
  } else if (add_points.size() > 0) {
    kdtree_ptr_->Add_Points(add_points, true);
  }

  if (del_points.size() > 0) {
    kdtree_ptr_->Delete_Points(del_points);
  }

  // retap_core: upstream regenerated the nav_msgs costmap here when
  // costmap_enable_ was set; visualization now goes through FineNav.
}

// 1: add, -1: delete, 0: dont care
int TerrainEvaluation::updateVoxelTerrainCollision(
    octomap::OcTreeKey iter_key, octomap::IgTreeNode *iter_node) {
  if (!iter_node || !octree_ptr_->isNodeOccupied(iter_node))
    return 0;  // unknown, free的直接跳过

  // bool old_val = iter_node->getCollision();
  if (checkNodeConstraints(iter_node) && checkNeighborCells(iter_key)) {
    iter_node->setCollision(false);  // 无碰撞，绿色
  } else {
    iter_node->setCollision(true);  // 碰撞，红色
  }

  // bool new_val = iter_node->getCollision();
  // if (old_val != new_val)
  // {  // 发生变化的，加上会导致部分不更新!!!!!
  //   if (new_val == 0.5) {  // 由未知/不可通行，变为可通行
  //     return 1;
  //   }
  //   // if (new_val == 1 && old_val == 0.5) {  // 由可通行，变为不可通行
  //   if (new_val == 1) {  // 由未知/可通行，变为不可通行
  //     return -1;
  //   }
  // }

  return iter_node->getCollision() ? -1 : 1;
}

// for global planning to query the nearest point to the goal.
// retap_core: body of queryDrivablePointServerCallback; the ROS service
// request/response is replaced by plain arguments.
bool TerrainEvaluation::queryNearestDrivable(
    const octomap::point3d &query_point, octomap::point3d *result_point,
    bool *in_disjoint_set) {
  std::lock_guard<std::mutex> lock(kdtree_mutex_);
  if (!kdtree_initialized_ || !drivable_disjoint_root_valid_) {
    std::cerr << "<TerrainEvaluation::queryNearestDrivable>: ikd-tree "
                 "or disjoint set NOT initialized!"
              << std::endl;
    return false;
  }

  auto p = query_point;
  auto key = octree_ptr_->coordToKey(p);
  // 先判断是否在disjoint set中，标记 success
  bool success = true;
  if (!checkDrivable(key)) {
    success = false;
  }
  if (in_disjoint_set) *in_disjoint_set = success;

  // 在disjoint set中，则直接返回coord，并且success为true
  if (success == true) {
    auto p3d = octree_ptr_->keyToCoord(key);
    if (result_point) *result_point = p3d;
    printf(
        "<TerrainEvaluation::queryNearestDrivable>: disjoint_set"
        " query [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f].\n",
        p.x(), p.y(), p.z(), p3d.x(), p3d.y(), p3d.z());
    return true;
  }

  // 不在disjoint set中，success为false，查询kdtree
  PointVector Nearest_Points;
  std::vector<float> Point_Distance;
  kdtree_ptr_->Nearest_Search(PointType(p.x(), p.y(), p.z()), 1,
                              Nearest_Points, Point_Distance);
  if (Nearest_Points.size() <= 0) {
    std::cerr << "<TerrainEvaluation::queryNearestDrivable>: ikd-tree "
                 "get no result for ["
              << p.x() << ", " << p.y() << ", " << p.z() << "]." << std::endl;
    return false;
  }

  octomap::point3d pr(Nearest_Points[0].x, Nearest_Points[0].y,
                      Nearest_Points[0].z);
  if (result_point) *result_point = pr;
  printf(
      "<TerrainEvaluation::queryNearestDrivable>: ikd-tree "
      "query [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f].\n",
      p.x(), p.y(), p.z(), pr.x(), pr.y(), pr.z());
  return true;
}

}  // namespace mapping_module
