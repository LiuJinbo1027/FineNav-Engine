// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// RetapTerrainMap implementation.  See the header for the role split.

#include "finenav_retap/retap_terrain_map.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <utility>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "finenav_core/algo_register_macros.hpp"
#include "finenav_visualizer/color.hpp"
#include "finenav_visualizer/point_cloud_viz.hpp"
#include "retap_core/map_io.h"

namespace finenav::retap {
namespace {

// Absolute names on purpose: the visualizer is built on the engine node, so a
// relative name would be namespaced under the engine's node name.  The plan's
// RViz configuration and the S3 acceptance criteria name these topics exactly.
constexpr char kTraversabilityTopic[] = "/retap/map/traversability";
constexpr char kCollisionTopic[] = "/retap/map/collision";
constexpr char kDrivableTopic[] = "/retap/map/drivable";

// Diagnostic republish throttle on the online path: the tree can grow every
// update cycle, but rebuilding three full point clouds at 10 Hz is wasted work.
constexpr double kDiagnosticPeriodSec = 1.0;

}  // namespace

// ── configuration ──────────────────────────────────────────────────────────

void RetapTerrainMap::configure(const ConfigType& cfg) {
    // TerrainConfig is the semantic source (retap_core/terrain_types.h);
    // every field is copied one for one so a calibration done with
    // retap_map_tool's terrain_sim.yaml transfers verbatim.
    retap_core::TerrainConfig terrain;
    terrain.roughness_threshold = static_cast<float>(cfg.roughness_threshold);
    terrain.sparsity_threshold = static_cast<float>(cfg.sparsity_threshold);
    terrain.slope_threshold = static_cast<float>(cfg.slope_threshold);
    terrain.roughness_weight = static_cast<float>(cfg.roughness_weight);
    terrain.sparsity_weight = static_cast<float>(cfg.sparsity_weight);
    terrain.slope_weight = static_cast<float>(cfg.slope_weight);
    terrain.traversability_weight = static_cast<float>(cfg.traversability_weight);
    terrain.points_number_threshold = static_cast<int>(cfg.points_number_threshold);
    terrain.traversability_threshold = static_cast<float>(cfg.traversability_threshold);
    terrain.collision_neighbor = static_cast<float>(cfg.collision_neighbor);
    terrain.frame_height = static_cast<float>(cfg.frame_height);
    terrain.filling_radius = static_cast<float>(cfg.filling_radius);
    terrain.update_free = static_cast<int>(cfg.update_free);
    terrain.terrain_update_period_sec = cfg.terrain_update_period_sec;
    terrain.terrain_update_batch = static_cast<int>(cfg.terrain_update_batch);
    terrain_cfg_ = terrain;

    map_format_ = cfg.map_format;
    map_path_ = cfg.map_path;
    resolution_ = cfg.resolution;
    publish_diagnostics_ = cfg.publish_diagnostics;
    downsample_diagnostics_ = std::max(1, static_cast<int>(cfg.downsample_diagnostics));
    diagnostics_period_sec_ = cfg.diagnostics_period_sec;
    online_mapping_ = cfg.online_mapping;
    scan_topic_ = cfg.scan_topic;
    scan_in_map_frame_ = cfg.scan_in_map_frame;
    update_free_ = static_cast<int>(cfg.update_free);
    max_range_ = cfg.max_range;
    origin_offset_ = Eigen::Vector3f(static_cast<float>(cfg.origin_offset_x),
                                     static_cast<float>(cfg.origin_offset_y),
                                     static_cast<float>(cfg.origin_offset_z));
    max_buffer_size_ = static_cast<int>(cfg.max_buffer_size);
    max_scans_per_cycle_ = static_cast<int>(cfg.max_scans_per_cycle);
    terrain_update_batch_ = static_cast<int>(cfg.terrain_update_batch);
    terrain_update_period_sec_ = cfg.terrain_update_period_sec;

    if (map_format_ != "bt" && map_format_ != "pcd") {
        RCLCPP_ERROR(logger_, "RetapTerrainMap: map_format '%s' is not 'bt' or 'pcd'; "
                              "the static map will not be loaded.",
                     map_format_.c_str());
    }
    if (update_free_ < 0 || update_free_ > 2) {
        RCLCPP_WARN(logger_, "RetapTerrainMap: update_free=%d is outside [0, 2]; "
                             "clamping to 2 (= ReTAP's online global-map policy).",
                    update_free_);
        update_free_ = 2;
    }
}

// ── lifecycle ──────────────────────────────────────────────────────────────

RetapTerrainMap::~RetapTerrainMap() {
    diagnostics_stop_.store(true);
    if (diagnostic_thread_.joinable()) diagnostic_thread_.join();
}

void RetapTerrainMap::selfManage(finenav::core::MapRuntime& runtime) {
    // MapServer runs selfManage() exactly once; the guard makes that explicit
    // in case a future server calls it again on a parameter update.
    if (loaded_) return;
    loaded_ = true;

    if (!map_path_.empty()) {
        loadStaticMap();
    } else {
        RCLCPP_INFO(logger_, "RetapTerrainMap: no map_path — starting empty%s.",
                    online_mapping_ ? " (online mapping enabled)" : "");
    }

    // Even without a prior map the read-only facade must be valid: the
    // WorldView and the Planner call terrain() unconditionally, and a null
    // grid there would be undefined behaviour rather than a clean failure.
    // An empty tree simply has no drivable voxel, so ready() stays false.
    if (model_ == nullptr) {
        model_.reset(new retap_core::TerrainModel(resolution_, terrain_cfg_));
        grid_.reset(new retap_core::TerrainGrid(model_->tree()));
        computeBounds();
        if (online_mapping_) {
            RCLCPP_INFO(logger_,
                        "RetapTerrainMap: online mapping starts from an empty tree.");
        }
    }

    if (publish_diagnostics_ && ready_) {
        publishDiagnostics();
        // The visualizer publishes volatile QoS, so a one-shot at load is lost
        // for any RViz started afterwards.  A small republish thread keeps the
        // terrain visible without making the MapServer's update cycle (which
        // needs a RobotState) responsible for it.
        const double period_sec = diagnostics_period_sec_;
        if (period_sec > 0.0) {
            diagnostic_thread_ = std::thread([this, period_sec]() {
                const auto period = std::chrono::duration<double>(period_sec);
                auto next = std::chrono::steady_clock::now() + period;
                while (!diagnostics_stop_.load()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    const auto now = std::chrono::steady_clock::now();
                    if (now < next) continue;
                    next = now + period;
                    restageDiagnostics();
                }
            });
        }
    }

    // ReTAP roots connectivity (and therefore the nearest-drivable index) at
    // the robot, so remember the pose from the first update cycle on, whether
    // or not online mapping is on.  Registered before the scan hook below so
    // the index exists before that cycle's scans are folded in.
    runtime.registerPostUpdateHook(
        [this](const finenav::core::RobotState& state) { rememberRobotPose(state); });

    if (online_mapping_) registerOnlineMapping(runtime);
}

void RetapTerrainMap::loadStaticMap() {
    std::string error;
    if (map_format_ == "pcd") {
        retap_core::TerrainBuildResult result;
        std::unique_ptr<retap_core::TerrainModel> model =
            retap_core::buildTerrainFromPcd(map_path_, resolution_, terrain_cfg_,
                                            &result, &error);
        if (model == nullptr) {
            RCLCPP_ERROR(logger_, "RetapTerrainMap: failed to build terrain from '%s': %s",
                         map_path_.c_str(), error.c_str());
            return;
        }
        RCLCPP_INFO(logger_,
                    "RetapTerrainMap: PCD '%s' -> %zu occupied, %zu drivable "
                    "(load %.0f ms, insert %.0f ms, analysis %.0f ms)",
                    map_path_.c_str(), result.occupied_voxels, result.drivable_voxels,
                    result.load_ms, result.insert_ms, result.analysis_ms);
        model_ = std::move(model);
    } else if (map_format_ == "bt") {
        std::unique_ptr<octomap::IgTree> tree = retap_core::loadTerrainBt(map_path_, &error);
        if (tree == nullptr) {
            RCLCPP_ERROR(logger_, "RetapTerrainMap: failed to load '%s': %s",
                         map_path_.c_str(), error.c_str());
            return;
        }
        // The .bt carries its analysis: adopt it instead of re-running the
        // terrain pass (which would need raw NDT statistics that the file does
        // not store — see map_io.h).
        model_.reset(new retap_core::TerrainModel(
            std::shared_ptr<octomap::IgTree>(std::move(tree)), terrain_cfg_));
        RCLCPP_INFO(logger_, "RetapTerrainMap: loaded '%s' at %.2f m resolution",
                    map_path_.c_str(), model_->resolution());
    } else {
        return;  // configure() already reported the bad format
    }

    resolution_ = model_->resolution();
    grid_.reset(new retap_core::TerrainGrid(model_->tree()));
    computeBounds();
    validateLoadedMap();
}

void RetapTerrainMap::validateLoadedMap() {
    const retap_core::TerrainStats stats = retap_core::computeStats(model_->tree());
    occupied_voxels_ = stats.occupied_voxels;
    drivable_voxels_ = stats.drivable_voxels;
    ready_ = drivable_voxels_ > 0;

    if (!ready_) {
        // IgTreeNode::collision_ defaults to 0 and getCollision() is
        // collision_ != 0.5, so EVERY un-analysed node reports blocked.  A map
        // with no drivable voxel is either un-analysed or entirely untraversable;
        // either way planning on it would silently fail.
        RCLCPP_ERROR(logger_,
                     "RetapTerrainMap: '%s' has %zu occupied voxels but ZERO drivable "
                     "ones — was it terrain-analysed (retap_map_tool analyze)? "
                     "Refusing to report ready.",
                     map_path_.c_str(), occupied_voxels_);
        return;
    }
    RCLCPP_INFO(logger_, "RetapTerrainMap ready: %zu occupied, %zu drivable voxels "
                         "(res %.2f m, bbox [%.1f, %.1f, %.1f]..[%.1f, %.1f, %.1f])",
                occupied_voxels_, drivable_voxels_, resolution_,
                bounds_.min.x, bounds_.min.y, bounds_.min.z,
                bounds_.max.x, bounds_.max.y, bounds_.max.z);
}

void RetapTerrainMap::computeBounds() {
    const double half = model_->resolution() * 0.5;
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = min_x, min_z = min_x;
    double max_x = -min_x, max_y = -min_x, max_z = -min_x;
    bool any = false;
    octomap::IgTree& tree = model_->tree();
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
        if (!tree.isNodeOccupied(&*it)) continue;
        const octomap::point3d p = tree.keyToCoord(it.getKey());
        const double px = static_cast<double>(p.x());
        const double py = static_cast<double>(p.y());
        const double pz = static_cast<double>(p.z());
        min_x = std::min(min_x, px);
        min_y = std::min(min_y, py);
        min_z = std::min(min_z, pz);
        max_x = std::max(max_x, px);
        max_y = std::max(max_y, py);
        max_z = std::max(max_z, pz);
        any = true;
    }
    if (!any) {
        bounds_ = finenav::common::Region3D{};
        return;
    }
    bounds_.min = {min_x - half, min_y - half, min_z - half};
    bounds_.max = {max_x + half, max_y + half, max_z + half};
}

// ── queries ────────────────────────────────────────────────────────────────

bool RetapTerrainMap::nearestDrivable(const finenav::common::Position3D& q,
                                      finenav::common::Position3D& out,
                                      double max_radius) const {
    if (model_ == nullptr) return false;

    const Eigen::Vector3f query(static_cast<float>(q.x), static_cast<float>(q.y),
                                static_cast<float>(q.z));
    // Build the disjoint set + ikd-tree on first use.  No drivable voxel yet
    // (empty map, or online mapping before the first scan) -> no answer.
    if (!ensureNearestIndex(query)) return false;

    // queryNearestDrivable() only reads the disjoint set and the ikd-tree, and
    // the evaluator's const-ness was not preserved by the port, hence the cast.
    retap_core::TerrainModel& model = const_cast<retap_core::TerrainModel&>(*model_);
    octomap::point3d snapped;
    if (!model.evaluator().queryNearestDrivable(octomap::point3d(q.x, q.y, q.z),
                                                &snapped)) {
        return false;
    }

    out = {snapped.x(), snapped.y(), snapped.z()};

    // max_radius is a warning threshold, not a filter: ReTAP never rejects a
    // snap because of distance (a 2-D goal arrives with z = 0 while the
    // surface can be metres above it).  Throttled so a periodic planner does
    // not fill the log.
    if (max_radius > 0.0) {
        const double dx = out.x - q.x, dy = out.y - q.y, dz = out.z - q.z;
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance > max_radius) {
            const auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(index_mutex_);
            if (last_snap_warn_.time_since_epoch().count() == 0 ||
                std::chrono::duration<double>(now - last_snap_warn_).count() >= 2.0) {
                last_snap_warn_ = now;
                RCLCPP_WARN(logger_,
                            "RetapTerrainMap: snapped (%.2f, %.2f, %.2f) -> "
                            "(%.2f, %.2f, %.2f), %.2f m away (snap_radius %.2f is a "
                            "warning threshold only)",
                            q.x, q.y, q.z, out.x, out.y, out.z, distance, max_radius);
            }
        }
    }
    return true;
}

bool RetapTerrainMap::ensureNearestIndex(const Eigen::Vector3f& fallback_root) const {
    std::lock_guard<std::mutex> lock(index_mutex_);
    if (nearest_index_ready_) return true;
    if (model_ == nullptr) return false;

    retap_core::TerrainModel& model = const_cast<retap_core::TerrainModel&>(*model_);
    // The PCD path already built the index inside evaluateAll(); rebuilding it
    // would only duplicate ikd-tree points.
    if (model.evaluator().hasDrivableIndex()) {
        nearest_index_ready_ = true;
        return true;
    }

    // A .bt load leaves the index empty, so seed it from every occupied,
    // collision-free leaf.  The tree is only read here; the cast is the same
    // formality as in publishDiagnostics().
    octomap::IgTree& tree = model.tree();
    std::vector<octomap::OcTreeKey> keys;
    std::vector<octomap::IgTreeNode*> nodes;
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
        if (!tree.isNodeOccupied(&*it) || it->getCollision()) continue;
        keys.push_back(it.getKey());
        nodes.push_back(&*it);
    }
    if (keys.empty()) return false;  // nothing to index yet; retry later

    const Eigen::Vector3f root = robot_position_valid_ ? robot_position_ : fallback_root;
    std::vector<octomap::OcTreeKey> no_revoke_keys;
    std::vector<octomap::IgTreeNode*> no_revoke_nodes;
    const auto start = std::chrono::steady_clock::now();
    // Same call ReTAP's terrain init makes on a freshly built map: root at the
    // drivable voxel nearest the robot, 6-connected merges, and only the root
    // component enters the ikd-tree (upstream: keep isolated roof tops out).
    model.evaluator().updateDrivableDisjointSet(
        keys, nodes, no_revoke_keys, no_revoke_nodes,
        octomap::point3d(root.x(), root.y(), root.z()));
    nearest_index_ready_ = true;
    const double cost_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
            .count();
    RCLCPP_INFO(logger_,
                "RetapTerrainMap: nearest-drivable index built from %zu drivable "
                "voxels around root (%.2f, %.2f, %.2f)%s in %.0f ms",
                keys.size(), root.x(), root.y(), root.z(),
                robot_position_valid_ ? "" : " [query point, no robot pose yet]", cost_ms);
    return true;
}

void RetapTerrainMap::rememberRobotPose(const finenav::core::RobotState& state) {
    const Eigen::Vector3f position(static_cast<float>(state.pose.position.x),
                                   static_cast<float>(state.pose.position.y),
                                   static_cast<float>(state.pose.position.z));
    {
        std::lock_guard<std::mutex> lock(index_mutex_);
        robot_position_ = position;
        robot_position_valid_ = true;
    }
    // One-shot: the index is built on the first cycle that has a pose, so its
    // root is the robot rather than a query point.  A no-op afterwards.
    ensureNearestIndex(position);
}

// ── diagnostics ────────────────────────────────────────────────────────────

void RetapTerrainMap::publishDiagnostics() {
    if (model_ == nullptr) return;

    std::vector<DiagnosticPoint> points;
    points.reserve(occupied_voxels_);
    size_t drivable_count = 0;

    octomap::IgTree& tree = model_->tree();
    size_t visited = 0;
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
        if (!tree.isNodeOccupied(&*it)) continue;
        if ((static_cast<int>(visited) % downsample_diagnostics_) != 0) {
            ++visited;
            continue;
        }
        ++visited;
        const octomap::point3d p = tree.keyToCoord(it.getKey());
        DiagnosticPoint point;
        point.x = static_cast<float>(p.x());
        point.y = static_cast<float>(p.y());
        point.z = static_cast<float>(p.z());
        point.value = it->getTraversability();
        point.drivable = !it->getCollision();
        if (point.drivable) ++drivable_count;
        points.push_back(point);
    }

    const size_t cached = points.size();
    {
        std::lock_guard<std::mutex> lock(diag_mutex_);
        diagnostic_points_ = std::move(points);
        cached_drivable_ = drivable_count;
    }
    last_published_generation_ = model_->generation();
    restageDiagnostics();
    RCLCPP_DEBUG(logger_, "RetapTerrainMap: cached %zu diagnostic voxels", cached);
}

void RetapTerrainMap::restageDiagnostics() {
    auto& traversability = viz_.pointCloud(kTraversabilityTopic);
    auto& collision = viz_.pointCloud(kCollisionTopic);
    auto& drivable = viz_.pointCloud(kDrivableTopic);

    {
        std::lock_guard<std::mutex> lock(diag_mutex_);
        if (diagnostic_points_.empty()) return;
        traversability.reserve(diagnostic_points_.size());
        collision.reserve(diagnostic_points_.size() - cached_drivable_);
        drivable.reserve(cached_drivable_);

        // ReTAP's traversability score is a cost-like measure: higher is worse.
        // Values above the hard threshold are collision anyway, so the scale
        // saturates there and the colour stays interpretable.
        const finenav::viz::ColorScale scale(
            0.0, std::max(1e-3, static_cast<double>(terrain_cfg_.traversability_threshold)),
            finenav::viz::ColorMap::TURBO);

        for (const DiagnosticPoint& p : diagnostic_points_) {
            // Every occupied voxel carries a score; collision/drivable is the
            // binary split on top of it.
            traversability.addScalarPoint(p.x, p.y, p.z, p.value, scale);
            if (p.drivable) {
                drivable.addPoint(p.x, p.y, p.z, finenav::viz::ColorName::GREEN);
            } else {
                collision.addPoint(p.x, p.y, p.z, finenav::viz::ColorName::RED);
            }
        }
    }

    viz_.publish(traversability);
    viz_.publish(collision);
    viz_.publish(drivable);
    last_publish_ = std::chrono::steady_clock::now();
}

// ── online incremental path (S5) ───────────────────────────────────────────

void RetapTerrainMap::registerOnlineMapping(finenav::core::MapRuntime& runtime) {
    if (!scan_in_map_frame_) {
        RCLCPP_ERROR(logger_,
                     "RetapTerrainMap: scan_in_map_frame=false needs a TF lookup, which "
                     "this build does not wire; online scans will be dropped.  "
                     "Republish the cloud in the map frame (e.g. /cloud_registered).");
    }

    finenav::core::ObservationBufferPolicy policy;
    policy.max_buffer_size = max_buffer_size_;  // ReTAP: global_map/max_buffer_size = 2
    policy.observation_keep_time_sec = 0.0;     // ReTAP consumed every frame
    policy.source_timeout_sec = 0.0;

    runtime.addObservationSource<sensor_msgs::msg::PointCloud2>(
        "lidar", scan_topic_, rclcpp::SensorDataQoS(), policy,
        [this](const sensor_msgs::msg::PointCloud2& msg, const rclcpp::Time&,
               const finenav::core::RobotState& state) {
            // Copy into our own queue; the tree is only touched from the
            // post-update hook, under the MapServer write lock.  `state` is
            // the pose interpolated at this cloud's header.stamp, so the ray
            // origin is fixed here instead of at flush time.
            enqueueScan(msg, sensorOrigin(state));
        });

    runtime.registerPostUpdateHook(
        [this](const finenav::core::RobotState& state) { processQueuedScans(state); });

    RCLCPP_INFO(logger_,
                "RetapTerrainMap: online mapping on '%s' (update_free=%d, batch>=%d or "
                "%.2f s, max %d scans/cycle)",
                scan_topic_.c_str(), update_free_, terrain_update_batch_,
                terrain_update_period_sec_, max_scans_per_cycle_);
}

Eigen::Vector3f RetapTerrainMap::sensorOrigin(
    const finenav::core::RobotState& state) const {
    // Ray casting starts at the sensor, not base_link, so the mount offset is
    // applied in the map frame (ReTAP: getSensorPoseTF(lidar_frame_)).  Only
    // yaw is applied — the mount's pitch/roll move the origin by millimetres.
    const auto& q = state.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Eigen::Vector3f(
        static_cast<float>(state.pose.position.x) +
            static_cast<float>(c * origin_offset_.x() - s * origin_offset_.y()),
        static_cast<float>(state.pose.position.y) +
            static_cast<float>(s * origin_offset_.x() + c * origin_offset_.y()),
        static_cast<float>(state.pose.position.z) + origin_offset_.z());
}

void RetapTerrainMap::enqueueScan(const sensor_msgs::msg::PointCloud2& msg,
                                  const Eigen::Vector3f& origin) {
    // Every delivered cloud ends up either processed or dropped, so the
    // counters at the bottom of this file always balance.
    scans_received_.fetch_add(1, std::memory_order_relaxed);

    if (!scan_in_map_frame_) {  // registerOnlineMapping() already warned
        scans_dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    std::vector<Eigen::Vector3f> points;
    points.reserve(static_cast<size_t>(msg.width) * msg.height);
    try {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) ||
                !std::isfinite(*iter_z)) {
                continue;
            }
            points.emplace_back(*iter_x, *iter_y, *iter_z);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN_ONCE(logger_, "RetapTerrainMap: cannot read xyz from '%s': %s",
                         scan_topic_.c_str(), e.what());
        scans_dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    if (points.empty()) {
        scans_dropped_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Bounded queue: the cycle consumes at most max_scans_per_cycle_ per
    // flush, so a queue deeper than that only grows if the MapServer's update
    // loop cannot keep up.  Drop the OLDEST scan in that case (ReTAP's
    // global_cloud_list_ does the same at max_buffer_size) and count it.
    const size_t queue_cap =
        std::max<size_t>(1, static_cast<size_t>(max_buffer_size_)) *
        std::max<size_t>(1, static_cast<size_t>(max_scans_per_cycle_));

    bool dropped_oldest = false;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        scan_queue_.push_back(QueuedScan{std::move(points), origin});
        while (scan_queue_.size() > queue_cap) {
            scan_queue_.pop_front();
            dropped_oldest = true;
        }
    }
    if (dropped_oldest) {
        scans_dropped_.fetch_add(1, std::memory_order_relaxed);
        static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(logger_, throttle_clock, 5000,
                             "RetapTerrainMap: scan queue full (%zu); dropped the "
                             "oldest scan",
                             queue_cap);
    }
}

void RetapTerrainMap::processQueuedScans(const finenav::core::RobotState& state) {
    if (model_ == nullptr) return;

    // ReTAP's terrainUpdate() waits until the update list has >= 3 entries or
    // 0.5 s have passed since the last analysis.  The same rule applies here;
    // max_scans_per_cycle then bounds the work done under the map write lock.
    std::vector<QueuedScan> batch;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (scan_queue_.empty()) return;
        const auto now = std::chrono::steady_clock::now();
        const bool period_due =
            last_flush_.time_since_epoch().count() == 0 ||
            std::chrono::duration<double>(now - last_flush_).count() >=
                terrain_update_period_sec_;
        const bool batch_due =
            scan_queue_.size() >= static_cast<size_t>(terrain_update_batch_);
        if (!period_due && !batch_due) return;

        const size_t take = std::min(scan_queue_.size(),
                                     static_cast<size_t>(max_scans_per_cycle_));
        batch.reserve(take);
        for (size_t i = 0; i < take; ++i) {
            batch.push_back(std::move(scan_queue_.front()));
            scan_queue_.pop_front();
        }
        last_flush_ = now;
    }

    // The ray origins were fixed at each scan's header.stamp when it was
    // enqueued, so a batch spanning several stamps still carves free space
    // from the right sensor poses.  The connectivity pass, on the other hand,
    // grows from the robot's current position, which is what `state` is.
    const Eigen::Vector3f center = sensorOrigin(state);

    retap_core::ScanOptions opts;
    opts.update_free = update_free_;
    opts.max_range = static_cast<float>(max_range_);

    octomap::KeySet dirty;
    size_t fused = 0;
    for (const QueuedScan& scan : batch) {
        const retap_core::ScanResult result =
            model_->insertScan(scan.points, scan.origin, opts);
        dirty.insert(result.occupied_keys.begin(), result.occupied_keys.end());
        ++fused;
    }
    scans_processed_.fetch_add(fused, std::memory_order_relaxed);
    if (dirty.empty()) return;
    model_->evaluateDirty(dirty, center);

    // Counters are only maintained on the stats clock: a full-tree walk per
    // cycle would scale with the map, not with the scan.  This must run even
    // when publishing is off — ready() and bounds() are what the WorldView
    // gates on, and an online map starts empty (ready_ == false).
    //
    // It has its own timestamp: the diagnostics republish thread also advances
    // last_publish_, so gating on that starved this block to one run per few
    // minutes whenever a prior map had started the thread.
    const auto now = std::chrono::steady_clock::now();
    const bool stats_due =
        last_stats_tick_.time_since_epoch().count() == 0 ||
        std::chrono::duration<double>(now - last_stats_tick_).count() >=
            kDiagnosticPeriodSec;
    if (stats_due) {
        last_stats_tick_ = now;
        const retap_core::TerrainStats stats = retap_core::computeStats(model_->tree());
        occupied_voxels_ = stats.occupied_voxels;
        drivable_voxels_ = stats.drivable_voxels;
        ready_ = drivable_voxels_ > 0;
        computeBounds();
        if (publish_diagnostics_) publishDiagnostics();

        // Every 5 s: account for the scans.  received == processed + dropped
        // by construction, so a non-zero dropped count is the whole story of
        // any loss (unreadable cloud, wrong frame or a full queue).
        if (last_stats_log_.time_since_epoch().count() == 0 ||
            std::chrono::duration<double>(now - last_stats_log_).count() >= 5.0) {
            size_t queued = 0;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                queued = scan_queue_.size();
            }
            RCLCPP_INFO(logger_,
                        "RetapTerrainMap: online %zu scans (processed %zu, "
                        "dropped %zu, queued %zu); map %zu occupied, %zu drivable",
                        scans_received_.load(std::memory_order_relaxed),
                        scans_processed_.load(std::memory_order_relaxed),
                        scans_dropped_.load(std::memory_order_relaxed), queued,
                        occupied_voxels_, drivable_voxels_);
            last_stats_log_ = now;
        }
    }

    RCLCPP_DEBUG(logger_, "RetapTerrainMap: fused %zu scans, %zu dirty voxels",
                 fused, dirty.size());
}

}  // namespace finenav::retap

// "retap_terrain_map" must match the top-level key in retap_terrain_map.yaml.
FINENAV_REGISTER_MAP(finenav::retap::RetapTerrainMap, retap_terrain_map)
