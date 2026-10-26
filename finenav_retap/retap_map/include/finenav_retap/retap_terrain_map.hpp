// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// RetapTerrainMap — FineNav Map plugin wrapping ReTAP's global terrain map.
//
// Role split (see the migration plan, §2.4):
//   - the static prior is loaded exactly once, in selfManage(), from a
//     terrain-analysed .bt (retap_map_tool's output) or a raw .pcd;
//   - the tree, its terrain attributes and the only read-only facade the
//     Planner is allowed to see (retap_core::TerrainGrid) live here;
//   - online incremental mapping is wired through MapRuntime, but is off by
//     default (online_mapping: false) until S5.
//
// The Map never publishes paths and never plans: it only answers terrain
// queries and publishes diagnostics through the injected visualizer.

#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <sensor_msgs/msg/point_cloud2.hpp>

// The map role's entry point — see finenav_core/plugin_map.hpp.
#include "finenav_core/plugin_map.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"

#include "finenav_retap/retap_terrain_map_params.hpp"
#include "retap_core/terrain_grid.h"
#include "retap_core/terrain_model.h"

namespace finenav::retap {

class RetapTerrainMap {
public:
    using ConfigType = retap_terrain_map::Params;
    // What a WorldView gets from this map.  The grid is a borrowed view over
    // the tree owned here; it stays valid for the map's lifetime and must not
    // be stored past a map reload (there is no reload: structural params are
    // read_only).
    using DataType = retap_core::TerrainGrid;

    RetapTerrainMap() = default;
    ~RetapTerrainMap();

    void configure(const ConfigType& cfg);

    // Loads the static prior exactly once and, when online_mapping is on,
    // registers the PointCloud2 observation source + post-update hook.
    // Deliberately NOT in configure(): configure() re-runs on every parameter
    // update, while loading must happen once.
    void selfManage(finenav::core::MapRuntime& runtime);

    // ── Framework injectors ──────────────────────────────────────────────
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }
    void setVisualizer(const finenav::viz::FineNavVisualizer& viz) { viz_ = viz; }

    // ── Queries (Planner / WorldView side) ───────────────────────────────
    // ready() is false until a map with at least one drivable voxel has been
    // loaded.  collision_ defaults to blocked, so an un-analysed map reports
    // zero drivable voxels — that is the guard against planning on garbage.
    bool ready() const { return ready_; }

    const retap_core::TerrainGrid& terrain() const { return *grid_; }
    const retap_core::TerrainModel& model() const { return *model_; }
    const octomap::IgTree& tree() const { return model_->tree(); }

    size_t occupiedVoxels() const { return occupied_voxels_; }
    size_t drivableVoxels() const { return drivable_voxels_; }

    // Bounding box of the loaded map, expanded by half a voxel.
    const finenav::common::Region3D& bounds() const { return bounds_; }

    // Nearest drivable voxel to q, with ReTAP's query_nearest_drivable_point
    // semantics (TerrainEvaluation::queryNearestDrivable):
    //   1. q's own voxel is drivable and belongs to the robot's connected
    //      component -> that voxel's centre;
    //   2. otherwise the 3-D nearest voxel of that component.  Distance is
    //      never a failure condition, so a 2-D goal published with z = 0 still
    //      snaps onto the surface metres above it.
    // max_radius > 0 is a *diagnostic* threshold only: a snap farther than it
    // logs a throttled warning and still returns the point.
    // Returns false only when the map holds no drivable voxel at all.
    // Single-layer scope: the component is chosen by the robot's position, so
    // overlapping decks would need the goal's z to disambiguate.
    bool nearestDrivable(const finenav::common::Position3D& q,
                         finenav::common::Position3D& out,
                         double max_radius) const;

    // ── Map concept extras (not required by finenav::core::Map) ──────────
    bool isInside(const finenav::common::Position3D& p) const {
        return contains(bounds_, p);
    }
    finenav::common::Region3D getWindowBounds() const { return bounds_; }
    finenav::common::Position3D getWindowCenter() const { return center(bounds_); }

private:
    // One diagnostic voxel, cached at build time so the periodic republish is a
    // restage rather than a full tree walk.
    struct DiagnosticPoint {
        float x = 0.f, y = 0.f, z = 0.f, value = 0.f;
        bool drivable = false;
    };

    void loadStaticMap();
    void validateLoadedMap();
    void computeBounds();

    // Rebuild the cache from the tree, then stage it and publish.
    void publishDiagnostics();
    // Stage the existing cache into the visualizer channels and publish.
    // Called by the republish thread; takes diag_mutex_.
    void restageDiagnostics();

    // ── Online incremental path (S5) ─────────────────────────────────────
    void registerOnlineMapping(finenav::core::MapRuntime& runtime);
    void enqueueScan(const sensor_msgs::msg::PointCloud2& msg);
    void processQueuedScans(const finenav::core::RobotState& state);

    // ── nearest-drivable index (ReTAP's disjoint set + ikd-tree) ─────────
    // Stored every update cycle and used as the connectivity root, so a map
    // loaded from a .bt keeps ReTAP's "root = drivable voxel nearest the
    // robot" semantics.  Runs under the MapServer write lock.
    void rememberRobotPose(const finenav::core::RobotState& state);
    // One-shot build of the index from every occupied, collision-free leaf of
    // the loaded tree.  Normally triggered by rememberRobotPose(); the query
    // path falls back to fallback_root when planning starts first (or when no
    // localizer feeds MapServer at all).  Returns false when the tree has no
    // drivable voxel yet, without latching, so an online map can retry.
    bool ensureNearestIndex(const Eigen::Vector3f& fallback_root) const;

    // ── configuration ────────────────────────────────────────────────────
    retap_core::TerrainConfig terrain_cfg_;
    std::string map_format_ = "bt";
    std::string map_path_;
    double resolution_ = 0.2;
    bool publish_diagnostics_ = true;
    int downsample_diagnostics_ = 1;
    double diagnostics_period_sec_ = 1.0;
    bool online_mapping_ = false;
    std::string scan_topic_ = "/cloud_registered";
    bool scan_in_map_frame_ = true;
    int update_free_ = 0;
    double max_range_ = 10.0;
    // Sensor origin in base_link; ray casting starts here, like ReTAP's
    // getSensorPoseTF(lidar_frame_).  Rotated by the robot's yaw.
    Eigen::Vector3f origin_offset_{Eigen::Vector3f::Zero()};
    int max_buffer_size_ = 2;
    int max_scans_per_cycle_ = 4;
    int terrain_update_batch_ = 3;
    double terrain_update_period_sec_ = 0.5;

    // ── map state ────────────────────────────────────────────────────────
    std::unique_ptr<retap_core::TerrainModel> model_;
    std::unique_ptr<retap_core::TerrainGrid> grid_;
    finenav::common::Region3D bounds_{};
    bool ready_ = false;
    bool loaded_ = false;
    size_t occupied_voxels_ = 0;
    size_t drivable_voxels_ = 0;
    uint64_t last_published_generation_ = 0;

    // ── diagnostics cache ────────────────────────────────────────────────
    // Publishes are volatile QoS, so a one-shot at load would be missed by any
    // RViz started afterwards; a low-rate republish keeps the picture alive.
    mutable std::mutex diag_mutex_;
    std::vector<DiagnosticPoint> diagnostic_points_;
    size_t cached_drivable_ = 0;
    std::thread diagnostic_thread_;
    std::atomic<bool> diagnostics_stop_{false};

    // ── online scan queue (guarded by queue_mutex_) ──────────────────────
    mutable std::mutex queue_mutex_;
    std::deque<std::vector<Eigen::Vector3f>> scan_queue_;
    std::chrono::steady_clock::time_point last_flush_{};
    std::chrono::steady_clock::time_point last_publish_{};
    bool scan_frame_warned_ = false;

    // ── nearest-drivable index state (guarded by index_mutex_) ───────────
    // A .bt keeps the terrain attributes but not connected_voxel_, so the
    // disjoint set and ikd-tree are empty after a load and are rebuilt here.
    mutable std::mutex index_mutex_;
    mutable bool nearest_index_ready_ = false;
    mutable Eigen::Vector3f robot_position_ = Eigen::Vector3f::Zero();
    mutable bool robot_position_valid_ = false;
    mutable std::chrono::steady_clock::time_point last_snap_warn_{};

    // ── online counters (every received scan is one of processed/dropped) ─
    std::atomic<size_t> scans_received_{0};
    std::atomic<size_t> scans_processed_{0};
    std::atomic<size_t> scans_dropped_{0};
    // Stats tick (1 s) and its log (5 s), independent of last_publish_ so the
    // diagnostics republish thread cannot starve them.
    std::chrono::steady_clock::time_point last_stats_tick_{};
    std::chrono::steady_clock::time_point last_stats_log_{};

    rclcpp::Logger logger_{rclcpp::get_logger("RetapTerrainMap")};
    finenav::viz::FineNavVisualizer viz_;
};

}  // namespace finenav::retap
