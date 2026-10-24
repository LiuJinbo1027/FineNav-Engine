// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// RetapAstarPlanner — FineNav Planner plugin around ReTAP's 26-neighbour 3D A*.
//
// Role split (see the migration plan, §2.5):
//   - the search itself is retap_core's ported Astar, unchanged;
//   - the planner owns only the parameters ReTAP's GlobalPlanning node read
//     from ROS and the conversions to/from FineNav's trajectory types;
//   - everything derived from the robot pose + goal (search region, snapping
//     radius reach) is the WorldView's job: plan() reads it from
//     IRetapWorldView, which is the ONLY thing it knows about the map.
//
// Output is pose-only (OutputProfile.pose), exactly as ReTAP's nav_msgs/Path.
// ChassisCmdVel therefore rejects the trajectory with a WARN — expected for a
// global planner in phase 1; the path is still delivered on __feedback and
// visualized on /retap/path.

#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

// The planner role's entry point: concept, geometry vocabulary, Trajectory.
#include "finenav_core/plugin_planner.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"

#include "finenav_retap/retap_astar_planner_params.hpp"
#include "retap_core/terrain_grid.h"

namespace finenav::retap {

/**
 * @brief What RetapAstarPlanner may ask the world.
 *
 * Deliberately small: the search reads voxels through TerrainGrid, and the
 * region bounds the search box.  nearestDrivable() is the in-process
 * replacement for ReTAP's query_nearest_drivable_point service.
 */
class IRetapWorldView {
public:
    virtual ~IRetapWorldView() = default;

    /// Read-only voxel facade over the terrain-analysed tree.
    virtual const retap_core::TerrainGrid& terrain() const = 0;

    /// Search box for this plan, computed by prepare() from the robot pose and
    /// the goal (already intersected with the map bounds).
    virtual const finenav::common::Region3D& searchRegion() const = 0;

    virtual bool isInside(const finenav::common::Position3D& p) const = 0;

    /// Nearest occupied, collision-free voxel to @p q within @p max_radius m.
    virtual bool nearestDrivable(const finenav::common::Position3D& q,
                                 finenav::common::Position3D& out,
                                 double max_radius) const = 0;

    /// Monotonic terrain generation.  A live map bumps it on every terrain
    /// update, so the planner can tell whether the voxels it last searched
    /// are still the current ones.  A static map returns a constant.
    virtual uint64_t mapGeneration() const = 0;
};

class RetapAstarPlanner {
public:
    using ConfigType = retap_astar_planner::Params;
    using IWorldView = IRetapWorldView;

    static constexpr finenav::core::IOProfile InputProfile{true, false, false, false};
    static constexpr finenav::core::IOProfile OutputProfile{true, false, false, false};

    void configure(const ConfigType& cfg);

    // ── Framework injectors (optional for the framework, used here) ──────
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }
    void setVisualizer(const finenav::viz::FineNavVisualizer& viz) { viz_ = viz; }

    /// Episode boundary: called by the framework when a new goal arrives.
    void reset();

    using Context = finenav::core::PlanningContext<RetapAstarPlanner>;
    using Result  = std::optional<finenav::core::OutputTrajectoryT<RetapAstarPlanner>>;

    Result plan(const Context& ctx, const IWorldView& view);

private:
    Result singlePose(const finenav::common::Position3D& p) const;

    // ── parameters (retap_astar_planner.yaml) ────────────────────────────
    float heuristic_weight_ = 0.5f;       // upstream astar_hueristic_weight
    float traversability_weight_ = 0.5f;  // upstream astar_traversability_weight
    float distance_weight_ = 1.0f;        // upstream astar_distance_weight
    int   max_expansions_ = 500000;
    double time_budget_ms_ = 500.0;
    int   goal_z_tolerance_voxels_ = 2;   // upstream |z - goal_h| <= 2
    double stop_distance_ = 0.2;
    double snap_radius_ = 1.0;
    bool  verbose_ = false;
    bool  publish_path_ = true;

    // ── replan throttle (upstream GlobalPlanning: moved > 0.2 m or > 2 s) ──
    // The periodic policy calls plan() every cycle; the search only runs when
    // one of these triggers fires.  A live map adds the generation trigger,
    // gated by replan_min_interval_sec_ so a 10 Hz map update does not turn
    // into a 10 Hz search.
    double replan_min_distance_ = 0.2;
    double replan_period_sec_ = 2.0;
    double replan_min_interval_sec_ = 0.5;
    bool   replan_on_map_change_ = false;

    bool has_plan_ = false;
    finenav::common::Position3D last_start_{};
    finenav::common::Position3D last_goal_{};
    uint64_t last_map_generation_ = 0;
    std::chrono::steady_clock::time_point last_plan_time_{};
    Result cached_;

    uint64_t plan_count_ = 0;

    rclcpp::Logger logger_{rclcpp::get_logger("RetapAstarPlanner")};
    finenav::viz::FineNavVisualizer viz_;
};

}  // namespace finenav::retap
