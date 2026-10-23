// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// RetapWorldView — the bridge between RetapTerrainMap and
// RetapAstarPlanner::IWorldView.
//
// Role split (see the migration plan, §2.6):
//   - the map owns the tree; the terrain analysis and the nearest-drivable
//     scan are done once, at load time;
//   - prepare() does only what needs the robot pose and the goal: the search
//     box, and the start/goal markers.  It runs once per plan(), inside the
//     map read lock, so it must stay cheap;
//   - the planner never sees the map — only this view's four queries.
//
// Dependency direction: Map ⊂ Planner ⊆ WorldView.  This is the only artifact
// that names both concrete types.

#pragma once

#include "finenav_core/plugin_world_view.hpp"
#include "finenav_visualizer/finenav_visualizer.hpp"

#include "finenav_retap/retap_planner.hpp"
#include "finenav_retap/retap_terrain_map.hpp"
#include "finenav_retap/retap_world_view_params.hpp"

namespace finenav::retap {

class RetapWorldView : public RetapAstarPlanner::IWorldView {
public:
    using ConfigType = retap_world_view::Params;

    /// Constructor convention: the layer's private sub-node first, then one
    /// const Map& per map type declared at registration.
    RetapWorldView(rclcpp::Node::SharedPtr node, const RetapTerrainMap& map);

    // ── MANDATORY: parameter channel ─────────────────────────────────────
    void configure(const ConfigType& cfg);

    // ── MANDATORY: per-plan derivation hook ──────────────────────────────
    void prepare(const finenav::core::PlanningContext<RetapAstarPlanner>& ctx);

    // ── OPTIONAL injections ──────────────────────────────────────────────
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }
    void setVisualizer(const finenav::viz::FineNavVisualizer& viz) { viz_ = viz; }

    // ── IRetapWorldView: must stay cheap, prepare() left everything behind ─
    const retap_core::TerrainGrid& terrain() const override { return map_.terrain(); }
    const finenav::common::Region3D& searchRegion() const override { return region_; }
    bool isInside(const finenav::common::Position3D& p) const override;
    bool nearestDrivable(const finenav::common::Position3D& q,
                         finenav::common::Position3D& out,
                         double max_radius) const override;

private:
    void publishGoalMarkers(const finenav::common::Position3D& start,
                            const finenav::common::Position3D& goal);

    const RetapTerrainMap& map_;

    // Search box for the current plan.  Initialised to the map bounds so a
    // query before the first prepare() still answers something sane.
    finenav::common::Region3D region_{};

    double search_margin_ = 3.0;
    bool publish_markers_ = true;

    rclcpp::Logger logger_{rclcpp::get_logger("RetapWorldView")};
    finenav::viz::FineNavVisualizer viz_;
};

}  // namespace finenav::retap
