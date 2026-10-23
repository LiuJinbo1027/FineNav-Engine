// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// RetapWorldView implementation.  See the header for the role split.

#include "finenav_retap/retap_world_view.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "finenav_core/algo_register_macros.hpp"
#include "finenav_visualizer/color.hpp"
#include "finenav_visualizer/marker_viz.hpp"

namespace finenav::retap {
namespace {

// Absolute, like the map's diagnostics and the planner's path.
constexpr char kGoalTopic[] = "/retap/goal";

finenav::common::Position3D toPosition(const geometry_msgs::msg::Point& p) {
    return {p.x, p.y, p.z};
}

finenav::common::Region3D intersect(const finenav::common::Region3D& a,
                                    const finenav::common::Region3D& b) {
    finenav::common::Region3D out;
    out.min = {std::max(a.min.x, b.min.x),
               std::max(a.min.y, b.min.y),
               std::max(a.min.z, b.min.z)};
    out.max = {std::min(a.max.x, b.max.x),
               std::min(a.max.y, b.max.y),
               std::min(a.max.z, b.max.z)};
    return out;
}

}  // namespace

RetapWorldView::RetapWorldView(rclcpp::Node::SharedPtr node,
                               const RetapTerrainMap& map)
    : map_(map),
      region_(map.bounds()),
      logger_(node->get_logger().get_child("retap_world_view")) {}

void RetapWorldView::configure(const ConfigType& cfg) {
    search_margin_ = cfg.search_margin;
    publish_markers_ = cfg.publish_markers;
}

void RetapWorldView::prepare(
    const finenav::core::PlanningContext<RetapAstarPlanner>& ctx) {
    if (!map_.ready()) {
        // The map refused to become ready (un-analysed file, missing file).
        // plan() will fail on the empty tree; keep the last region so the
        // failure surfaces from there rather than from a crash here.
        return;
    }
    if (ctx.ref_traj.poses.empty()) {
        region_ = map_.bounds();
        return;
    }

    const finenav::common::Position3D start =
        toPosition(ctx.robot_state.pose.position);
    const finenav::common::Position3D goal =
        toPosition(ctx.ref_traj.poses.back().position);

    // The box only bounds the search; it need not contain the snapped voxels
    // exactly, because snapping moves a point by at most the planner's
    // snap_radius (1 m by default), and the margin (3 m by default) is larger.
    finenav::common::Region3D box;
    box.min = {std::min(start.x, goal.x) - search_margin_,
               std::min(start.y, goal.y) - search_margin_,
               std::min(start.z, goal.z) - search_margin_};
    box.max = {std::max(start.x, goal.x) + search_margin_,
               std::max(start.y, goal.y) + search_margin_,
               std::max(start.z, goal.z) + search_margin_};

    region_ = intersect(box, map_.bounds());

    if (publish_markers_) {
        // Show what the planner will actually use, not the raw request: plan()
        // snaps both ends to the nearest drivable voxel inside this region.
        finenav::common::Position3D marked_start = start;
        finenav::common::Position3D marked_goal = goal;
        map_.nearestDrivable(start, marked_start, search_margin_);
        map_.nearestDrivable(goal, marked_goal, search_margin_);
        publishGoalMarkers(marked_start, marked_goal);
    }
}

void RetapWorldView::publishGoalMarkers(
    const finenav::common::Position3D& start,
    const finenav::common::Position3D& goal) {
    auto& markers = viz_.markers(kGoalTopic);
    markers.addSphere(Eigen::Vector3d(start.x, start.y, start.z), 0.15,
                      finenav::viz::ColorName::GREEN);
    markers.addSphere(Eigen::Vector3d(goal.x, goal.y, goal.z), 0.15,
                      finenav::viz::ColorName::RED);
    viz_.publish(markers);
}

bool RetapWorldView::isInside(const finenav::common::Position3D& p) const {
    return contains(region_, p);
}

bool RetapWorldView::nearestDrivable(const finenav::common::Position3D& q,
                                     finenav::common::Position3D& out,
                                     double max_radius) const {
    return map_.nearestDrivable(q, out, max_radius);
}

}  // namespace finenav::retap

// "retap_world_view" must match the top-level key of retap_world_view.yaml.
// The macro records only TYPES; the framework resolves them to the registered
// names "retap_astar_planner" / "retap_terrain_map" at assembly time.
FINENAV_REGISTER_WORLDVIEW(
    finenav::retap::RetapWorldView,
    retap_world_view,
    finenav::retap::RetapAstarPlanner,
    finenav::retap::RetapTerrainMap)
