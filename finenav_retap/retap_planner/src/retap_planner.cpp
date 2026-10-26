// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// RetapAstarPlanner implementation.  See the header for the role split and
// retap_core/astar_3d/astar_3d.h for the ported search itself.

#include "finenav_retap/retap_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "finenav_core/algo_register_macros.hpp"
#include "finenav_visualizer/color.hpp"
#include "finenav_visualizer/marker_viz.hpp"
#include "retap_core/astar_3d/astar_3d.h"

namespace finenav::retap {
namespace {

// Absolute, like the map's diagnostics: the visualizer lives on the engine
// node, so a relative name would land under the engine's node name.
constexpr char kPathTopic[] = "/retap/path";

finenav::common::Position3D toPosition(const geometry_msgs::msg::Point& p) {
    return {p.x, p.y, p.z};
}

/// A voxel-aligned box: Astar derives its index offsets with floor(), so
/// rounding the region to whole voxels keeps Coordinate -> index equal on both
/// sides of the call.
finenav::common::Region3D alignToVoxels(const finenav::common::Region3D& r,
                                        double res) {
    finenav::common::Region3D out;
    out.min = {std::floor(r.min.x / res) * res,
               std::floor(r.min.y / res) * res,
               std::floor(r.min.z / res) * res};
    out.max = {std::ceil(r.max.x / res) * res,
               std::ceil(r.max.y / res) * res,
               std::ceil(r.max.z / res) * res};
    // A degenerate axis would make the open[] row array empty.
    if (out.max.x <= out.min.x) out.max.x = out.min.x + res;
    if (out.max.y <= out.min.y) out.max.y = out.min.y + res;
    if (out.max.z <= out.min.z) out.max.z = out.min.z + res;
    return out;
}

}  // namespace

void RetapAstarPlanner::configure(const ConfigType& cfg) {
    heuristic_weight_ = static_cast<float>(cfg.heuristic_weight);
    traversability_weight_ = static_cast<float>(cfg.traversability_weight);
    distance_weight_ = static_cast<float>(cfg.distance_weight);
    max_expansions_ = static_cast<int>(cfg.max_expansions);
    time_budget_ms_ = cfg.time_budget_ms;
    goal_z_tolerance_voxels_ = static_cast<int>(cfg.goal_z_tolerance_voxels);
    stop_distance_ = cfg.stop_distance;
    snap_radius_ = cfg.snap_radius;
    verbose_ = cfg.verbose;
    publish_path_ = cfg.publish_path;
    replan_min_distance_ = cfg.replan_min_distance;
    replan_period_sec_ = cfg.replan_period_sec;
    replan_min_interval_sec_ = cfg.replan_min_interval_sec;
    replan_on_map_change_ = cfg.replan_on_map_change;
}

void RetapAstarPlanner::reset() {
    // The search is stateless across episodes (Astar is built per plan), so
    // the only state to drop is the replan-throttle cache: a new goal must
    // not be answered from the previous episode's path.
    has_plan_ = false;
    cached_.reset();
    plan_count_ = 0;
    RCLCPP_INFO(logger_, "reset(): new episode");
}

RetapAstarPlanner::Result RetapAstarPlanner::singlePose(
    const finenav::common::Position3D& p) const {
    using Trajectory = finenav::core::Trajectory<OutputProfile>;
    Trajectory result;
    geometry_msgs::msg::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = p.z;
    pose.orientation.w = 1.0;
    result.poses.push_back(pose);
    return result;
}

RetapAstarPlanner::Result RetapAstarPlanner::plan(const Context& ctx,
                                                  const IWorldView& view) {
    if (ctx.ref_traj.poses.empty()) {
        // The framework gates "no reference" out before plan(); reaching this
        // means the reference is degenerate, which is a legitimate failure.
        RCLCPP_WARN(logger_, "plan(): reference trajectory carries no pose");
        return std::nullopt;
    }

    const finenav::common::Position3D start =
        toPosition(ctx.robot_state.pose.position);
    const finenav::common::Position3D goal =
        toPosition(ctx.ref_traj.poses.back().position);

    // ── replan throttle ─────────────────────────────────────────────────
    // Upstream GlobalPlanning replanned only after moving 0.2 m or waiting
    // 2 s.  The gate sits before snapping and A*: under a periodic policy
    // plan() is called every cycle, and re-running an unchanged search would
    // burn the map read lock for nothing.  Triggers: new goal (a mid-episode
    // reference change; a new episode calls reset()), the robot moved, the
    // period elapsed, or the live map changed (rate-limited by
    // replan_min_interval_sec_ so terrain updates do not become searches).
    const auto now = std::chrono::steady_clock::now();
    const double since_last =
        has_plan_
            ? std::chrono::duration<double>(now - last_plan_time_).count()
            : std::numeric_limits<double>::infinity();
    const bool moved =
        !has_plan_ || norm(start - last_start_) > replan_min_distance_;
    const bool goal_changed =
        !has_plan_ || norm(goal - last_goal_) > 1e-6;
    const bool period_due =
        !has_plan_ || (replan_period_sec_ > 0.0 &&
                       since_last >= replan_period_sec_);
    const bool map_changed =
        replan_on_map_change_ && has_plan_ &&
        view.mapGeneration() != last_map_generation_ &&
        since_last >= replan_min_interval_sec_;
    if (has_plan_ && !moved && !goal_changed && !period_due && !map_changed) {
        RCLCPP_DEBUG(logger_, "reusing the previous path (%.2f s old)", since_last);
        return cached_;
    }

    finenav::common::Position3D start_snapped;
    finenav::common::Position3D goal_snapped;
    // nearestDrivable() follows ReTAP: it snaps to the nearest voxel of the
    // robot's component at any distance, so failure means the map has no
    // drivable voxel at all (snap_radius is only a warning threshold there).
    if (!view.nearestDrivable(start, start_snapped, snap_radius_)) {
        RCLCPP_WARN(logger_,
                    "no drivable voxel in the map for the robot at "
                    "(%.2f, %.2f, %.2f)",
                    start.x, start.y, start.z);
        return std::nullopt;
    }
    if (!view.nearestDrivable(goal, goal_snapped, snap_radius_)) {
        RCLCPP_WARN(logger_,
                    "no drivable voxel in the map for the goal at "
                    "(%.2f, %.2f, %.2f)",
                    goal.x, goal.y, goal.z);
        return std::nullopt;
    }

    if (norm(goal_snapped - start_snapped) < stop_distance_) {
        // ReTAP's GlobalPlanning stopped before planning when the snapped goal
        // coincided with the robot.  Emit the goal so the episode can finish
        // instead of reporting a fault.
        RCLCPP_INFO(logger_, "goal already reached (%.2f m)", stop_distance_);
        return singlePose(goal_snapped);
    }

    const retap_core::TerrainGrid& terrain = view.terrain();
    const double res = terrain.resolution();
    const finenav::common::Region3D region =
        alignToVoxels(view.searchRegion(), res);

    // retap_core::Astar only reads the tree (search() is non-const in octomap),
    // so the cast is a formality with no write behind it.
    octomap::IgTree* tree = const_cast<octomap::IgTree*>(&terrain.tree());

    const auto t0 = std::chrono::steady_clock::now();
    Astar astar(tree, heuristic_weight_, traversability_weight_,
                distance_weight_,
                octomap::point3d(region.min.x, region.min.y, region.min.z),
                octomap::point3d(region.max.x, region.max.y, region.max.z),
                max_expansions_, time_budget_ms_, verbose_);
    astar.goal_z_tolerance = goal_z_tolerance_voxels_;

    const Eigen::Vector3f start_f(static_cast<float>(start_snapped.x),
                                  static_cast<float>(start_snapped.y),
                                  static_cast<float>(start_snapped.z));
    const Eigen::Vector3f goal_f(static_cast<float>(goal_snapped.x),
                                 static_cast<float>(goal_snapped.y),
                                 static_cast<float>(goal_snapped.z));
    astar.SetStartAndGoal(start_f, goal_f);

    const std::vector<Eigen::Vector3f> path = astar.startSearch();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0)
            .count();

    if (path.empty()) {
        RCLCPP_WARN(logger_,
                    "A* found no path from (%.2f, %.2f, %.2f) to "
                    "(%.2f, %.2f, %.2f) in %.1f ms%s",
                    start_snapped.x, start_snapped.y, start_snapped.z,
                    goal_snapped.x, goal_snapped.y, goal_snapped.z, elapsed_ms,
                    astar.budgetExceeded() ? " (budget exceeded)" : "");
        return std::nullopt;
    }

    using Trajectory = finenav::core::Trajectory<OutputProfile>;
    Trajectory result;
    result.poses.reserve(path.size());
    std::vector<Eigen::Vector3d> viz_points;
    viz_points.reserve(path.size());

    for (size_t i = 0; i < path.size(); ++i) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = path[i].x();
        pose.position.y = path[i].y();
        pose.position.z = path[i].z();

        // Upstream left the quaternion at zero, which is not a rotation; use
        // the horizontal tangent (same fix as the S2 node).
        const size_t next = std::min(i + 1, path.size() - 1);
        const size_t prev = (i == 0) ? 0 : i - 1;
        const double dx = path[next].x() - path[prev].x();
        const double dy = path[next].y() - path[prev].y();
        const double yaw = (dx == 0.0 && dy == 0.0) ? 0.0 : std::atan2(dy, dx);
        pose.orientation.z = std::sin(yaw * 0.5);
        pose.orientation.w = std::cos(yaw * 0.5);

        result.poses.push_back(pose);
        viz_points.emplace_back(path[i].x(), path[i].y(), path[i].z());
    }

    ++plan_count_;
    RCLCPP_INFO(logger_,
                "plan #%lu: %zu waypoints in %.1f ms (%d nodes, %.2f m)",
                static_cast<unsigned long>(plan_count_), result.poses.size(),
                elapsed_ms, astar.expandedNodes(),
                static_cast<double>(astar.sresult.pathlength) * res);

    if (publish_path_) {
        auto& markers = viz_.markers(kPathTopic);
        markers.addPath(viz_points, 0.03, finenav::viz::ColorName::CYAN);
        viz_.publish(markers);
    }

    // Remember what this path was computed from.  Failures are not cached:
    // a nullopt faults the stage, so there is no next cycle to reuse it in.
    has_plan_ = true;
    last_start_ = start;
    last_goal_ = goal;
    last_map_generation_ = view.mapGeneration();
    last_plan_time_ = std::chrono::steady_clock::now();
    cached_ = result;
    return result;
}

}  // namespace finenav::retap

// "retap_astar_planner" must match the top-level key of retap_astar_planner.yaml.
FINENAV_REGISTER_PLANNER(finenav::retap::RetapAstarPlanner, retap_astar_planner)
