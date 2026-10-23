// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "finenav_core/robot_state/robot_state.hpp"
#include "finenav_localizer/sources/world_anchor.hpp"
#include "finenav_localizer/sources/odometry_cache.hpp"
#include "finenav_localizer/sources/integrator.hpp"
#include "finenav_localizer/sources/velocity_provider.hpp"
#include "finenav_util/ros/node_thread.hpp"
#include "finenav_localizer/localizer_params.hpp"

namespace finenav::localizer {

/**
 * @brief FineNavLocalizer — observation-driven robot-state provider.
 *
 * Architecture:
 *   WorldAnchor      — the odometry frame's placement in the world frame
 *   OdometryCache    — latest odometry reading (pure cache, no integration)
 *   Integrator       — dead-reckoning (twist mode only)
 *   VelocityProvider — current-velocity estimate
 *
 * Output state = world_anchor ⊕ odom_cache.
 * pushState() is event-driven: one sample per observation.
 */
class FineNavLocalizer : public finenav::core::RobotStateProvider {
public:
    using Params        = finenav_localizer::Params;
    using ParamListener = finenav_localizer::ParamListener;

    explicit FineNavLocalizer(rclcpp::Node::SharedPtr parent_node,
                              rclcpp::NodeOptions pnode_overrides =
                                  rclcpp::NodeOptions());

    FineNavLocalizer(const FineNavLocalizer&)            = delete;
    FineNavLocalizer& operator=(const FineNavLocalizer&) = delete;
    ~FineNavLocalizer() override = default;

private:
    using PoseMsg  = geometry_msgs::msg::PoseWithCovarianceStamped;
    using TwistMsg = geometry_msgs::msg::TwistWithCovarianceStamped;
    using OdomMsg  = nav_msgs::msg::Odometry;

    struct Routing {
        enum class PoseSrc { None, Pose };
        enum class IntegSrc { None, Twist, RelPose };
        enum class VelSrc  { None, Twist, Odom, AutoDiff };

        PoseSrc  pose;
        IntegSrc integration;
        VelSrc   velocity;
    };

    /// Build routing table from params + validate.  Throws on conflict.
    static Routing buildRouting(const Params& p);

    // ── ROS callbacks ────────────────────────────────────────────────────

    /// ROS Callbacks
    void onPose(const PoseMsg& msg);
    void onTwist(const TwistMsg& msg);
    void onOdometry(const OdomMsg& msg);
    void onInitialPose(const PoseMsg& msg);

    /// Shared body of onPose / onInitialPose: fold one absolute pose
    /// observation into the world anchor. Not gated on pose routing — a
    /// one-shot initial pose is valid even with pose_topic disabled.
    void applyAbsolutePose(const PoseMsg& msg);

    /// TimerCallback with frequency of kInternalTickRateHz (default = 100Hz)
    /// Used for:
    /// 1. Pose observation smoothing
    /// 2. Dynamic update parameters
    void timerCallback();

    // ── Output ───────────────────────────────────────────────────────────

    void force2D();
    void publishOutput(const rclcpp::Time& now);

    // Robot state
    finenav::core::RobotState state_{};
    bool initialized_{false};
    mutable std::mutex state_mutex_;

    // Effect sources
    Routing                    routing_;
    WorldAnchor                world_anchor_;
    OdometryCache              odom_cache_;
    Integrator                 integrator_;
    VelocityProvider           velocity_;

    // ROS infrastructure
    rclcpp::Node::SharedPtr        parent_node_;
    rclcpp::Node::SharedPtr        pnode_;
    std::unique_ptr<ParamListener> param_listener_;
    Params                         params_;

    std::shared_ptr<tf2_ros::TransformBroadcaster>        tf_br_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr nav_state_pub_;

    rclcpp::TimerBase::SharedPtr               update_timer_;
    std::unique_ptr<finenav::util::NodeThread> pnode_thread_;

    rclcpp::Subscription<PoseMsg>::SharedPtr  initial_pose_sub_;
    rclcpp::Subscription<PoseMsg>::SharedPtr  pose_sub_;
    rclcpp::Subscription<TwistMsg>::SharedPtr twist_sub_;
    rclcpp::Subscription<OdomMsg>::SharedPtr  odom_sub_;
};

}  // namespace finenav::localizer
