// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_localizer/finenav_localizer.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "finenav_util/common/log_macros.hpp"
#include "finenav_common/frames.hpp"
#include "finenav_core/robot_state/eigen_conversions.hpp"

namespace finenav::localizer {

namespace {
// Fixed internal timer rate [Hz] for the pose-smoothing tick.
inline constexpr double  kInternalTickRateHz = 100.0;
}  // namespace


FineNavLocalizer::Routing FineNavLocalizer::buildRouting(const Params& p) {
    Routing r{};

    const bool has_pose_topic  = !p.pose_topic.empty();
    const bool has_twist_topic = !p.twist_topic.empty();
    const bool has_odom        = !p.odom_topic.empty();

    // ── Pose source ──────────────────────────────────────────────────────
    r.pose = has_pose_topic ? Routing::PoseSrc::Pose
                            : Routing::PoseSrc::None;

    // ── Integration source ───────────────────────────────────────────────
    if (has_twist_topic && has_odom) {
        RCLCPP_WARN(rclcpp::get_logger("finenav_localizer"),
            "FineNavLocalizer: twist_topic ignored for position integration — "
            "odom_topic takes priority.");
    }

    if (has_odom) {
        r.integration = Routing::IntegSrc::RelPose;
    } else if (has_twist_topic) {
        r.integration = Routing::IntegSrc::Twist;
    } else {
        r.integration = Routing::IntegSrc::None;
    }

    // ── Velocity source ──────────────────────────────────────────────────
    if (has_twist_topic && has_odom && p.odom_use_twist) {
        RCLCPP_WARN(rclcpp::get_logger("finenav_localizer"),
            "FineNavLocalizer: odom_use_twist ignored for velocity — "
            "twist_topic takes priority.");
    }

    if (has_twist_topic) {
        r.velocity = Routing::VelSrc::Twist;
    } else if (has_odom && p.odom_use_twist) {
        r.velocity = Routing::VelSrc::Odom;
    } else if (r.pose != Routing::PoseSrc::None) {
        r.velocity = Routing::VelSrc::AutoDiff;
    } else {
        r.velocity = Routing::VelSrc::None;
    }

    return r;
}

FineNavLocalizer::FineNavLocalizer(rclcpp::Node::SharedPtr parent_node,
                                   rclcpp::NodeOptions pnode_overrides)
    : parent_node_(std::move(parent_node))
{
    pnode_overrides.append_parameter_override(
        "use_sim_time", parent_node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(
        "finenav_localizer", parent_node_->get_name(), pnode_overrides);

    param_listener_ = std::make_unique<ParamListener>(pnode_);
    params_         = param_listener_->get_params();

    // Resolve and validate routing
    routing_ = buildRouting(params_);

    // Create effect sources
    world_anchor_.configure(params_.pose_smoothing_time, kInternalTickRateHz);

    // Subscriptions
    if (!params_.initial_pose_topic.empty()) {
        initial_pose_sub_ = pnode_->create_subscription<PoseMsg>(
            params_.initial_pose_topic, rclcpp::SystemDefaultsQoS(),
            [this](PoseMsg::ConstSharedPtr msg) {
                onInitialPose(*msg);
                initial_pose_sub_.reset();  // one-shot
            });
    }
    if (!params_.pose_topic.empty()) {
        pose_sub_ = pnode_->create_subscription<PoseMsg>(
            params_.pose_topic, rclcpp::SystemDefaultsQoS(),
            [this](PoseMsg::ConstSharedPtr msg) { onPose(*msg); });
    }
    if (!params_.twist_topic.empty()) {
        twist_sub_ = pnode_->create_subscription<TwistMsg>(
            params_.twist_topic, rclcpp::SystemDefaultsQoS(),
            [this](TwistMsg::ConstSharedPtr msg) { onTwist(*msg); });
    }
    if (!params_.odom_topic.empty()) {
        odom_sub_ = pnode_->create_subscription<OdomMsg>(
            params_.odom_topic, rclcpp::SystemDefaultsQoS(),
            [this](OdomMsg::ConstSharedPtr msg) { onOdometry(*msg); });
    }

    // Optional outputs
    if (params_.publish_tf) {
        tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(pnode_);
    }
    if (params_.publish_odom) {
        nav_state_pub_ = pnode_->create_publisher<nav_msgs::msg::Odometry>(
            "~/nav_state", 10);
    }

    // Timer
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / kInternalTickRateHz));

    update_timer_ = rclcpp::create_timer(
        pnode_, pnode_->get_clock(), period, [this]() { timerCallback(); });

    pnode_thread_ = std::make_unique<finenav::util::NodeThread>(pnode_);
}


void FineNavLocalizer::onInitialPose(const PoseMsg& msg) {
    // A one-shot initial pose is an absolute observation in its own right:
    // apply it even when the continuous pose channel (pose_topic) is disabled.
    applyAbsolutePose(msg);
}

void FineNavLocalizer::onPose(const PoseMsg& msg) {
    if (routing_.pose != Routing::PoseSrc::Pose) return;
    applyAbsolutePose(msg);
}

void FineNavLocalizer::applyAbsolutePose(const PoseMsg& msg) {
    const auto& pose = msg.pose.pose;
    const Eigen::Vector3d    obs_p(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Quaterniond obs_q(pose.orientation.w, pose.orientation.x,
                                    pose.orientation.y, pose.orientation.z);
    const auto stamp = rclcpp::Time(msg.header.stamp);

    {
        std::lock_guard<std::mutex> lk(state_mutex_);

        // ── Resolve odometry at the observation timestamp ────────────────
        Eigen::Vector3d    odom_p_at_obs = odom_cache_.position();
        Eigen::Quaterniond odom_q_at_obs = odom_cache_.orientation();
        bool               fallback      = false;

        if (world_anchor_.isReady()) {
            auto state_at_obs = getRobotState(stamp);
            if (!state_at_obs.has_value()) {
                FINENAV_LOG_WARN_THROTTLE(pnode_->get_logger(), 5000,
                    "FineNavLocalizer: pose stamp %.3f s is outside the "
                    "retained state history — falling back to latest odometry.",
                    stamp.seconds());
                fallback = true;
            } else {
                Eigen::Vector3d    state_pos_at_obs;
                Eigen::Quaterniond state_ori_at_obs;
                core::fromMsg(*state_at_obs, state_pos_at_obs);
                core::fromMsg(*state_at_obs, state_ori_at_obs);

                odom_q_at_obs = world_anchor_.orientation().conjugate()
                                * state_ori_at_obs;
                odom_p_at_obs = world_anchor_.orientation().conjugate()
                                * (state_pos_at_obs - world_anchor_.position());
            }
        }

        // ── Compute the new anchor ───────────────────────────────────────
        const Eigen::Quaterniond anchor_ori = obs_q * odom_q_at_obs.conjugate();
        const Eigen::Vector3d    anchor_pos = obs_p - anchor_ori * odom_p_at_obs;

        if (params_.pose_smoothing_time <= 0.0) {
            world_anchor_.set(anchor_pos, anchor_ori);
        } else {
            world_anchor_.setTarget(anchor_pos, anchor_ori);
        }

        // ── Compose state from the updated anchor ────────────────────────
        {
            Eigen::Vector3d    tmp_pos;
            Eigen::Quaterniond tmp_ori;
            composeState(world_anchor_.position(), world_anchor_.orientation(),
                         odom_cache_.position(), odom_cache_.orientation(),
                         tmp_pos, tmp_ori);
            core::toMsg(tmp_pos, state_);
            core::toMsg(tmp_ori, state_);
        }

        // Velocity: auto-diff from consecutive poses
        if (routing_.velocity == Routing::VelSrc::AutoDiff) {
            velocity_.autoDiffFromPose(obs_p, obs_q, stamp, state_);
        }

        initialized_ = true;
    }

    state_.header.stamp = stamp;
    pushState(state_);
}

void FineNavLocalizer::onTwist(const TwistMsg& msg) {
    const auto& t = msg.twist.twist;
    const Eigen::Vector3d v(t.linear.x, t.linear.y, t.linear.z);
    const Eigen::Vector3d omega(t.angular.x, t.angular.y, t.angular.z);
    const auto stamp = rclcpp::Time(msg.header.stamp);

    {
        std::lock_guard<std::mutex> lk(state_mutex_);

        if (routing_.integration == Routing::IntegSrc::Twist) {
            integrator_.acceptTwist(v, omega, stamp, state_);
        }

        if (routing_.velocity == Routing::VelSrc::Twist) {
            velocity_.setFromTwist(v, omega, state_);
        }

        initialized_ = true;

        if (params_.force_2d_mode) {
            force2D();
        }
        state_.header.stamp = stamp;
        pushState(state_);
    }
}

void FineNavLocalizer::onOdometry(const OdomMsg& msg) {
    const auto& pose = msg.pose.pose;
    const Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                                pose.orientation.y, pose.orientation.z);
    const Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);

    std::lock_guard<std::mutex> lk(state_mutex_);

    // Always cache the latest odometry
    odom_cache_.apply(p, q);

    // Compose state = anchor ⊕ odometry
    if (world_anchor_.isReady()) {
        Eigen::Vector3d    tmp_pos;
        Eigen::Quaterniond tmp_ori;
        composeState(world_anchor_.position(), world_anchor_.orientation(),
                     odom_cache_.position(), odom_cache_.orientation(),
                     tmp_pos, tmp_ori);
        core::toMsg(tmp_pos, state_);
        core::toMsg(tmp_ori, state_);
    } else {
        core::toMsg(p, state_);
        core::toMsg(q, state_);
    }

    // ── Velocity: odom.twist as explicit velocity ───────────────────────
    if (routing_.velocity == Routing::VelSrc::Odom) {
        const auto& t = msg.twist.twist;
        const Eigen::Vector3d v(t.linear.x, t.linear.y, t.linear.z);
        const Eigen::Vector3d omega(t.angular.x, t.angular.y, t.angular.z);
        velocity_.setFromTwist(v, omega, state_);
    }

    initialized_ = true;

    if (params_.force_2d_mode) {
        force2D();
    }
    state_.header.stamp = rclcpp::Time(msg.header.stamp);
    pushState(state_);
}

void FineNavLocalizer::timerCallback() {
    // ── Refresh dynamic parameters ───────────────────────────────────────
    if (param_listener_->is_old(params_)) {
        const auto old_smoothing = params_.pose_smoothing_time;
        params_ = param_listener_->get_params();
        if (params_.pose_smoothing_time != old_smoothing) {
            world_anchor_.configure(params_.pose_smoothing_time, kInternalTickRateHz);
        }
    }

    const rclcpp::Time now = pnode_->now();

    {
        std::lock_guard<std::mutex> lk(state_mutex_);

        // ── EMA smoothing tick ──────────────────────────────────────────
        if (world_anchor_.isReady()) {
            auto anchor_pos = world_anchor_.position();
            auto anchor_ori = world_anchor_.orientation();
            if (params_.pose_smoothing_time > 0.0 &&
                world_anchor_.tick(anchor_pos, anchor_ori)) {
                if (odom_cache_.hasData()) {
                    Eigen::Vector3d    tmp_pos;
                    Eigen::Quaterniond tmp_ori;
                    composeState(anchor_pos, anchor_ori,
                                 odom_cache_.position(), odom_cache_.orientation(),
                                 tmp_pos, tmp_ori);
                    core::toMsg(tmp_pos, state_);
                    core::toMsg(tmp_ori, state_);
                }
                if (params_.force_2d_mode) {
                    force2D();
                }
                state_.header.stamp = now;
                pushState(state_);
            }
        }

        publishOutput(now);

        // ── Warning: no absolute pose data ───────────────────────────────
        if (!world_anchor_.hasPoseData()) {
            FINENAV_LOG_WARN_THROTTLE(pnode_->get_logger(), 5000,
                "FineNavLocalizer: no absolute pose source configured or no pose "
                "data received — position will drift without absolute reference.");
        }
    }
}

void FineNavLocalizer::publishOutput(const rclcpp::Time& now) {
    if (tf_br_) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp    = now;
        tf.header.frame_id = common::frames().world_fixed;
        tf.child_frame_id  = common::frames().robot;
        tf.transform.translation.x = state_.pose.position.x;
        tf.transform.translation.y = state_.pose.position.y;
        tf.transform.translation.z = state_.pose.position.z;
        tf.transform.rotation.w = state_.pose.orientation.w;
        tf.transform.rotation.x = state_.pose.orientation.x;
        tf.transform.rotation.y = state_.pose.orientation.y;
        tf.transform.rotation.z = state_.pose.orientation.z;
        tf_br_->sendTransform(tf);
    }

    if (nav_state_pub_) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = now;
        odom.header.frame_id = common::frames().world_fixed;
        odom.child_frame_id  = common::frames().robot;
        odom.pose.pose.position.x    = state_.pose.position.x;
        odom.pose.pose.position.y    = state_.pose.position.y;
        odom.pose.pose.position.z    = state_.pose.position.z;
        odom.pose.pose.orientation.w = state_.pose.orientation.w;
        odom.pose.pose.orientation.x = state_.pose.orientation.x;
        odom.pose.pose.orientation.y = state_.pose.orientation.y;
        odom.pose.pose.orientation.z = state_.pose.orientation.z;
        odom.twist.twist.linear.x  = state_.twist.linear.x;
        odom.twist.twist.linear.y  = state_.twist.linear.y;
        odom.twist.twist.linear.z  = state_.twist.linear.z;
        odom.twist.twist.angular.x = state_.twist.angular.x;
        odom.twist.twist.angular.y = state_.twist.angular.y;
        odom.twist.twist.angular.z = state_.twist.angular.z;
        nav_state_pub_->publish(odom);
    }
}

void FineNavLocalizer::force2D() {
    state_.pose.position.z = 0.0;

    Eigen::Vector3d    pos;
    Eigen::Quaterniond ori;
    core::fromMsg(state_, pos);
    core::fromMsg(state_, ori);

    const Eigen::Vector3d x_axis = ori.matrix().col(0);
    const double yaw = std::atan2(x_axis.y(), x_axis.x());
    Eigen::Quaterniond q_2d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    core::toMsg(q_2d, state_);
}

}  // namespace finenav::localizer
