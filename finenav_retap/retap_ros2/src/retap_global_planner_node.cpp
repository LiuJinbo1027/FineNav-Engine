/*
 * retap_global_planner_node - ROS 2 replacement for ReTAP's
 * GlobalPlanning node (migration stage S2).
 *
 * It does exactly what upstream GlobalPlanning did, minus the parts that
 * belong to FineNav in S4:
 *   - map:  a processed .bt file (or an octomap_msgs topic when built with
 *           RETAP_HAS_OCTOMAP_MSGS)
 *   - start: TF map <- base_link
 *   - goal:  ~/goal (PoseStamped) or /clicked_point (PointStamped)
 *   - snap both to the nearest drivable voxel, run the upstream 26-neighbour
 *     A*, publish ~/path (nav_msgs/Path)
 *
 * It deliberately has no planner plugin machinery; S4 turns the same core
 * calls into RetapWorldView::prepare() + RetapAstarPlanner::plan().
 */

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#ifdef RETAP_HAS_OCTOMAP_MSGS
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#endif

#include "retap_core/astar_3d/astar_3d.h"
#include "retap_core/map_io.h"

class RetapGlobalPlannerNode : public rclcpp::Node {
 public:
  RetapGlobalPlannerNode() : rclcpp::Node("retap_global_planner") {
    map_path_ = declare_parameter<std::string>("map_path", "");
    map_topic_ = declare_parameter<std::string>("map_topic", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_link_frame_ =
        declare_parameter<std::string>("base_link_frame", "base_link");
    heuristic_weight_ = declare_parameter<double>("heuristic_weight", 0.5);
    traversability_weight_ =
        declare_parameter<double>("traversability_weight", 0.5);
    distance_weight_ = declare_parameter<double>("distance_weight", 1.0);
    snap_radius_ = declare_parameter<double>("snap_radius", 1.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Private topic names, like ReTAP's global_planning/path: the node name is
    // not part of a ROS 2 topic namespace, so "path" alone would resolve to
    // /path.
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 10);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/goal", 10,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          Eigen::Vector3f goal;
          goal << static_cast<float>(msg->pose.position.x),
              static_cast<float>(msg->pose.position.y),
              static_cast<float>(msg->pose.position.z);
          planFromCurrentPose(goal);
        });
    clicked_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          Eigen::Vector3f goal;
          goal << static_cast<float>(msg->point.x),
              static_cast<float>(msg->point.y),
              static_cast<float>(msg->point.z);
          planFromCurrentPose(goal);
        });

    loadMap();
  }

 private:
  void loadMap() {
    if (!map_path_.empty()) {
      std::string error;
      std::unique_ptr<octomap::IgTree> tree =
          retap_core::loadTerrainBt(map_path_, &error);
      if (tree == nullptr) {
        RCLCPP_ERROR(get_logger(), "failed to load map: %s", error.c_str());
        return;
      }
      adoptTree(std::shared_ptr<octomap::IgTree>(std::move(tree)));
      return;
    }
    if (!map_topic_.empty()) {
#ifdef RETAP_HAS_OCTOMAP_MSGS
      map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
          map_topic_, rclcpp::QoS(1).transient_local(),
          [this](octomap_msgs::msg::Octomap::ConstSharedPtr msg) {
            // msgToMap() returns a caller-owned raw pointer; shared_ptr's
            // constructor from a raw pointer is explicit, so this must be
            // direct-initialised ('=' would not compile).
            std::shared_ptr<octomap::AbstractOcTree> tree{
                octomap_msgs::msgToMap(*msg)};
            auto ig_tree = std::dynamic_pointer_cast<octomap::IgTree>(tree);
            if (ig_tree == nullptr) {
              RCLCPP_ERROR(get_logger(),
                           "map_topic carries '%s', not an IgTree "
                           "(publisher not registering the type?)",
                           tree ? tree->getTreeType().c_str() : "nothing");
              return;
            }
            adoptTree(ig_tree->deepClone());
          });
      RCLCPP_INFO(get_logger(), "waiting for a map on %s",
                  map_topic_.c_str());
#else
      RCLCPP_ERROR(get_logger(),
                   "map_topic requires octomap_msgs: install "
                   "ros-humble-octomap-msgs and rebuild, or use map_path");
#endif
      return;
    }
    RCLCPP_ERROR(get_logger(),
                 "no map source: set map_path (or map_topic when built with "
                 "octomap_msgs)");
  }

  void adoptTree(std::shared_ptr<octomap::IgTree> tree) {
    tree_ = std::move(tree);
    const retap_core::TerrainStats stats = retap_core::computeStats(*tree_);
    RCLCPP_INFO(get_logger(),
                "map loaded: %zu occupied, %zu drivable, res %.2f m%s",
                stats.occupied_voxels, stats.drivable_voxels,
                tree_->getResolution(),
                stats.drivable_voxels == 0
                    ? " -- NO DRIVABLE VOXEL: was this map terrain-analysed?"
                    : "");
    if (stats.drivable_voxels == 0) {
      RCLCPP_ERROR(get_logger(),
                   "collision_ defaults to blocked; refusing to plan on an "
                   "unanalysed map");
      tree_.reset();
    }
  }

  bool lookupRobotPose(Eigen::Vector3f *pose) {
    try {
      const geometry_msgs::msg::TransformStamped tf =
          tf_buffer_->lookupTransform(map_frame_, base_link_frame_,
                                      tf2::TimePointZero);
      pose->x() = static_cast<float>(tf.transform.translation.x);
      pose->y() = static_cast<float>(tf.transform.translation.y);
      pose->z() = static_cast<float>(tf.transform.translation.z);
      return true;
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN(get_logger(), "TF %s <- %s unavailable: %s",
                  map_frame_.c_str(), base_link_frame_.c_str(), e.what());
      return false;
    }
  }

  void planFromCurrentPose(const Eigen::Vector3f &goal) {
    if (!tree_) {
      RCLCPP_WARN(get_logger(), "no map loaded, ignoring goal");
      return;
    }
    Eigen::Vector3f start;
    if (!lookupRobotPose(&start)) return;

    Eigen::Vector3f start_snapped;
    Eigen::Vector3f goal_snapped;
    const float radius = static_cast<float>(snap_radius_);
    if (!retap_core::nearestDrivableVoxel(*tree_, start, radius,
                                          &start_snapped)) {
      RCLCPP_WARN(get_logger(),
                  "no drivable voxel within %.2f m of the robot at "
                  "[%.2f, %.2f, %.2f]",
                  snap_radius_, start.x(), start.y(), start.z());
      return;
    }
    if (!retap_core::nearestDrivableVoxel(*tree_, goal, radius,
                                          &goal_snapped)) {
      RCLCPP_WARN(get_logger(),
                  "no drivable voxel within %.2f m of the goal at "
                  "[%.2f, %.2f, %.2f]",
                  snap_radius_, goal.x(), goal.y(), goal.z());
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();
    Astar astar(tree_.get(), static_cast<float>(heuristic_weight_),
                static_cast<float>(traversability_weight_),
                static_cast<float>(distance_weight_));
    astar.SetStartAndGoal(start_snapped, goal_snapped);
    const std::vector<Eigen::Vector3f> path = astar.startSearch();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0)
            .count();
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "A* found no path (%.1f ms)", elapsed_ms);
      return;
    }

    nav_msgs::msg::Path msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    msg.poses.reserve(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = path[i].x();
      pose.pose.position.y = path[i].y();
      pose.pose.position.z = path[i].z();
      // Upstream leaves the quaternion zero, which is not a valid rotation;
      // use the horizontal tangent instead.
      const size_t next = std::min(i + 1, path.size() - 1);
      const size_t prev = (i == 0) ? 0 : i - 1;
      const double dx = path[next].x() - path[prev].x();
      const double dy = path[next].y() - path[prev].y();
      const double yaw = (dx == 0.0 && dy == 0.0) ? 0.0 : std::atan2(dy, dx);
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
      msg.poses.push_back(pose);
    }
    path_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "path: %zu waypoints in %.1f ms", path.size(),
                elapsed_ms);
  }

  std::string map_path_;
  std::string map_topic_;
  std::string map_frame_;
  std::string base_link_frame_;
  double heuristic_weight_ = 0.5;
  double traversability_weight_ = 0.5;
  double distance_weight_ = 1.0;
  double snap_radius_ = 1.0;

  std::shared_ptr<octomap::IgTree> tree_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      clicked_sub_;
#ifdef RETAP_HAS_OCTOMAP_MSGS
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
#endif
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RetapGlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
