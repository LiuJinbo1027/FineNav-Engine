// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// retap_demo — assemble the ReTAP integration inside FineNav-Engine.
//
// S4: the Map (static terrain .bt), the Planner (ReTAP's 26-neighbour A*) and
// the WorldView (search box + snapping) are assembled into one control layer;
// the navigator's behaviour tree drives it:
//
//   goal ─► _root ─► retap_layer (EventDriven) ─► __feedback ─► Trajectory
//
// The map server is created explicitly first so its params come from the
// <engine>/retap_terrain_map block; createControlLayerByWorldView() then
// resolves the same map type back to that server instead of creating a second
// one (FineNavEngine::ensureMapServer).
//
// The algorithm libraries are deliberately NOT linked (see CMakeLists.txt):
// loadAlgoPlugins() dlopens them so their static registrars fire.

#include <memory>
#include <string>
#include <utility>
#include <variant>

#include <rclcpp/rclcpp.hpp>

#include "finenav_engine_cpp/finenav_engine.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("retap_demo");
    auto engine = std::make_shared<finenav::FineNavEngine>(node);

    // 0. The tree used by /goal_pose and navigateTo() without a bt_id.
    engine->setDefaultBehaviorTree("RetapNavigate");

    // 1. dlopen install/finenav_retap/lib — the map, planner and world view
    //    plugins register themselves there.
    if (!engine->loadAlgoPlugins({"finenav_retap/lib"})) {
        RCLCPP_ERROR(node->get_logger(), "failed to load finenav_retap plugins");
        rclcpp::shutdown();
        return 1;
    }

    // 2. Create the map server first.  Its private node is
    //    /retap_demo/retap_terrain_map, so the parameters live under that key
    //    in demo/params/retap_demo.yaml.
    if (!engine->createMapServer("retap_terrain_map", "retap_terrain_map")) {
        RCLCPP_ERROR(node->get_logger(),
                     "failed to create 'retap_terrain_map' — is the plugin registered?");
        rclcpp::shutdown();
        return 1;
    }

    // 3. Assemble the layer from the WorldView bridge by NAME.  The framework
    //    resolves the planner and map TYPES the bridge declares back to their
    //    registered names, reuses the map server created above, builds the
    //    view and attaches it.
    //
    //    S4 runs EventDriven: the robot does not move and the map is static, so
    //    "plan once per goal" is exactly right.  S5 runs Periodic: the map
    //    grows under the robot, so plan() is called on a timer and its own
    //    replan throttle (replan_min_distance / replan_period_sec /
    //    replan_on_map_change) decides whether to search again.
    const std::string planner_policy =
        node->declare_parameter<std::string>("planner_policy", "event");

    finenav::core::ExecutionPolicyVariant policy;
    if (planner_policy == "periodic") {
        // PeriodicPolicy::declare_parameters() declares frequency_hz on the
        // layer's private node, so the rate is set there:
        //   <engine>/<layer>/ros__parameters/frequency_hz
        policy = finenav::core::PeriodicPolicy{};
    } else if (planner_policy == "event") {
        policy = finenav::core::EventDrivenPolicy{};
    } else {
        RCLCPP_ERROR(node->get_logger(),
                     "planner_policy must be 'event' or 'periodic' (got '%s').",
                     planner_policy.c_str());
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "planner_policy=%s", planner_policy.c_str());

    if (!engine->createControlLayerByWorldView(
            "retap_world_view", "retap_layer", std::move(policy))) {
        RCLCPP_ERROR(node->get_logger(),
                     "failed to assemble 'retap_layer' from WorldView "
                     "'retap_world_view'.");
        rclcpp::shutdown();
        return 1;
    }

    // 4. Declare the branch the behaviour tree activates.
    if (!engine->registerControlBranch("navigate", {"retap_layer"})) {
        RCLCPP_ERROR(node->get_logger(), "failed to register branch 'navigate'.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(),
                "ReTAP navigation ready: send a goal on /retap_demo/navigate_to_pose.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
