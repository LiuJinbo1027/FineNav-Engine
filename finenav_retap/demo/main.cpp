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
    //    EventDriven, not Periodic: the robot does not move in phase 1, and a
    //    periodic policy would re-plan the same path and log the same WARN.
    //    S5's online mapping switches this back to PeriodicPolicy.
    if (!engine->createControlLayerByWorldView(
            "retap_world_view", "retap_layer",
            finenav::core::EventDrivenPolicy{})) {
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
