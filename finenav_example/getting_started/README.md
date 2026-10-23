# Getting Started — Simulation

Minimal FineNav-Engine navigation stack driven by a simulator.

```
/odom → FineNavLocalizer (ESKF) → BtNavigator → ComputePlan("my_layer")
                                                    ↓
                                            ControlLayer<DummyPlanner>
                                                    ↓
                                               /cmd_vel
/goal_pose → FineNavEngine → navigateTo()
```

## Prerequisites

- Ubuntu 22.04 + ROS 2 Humble
- Workspace built: `colcon build --symlink-install && source install/setup.bash`

## Quick Start

### Terminal 1 — Simulator

```bash
ros2 launch finenav_simulator finenav_simulator.launch.py \
  lio_type:=static \
  use_sim_time:=true \
  world_name:=flat_small_scale_maze.world \
  publish_map_tf:=true
```

### Terminal 2 — FineNav-Engine

```bash
cd ~/FineNav_Engine_ws && source install/setup.bash
ros2 run finenav_example hello_finenav \
  --ros-args --params-file src/FineNav-Engine/finenav_example/getting_started/params/sim_localizer.yaml
```

### Terminal 3 — Set Initial Pose

`/initialpose` is a **one-shot absolute pose**: the localizer folds it into its
world anchor once, then drops the subscription.  It works even though
`pose_topic` is empty here.  Rviz2's *2D Pose Estimate* tool publishes to the
same topic — either is fine.
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {
    stamp: {sec: 16, nanosec: 360000000},
    frame_id: 'odom'
  },
  pose: {
    pose: {
      position: {
        x: 2.000000000423794,
        y: 2.0,
        z: 0.7500389896494932
      },
      orientation: {
        x: 1.617940655980588e-18,
        y: 2.638455507896242e-10,
        z: 4.989162198923731e-21,
        w: 1.0
      }
    },
    covariance: [
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ]
  }
}"
```

> **The pose must match where the robot actually is.**  The message is taken as
> the robot pose in the world frame — `frame_id` is ignored, no TF lookup is
> done — and the estimator then holds `state = anchor ⊕ odom`.  A wrong initial
> pose therefore shifts *every* later estimate by the same offset: publish
> x = 3.0 for a robot at x = 2.0 and the pose reads 1 m off forever.
> The values above are the true pose for this world (spawn 2.0, 2.0 → `base_link`
> at z ≈ 0.75).  `stamp` only tags the history sample — the example keeps a
> stale one from a recording, which is harmless for a single publish.
>
> Because it is one-shot, a second publish finds no subscriber.  To re-set the
> pose, restart Terminal 2.

Now, you can use Rviz2 to publish a goal pose. The dummy planner computes a twist toward the goal.  Verify with `ros2 topic echo /cmd_vel`.

## How It Works

`FineNavEngine` subscribes to `/goal_pose` and triggers `navigateTo()`.  The BT ID is set in `main.cpp`:

```cpp
engine->setDefaultBehaviorTree("GettingStartedNavigate");
```

The behavior tree (`behavior_trees/navigate_to_pose.xml`) wires the example's
`layer="my_layer"` and `planner_id="dummy_planner"` via TaskChannel IPC.
Navigator reads feedback from the layer and publishes `/cmd_vel`.

## Key Parameters (`sim_localizer.yaml`)

| Section | Parameter | Purpose |
|---|---|---|
| `hello_finenav/finenav_localizer` | `odom_topic: "/odom"` | Subscribe to simulator ground truth |
| | `odom_use_twist: true` | Take velocity from `/odom.twist` |
| | `initial_pose_topic: "/initialpose"` | One-shot absolute pose (Terminal 3) |
| | `publish_tf: false` | TF is the simulator's job — see below |
| | `force_2d_mode: true` | Planar constraint |
| `hello_finenav/finenav_navigator` | `behavior_trees` | Load the example's BT XML |

Keep `publish_tf: false` here.  The simulator already supplies the whole chain
(`map→odom` identity, plus `odom→base_link` from `simulator_helper`), while the
localizer broadcasts `map→base_link` **directly** — the framework has no odom
frame — so enabling it gives `base_link` two parents.

## Troubleshooting

| Symptom | Check |
|---|---|
| Robot doesn't move | `ros2 topic echo /cmd_vel` |
| No TF | Simulator's `map→odom` + `odom→base_link` (keep localizer `publish_tf: false`) |
| `no absolute pose source ... will drift` WARN | Harmless with ground-truth `/odom` — publish `/initialpose` to silence it |
| Robot pose offset by a constant | A wrong `/initialpose` shifted the anchor |
| "behavior tree ID is empty" | `setDefaultBehaviorTree()` was not called |
| "Failed to create tree" | Ensure `behavior_trees` includes the example's BT directory |
