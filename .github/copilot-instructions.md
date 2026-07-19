# ROS 2 / Nav 2 + Gazebo Sim — Copilot Instructions

This workspace follows **Clean Architecture** for ROS 2 (Python + C++) and
the **Entity-Component-System** model for Gazebo Sim (`gz-sim`) plugins.
Full reference material (not all loaded here) lives in `.claude/rules/`
and `.claude/skills/` — read those files directly for anything not
covered below (Nav 2 parameter tuning, VDA 5050 fleet protocol,
ros2_control, BehaviorTree.CPP, etc.).

## Layering (ROS 2 packages)

```
src/<pkg>/<pkg>/
├── domain/           # entities, value objects, ports — NO ROS imports
├── application/      # use cases — depends only on domain
├── infrastructure/   # rclpy/rclcpp nodes, TF, repositories — implements domain ports
└── presentation/     # CLI, launch entrypoints
```

**Hard rule:** `domain/` must never import `rclpy`, `rclcpp`, or any
`*_msgs` package. If code needs ROS, it does not belong in `domain/`.
Dependencies point inward: presentation → application → domain ←
infrastructure (infrastructure implements domain interfaces, it does not
get depended upon).

## Node conventions

- **Lifecycle by default** for anything owning a resource (sensor,
  actuator, costmap, hardware bridge) — use `rclcpp_lifecycle::LifecycleNode`
  / `rclpy` managed nodes, not plain nodes, when the resource has
  meaningful configure/activate/cleanup states.
- **`declare_parameter` for every parameter.** Calling `get_parameter` on
  an undeclared name is a bug, not a shortcut.
- Package names: snake_case, `<org>_<component>_<type>`.

## QoS — match reliability to semantics

| Data type | Reliability | Durability | Depth |
|---|---|---|---|
| Sensor data (LiDAR, camera, IMU) | BEST_EFFORT | VOLATILE | 1–5 |
| Commands (`cmd_vel`) | RELIABLE | VOLATILE | 10 |
| State/config (latched) | RELIABLE | TRANSIENT_LOCAL | 1 |
| TF | RELIABLE | VOLATILE | 100 |

## Testing

- Tests run via `colcon test` (`colcon test-result --all` to see
  failures). Never `sleep(N)` to synchronize a test — use futures,
  conditions, or `launch_testing.ReadyToTest`.
- Structure: `test/unit/`, `test/integration/`, `test/e2e/` (launch tests).
- Python → pytest; C++ → GTest.

## Common commands

```bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select <pkg> && colcon test-result --all
ros2 launch <pkg> <file>.launch.py
```

## Where to look for more (do not duplicate here — read the source file)

| Need | File |
|---|---|
| Full layer-dependency rules + code examples | `.claude/rules/clean_architecture.md` |
| Node/lifecycle/parameter patterns | `.claude/rules/ros2_nodes.md` |
| Topic naming, QoS, custom msg/srv/action | `.claude/rules/ros2_communication.md` |
| Test patterns (unit/integration/launch) | `.claude/rules/testing.md` |
| Nav 2 architecture / parameters / msgs | `.claude/rules/nav2_*.md` |
| ros2_control framework + controller catalog | `.claude/rules/ros2_control_*.md` |
| BehaviorTree.CPP / BehaviorTree.ROS2 | `.claude/rules/behaviortree_*.md` |
| VDA 5050 fleet protocol | `.claude/rules/vda5050_*.md` |
| Step-by-step recipes (scaffold a node, package, plugin...) | `.claude/skills/*/SKILL.md` |

## Note on paths in `.claude/rules/`

Some rule files reference a `~/nav2_ws/src/...` source tree (vendored
copies of Nav 2, ros2_control, BehaviorTree.CPP, VDA 5050 repos, etc.)
that does not exist on this machine yet. Treat those as illustrative
source pointers, not live paths, until that tree is actually checked
out.
