# Test report — TurtleBot3 Burger maze navigation

Recorded: 2026-10-17 15:13 +0530 (Asia/Colombo). Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.11.0 / Harmonic. All movement tests used Gazebo, not physical hardware.

## Result

**Working autonomous Nav2 navigation verified.** Two fresh simulation runs each completed a short goal and then three consecutive reachable goals (8 successful action results total). A separate earlier short goal also returned success, but its ground-truth capture was incomplete; that trial remains marked failed by the verifier and is not counted among these eight.

The final run used the default Gazebo GUI and RViz launch options. The final project RViz view was loaded separately and the delivered startup uses that view. GUI processes/logs were checked; no screenshot-based visual QA or manual RViz goal clicking was performed. Goals were sent through the same Nav2 `NavigateToPose` action used by RViz.

## Diagnosed causes and repairs

1. The project simulation and Nav2 launch files were empty. Startup depended on separately launched stock commands with no project readiness or duplicate checks. Implemented both launch files and per-domain runtime locks/graph guards.
2. AMCL had `set_initial_pose: false` and zero-valued pose parameters. Existing logs repeatedly say AMCL cannot publish its pose/transform until initialized, followed by global-costmap waits for the missing `map` frame. Added a simulation-stamped pose at `(-0.764, -1.250, 0)` with nonzero covariance. Localization has a separate lifecycle manager; navigation processes start only after AMCL, sensor/map data and TF are ready.
3. The audit saw duplicate Nav2 node names, and historical launches overlapped. Replaced composed stock bringup with explicit standalone project servers. The logs do not establish one exact middleware cause for every historical unconfigured-AMCL episode; the demonstrated fix removes ambiguous concurrent initialization and the missing-pose dependency.
4. The stock robot-state-publisher launch declares simulation time false by default (the live audited node had already been manually corrected). The project starts it directly with simulation time true and no frame prefix, avoiding reliance on that manual correction.
5. Copied parameters used 0.30 m/s DWB speed, 0.25 m goal tolerance, duplicate obstacle/voxel layers for one laser, and a 0.2 s collision-monitor timeout versus a roughly 0.2 s scan interval. Retained DWB and NavFn Dijkstra, reduced speed/tolerances, used a conservative Burger envelope, simplified planar costmaps and allowed a 1.0 s scan timeout. The wrong single-pose BT override for through-poses was removed in favor of installed defaults.
6. The project SLAM launch instantiated a plain node without the installed SLAM Toolbox lifecycle startup sequence. It now includes the lifecycle-aware asynchronous launch and is mutually exclusive with AMCL.
7. The world install rule referenced an empty, untracked models directory. It now installs actual launch/world assets and uses the installed TurtleBot3 models. Runtime/test dependencies and executable installation were completed. Offline XML lint no longer depends on an unreachable remote schema hint; package parsing and XML checks pass.
8. The stock RViz view targeted x≈-5.41 m and disabled the robot model. A project view centers the 3 m maze at (0, 0), enables the Burger and removes unrelated dock, camera and voxel displays.

AMCL initialization and stamped velocity behavior follow the [Nav2 Jazzy AMCL guide](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/) and [Gazebo odometry guide](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/odom/setup_odom_gz/); installed launch/model files were also inspected directly.

## Actual checks

- `colcon build --symlink-install --packages-select maze_world maze_navigation`: passed; [build log](build.log).
- `colcon test --packages-select maze_world maze_navigation` and `colcon test-result --verbose`: **13 tests, 0 errors/failures/skips**; [result](colcon-test-result.txt), [full log](colcon-test.log). These are lint/package checks; runtime behavior was tested separately below.
- Python parsing/compilation, shell environment syntax and `git diff --check`: passed. Additional navigation/audit style check: [log](navigation-style.log).
- `gz sdf -k src/maze_world/worlds/maze_fixed.sdf`: valid. Read-only reference-map audit found **20 wall links, 1,895 occupied pixels and 0 mismatched pixels** in the 128 × 128 map; [JSON](static-audit.json).
- Map server, AMCL, controller, smoother, planner, behaviors, velocity smoother, collision monitor, BT navigator and both costmaps: **active**. Required nodes and robot_state_publisher: simulation time true. Map→odom→base_footprint→base_scan TF and both planning/navigation action servers available; [runtime evidence](navigation-final-trials.json).
- Laser frame `base_scan`, roughly 5 Hz simulated scan and 50 Hz odometry; measured wall rates are included in JSON. `/map` was 128 × 128 at 0.025 m. `/cmd_vel` was `geometry_msgs/msg/TwistStamped`.
- Duplicate simulation, duplicate navigation, SLAM while AMCL active and AMCL while SLAM active were rejected before conflicting nodes started; [guard checks](duplicate-guards.json), [reverse conflict log](nav2-slam-conflict.log).
- Corrected/original SDFs, reference-map files and all existing DQN files match pre-edit SHA-256 checksums; [manifest](preserved_assets.sha256), [result](preserved-assets-result.txt). Pre-edit source copies are in `backups/2026-10-17-before-nav2-fix/`.

## Final four-goal run

| Trial | Goal (x, y, yaw) | Action / error | Wall s | Map error cm | Gazebo error cm | Min circle-to-wall cm | Gazebo samples |
|---|---|---|---:|---:|---:|---:|---:|
| Short | (-0.45, -1.25, 0.000) | SUCCEEDED / 0 | 7.1 | 5.9 | 4.9 | 9.4 | 414 |
| 1 | (0.75, -1.25, 0.000) | SUCCEEDED / 0 | 18.3 | 5.2 | 2.8 | 7.1 | 1066 |
| 2 | (1.25, -1.25, 1.571) | SUCCEEDED / 0 | 16.5 | 4.5 | 4.8 | 8.1 | 960 |
| 3 | (1.25, -0.75, 1.571) | SUCCEEDED / 0 | 7.7 | 5.9 | 3.6 | 7.8 | 448 |

All trials generated nonzero stamped commands and measurable Gazebo displacement. Before movement, NavFn successfully planned each goal and the planned path passed a wall-clearance check. No watchdog tripped. Collision checking sampled a conservative 0.13 m enclosing circle against corrected SDF wall boxes, with positive clearance throughout; no contact sensor was installed, so no contact-event measurement is claimed.

Evidence: [final JSON](navigation-final-trials.json), [final trial log](navigation-final-trials.log), [startup log](nav2-final-start.log); independent prior run: [JSON](navigation-trials.json).

## Separate genuine LiDAR SLAM check

Stopped AMCL/Nav2 before mapping. SLAM Toolbox automatically configured and activated, registered the LiDAR and published a **44 × 62 map at 0.025 m**. Saved it to [maze_lidar_smoke.yaml](slam-smoke/maze_lidar_smoke.yaml) and its PGM. This is a stationary local observation after the navigation trials, **not a complete maze survey**. The geometry-generated reference map was not overwritten or relabeled.

The first map-save attempt hit the CLI's default 2 s subscription timeout. With a 10 s timeout and transient-local map subscription it saved successfully. Both attempts are retained: [initial log](slam-save.log), [successful log](slam-save-with-timeout.log), [summary](slam-smoke-result.json). The README uses the successful command.

## Limits and remaining observations

- No blocker remains for the demonstrated simulation startup/localization/goal route. Other maze routes, prolonged reliability, physical hardware, and a complete SLAM survey were not tested. EGL `headless:=true` was not exercised; server-only with a display and default GUI startup were exercised.
- RViz emits one first-map `indexed_8bit_image` GLSL sampler-link error. The process remained alive and received map/costmap updates; this matches the [open upstream RViz issue](https://github.com/ros2/rviz/issues/463). No changes were made under `/opt/ros/jazzy`; visual rendering correctness was not asserted from logs alone.
- Brief initial TF extrapolation messages appeared while separate DDS/clock/TF listeners populated their caches. Localization and navigation then reached active automatically; no persistent TF wait or manual lifecycle recovery was required. The installed Burger model also emits its usual `gz_frame_id` extension warning.
- The included startup commands are for the default spawn. Restarting only Nav2 after movement requires current-pose arguments. Start from a fresh simulation to repeat the supplied sequence.

See [README startup and demonstration commands](../README.md). The delivered live session is returned to the default spawn with simulation, AMCL/Nav2 and one project RViz view running. Its final read-only verification passed: [readiness JSON](delivered-session-readiness.json), [process inventory](delivered-processes.txt).

Follow-up: a later manual top-row trial exposed localization input error and NavFn's fixed long-route extraction budget. Both were diagnosed; see [the repair report](TOP_RIGHT_REPAIR.md). The new plugin and five regression cases bring the current package-check total to 20. The long drive itself remains to be demonstrated manually.
