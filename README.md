# TurtleBot3 Burger maze navigation

Existing ROS 2 Jazzy / Gazebo Harmonic project with reference-map AMCL, Nav2 navigation and DQN maze-route learning. The corrected world is **`src/maze_world/worlds/maze_fixed.sdf`**. The original world and both map files are preserved. The formerly empty DQN package is now enabled; its original files are backed up in `backups/2026-10-17-before-dqn/`.

The maze has 6 × 6 cells at 0.5 m spacing, 0.05 m walls, and approximately 0.45 m clear corridors. `maze_map.pgm` is a **geometry-generated reference occupancy map, not a SLAM result**. Its 128 × 128 pixels at 0.025 m/pixel and origin `[-1.6, -1.6, 0]` match the corrected SDF wall boxes exactly at pixel centers. See [the test report](reports/TEST_REPORT.md) for measured navigation results and limitations.

## Build and start

Use the installed Ubuntu 24.04 system ROS/Python packages, not a Python virtual environment. Required ROS packages are declared in the two active `package.xml` files: TurtleBot3 Gazebo, ros_gz, robot_state_publisher, Nav2 (including DWB, NavFn, collision monitor and velocity smoother), SLAM Toolbox and RViz. No files under `/opt/ros/jazzy` are modified.

From this workspace:

```bash
cd /mnt/storage/Projects-external/robotics/turtlebot3_maze_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select maze_world maze_navigation
source scripts/maze_env.bash
```

Terminal 1 — simulation (Gazebo GUI enabled by default):

```bash
cd /mnt/storage/Projects-external/robotics/turtlebot3_maze_ws
source scripts/maze_env.bash
ros2 launch maze_world simulation.launch.py
```

Use `gui:=false` for server-only operation with an available display; `gui:=false headless:=true` requests EGL rendering without a display. The tested rendering mode is recorded in the report. The launch starts the corrected maze, spawns one Burger at `(-0.75, -1.25)`, starts one ROS/Gazebo bridge and starts robot_state_publisher with simulation time and unprefixed frame names. Settled robot x is approximately `-0.764`, so localization uses that coordinate.

Terminal 2 — AMCL and Nav2 (RViz enabled by default):

```bash
cd /mnt/storage/Projects-external/robotics/turtlebot3_maze_ws
source scripts/maze_env.bash
ros2 launch maze_navigation nav2.launch.py
```

The project RViz view centers the maze and shows the Burger, laser and paths. Use `rviz:=false` to omit RViz or `rviz_config:=...` for another view. Wait for **`Localization ready`**, followed by navigation's **`Managed nodes are active`**. No manual lifecycle transitions or initial-pose clicks are needed at the default spawn.

The launch supports `map:=/absolute/path/to/map.yaml`, `params_file:=...`, and `initial_x:=... initial_y:=... initial_yaw:=...`. Set these to the robot's **current** pose when restarting Nav2 after moving; the defaults describe the initial spawn, not its last pose. For a repeatable demonstration, restart the simulation and Nav2 together.

For a clean reset, including a leftover maze server after a terminal was closed, run from this workspace:

```bash
./scripts/stop_maze.py --dry-run  # Show this workspace's matching processes.
./scripts/stop_maze.py            # Stop them, then start Terminals 1 and 2 again.
```

This helper targets project launches identified by their workspace log environment, their children, and a Gazebo server loading this workspace's exact corrected-world path. It checks process start times before signaling and leaves unrelated processes alone. It does not delete lock files.

Stop a launch with **Ctrl+C**, and wait for its children to exit before restarting. Never use Ctrl+Z. Runtime locks prevent duplicate project launches in one ROS domain; graph checks also reject legacy AMCL/SLAM/navigation nodes. Simulation checks for an existing maze server in its Gazebo partition. If a guard reports a conflict, inspect `ros2 node list --no-daemon --spin-time 5` and stop the relevant owning launch. Do not kill unrelated ROS sessions. Keep the same `ROS_DOMAIN_ID` and Gazebo partition across these terminals.

## Navigation configuration

The classical global planner is **NavFn Dijkstra** (`maze_navigation::MazeNavfnPlanner`, a thin extension of `nav2_navfn_planner::NavfnPlanner`, `use_astar: false`, `allow_unknown: false`, goal-search tolerance 0.05 m). The project plugin retries path extraction only when the stock planner exhausts its fixed `4 * max(map width, map height)` half-cell step budget. This handles winding routes longer than approximately 6.4 m on the unchanged 128-pixel reference map. It retains NavFn's potential, collision costs and Dijkstra algorithm; disconnected goals still fail. See [the long-route repair report](reports/TOP_RIGHT_REPAIR.md). DWB remains the local controller. Its forward pruning distance is 0.40 m, backward pruning distance 0.20 m, and PathAlign weight 16, preventing distant maze segments and excessive alignment cost from stalling progress around bends. Speeds are limited to 0.10 m/s and 0.65 rad/s, with gentle acceleration; the goal checker uses 0.06 m position and 0.12 rad yaw tolerances.

Both costmaps use `base_footprint`, 0.025 m resolution, one 2D laser obstacle layer and 0.20 m inflation. The 0.12 m robot radius plus 0.01 m padding conservatively covers the Burger collision body. Removing duplicate obstacle/voxel processing avoids counting one planar laser twice. Inflation costs encourage a corridor center without making the entire 0.45 m corridor lethal.

The command chain is:

```text
controller_server / behavior_server
  -> /cmd_vel_nav (TwistStamped)
  -> velocity_smoother
  -> /cmd_vel_smoothed (TwistStamped)
  -> collision_monitor
  -> /cmd_vel (TwistStamped)
  -> Gazebo bridge -> Burger DiffDrive
```

The collision monitor uses a 0.13 m stop circle and a 1.0 s scan timeout. At approximately 5 Hz laser frequency, the old 0.2 s timeout was too short. The monitor stops commands if observations become stale.

Localization has its **own lifecycle manager and separate processes**. The initialization helper waits for an active AMCL, map, clock, laser, odometry and robot TF, publishes `/initialpose` with simulation time, a unit quaternion and nonzero x/y/yaw covariance, then waits for AMCL pose and `map -> odom -> base_footprint -> base_scan`. Navigation servers are created only after this helper succeeds. A 90 s wall-time timeout stops startup with a diagnostic rather than activating navigation without localization. Costmap lifecycle is owned by its planner/controller server; do not add the costmaps separately to the lifecycle manager.

## Manual pose estimate for the assignment

After a fresh simulation startup, the robot is in the bottom row, second cell from the left, facing **right**. The launch already initializes this pose. To demonstrate the manual tool, select **2D Pose Estimate**, press at the robot's actual location, drag **right**, and release. This arrow describes its **current** heading. For all destinations, use **Nav2 Goal**: the press location is the destination and the drag direction is its desired arrival heading. The far top-right cell is `(1.25, 1.25)`; `(0.75, 1.25)` is the adjacent cell to its left.

An incorrect pose estimate changes AMCL's belief without moving the Gazebo robot. Misplaced laser observations can then block valid corridors in the costmaps. If this happens, perform a clean simulation/Nav2 restart with `./scripts/stop_maze.py`, then use the known spawn pose. Do not click the destination with **2D Pose Estimate**.

## Verify and demonstrate goals

Terminal 3, after Nav2 reports active:

```bash
cd /mnt/storage/Projects-external/robotics/turtlebot3_maze_ws
source scripts/maze_env.bash
ros2 run maze_navigation verify_navigation.py --output reports/readiness.json
```

The default is read-only: it checks all required lifecycle states, simulation-time parameters, fresh `/scan` and `/odom`, `/map`, TF, action servers, duplicate names and the stamped command type. It also reads named Gazebo `burger` poses through `gz topic --json-output`.

ROS client creation and cleanup took over a minute in some observed diagnostic runs. Allow the verifier to finish; its JSON output separates readiness failures from navigation results.

For the full demonstration from a fresh default spawn:

```bash
ros2 run maze_navigation verify_navigation.py \
  --goals all --output reports/my-navigation-trials.json
```

This first tests a short goal `(-0.45, -1.25, 0)`, then these three reachable goals in order:

| Goal | x (m) | y (m) | yaw (rad) |
|---|---:|---:|---:|
| 1 | 0.75 | -1.25 | 0.0 |
| 2 | 1.25 | -1.25 | 1.5708 |
| 3 | 1.25 | -0.75 | 1.5708 |

Use `--goals short` for only the short test or `--goals three` for only the three goals. Every goal is preplanned using NavFn, checked against SDF walls, then sent through `NavigateToPose`. The verifier stops the sequence on failure and cancels timed-out goals. JSON records action status/error, map and Gazebo final poses, actual displacement, nonzero velocity commands and sampled wall clearance. Clearance is measured from a conservative 0.13 m circle to actual wall boxes; there is no contact sensor, so this is a geometric collision check at sampled poses.

To test the winding route to the far top-right cell, allow up to five minutes:

```bash
ros2 run maze_navigation verify_navigation.py \
  --goals top_right --timeout 300 --output reports/my-top-right-trial.json
```

Keep RViz idle while an automated trial is running. For another manual destination afterward, select **Nav2 Goal**; localization continues tracking the robot as it moves.

In RViz, use **Nav2 Goal** to send the same three coordinates/orientations in order, waiting for each result. `ros2 action echo` is not needed: the automated verifier gives explicit action results. This route verifies translation, a turn, and travel through a 0.5 m cell corridor; it does not establish coverage of every maze route.

Offline checks:

```bash
/usr/bin/python3 scripts/audit_project.py
source /opt/ros/jazzy/setup.bash
gz sdf -k src/maze_world/worlds/maze_fixed.sdf
colcon test --packages-select maze_world maze_navigation
colcon test-result --verbose
sha256sum -c reports/preserved_navigation_assets.sha256
```

## Deep Q-learning route demonstration

The [Nav2 versus DQN evaluation](evaluation/README.md) compares actual Nav2 planning
and the trained policy on 105 identical start–goal pairs, with CSV/JSON data,
clearance checks, PNG/PDF plots and commands to reproduce the experiment.

See [DQN setup, training and evaluation](src/maze_dqn/README.md) and the [measured results](reports/DQN_REPORT.md). DQN learns cell-to-cell routes from rewards; Nav2's DWB controller executes the learned path directly through `FollowPath`. RViz's **Nav2 Goal** tool continues using the classical NavFn planner. Run DQN from its own command below.

With the simulation and Nav2 terminals running, preview a learned route without moving:

```bash
source scripts/dqn_env.bash
python -m maze_dqn.follow --model models/dqn/maze_dqn.pt --goal-cell 4
```

Add `--execute` to drive to cell 4 in the bottom corridor, or choose the other trained goals 11 and 35 (top-right). Do not send a competing RViz goal during execution:

```bash
python -m maze_dqn.follow --model models/dqn/maze_dqn.pt \
  --goal-cell 4 --yaw 0 --execute --output reports/my-dqn-bottom.json
python -m maze_dqn.follow --model models/dqn/maze_dqn.pt \
  --goal-cell 35 --yaw 1.5707963268 --execute --output reports/my-dqn-top-right.json
```

Training is offline in the fixed SDF-derived grid, followed by separate Gazebo execution. It does not learn steering from LiDAR or establish performance on new mazes. The supplied checkpoint and all three training seeds' results are documented in the DQN report.

## Genuine LiDAR SLAM (separate mode)

Stop **Nav2/AMCL completely** with Ctrl+C; keep only the simulation running. Then:

```bash
source scripts/maze_env.bash
ros2 launch maze_navigation slam.launch.py
```

This uses the installed SLAM Toolbox lifecycle-aware `online_async_launch.py` with the existing project parameters. It configures and activates mapping automatically. The launch refuses to run alongside AMCL. Its map frame starts at the robot's initial odometry origin, so a saved SLAM map will generally have different world coordinates from the geometry-generated reference.

In another terminal, drive slowly with stamped teleoperation, explore all corridors, and save to a **new** filename:

```bash
source scripts/maze_env.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_footprint \
  -p speed:=0.08 -p turn:=0.5
# In another terminal after exploration:
source scripts/maze_env.bash
mkdir -p maps/slam
ros2 run nav2_map_server map_saver_cli \
  -f "$PWD/maps/slam/maze_lidar" --ros-args -p use_sim_time:=true \
  -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

The teleop keyboard requires its ROS package to be installed. Saving does not complete a mapping survey: unobserved areas remain unknown until explored. For returning to AMCL, stop SLAM, use the saved YAML via `map:=...`, and provide the initial pose expressed in that saved map. A genuine mapping smoke test and any map-save test are reported separately from autonomous navigation; no geometry map is relabeled as SLAM.

## Files and evidence

- `src/maze_world/launch/simulation.launch.py`: corrected-world simulation and Burger startup.
- `src/maze_navigation/launch/nav2.launch.py`: staged localization and standalone Nav2 lifecycle.
- `src/maze_navigation/config/nav2_params.yaml`: Burger DWB, NavFn Dijkstra, costmaps and stamped velocity chain.
- `src/maze_navigation/config/maze_navigation.rviz`: maze-centered robot, laser, map, costmap and goal view.
- `src/maze_navigation/scripts/initialize_localization.py`: automatic stamped AMCL initialization and TF gate.
- `src/maze_navigation/scripts/verify_navigation.py`: read-only checks and explicit goal trials.
- `src/maze_navigation/launch/slam.launch.py`: separate lifecycle-aware mapping mode.
- `scripts/audit_project.py`: structural/map alignment validation; `scripts/maze_env.bash`: common shell environment.
- `reports/TEST_REPORT.md` and JSON/log evidence: what was actually verified.
- `backups/2026-10-17-before-nav2-fix/`: pre-edit copies; `backups/COLCON_IGNORE` excludes backups from package discovery.
