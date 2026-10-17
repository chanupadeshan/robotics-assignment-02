# Top-row goal repair — 2026-10-17

The user's Nav2 goal aborted after repeated planning failures and recoveries. The recorded click was `(0.74, 1.23)`, the cell adjacent to the far top-right cell `(1.25, 1.25)`.

Two independent causes were reproduced:

1. **Incorrect manual localization.** Gazebo measured the robot at `(-0.4602, -1.2414, -1.7332 rad)`, while AMCL estimated `(-0.4956, -0.9422, -1.7370 rad)`, roughly 30 cm north of its actual position. AMCL logs recorded earlier manual pose estimates at `(-0.778, -1.231, 1.648 rad)` and `(-0.872, -0.781, 0.881 rad)`. Misplaced laser returns and their inflation put the estimated start cell at cost 253. Planning even back to the initial spawn failed. A simulation-stamped initial pose from stationary Gazebo ground truth, followed by clearing both costmaps, restored localization to within about 4 mm and restored planning to `(0.74, 1.23)` without moving the robot. Evidence: [before](top-route-diagnosis-before.json), [pose correction](top-route-relocalized.json), [after](top-route-diagnosis-after.json).
2. **Stock NavFn extraction budget.** After localization was corrected, the start and far top-right goal were in the same traversable costmap component, but stock NavFn still failed. Jazzy limits extracted paths to `4 * max(width_cells, height_cells)` steps; on this 128 × 128 map that is 512 half-cell steps, approximately 6.4 m. The longer winding route exceeded this limit. [Upstream source](https://api.nav2.org/nav2-jazzy/html/navfn__planner_8cpp_source.html) documents the fixed budget in `getPlanFromPotential()`.

## Project change

Added `maze_navigation::MazeNavfnPlanner`, a small subclass of the installed NavFn planner. It runs stock NavFn first and retries extraction only when the exact fixed extraction budget was exhausted. The retry reuses the existing potential, costmap and half-cell step size, with a finite budget proportional to map area. Dijkstra remains selected. It does not reduce robot clearance, allow unknown space, enlarge goal tolerance or change either reference-map file. Genuine disconnected goals and cancellations remain failures. The plugin is installed and exported through pluginlib, with configuration and package/build dependencies updated.

The stock implementation can log an extraction error before the subclass successfully retries. A subsequent `NavFn extended extraction succeeded` message identifies recovery; the action result is authoritative. No files under `/opt/ros/jazzy` were modified.

## Validation

- Build passed. `colcon test-result --verbose`: **20 tests, 0 errors/failures/skips** across both active packages. [Result](long-route-test-result.txt), [test log](long-route-tests.log), [build](long-route-build.log).
- A captured raw costmap fixture reproduces the failure: stock extraction stops at exactly 512 steps, while the new plugin produces the longer valid path.
- Five regression cases cover the long connected route, identical short-route behavior, rejection of a disconnected route after a previous long plan, immediate cancellation, and cancellation before extended extraction.
- A temporary, separately namespaced planner loaded the exported plugin against the live map. It successfully planned short, clicked-top-row and far-top-right goals. All planned poses passed the conservative 0.13 m enclosing-circle clearance check against corrected SDF walls. [Live planning evidence](long-route-planner-verification.json).
- Corrected/original worlds, reference maps and DQN files retained their original checksums. [Preservation check](preserved-assets-after-long-route.txt), [static audit](static-audit-after-long-route.json).

## Follow-up drive and controller correction

The next user session loaded the new planner correctly but recorded a manual initial heading of `0.93044 rad` (53 degrees) at the fresh spawn, whose actual heading was zero. Following recovery rotations, reinitializing AMCL from the stationary simulator pose and clearing both costmaps restored planning again. [Pose correction and successful plans](repeated-goal-relocalized.json).

An autonomous drive then reached the top row but repeatedly lost progress at a bend and was canceled at the verifier's 300 s limit. Localization remained aligned, and the sampled 0.13 m circle retained at least 4.8 cm clearance. This attempt is explicitly a **failed** trial: [JSON](top-right-drive-trial.json).

DWB evaluation showed stationary/rotating candidates could beat moving candidates on alignment cost. With the existing 2 m local map, the default forward pruning distance allowed path segments up to 1 m away to influence the local target, spanning several 0.5 m maze cells. The trial settings change `FollowPath.forward_prune_distance` to 0.40 m, `prune_distance` to 0.20 m and `PathAlign.scale` from 32 to 16. Speed limits, robot radius, inflation and collision checks are unchanged. These settings use the installed DWB implementation; see its [path transformation source](https://api.nav2.org/nav2-jazzy/html/dwb__local__planner_8cpp_source.html) and [parameter guide](https://ros-navigation.github.io/mkdocs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/dwb_params/controller/).

After reloading the navigation lifecycle while retaining localization, a continuation from `(0.190, 1.142)` to the far top-right goal succeeded. Simulator position error was 4.8 cm, with at least 5.1 cm sampled enclosing-circle clearance. [Successful continuation](top-right-tuned-continuation.json).

Two complete winding-route drives then succeeded with the saved controller settings:

| Drive | Planned length | Path poses | Wall time | Gazebo position error | Min circle-to-wall clearance |
|---|---:|---:|---:|---:|---:|
| Top-right to starting area | 6.36 m | 508 | 88.8 s | 7.8 cm | 6.7 cm |
| Starting area to top-right | 6.78 m | 541 | 83.5 s | 4.7 cm | 6.6 cm |

Both returned `NavigateToPose` status 4, error code 0, generated nonzero stamped velocity commands and passed final position/heading checks against Gazebo. The forward plan exceeds stock NavFn's 512-point extraction budget. These were continuous autonomous drives, without teleporting the robot or injecting pose corrections during motion. The second drive started at the returned pose `(-0.748, -1.174, -0.116 rad)`, approximately 8 cm from the default spawn, rather than from a newly restarted simulation. Clearance is a sampled geometric check of a conservative 0.13 m circle against SDF walls; no contact sensor was installed. Evidence: [return drive](top-right-return-trial.json), [full forward drive](top-right-full-route.json), [running controller parameters](controller-tuned-parameters.json).

The verifier's startup checks also initially encountered stale TF after blocking service-discovery waits. Discovery now keeps clock/sensor/TF callbacks running, and observations are refreshed after service checks. The normal read-only verifier subsequently passed: [readiness](controller-tuned-readiness-retry.json). Added `--goals top_right --timeout 300` for repeatable goal testing. Package checks passed again: [20 tests, no failures](controller-tuning-test-result.txt). The [static audit](static-audit-after-controller-tuning.json) and [asset checksum check](preserved-assets-after-top-right.txt) also passed.

The delivered session is left stopped at the top-right goal, facing approximately upward. Simulation, AMCL/Nav2 and the user's RViz remain running. A further stationary check at this pose also passed: [final readiness](top-right-final-readiness.json). ROS middleware entity creation/destruction was slow in the diagnostic process; a captured [Python stack trace](readiness-debug.log) identifies waits inside `rclpy` client/subscription creation and cleanup rather than navigation execution.

## Load the repair and demonstrate

For a repeatable demonstration, restart the project at the default spawn. In the two existing terminals, press Ctrl+C and wait for command prompts. In Terminal 1 run `./scripts/stop_maze.py`, source `scripts/maze_env.bash`, then launch `maze_world simulation.launch.py`. In Terminal 2 source the same environment and launch `maze_navigation nav2.launch.py`.

After startup the robot is in the bottom row, second cell from the left, facing right. For the assignment, an optional manual **2D Pose Estimate** must begin at that actual robot location and point right. It describes the **current** pose. Use **Nav2 Goal** for the destination; its press location sets the destination and dragging sets the desired arrival heading. Select the far top-right cell `(1.25, 1.25)` and point the arrival arrow upward.
