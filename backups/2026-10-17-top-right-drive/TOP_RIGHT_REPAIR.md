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
- **No autonomous motion to the top row was commanded during this repair.** The full long drive still needs the user's RViz trial; previous bottom/right-corridor navigation successes remain separate evidence.

## Load the repair and demonstrate

The running main Nav2 process still has the old plugin loaded; restart the project to use the new one. In the two existing terminals, press Ctrl+C and wait for command prompts. In Terminal 1 run `./scripts/stop_maze.py`, source `scripts/maze_env.bash`, then launch `maze_world simulation.launch.py`. In Terminal 2 source the same environment and launch `maze_navigation nav2.launch.py`.

After startup the robot is in the bottom row, second cell from the left, facing right. For the assignment, an optional manual **2D Pose Estimate** must begin at that actual robot location and point right. It describes the **current** pose. Use **Nav2 Goal** for the destination; its press location sets the destination and dragging sets the desired arrival heading. Select the far top-right cell `(1.25, 1.25)` and point the arrival arrow upward.
