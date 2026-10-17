# DQN maze route learning

This package learns **which adjacent maze cell to visit next**. A PyTorch neural
network is trained from rewards in an offline environment derived from the unchanged
corrected SDF. During Gazebo demonstrations, its learned cell route is densified and
executed one learned waypoint at a time through Nav2's **FollowPath/DWB local
controller**, using the existing velocity smoother and collision monitor. Each
waypoint has a checked straight path and arrival heading for the next move. No
NavFn global planner is called to produce
or replace a DQN route. This is global route learning, not LiDAR-to-steering learning.
When the robot faces away from the next path, a collision-checked Nav2 Spin action
first aligns it with that learned edge. This also allows returning from a previous goal.
At the destination, translation finishes first; a separate Spin action then sets
the requested arrival heading. Both phases must pass for the route to pass.

## Setup

Run from the workspace root. Use system Python 3.12 so ROS modules remain available.
The CPU-only environment avoids changing the installed ROS packages:

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-dqn
.venv-dqn/bin/python -m pip install torch==2.10.0 --index-url https://download.pytorch.org/whl/cpu
source scripts/dqn_env.bash
python /usr/bin/colcon build --symlink-install --packages-select maze_dqn
source scripts/dqn_env.bash
```

The package is enabled for colcon. Build it with the DQN environment's Python so
its ROS console entry points use that interpreter. The `python -m` commands below
also explicitly use the activated environment.

The supplied deployment checkpoint is `models/dqn/maze_dqn.pt` (seed 17). Use it
instead of training again for the verified demonstration. Models from seeds 7 and
27, raw training CSV/metadata in `reports/dqn-training/`, and aggregate results
are retained. The deployment model was chosen after exhaustive same-maze
qualification; its held-out-reset score is not an unbiased model-selection test.

## Train and compare

Training and offline evaluation do **not** require Gazebo or Nav2:

```bash
source scripts/dqn_env.bash
python -m maze_dqn.train --episodes 6000 --seed 17 --output runs/dqn/my-seed17
python -m maze_dqn.evaluate --model runs/dqn/my-seed17/best.pt --output reports/my-dqn-evaluation.json
```

`best.pt` is selected using greedy returns/success on training reset pairs only.
`last.pt`, episode CSV, configuration, seeds and periodic training summaries are
saved separately. Checkpoints store inference weights and metadata; resuming the
optimizer/replay state is not implemented. Re-running training starts a fresh run;
use another output directory to preserve previous results.

The default goals are cells **4**, **11**, **35**, corresponding to `(0.75,-1.25)`,
`(1.25,-0.75)` and `(1.25,1.25)`. Cells use `row*6+column`, row zero at the bottom,
column zero at the left. The initial spawn is cell **1**. Cell centers are
`x=-1.25+0.5*column`, `y=-1.25+0.5*row`.

There are 105 distinct start/goal pairs for the three default goals. Training reset
pairs and held-out initial pairs use an 80/20 seeded split. A held-out pair can still
occur as an intermediate state during training: this tests new **initial pairs in
the same known maze**, not unseen maps or independent state generalization. The
evaluation reports both groups, wall attempts, optimality, grid route lengths and
CPU planning time. Dijkstra searches the **same cell graph**, making these lengths
comparable. Neither method's grid length is a continuous NavFn trajectory length.

## Learned policy

- Input: 36 current-cell indicators, 36 goal-cell indicators, 4 blocked-neighbor
  indicators, and normalized relative goal x/y: **78 values**.
- Output: four Q values for **east, north, west, south**. All four actions remain
  available: invalid wall moves are learned from penalties, not masked out.
- Network: 78 → 128 ReLU → 128 ReLU → 4, trained with Adam and Huber loss.
- DQN: uniform replay, epsilon-greedy exploration, frozen target network copied
  every 250 optimizer updates, discount 0.98, batch 64, update every four transitions.
- Rewards: +10 for reaching the goal, −1 for attempting a wall move, −0.05 for other
  moves; plus potential shaping `gamma*Phi(next)-Phi(current)`, where Phi is negative
  Manhattan distance divided by ten and terminal Phi is zero. Dijkstra is not used
  for reward labels, action selection or replay generation.
- Episodes end at the goal or 72 steps. Time-limit truncation still bootstraps;
  actual goal termination does not. Training can stop after sustained perfect
  success on training reset pairs, following at least 2,000 episodes.

## Preview or drive a learned route

Offline preview from the spawn to the top-right goal sends **no ROS commands**:

```bash
python -m maze_dqn.follow --model models/dqn/maze_dqn.pt --start-cell 1 --goal-cell 35
```

For Gazebo execution, keep the project's simulation and AMCL/Nav2 terminals
running. Do not send an RViz goal during DQN execution. In a separate terminal:

```bash
cd /mnt/storage/Projects-external/robotics/turtlebot3_maze_ws
source scripts/dqn_env.bash
python -m maze_dqn.follow --model models/dqn/maze_dqn.pt \
  --goal-cell 35 --yaw 1.5707963268 --execute --output reports/my-dqn-drive.json
```

Omit `--execute` for a live-pose preview. Execution reads the current robot pose;
do not pass `--start-cell`. If already at the goal it sends no motion action.
Choose another trained goal, such as `--goal-cell 4 --yaw 0`, to return to the
bottom corridor. Preview/execution rejects failed policies, loops, changed maze
geometry, active competing goals, inaccurate localization and unsafe connectors.
Execution cancels on timeout, stale simulator data, localization divergence or
crossing the conservative 0.13 m wall-clearance envelope. Failures are reported,
without substituting a Dijkstra route. Geometric checks are sampled; no physical
contact sensor is installed.

The total route timeout defaults to 360 seconds. Ctrl+C cancels the active action
before shutting down its ROS context. A mid-passage start connects to the next
untraversed part of the first learned edge when safe, avoiding a reversal solely
to visit an already-passed cell center. The robot's 0.13 m envelope is unchanged.
Intermediate waypoints temporarily use an 8 cm controller position tolerance to
allow finishing their turns despite small localization updates. The original
6 cm final-goal tolerance is restored for the destination and again on exit,
including cancellation/error cleanup; the report records restoration. Do not
send RViz goals while these temporary settings are in use.
The independent simulator-heading check allows 0.30 rad at intermediate targets
and 0.20 rad at the destination; intermediate headings guide the next leg.
After the final rotation, independent position checks require less than 9 cm map
error and 12 cm simulator error. The controller's 6 cm position threshold applies
to the final approach; rotation can slightly change the final measured position.

The learned path is published as `/dqn_path` (`nav_msgs/Path`). Add a Path display
with that topic in an already-open RViz, or use the project's updated RViz config
on your next startup. The stored path may differ from DWB's actual corner tracking.

## Tests and interpretation

```bash
source scripts/dqn_env.bash
python -m pytest src/maze_dqn/test -q
```

The tests cover geometry, collision rewards, terminal bootstrapping, replay,
neural-gradient/target updates, checkpoint round trips, failed-policy handling
and the held-out split. Trained success and actual Gazebo motion are checked
separately and reported in `reports/DQN_REPORT.md`. A small fixed maze does not
establish that DQN is faster or more reliable than Dijkstra, and learning a grid
policy is not an end-to-end autonomous-driving result.

Implementation references: [PyTorch DQN tutorial](https://docs.pytorch.org/tutorials/intermediate/reinforcement_q_learning)
and [CPU installation](https://pytorch.org/get-started/locally/).
