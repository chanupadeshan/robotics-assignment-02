# DQN maze route learning

Reward-trained global routes on the corrected 6×6 maze.

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
