# Source from the workspace before DQN training/evaluation/execution.
dqn_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$dqn_workspace/scripts/maze_env.bash"
if [ ! -f "$dqn_workspace/.venv-dqn/bin/activate" ]; then
  echo 'DQN environment missing; follow src/maze_dqn/README.md setup instructions.' >&2
  return 1
fi
source "$dqn_workspace/.venv-dqn/bin/activate"
unset dqn_workspace
