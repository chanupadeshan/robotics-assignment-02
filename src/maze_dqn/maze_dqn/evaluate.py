"""Compare greedy DQN and Dijkstra on the same cell graph; never substitute a route."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from .agent import Agent
from .environment import Maze, rollout


def evaluate(maze, agent, metadata):
    if metadata['maze_signature'] != maze.signature:
        raise ValueError('Checkpoint does not match the corrected maze/robot geometry')
    heldout = {tuple(p) for p in metadata['heldout_initial_pairs']}
    trials = []
    for goal in metadata['goals']:
        for start in range(maze.cells):
            if start == goal:
                continue
            beginning = time.perf_counter()
            result = rollout(maze, agent, start, goal)
            result['dqn_planning_ms'] = (time.perf_counter()-beginning)*1000
            beginning = time.perf_counter()
            baseline = maze.dijkstra(start, goal)
            result.update({'dijkstra_planning_ms': (time.perf_counter()-beginning)*1000,
                           'dijkstra_path': baseline, 'dijkstra_steps': len(baseline)-1,
                           'dqn_length_m': result['steps']*maze.spacing if result['success'] else None,
                           'dijkstra_length_m': (len(baseline)-1)*maze.spacing,
                           'heldout_initial_pair': (start, goal) in heldout,
                           'optimal': result['success'] and result['steps'] == len(baseline)-1})
            trials.append(result)

    def summary(rows):
        return {'trials': len(rows),
                'success_rate': sum(t['success'] for t in rows)/len(rows),
                'optimal_rate': sum(t['optimal'] for t in rows)/len(rows),
                'wall_attempts': sum(t['wall_attempts'] for t in rows),
                'mean_dqn_planning_ms': float(np.mean([t['dqn_planning_ms'] for t in rows])),
                'mean_dijkstra_planning_ms': float(np.mean([t['dijkstra_planning_ms'] for t in rows]))}

    return {'maze_signature': maze.signature, 'checkpoint_metadata': metadata,
            'all': summary(trials),
            'heldout_initial_pairs': summary([t for t in trials if t['heldout_initial_pair']]),
            'scope': 'Same fixed maze and trained goals; held-out reset pairs are not unseen maps.',
            'timing_scope': 'CPU route construction after model load; excludes training, ROS and motion.',
            'trials': trials}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('reports/dqn-evaluation.json'))
    args = parser.parse_args()
    agent, metadata = Agent.load(args.model)
    result = evaluate(Maze(), agent, metadata)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: result[k] for k in ['all', 'heldout_initial_pairs', 'scope']}, indent=2))
    return 0 if result['all']['success_rate'] == 1. else 1


if __name__ == '__main__':
    raise SystemExit(main())
