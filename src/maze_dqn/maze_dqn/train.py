"""Train a goal-conditioned DQN on cell transitions in the fixed reference maze."""
import argparse
import csv
import json
import math
from pathlib import Path
import time

import numpy as np

from .agent import Agent
from .environment import DEFAULT_GOALS, Maze, rollout


def split_pairs(goals, seed):
    rng = np.random.default_rng(seed)
    pairs = [(s, g) for g in goals for s in range(36) if s != g]
    indices = rng.permutation(len(pairs))
    heldout = set(int(i) for i in indices[:max(1, len(pairs)//5)])
    return ([p for i, p in enumerate(pairs) if i not in heldout],
            [p for i, p in enumerate(pairs) if i in heldout])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes', type=int, default=6000)
    parser.add_argument('--max-steps', type=int, default=72)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--goals', type=int, nargs='+', default=list(DEFAULT_GOALS))
    parser.add_argument('--output', type=Path, default=Path('runs/dqn/seed7'))
    args = parser.parse_args()
    if args.episodes < 1 or args.max_steps < 1 or not set(args.goals) <= set(range(36)):
        parser.error('Episodes/steps must be positive and goal cells must be 0..35')
    maze, rng = Maze(), np.random.default_rng(args.seed)
    goals = sorted(set(args.goals))
    train_pairs, heldout_pairs = split_pairs(goals, args.seed)
    agent = Agent(maze.observation_size, args.seed)
    args.output.mkdir(parents=True, exist_ok=True)
    metadata = {'maze_signature': maze.signature, 'world_sha256': maze.world_hash,
                'goals': goals, 'train_pairs': train_pairs, 'heldout_initial_pairs': heldout_pairs,
                'method': 'DQN; replay; hard target update; Huber; Adam; no search labels',
                'training_environment': 'offline SDF-derived 6x6 cell MDP, not Gazebo',
                'seed': args.seed, 'gamma': agent.config.gamma}
    (args.output/'config.json').write_text(json.dumps(metadata, indent=2)+'\n')
    start, frames, best_score, perfect_streak = time.monotonic(), 0, (-1., -math.inf), 0
    history = []
    with (args.output/'episodes.csv').open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=['episode', 'frames', 'start', 'goal',
                                'steps', 'return', 'success', 'wall_attempts', 'epsilon', 'loss'])
        writer.writeheader()
        for episode in range(1, args.episodes+1):
            initial, goal = train_pairs[int(rng.integers(len(train_pairs)))]
            cell, total, collisions, terminated, loss = initial, 0., 0, False, None
            epsilon = 1.
            for step in range(1, args.max_steps+1):
                epsilon = 0.05+0.95*math.exp(-frames/20_000)
                state = maze.observe(cell, goal)
                action = agent.act(state, epsilon)
                successor, reward, terminated, collision = maze.transition(
                    cell, goal, action, agent.config.gamma)
                agent.replay.add(state, action, reward, maze.observe(successor, goal), terminated)
                frames += 1
                if frames >= 512 and frames % 4 == 0:
                    loss = agent.learn()
                cell, total, collisions = successor, total+reward, collisions+int(collision)
                if terminated:
                    break
            writer.writerow({'episode': episode, 'frames': frames, 'start': initial,
                             'goal': goal, 'steps': step, 'return': total, 'success': terminated,
                             'wall_attempts': collisions, 'epsilon': epsilon, 'loss': loss})
            if episode % 250 == 0 or episode == args.episodes:
                trials = [rollout(maze, agent, s, g, args.max_steps) for s, g in train_pairs]
                success = sum(t['success'] for t in trials)/len(trials)
                mean_return = float(np.mean([t['return'] for t in trials]))
                score = (success, mean_return)
                progress = {'episode': episode, 'frames': frames, 'train_success': success,
                            'train_greedy_mean_return': mean_return,
                            'seconds': time.monotonic()-start}
                history.append(progress)
                print(json.dumps(progress), flush=True)
                current_metadata = dict(metadata, episode=episode, frames=frames,
                                        training_seconds=progress['seconds'], validation=progress)
                agent.save(args.output/'last.pt', current_metadata)
                if score > best_score:
                    best_score = score
                    agent.save(args.output/'best.pt', current_metadata)
                perfect_streak = perfect_streak+1 if success == 1. else 0
                file.flush()
                if episode >= 2000 and perfect_streak >= 3:
                    break
    (args.output/'training-summary.json').write_text(json.dumps({
        **metadata, 'episodes_completed': episode, 'frames': frames,
        'training_seconds': time.monotonic()-start, 'history': history,
        'best_selection': 'training initial pairs only; success then greedy return',
        'heldout_note': 'Initial pairs excluded from resets; their states can occur mid-episode.'
    }, indent=2)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
