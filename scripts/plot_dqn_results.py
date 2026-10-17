#!/usr/bin/python3
"""Render measured DQN learning curves and a learned-route/Dijkstra comparison."""
import csv
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR', str(ROOT/'log/matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from maze_dqn.agent import Agent
from maze_dqn.environment import Maze, rollout

def main():
    seeds = [7, 17, 27]
    colors = ['#2463a6', '#dc7c16', '#2f865e']
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2), layout='constrained')
    evaluation = []
    for seed, color in zip(seeds, colors):
        run = ROOT/f'reports/dqn-training/seed{seed}'
        with (run/'episodes.csv').open() as file:
            episodes = list(csv.DictReader(file))
        returns = np.array([float(r['return']) for r in episodes])
        smooth = np.convolve(returns, np.ones(100)/100, mode='valid')
        axes[0].plot(np.arange(100, len(returns)+1), smooth, color=color, label=f'Seed {seed}')
        history = json.loads((run/'training-summary.json').read_text())['history']
        axes[1].plot([h['episode'] for h in history], [100*h['train_success'] for h in history],
                     marker='o', markersize=3, color=color, label=f'Seed {seed}')
        evaluation.append(json.loads((ROOT/f'reports/dqn-seed{seed}-evaluation.json').read_text()))
    axes[0].set(title='Training returns', xlabel='Episode', ylabel='Mean return (100 episodes)')
    axes[1].set(title='Greedy training-pair success', xlabel='Episode', ylabel='Success (%)', ylim=(0, 105))
    positions = np.arange(3)
    axes[2].bar(positions-.18, [100*r['all']['success_rate'] for r in evaluation], .35,
                color='#456c99', label='All 105 initial pairs')
    axes[2].bar(positions+.18, [100*r['heldout_initial_pairs']['success_rate'] for r in evaluation],
                .35, color='#e59b48', label='21 held-out initial pairs')
    axes[2].axhline(100, color='#30865d', linestyle='--', linewidth=1.3, label='Dijkstra: 100%')
    axes[2].set(title='Fixed-maze evaluation', xticks=positions,
                xticklabels=[f'Seed {s}' for s in seeds], ylabel='Success (%)', ylim=(0, 118))
    for ax in axes:
        ax.spines[['top', 'right']].set_visible(False)
        ax.grid(axis='y', alpha=.18)
        ax.set_axisbelow(True)
        ax.legend(loc='lower right', fontsize=8)
    fig.suptitle('DQN route learning — three seeded runs, same corrected maze', fontsize=14)
    fig.savefig(ROOT/'reports/dqn-learning-curves.png', dpi=180)
    fig.savefig(ROOT/'reports/dqn-learning-curves.pdf')
    plt.close(fig)

    maze = Maze()
    agent, _ = Agent.load(ROOT/'models/dqn/maze_dqn.pt')
    learned = rollout(maze, agent, 1, 35)
    baseline = maze.dijkstra(1, 35)
    fig, ax = plt.subplots(figsize=(7, 7), layout='constrained')
    for x, y, hx, hy in maze.walls:
        ax.add_patch(Rectangle((x-hx, y-hy), 2*hx, 2*hy, color='#303943'))
    for cell in range(36):
        x, y = maze.xy(cell)
        ax.text(x+.04, y+.05, str(cell), fontsize=8, color='#777777')
    for route, color, width, style, label in [
        (baseline, '#52a2c6', 6, '-', f'Dijkstra: {len(baseline)-1} moves'),
        (learned['path'], '#d36d16', 2.4, '--', f'DQN seed 17: {learned["steps"]} moves')]:
        points = np.array([maze.xy(c) for c in route])
        ax.plot(points[:, 0], points[:, 1], color=color, linewidth=width,
                linestyle=style, label=label, solid_capstyle='round')
    ax.scatter(*maze.xy(1), c='#21845b', s=95, marker='o', zorder=5, label='Start: cell 1')
    ax.scatter(*maze.xy(35), c='#bd3333', s=170, marker='*', zorder=5, label='Goal: cell 35')
    ax.set(xlim=(-1.62, 1.62), ylim=(-1.62, 1.62), aspect='equal', xlabel='World x (m)',
           ylabel='World y (m)', title='Learned route vs Dijkstra on the same cell graph')
    ax.legend(loc='upper center', bbox_to_anchor=(.5, -.10), ncol=2, fontsize=9)
    fig.savefig(ROOT/'reports/dqn-route-comparison.png', dpi=180)
    fig.savefig(ROOT/'reports/dqn-route-comparison.pdf')
    plt.close(fig)


if __name__ == '__main__':
    main()
