#!/usr/bin/python3
"""Plot verified simulator trajectories against DQN-selected cell routes."""
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

from maze_dqn.environment import Maze


def main():
    maze = Maze()
    names = ['dqn-final-bottom', 'dqn-final-top-right']
    results = [json.loads((ROOT/f'reports/{name}.json').read_text()) for name in names]
    if not all(result['passed'] for result in results):
        raise RuntimeError('Execution figure requires two passing trials')
    fig, axes = plt.subplots(1, 2, figsize=(12, 6.4), layout='constrained')
    for ax, name, result in zip(axes, names, results):
        navigation = result['navigation']
        for x, y, hx, hy in maze.walls:
            ax.add_patch(Rectangle((x-hx, y-hy), 2*hx, 2*hy, color='#303943'))
        learned = np.array(result['route_xy'])
        measured = np.array([[sample['gazebo']['x'], sample['gazebo']['y']]
                             for sample in navigation['trajectory']])
        ax.plot(learned[:, 0], learned[:, 1], color='#d88426', linestyle='--',
                linewidth=2, label='DQN cell route')
        ax.plot(measured[:, 0], measured[:, 1], color='#256cb3', linewidth=1.6,
                label='Measured Gazebo trajectory')
        ax.scatter(*learned[-1], color='#bd3333', marker='*', s=120, zorder=5)
        ax.set(aspect='equal', xlim=(-1.6, 1.6), ylim=(-1.6, 1.6),
               xlabel='World x (m)', ylabel='World y (m)',
               title=f'{name.removeprefix("dqn-final-").replace("-", " ").title()} goal\n'
                     f'{navigation["wall_seconds"]:.1f} s; final error '
                     f'{100*navigation["gazebo_position_error"]:.1f} cm')
        ax.legend(loc='upper center', bbox_to_anchor=(.5, -.11), fontsize=9)
    fig.suptitle('DQN selects the route; DWB executes each learned waypoint', fontsize=14)
    fig.savefig(ROOT/'reports/dqn-gazebo-trajectories.png', dpi=180)
    fig.savefig(ROOT/'reports/dqn-gazebo-trajectories.pdf')
    plt.close(fig)


if __name__ == '__main__':
    main()
