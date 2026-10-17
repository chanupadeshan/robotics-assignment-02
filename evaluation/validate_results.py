#!/usr/bin/env python3
"""Audit result provenance and counts, independently of planner pass/fail verdict."""
import csv
import hashlib
import json
import math
from pathlib import Path

from maze_dqn.environment import Maze
from run_comparison import nav2_success, path_metrics

ROOT = Path(__file__).resolve().parents[1]
RESULTS = Path(__file__).resolve().parent/'results'


def main():
    manifest = json.loads((RESULTS/'source_manifest.json').read_text())
    for name, expected in manifest.items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == expected, name
    summary = json.loads((RESULTS/'comparison.json').read_text())
    raw = json.loads((RESULTS/'nav2_plans.json').read_text())
    with (RESULTS/'planning_trials.csv').open() as file:
        rows = list(csv.DictReader(file))
    n = summary['pairs']
    assert len(raw['trials']) == len(rows) == n
    pairs = {(int(r['start_cell']), int(r['goal_cell'])) for r in rows}
    assert len(pairs) == n
    assert pairs == {(s,g) for g in summary['goals'] for s in range(36) if s != g}
    maze = Maze()
    assert maze.signature == summary['maze_signature'] == raw['maze_signature']
    by_pair = {(int(r['start_cell']),int(r['goal_cell'])): r for r in rows}
    for trial in raw['trials']:
        assert trial['success'] == nav2_success(trial)
        checked = path_metrics(maze, trial['path_xy'])
        for key in checked:
            assert (checked[key] is None and trial[key] is None or
                    checked[key] is not None and math.isclose(checked[key],trial[key],abs_tol=1e-10))
        row = by_pair[trial['start'],trial['goal']]
        assert (row['nav2_success'] == 'True') == trial['success']
        assert (row['nav2_action_success'] == 'True') == (
            trial['status'] == 4 and trial['error_code'] == 0 and bool(trial['path_xy']))
        assert int(row['dijkstra_steps']) == len(maze.dijkstra(trial['start'],trial['goal']))-1
        if row['dqn_grid_optimal'] == 'True':
            assert row['dqn_steps'] == row['dijkstra_steps']
    for field, key in [('nav2_success','nav2_successes'),
                       ('nav2_action_success','nav2_action_successes'),
                       ('dqn_success','dqn_successes'),
                       ('dqn_grid_optimal','dqn_grid_optimal_routes')]:
        assert sum(r[field] == 'True' for r in rows) == summary[key]
    assert sum(bool(r['nav2_min_clearance_m']) and float(r['nav2_min_clearance_m']) < 0 for r in rows) == summary['nav2_clearance_flags']
    timings = json.loads((RESULTS/'cpu_timings.json').read_text())
    assert {(r['start'],r['goal']) for r in timings} == pairs
    assert all(len(r[m]) == summary['cpu_repeats_per_pair'] for r in timings for m in ['dqn','dijkstra'])
    for filename in ['planning_comparison','route_comparison','clearance_audit']:
        for extension in ['png','pdf']:
            assert (RESULTS.parent/'plots'/f'{filename}.{extension}').stat().st_size > 1000
    result = {'passed': True, 'source_hashes_checked': len(manifest), 'pairs_checked': n,
              'nav2_returned_routes': summary['nav2_action_successes'],
              'nav2_envelope_checked_routes': summary['nav2_successes'],
              'nav2_clearance_flags': summary['nav2_clearance_flags'],
              'dqn_successes': summary['dqn_successes'],
              'note': 'Artifact audit passes even when planned-clearance flags remain in results.'}
    (RESULTS/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
