import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_comparison import nav2_success, path_metrics, recorded_execution
from maze_dqn.environment import Maze


def test_segment_check_detects_wall_between_clear_endpoints():
    maze = Maze()
    points = [maze.xy(0), maze.xy(6)]
    assert all(maze.clearance(*p) > 0 for p in points)
    assert path_metrics(maze, points)['minimum_clearance_m'] < 0


def test_valid_corridor_length_and_empty_failed_path():
    maze = Maze()
    metrics = path_metrics(maze, [maze.xy(1), maze.xy(7)])
    assert metrics['length_m'] == pytest.approx(.5)
    assert metrics['minimum_clearance_m'] > 0
    assert path_metrics(maze, []) == {'length_m': None, 'minimum_clearance_m': None}


@pytest.mark.parametrize('changed', [
    {'status': 6}, {'error_code': 208}, {'path_xy': []},
    {'minimum_clearance_m': -.001}, {'goal_error_m': .06}, {'start_error_m': .06}])
def test_invalid_nav2_result_is_never_counted_as_success(changed):
    row = {'status': 4, 'error_code': 0, 'path_xy': [(0., 0.)],
           'minimum_clearance_m': .05, 'goal_error_m': .001, 'start_error_m': .001}
    assert nav2_success(row)
    row.update(changed)
    assert not nav2_success(row)


def test_recorded_drives_keep_different_endpoints_and_path_definitions():
    rows = recorded_execution()
    classical = rows[1]
    learned = rows[3]
    assert classical['goal_x_m'] == learned['goal_x_m'] == 1.25
    assert classical['start_x_m'] != learned['start_x_m']
    assert classical['planned_length_kind'] != learned['planned_length_kind']
    assert all('no speed ranking' in row['comparison_scope'] for row in rows)
