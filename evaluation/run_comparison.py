#!/usr/bin/env python3
"""Compare actual Nav2 plans and reward-trained DQN routes on identical endpoints.

--live-nav2 collects ComputePathToPose results without moving the robot. Otherwise
reuse the saved Nav2 results. Dijkstra on the cell graph is a separate reference.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import time

import numpy as np

from maze_dqn.agent import Agent
from maze_dqn.environment import Maze, rollout

ROOT = Path(__file__).resolve().parents[1]
HERE = Path(__file__).resolve().parent
RESULTS = HERE/'results'


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def write_csv(path, rows):
    with path.open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def path_metrics(maze, points):
    """Check segments as well as vertices using <= 1.25 cm samples."""
    if not points:
        return {'length_m': None, 'minimum_clearance_m': None}
    sampled = [points[0]]
    length = 0.
    for a, b in zip(points, points[1:]):
        distance = math.dist(a, b)
        length += distance
        count = max(1, math.ceil(distance/.0125))
        sampled.extend((a[0]+(b[0]-a[0])*i/count, a[1]+(b[1]-a[1])*i/count)
                       for i in range(1, count+1))
    return {'length_m': length,
            'minimum_clearance_m': min(maze.clearance(*p) for p in sampled)}


def nav2_success(row):
    return (row['status'] == 4 and row['error_code'] == 0 and bool(row['path_xy'])
            and row['goal_error_m'] <= .050001 and row['start_error_m'] <= .050001
            and row['minimum_clearance_m'] >= 0)


def collect_nav2(maze, pairs):
    # Lazy ROS imports keep replay/plot generation usable without ROS processes.
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import GetParameters
    from nav_msgs.msg import OccupancyGrid
    from nav2_msgs.action import ComputePathToPose
    from nav2_msgs.srv import GetCostmap

    rclpy.init(args=[])
    node = Node('maze_planner_comparison',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    states = node.create_client(GetState, '/planner_server/get_state')
    params = node.create_client(GetParameters, '/planner_server/get_parameters')
    costmap = node.create_client(GetCostmap, '/global_costmap/get_costmap')
    maps = []
    subscription = node.create_subscription(OccupancyGrid, '/map', maps.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    active_handle = None

    def spin_until(probe, seconds=25):
        end = time.monotonic()+seconds
        while not probe() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        if not probe():
            raise RuntimeError('Nav2 discovery/response timed out; keep simulation and Nav2 running')

    def wait(future, seconds=25):
        spin_until(future.done, seconds)
        return future.result()

    def snapshot():
        msg = wait(costmap.call_async(GetCostmap.Request())).map
        meta = msg.metadata
        return {'sha256': hashlib.sha256(bytes(msg.data)).hexdigest(),
                'width': meta.size_x, 'height': meta.size_y, 'resolution': meta.resolution,
                'frame': msg.header.frame_id,
                'update_sim_seconds': meta.update_time.sec+meta.update_time.nanosec/1e9}

    def plan(start, goal):
        nonlocal active_handle
        def pose(cell):
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y = maze.xy(cell)
            msg.pose.orientation.w = 1.
            return msg
        request = ComputePathToPose.Goal(start=pose(start), goal=pose(goal),
                                        use_start=True, planner_id='GridBased')
        beginning = time.perf_counter_ns()
        active_handle = wait(planner.send_goal_async(request))
        if not active_handle.accepted:
            active_handle = None
            return {'start': start, 'goal': goal, 'status': 0, 'error_code': None,
                    'error_msg': 'Action rejected', 'path_xy': [], 'success': False,
                    'server_planning_ms': None, 'action_round_trip_ms': None,
                    'start_error_m': None, 'goal_error_m': None,
                    **path_metrics(maze, [])}
        response = wait(active_handle.get_result_async())
        elapsed = (time.perf_counter_ns()-beginning)/1e6
        active_handle = None
        result = response.result
        points = [(p.pose.position.x, p.pose.position.y) for p in result.path.poses]
        row = {'start': start, 'goal': goal, 'status': response.status,
               'error_code': result.error_code, 'error_msg': result.error_msg,
               'server_planning_ms': (result.planning_time.sec*1e3+
                                      result.planning_time.nanosec/1e6),
               'action_round_trip_ms': elapsed, 'path_xy': points,
               'start_error_m': math.dist(points[0], maze.xy(start)) if points else None,
               'goal_error_m': math.dist(points[-1], maze.xy(goal)) if points else None,
               **path_metrics(maze, points)}
        row['success'] = nav2_success(row)
        return row

    try:
        spin_until(lambda: maps and node.get_clock().now().nanoseconds > 0)
        for client in [states, params, costmap]:
            spin_until(client.service_is_ready)
        spin_until(planner.server_is_ready)
        if wait(states.call_async(GetState.Request())).current_state.label != 'active':
            raise RuntimeError('Planner server is not active')
        if not maze.matches_map(maps[-1].info, maps[-1].data):
            raise RuntimeError('Live reference map does not match DQN training geometry')
        graph = node.get_node_names_and_namespaces()
        if sum(n == 'planner_server' and ns == '/' for n, ns in graph) != 1:
            raise RuntimeError('Expected one project planner server')
        names = ['GridBased.plugin', 'GridBased.use_astar', 'GridBased.tolerance',
                 'GridBased.allow_unknown']
        values = wait(params.call_async(GetParameters.Request(names=names))).values
        configuration = dict(zip(names, [values[0].string_value, values[1].bool_value,
                                         values[2].double_value, values[3].bool_value]))
        if configuration['GridBased.plugin'] != 'maze_navigation::MazeNavfnPlanner' or values[1].bool_value:
            raise RuntimeError('Expected project NavFn Dijkstra configuration')
        before = snapshot()
        for start, goal in pairs[:3]:
            plan(start, goal)  # Warm DDS/action/planner allocation; not measured below.
        trials = []
        for i, (start, goal) in enumerate(pairs, 1):
            trials.append(plan(start, goal))
            if i % 21 == 0:
                print(f'Nav2: {i}/{len(pairs)} planning requests recorded', flush=True)
        result = {'recorded_utc': datetime.now(timezone.utc).isoformat(),
                  'maze_signature': maze.signature, 'configuration': configuration,
                  'costmap_before': before, 'costmap_after': snapshot(),
                  'scope': 'ComputePathToPose only; explicit cell-center start/goal; no motion',
                  'warmup_requests': 3, 'trials': trials}
        dump(RESULTS/'nav2_plans.json', result)
        return result
    finally:
        if active_handle is not None:
            wait(active_handle.cancel_goal_async())
        node.destroy_node()
        rclpy.shutdown()


def benchmark_grid(maze, agent, pairs, repeats):
    timing = {pair: {'dqn': [], 'dijkstra': []} for pair in pairs}
    for start, goal in pairs[:3]:
        rollout(maze, agent, start, goal)
        maze.dijkstra(start, goal)
    rng = np.random.default_rng(123)
    for repeat in range(repeats):
        for index in rng.permutation(len(pairs)):
            start, goal = pair = pairs[index]
            methods = ['dqn', 'dijkstra'] if (repeat+index) % 2 else ['dijkstra', 'dqn']
            for method in methods:
                beginning = time.perf_counter_ns()
                if method == 'dqn':
                    rollout(maze, agent, start, goal)
                else:
                    maze.dijkstra(start, goal)
                timing[pair][method].append((time.perf_counter_ns()-beginning)/1e6)
    return timing


def recorded_execution():
    rows = []
    for name in ['top-right-return-trial', 'top-right-full-route',
                 'dqn-final-bottom', 'dqn-final-top-right']:
        report = json.loads((ROOT/f'reports/{name}.json').read_text())
        dqn = name.startswith('dqn')
        motion = report['navigation'] if dqn else report
        target = report['route_xy'][-1] if dqn else report['goal'][:2]
        rows.append({'method': 'DQN + Nav2 DWB' if dqn else 'Nav2 NavFn + DWB',
            'source': f'reports/{name}.json', 'passed': report['passed'],
            'start_x_m': motion['start_truth']['x'], 'start_y_m': motion['start_truth']['y'],
            'start_yaw_rad': motion['start_truth']['yaw'],
            'goal_x_m': target[0], 'goal_y_m': target[1],
            'goal_yaw_rad': report['goal_yaw'] if dqn else report['goal'][2],
            'wall_seconds': motion['wall_seconds'],
            'planned_length_m': report['policy']['steps']*.5 if dqn else report['path_length'],
            'planned_length_kind': 'cell-center polyline' if dqn else 'continuous NavFn path',
            'gazebo_position_error_m': motion['gazebo_position_error'] if dqn else report['truth_error'],
            'gazebo_yaw_error_rad': motion['gazebo_yaw_error'] if dqn else report['truth_yaw_error'],
            'minimum_clearance_m': motion['minimum_clearance'],
            'comparison_scope': 'Historical demonstration; endpoints differ; no speed ranking'})
    return rows


def percentile(values, quantile):
    valid = [v for v in values if v is not None]
    return float(np.percentile(valid, quantile)) if valid else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live-nav2', action='store_true', help='Collect fresh plans; no robot motion')
    parser.add_argument('--repeats', type=int, default=30, help='CPU repeats per pair, at least 5')
    args = parser.parse_args()
    if args.repeats < 5:
        parser.error('--repeats must be at least 5')
    RESULTS.mkdir(parents=True, exist_ok=True)
    maze = Maze()
    agent, metadata = Agent.load(ROOT/'models/dqn/maze_dqn.pt')
    if metadata['maze_signature'] != maze.signature:
        raise ValueError('Deployed checkpoint differs from the current maze')
    pairs = [(s, g) for g in metadata['goals'] for s in range(maze.cells) if s != g]
    if args.live_nav2:
        live = collect_nav2(maze, pairs)
    else:
        cache = RESULTS/'nav2_plans.json'
        if not cache.exists():
            parser.error('No cached Nav2 plans. Run once with --live-nav2 and Nav2 running.')
        live = json.loads(cache.read_text())
    if live['maze_signature'] != maze.signature:
        raise ValueError('Cached Nav2 plans differ from the current maze')
    nav = {(r['start'], r['goal']): r for r in live['trials']}
    if len(nav) != len(pairs) or set(nav) != set(pairs) or len(live['trials']) != len(pairs):
        raise ValueError('Nav2 results must contain each requested start-goal pair exactly once')
    timing = benchmark_grid(maze, agent, pairs, args.repeats)
    trials = []
    for start, goal in pairs:
        learned, baseline = rollout(maze, agent, start, goal), maze.dijkstra(start, goal)
        learned_metrics = path_metrics(maze, [maze.xy(c) for c in learned['path']])
        n = nav[start, goal]
        if n['success'] != nav2_success(n):
            raise ValueError('Cached Nav2 success disagrees with its recorded result')
        t = timing[start, goal]
        trials.append({'start_cell': start, 'goal_cell': goal,
            'nav2_success': n['success'],
            'nav2_action_success': n['status'] == 4 and n['error_code'] == 0 and bool(n['path_xy']),
            'nav2_clearance_pass': n['minimum_clearance_m'] is not None and n['minimum_clearance_m'] >= 0,
            'nav2_status': n['status'], 'nav2_error_code': n['error_code'],
            'nav2_server_planning_ms': n['server_planning_ms'],
            'nav2_action_round_trip_ms': n['action_round_trip_ms'],
            'nav2_continuous_length_m': n['length_m'], 'nav2_min_clearance_m': n['minimum_clearance_m'],
            'dqn_success': learned['success'] and learned['wall_attempts'] == 0 and learned_metrics['minimum_clearance_m'] >= 0,
            'dqn_policy_success': learned['success'], 'dqn_wall_attempts': learned['wall_attempts'],
            'dqn_steps': learned['steps'], 'dqn_grid_length_m': learned['steps']*.5 if learned['success'] else None,
            'dqn_min_clearance_m': learned_metrics['minimum_clearance_m'],
            'dqn_cpu_median_ms': percentile(t['dqn'], 50), 'dqn_cpu_p95_ms': percentile(t['dqn'], 95),
            'dijkstra_steps': len(baseline)-1, 'dijkstra_grid_length_m': (len(baseline)-1)*.5,
            'dijkstra_cpu_median_ms': percentile(t['dijkstra'], 50),
            'dijkstra_cpu_p95_ms': percentile(t['dijkstra'], 95),
            'dqn_grid_optimal': learned['success'] and learned['steps'] == len(baseline)-1})
    seeds = []
    for seed in [7, 17, 27]:
        evaluation = json.loads((ROOT/f'reports/dqn-seed{seed}-evaluation.json').read_text())
        training = json.loads((ROOT/f'reports/dqn-training/seed{seed}/training-summary.json').read_text())
        seeds.append({'seed': seed, 'episodes': training['episodes_completed'],
                      'transitions': training['frames'], 'training_seconds': training['training_seconds'],
                      'successes': sum(r['success'] for r in evaluation['trials']),
                      'trials': evaluation['all']['trials'],
                      'heldout_initial_successes': sum(r['success'] for r in evaluation['trials'] if r['heldout_initial_pair']),
                      'deployed': seed == metadata['seed']})
    summary = {'recorded_utc': datetime.now(timezone.utc).isoformat(),
        'maze_signature': maze.signature, 'pairs': len(pairs), 'goals': metadata['goals'],
        'deployed_seed': metadata['seed'], 'cpu_repeats_per_pair': args.repeats,
        'python': platform.python_version(), 'cpu': platform.processor(),
        'nav2_successes': sum(r['nav2_success'] for r in trials),
        'nav2_action_successes': sum(r['nav2_action_success'] for r in trials),
        'nav2_clearance_flags': sum(r['nav2_min_clearance_m'] is not None and r['nav2_min_clearance_m'] < 0 for r in trials),
        'nav2_minimum_planned_clearance_m': min((r['nav2_min_clearance_m'] for r in trials if r['nav2_min_clearance_m'] is not None),default=None),
        'dqn_successes': sum(r['dqn_success'] for r in trials),
        'dqn_grid_optimal_routes': sum(r['dqn_grid_optimal'] for r in trials),
        'dqn_wall_attempts': sum(r['dqn_wall_attempts'] for r in trials),
        'dijkstra_successes': len(trials),
        'nav2_server_planning_median_ms': percentile([r['nav2_server_planning_ms'] for r in trials], 50),
        'nav2_action_round_trip_median_ms': percentile([r['nav2_action_round_trip_ms'] for r in trials], 50),
        'dqn_pair_median_cpu_ms': percentile([r['dqn_cpu_median_ms'] for r in trials], 50),
        'dijkstra_pair_median_cpu_ms': percentile([r['dijkstra_cpu_median_ms'] for r in trials], 50),
        'timing_scope': 'Nav2 node-clock duration (simulation time here), ROS wall-clock action latency; separate in-process wall-clock CPU benchmark. No cross-context speed ratio.',
        'path_scope': 'Same endpoints; continuous NavFn geometry and cell-center routes differ. Grid optimality only against cell-graph Dijkstra.',
        'generalization_scope': 'One known maze, three trained goals; deployment selected after same-maze qualification.',
        'nav2_capture': {k:v for k,v in live.items() if k != 'trials'}}
    write_csv(RESULTS/'planning_trials.csv', trials)
    flags = [r for r in trials if r['nav2_min_clearance_m'] is not None and r['nav2_min_clearance_m'] < 0]
    if flags:
        write_csv(RESULTS/'nav2_clearance_flags.csv', flags)
    else:
        (RESULTS/'nav2_clearance_flags.csv').write_text('start_cell,goal_cell\n')
    write_csv(RESULTS/'dqn_seeds.csv', seeds)
    execution = recorded_execution()
    write_csv(RESULTS/'recorded_execution.csv', execution)
    dump(RESULTS/'cpu_timings.json', [{'start': s, 'goal': g, **timing[s, g]} for s, g in pairs])
    dump(RESULTS/'comparison.json', summary)
    sources = ['models/dqn/maze_dqn.pt', 'src/maze_world/worlds/maze_fixed.sdf',
               'src/maze_navigation/maps/maze_map.pgm', 'src/maze_navigation/maps/maze_map.yaml',
               'src/maze_navigation/config/nav2_params.yaml', 'evaluation/results/nav2_plans.json',
               'evaluation/run_comparison.py', 'evaluation/render_report.py',
               'evaluation/validate_results.py',
               'src/maze_dqn/maze_dqn/agent.py', 'src/maze_dqn/maze_dqn/environment.py']
    sources += [r['source'] for r in execution]
    sources += [f'reports/dqn-seed{s}-evaluation.json' for s in [7,17,27]]
    sources += [f'reports/dqn-training/seed{s}/training-summary.json' for s in [7,17,27]]
    dump(RESULTS/'source_manifest.json', {s: hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources})
    from render_report import render
    render(maze, agent, trials, summary, live, seeds, execution)
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary['nav2_successes'] == summary['dqn_successes'] == len(pairs) else 1


if __name__ == '__main__':
    raise SystemExit(main())
