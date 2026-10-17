"""Render the measured comparison; no synthetic navigation results."""
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', str(Path(__file__).resolve().parents[1]/'log/matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from maze_dqn.environment import rollout

HERE = Path(__file__).resolve().parent


def render(maze, agent, rows, summary, live, seeds, execution):
    plots = HERE/'plots'
    plots.mkdir(exist_ok=True)
    blue, orange, green = '#2874a6', '#d77d20', '#36855b'
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    n = summary['pairs']
    values = [summary['nav2_successes'], summary['dqn_successes'], summary['dijkstra_successes']]
    returned = [summary['nav2_action_successes'], sum(r['dqn_policy_success'] for r in rows), n]
    x = np.arange(3)
    for offset, data, alpha, label in [(-.18, returned, .35, 'Returned a route'),
                                      (.18, values, 1., 'Passed envelope check')]:
        bars = axes[0,0].bar(x+offset, [100*v/n for v in data], width=.34,
                             color=[blue,orange,green],alpha=alpha,label=label)
        axes[0,0].bar_label(bars, labels=[f'{v}/{n}' for v in data], padding=3,fontsize=8)
    axes[0,0].set(title='Same 105 start–goal pairs', ylabel='Route success (%)', ylim=(0, 122),
                  xticks=x,xticklabels=['Nav2\nNavFn',f'DQN\nseed {summary["deployed_seed"]}','Cell-graph\nDijkstra'])
    axes[0,0].legend(loc='lower right',fontsize=8)
    cpu = [summary['dqn_pair_median_cpu_ms'], summary['dijkstra_pair_median_cpu_ms']]
    bars = axes[0,1].bar(['DQN rollout', 'Cell-graph Dijkstra'], cpu, color=[orange, green])
    axes[0,1].bar_label(bars, labels=[f'{v:.3f} ms' for v in cpu], padding=4)
    axes[0,1].set(title='In-process CPU timing; same cell graph', ylabel='Median of per-pair median times (ms)',
                  ylim=(0, max(cpu)*1.3))
    server = [r['nav2_server_planning_ms'] for r in rows if r['nav2_action_success']]
    axes[1,0].hist(server, bins=15, color=blue, edgecolor='white')
    axes[1,0].set(title='Nav2 server duration (simulation clock)', xlabel='Server-reported planning time (sim ms)', ylabel='Pairs')
    matched = [r for r in rows if r['nav2_action_success'] and r['dqn_policy_success']]
    lengths = [[r['nav2_continuous_length_m'] for r in matched],
               [r['dqn_grid_length_m'] for r in matched]]
    for data, color, label in zip(lengths, [blue, orange], ['Continuous NavFn path', 'DQN cell-center polyline']):
        ordered = np.sort(data)
        axes[1,1].plot(ordered, np.arange(1, len(ordered)+1)/len(ordered), color=color, label=label)
    axes[1,1].set(title=f'Returned paths: {len(matched)} matched pairs; representations differ', xlabel='Planned geometric length (m)', ylabel='Cumulative fraction')
    axes[1,1].legend(fontsize=8)
    for ax in axes.flat:
        ax.spines[['top','right']].set_visible(False)
        ax.grid(axis='y', alpha=.18)
        ax.set_axisbelow(True)
    fig.suptitle('Nav2 global planning vs DQN route learning — fixed reference maze', fontsize=14)
    fig.savefig(plots/'planning_comparison.png', dpi=180)
    fig.savefig(plots/'planning_comparison.pdf')
    plt.close(fig)

    fig, axes = plt.subplots(1,2,figsize=(12,6.2),layout='constrained')
    for field, color, label in [('nav2_min_clearance_m',blue,'Nav2 planned paths'),
                                 ('dqn_min_clearance_m',orange,'DQN cell-center paths')]:
        data=np.sort([100*r[field] for r in rows if r[field] is not None])
        axes[0].plot(data,np.arange(1,len(data)+1)/len(data),color=color,label=label)
    axes[0].axvline(0,color='#bd3333',linestyle='--',label='0.13 m envelope boundary')
    axes[0].set(xlabel='Minimum sampled circle-to-wall clearance (cm)',ylabel='Cumulative fraction',
                title='Planned geometry; no robot motion')
    axes[0].grid(alpha=.2)
    axes[0].legend(fontsize=8,loc='lower right')
    worst=min(live['trials'],key=lambda r:r['minimum_clearance_m'] if r['minimum_clearance_m'] is not None else float('inf'))
    ax=axes[1]
    for x,y,hx,hy in maze.walls:
        ax.add_patch(Rectangle((x-hx,y-hy),2*hx,2*hy,color='#303943'))
    xy=np.array(worst['path_xy'])
    if len(xy):
        ax.plot(xy[:,0],xy[:,1],color=blue,label='Returned NavFn path',linewidth=2)
        flagged=np.array([p for p in xy if maze.clearance(*p)<0])
        if len(flagged):
            ax.scatter(flagged[:,0],flagged[:,1],c='#bd3333',s=24,zorder=5,label='Envelope flags at path vertices')
    ax.set(aspect='equal',xlim=(-1.6,1.6),ylim=(-1.6,1.6),xlabel='World x (m)',ylabel='World y (m)',
           title=f'Flagged plan: cell {worst["start"]} → {worst["goal"]}\nEnvelope encroachment ≠ observed collision')
    ax.legend(loc='upper center',bbox_to_anchor=(.5,-.12),fontsize=8)
    fig.suptitle('Independent clearance audit of the same planned paths',fontsize=14)
    fig.savefig(plots/'clearance_audit.png',dpi=180)
    fig.savefig(plots/'clearance_audit.pdf')
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 6.3), layout='constrained')
    lookup = {(r['start'],r['goal']):r for r in live['trials']}
    for ax, start in zip(axes, [1, 4]):
        for x, y, hx, hy in maze.walls:
            ax.add_patch(Rectangle((x-hx,y-hy),2*hx,2*hy,color='#303943'))
        n = lookup[start,35]
        learned = rollout(maze, agent, start,35)
        for points, color, style, label in [
                (n['path_xy'],blue,'-',f'Actual NavFn: {n["length_m"]:.2f} m' if n['path_xy'] else 'NavFn failed'),
                ([maze.xy(c) for c in learned['path']],orange,'--',f'DQN cells: {learned["steps"]*.5:.2f} m')]:
            if points:
                xy = np.asarray(points)
                ax.plot(xy[:,0],xy[:,1],color=color,linestyle=style,linewidth=2,label=label)
        ax.scatter(*maze.xy(start),color=green,s=70,zorder=5,label=f'Start: cell {start}')
        ax.scatter(*maze.xy(35),color='#bd3333',marker='*',s=120,zorder=5,label='Goal: cell 35')
        ax.set(aspect='equal',xlim=(-1.6,1.6),ylim=(-1.6,1.6),xlabel='World x (m)',ylabel='World y (m)',
               title=f'Identical endpoints: cell {start} → 35')
        ax.legend(loc='upper center',bbox_to_anchor=(.5,-.11),fontsize=8,ncol=2)
    fig.suptitle('Actual Nav2 paths and learned DQN routes; planning only, no motion',fontsize=14)
    fig.savefig(plots/'route_comparison.png',dpi=180)
    fig.savefig(plots/'route_comparison.pdf')
    plt.close(fig)

    seed_table = '\n'.join(f"| {s['seed']}{' (deployed)' if s['deployed'] else ''} | {s['episodes']:,} | {s['training_seconds']:.1f} | {s['successes']}/{s['trials']} | {s['heldout_initial_successes']}/21 |" for s in seeds)
    flags_table = '\n'.join(f"| {r['start_cell']} | {r['goal_cell']} | {r['nav2_status']} / {r['nav2_error_code']} | {1000*r['nav2_min_clearance_m']:.2f} |" for r in rows if r['nav2_min_clearance_m'] is not None and r['nav2_min_clearance_m'] < 0)
    number = lambda value: f'{value:.3f}' if value is not None else 'unavailable'
    history_table = '\n'.join(f"| {r['method']} | ({r['start_x_m']:.2f}, {r['start_y_m']:.2f}) | ({r['goal_x_m']:.2f}, {r['goal_y_m']:.2f}) | {r['wall_seconds']:.1f} | {100*r['gazebo_position_error_m']:.1f} | {100*r['minimum_clearance_m']:.1f} | [JSON](../{r['source']}) |" for r in execution)
    report = f'''# Nav2 and Deep Q-learning evaluation

Generated from measured project results: {summary['recorded_utc']}.

## Findings

The actual Nav2 planner **returned {summary['nav2_action_successes']}/{summary['pairs']} routes**;
**{summary['nav2_successes']}/{summary['pairs']} passed the independent 0.13 m envelope check**.
The deployed DQN produced **{summary['dqn_successes']}/{summary['pairs']} successful, envelope-checked routes**,
with **{summary['dqn_grid_optimal_routes']}/{summary['pairs']} shortest cell-graph routes** and
**{summary['dqn_wall_attempts']} wall attempts**. Both were asked to plan between exactly
the same cell-center coordinates. This is a fixed-maze functional comparison.

**{summary['nav2_clearance_flags']} Nav2 paths were flagged for planned envelope clearance**,
even though their planning actions succeeded. These are conservative geometric
flags on planned paths, not observed robot collisions or failed driving trials.
The shared DWB controller can track a different trajectory from the global plan.

The in-process cell-graph Dijkstra reference is faster than DQN in these measurements.
The actual Nav2 planner is measured separately through ROS; its server duration
and action latency are not equivalent to an in-process Python CPU timer.
The historical driving trials do not support a robot-speed ranking because their
start and goal locations differ. DQN demonstrates learned route selection;
these results do not establish an advantage over classical planning.

## What is being compared

| Property | Nav2 classical navigation | Deep Q-learning navigation |
|---|---|---|
| Global route decision | `maze_navigation::MazeNavfnPlanner`, NavFn with Dijkstra (`use_astar: false`) | Reward-trained neural DQN, deployed seed {summary['deployed_seed']} |
| Planning representation | 0.025 m costmap, continuous path coordinates | 6×6 cells, four adjacent-cell actions |
| Training | None | 2,000 offline grid episodes per seed; replay and target network |
| Local movement | DWB, velocity smoother, collision monitor | Same Nav2 components, learned waypoints via FollowPath and Spin |
| Learned inputs | Not applicable | Current/goal cell indicators, blocked neighbors, relative goal: 78 values |
| Goal set in this evaluation | Cells 4, 11, 35 | Same three trained goals |

DQN is a global route learner using Nav2 for local execution. It is not a complete
replacement for Nav2 and does not learn motor commands directly from LiDAR.
**Cell-graph Dijkstra is an additional reference, not a substitute for actual Nav2.**

## Matched planning experiment

The experiment requests each of 35 non-goal starting cells for each of three goals:
105 identical start–goal pairs. Nav2 uses `ComputePathToPose`, `use_start=true`,
planner `GridBased`, map-frame cell centers and yaw zero at both endpoints.
No robot motion actions are sent. The live map must match the DQN training world,
the planner must be active, and the expected NavFn Dijkstra plugin must be loaded.

Nav2 success requires action status 4, error code 0, a nonempty path, start and goal
errors at most 0.05 m, and nonnegative circle-to-wall clearance. Segment clearance
is sampled at intervals no larger than 0.0125 m with the same conservative 0.13 m
robot envelope. This is geometric checking, not a contact sensor.
DQN success uses the greedy learned policy with no search fallback or action masking.
Cell-graph optimality is checked against Dijkstra on the identical cell graph.
Continuous NavFn paths and cell-center polylines use different feasible path sets;
their geometric lengths are reported without treating their ratio as optimality.

| Metric | Measured result | Timing or representation scope |
|---|---:|---|
| Actual Nav2 returned routes | {summary['nav2_action_successes']}/{summary['pairs']} | Action status 4, error 0, nonempty path |
| Actual Nav2 envelope-checked routes | {summary['nav2_successes']}/{summary['pairs']} | Also requires endpoints and segment clearance |
| DQN successful routes | {summary['dqn_successes']}/{summary['pairs']} | Greedy fixed-maze policy |
| Cell-graph Dijkstra routes | {summary['dijkstra_successes']}/{summary['pairs']} | Separate discrete reference |
| DQN grid-optimal routes | {summary['dqn_grid_optimal_routes']}/{summary['pairs']} | Compared only with cell-graph Dijkstra |
| Nav2 planning median | {number(summary['nav2_server_planning_median_ms'])} sim ms | Server node clock, one measured request per pair |
| Nav2 action round-trip median | {number(summary['nav2_action_round_trip_median_ms'])} ms | Client wall clock, includes ROS transport/scheduling |
| DQN median route CPU time | {summary['dqn_pair_median_cpu_ms']:.3f} ms | Median of per-pair medians; {summary['cpu_repeats_per_pair']} repeats |
| Cell-graph Dijkstra median CPU time | {summary['dijkstra_pair_median_cpu_ms']:.3f} ms | Same process and repeat count |

CPU timers exclude model loading, training, path-clearance checks and ROS.
DQN timing includes greedy rollout/transition bookkeeping; Dijkstra returns its
cell route. CPU pairs are shuffled with seed 123 and method order alternates.
Nav2 has three unmeasured warm-up requests. Nav2 costmaps remain live; before/after
hashes and update stamps are saved, rather than claiming a frozen costmap.
No confidence interval is inferred from repeated CPU timings or these dependent,
same-map pairs. Timing values can vary when the script is rerun.
Nav2 Jazzy computes `planning_time` from its node clock; in this project that uses
simulation time. It is not a wall-clock CPU benchmark.
[Planner-server source](https://api.nav2.org/nav2-jazzy/html/planner__server_8cpp_source.html).

![Planning comparison](plots/planning_comparison.png)

![Same endpoints, actual Nav2 paths and DQN cell routes](plots/route_comparison.png)

## Independent planned-clearance flags

The evaluator preserves action success separately from its stricter circle-envelope
criterion. Negative values below are circle-to-wall distances, not body penetration
measurements. A 5 mm negative distance means the path center is 5 mm too close to
the SDF wall for the evaluator's 0.13 m enclosing circle. These plans were not driven
during this planning experiment. No collision count or safety superiority is inferred.

| Start cell | Goal cell | Action status / error | Minimum envelope clearance (mm) |
|---|---|---|---:|
{flags_table}

![Planned circle-envelope audit](plots/clearance_audit.png)

## Training variation

| DQN seed | Episodes | Training time (s) | Successes | Held-out initial pairs |
|---|---:|---:|---:|---:|
{seed_table}

Seed 7's three failed pairs remain failures. The deployment checkpoint was chosen
after same-maze qualification; its held-out score is not an unbiased model-selection
estimate. Held-out pairs are excluded from training resets, but their states may
appear during episodes. This does not test unseen maps or new goals.

## Recorded Gazebo demonstrations

These four selected final demonstrations passed their respective action and
position/heading checks. They document execution, not a matched physical benchmark
or an overall success rate across development attempts. Earlier failures and
repairs remain in [the Nav2 report](../reports/TOP_RIGHT_REPAIR.md) and
[the DQN report](../reports/DQN_REPORT.md).

| Method | Actual start x,y (m) | Goal x,y (m) | Execution (s) | Gazebo position error (cm) | Minimum sampled clearance (cm) | Source |
|---|---|---|---:|---:|---:|---|
{history_table}

Nav2 follows a continuous path; DQN stops/turns at learned waypoints. DQN also
temporarily uses an 8 cm intermediate waypoint tolerance, restoring the original
6 cm controller position tolerance for its destination and on exit. Its final
rotation is separate; independent end checks permit 9 cm map error, 12 cm Gazebo
error and 0.20 rad heading error. These execution-policy differences prevent
attributing historical timing differences solely to the global planner.

## Reproduce

From the workspace root, with the existing simulation and Nav2 running:

```bash
source scripts/dqn_env.bash
python evaluation/run_comparison.py --live-nav2
```

This collects planning results and regenerates all evaluation artifacts without
moving the robot or changing controller parameters. To regenerate from saved Nav2
plans without running ROS processes:

```bash
source scripts/dqn_env.bash
python evaluation/run_comparison.py
```

The second command re-times DQN/Dijkstra on this computer but retains the recorded
Nav2 capture time. `--repeats 30` controls CPU repeats per pair. The script exits
nonzero if either planner fails any pair; failed rows are retained in its outputs.
For this capture, exit code 1 is expected because nine plans fail the independent
envelope criterion; the report and all data are still generated successfully.

Check source hashes, saved geometry, row counts and metric consistency:

```bash
python evaluation/validate_results.py
python -m pytest evaluation/tests -q
```

The artifact audit checks consistency, rather than requiring every planner metric
to pass. [Audit result](results/validation.json), [nine regression checks](test_results.txt).

## Files and provenance

- [Machine-readable summary](results/comparison.json)
- [All 105 planning pairs](results/planning_trials.csv)
- [Actual Nav2 action results and paths](results/nav2_plans.json)
- [Raw repeated CPU timings](results/cpu_timings.json)
- [Three DQN seeds](results/dqn_seeds.csv)
- [Recorded Gazebo demonstration metrics](results/recorded_execution.csv)
- [Flagged Nav2 planned clearances](results/nav2_clearance_flags.csv)
- [Input checksums](results/source_manifest.json)
- Plot exports: `plots/*.png` and `plots/*.pdf`.

The reference maze, maps, checkpoint and controller configuration are unchanged by
this evaluation. The comparison covers this one known maze and three trained goals.
'''
    (HERE/'README.md').write_text(report)
