import numpy as np
import pytest
import torch

from maze_dqn.agent import Agent, Config, Replay, bellman_target
from maze_dqn.environment import Maze, rollout
from maze_dqn.follow import route_points
from maze_dqn.train import split_pairs


def test_geometry_blocks_walls_and_preserves_real_corridors():
    maze = Maze()
    assert maze.edges.shape == (36, 4)
    assert (maze.edges >= 0).sum() == 72  # 36 undirected open passages
    assert maze.edges[1, 1] == 7  # Spawn cell has a north passage.
    assert maze.edges[1, 3] == -1  # Southern perimeter.
    assert maze.edges[1, 0] == 2
    for cell in range(36):
        assert len(maze.distances(cell)) == 36
        for other in maze.edges[cell]:
            if other >= 0:
                assert maze.clearance(*((np.array(maze.xy(cell)) + maze.xy(other))/2)) > 0


def test_collision_is_penalized_without_moving_and_goal_terminates():
    maze = Maze()
    cell, reward, done, collision = maze.transition(1, 7, 3)
    assert cell == 1 and collision and not done and reward < -0.9
    cell, reward, done, collision = maze.transition(1, 7, 1)
    assert cell == 7 and done and not collision and reward > 9
    with pytest.raises(ValueError):
        maze.transition(7, 7, 1)


def test_terminal_bellman_target_and_time_limit_bootstrapping():
    targets = bellman_target(torch.tensor([10., -0.05]), torch.tensor([1., 0.]),
                             torch.tensor([100., 2.]), .98)
    assert torch.allclose(targets, torch.tensor([10., 1.91]))


def test_replay_wraps_without_mixing_transition_components():
    replay = Replay(3, 2, 7)
    for i in range(5):
        replay.add(np.full(2, i), i, i, np.full(2, i+1), False)
    states, actions, rewards, next_states, terminal = replay.sample(3)
    assert set(actions.tolist()) == {2, 3, 4}
    assert torch.equal(states[:, 0], rewards)
    assert torch.equal(next_states, states+1)
    assert not terminal.any()


def test_gradient_updates_online_and_periodically_copies_frozen_target(tmp_path):
    agent = Agent(2, config=Config(hidden=16, batch_size=8, target_interval=2))
    for i in range(8):
        agent.replay.add(np.array([1., 0.]), 1, 10., np.array([0., 1.]), True)
    original = [p.clone() for p in agent.target.parameters()]
    assert np.isfinite(agent.learn())
    assert all(torch.equal(a, b) for a, b in zip(original, agent.target.parameters()))
    assert any(not torch.equal(a, b) for a, b in zip(original, agent.online.parameters()))
    agent.learn()
    assert all(torch.equal(a, b) for a, b in zip(agent.online.parameters(),
                                               agent.target.parameters()))
    assert all(p.grad is None for p in agent.target.parameters())
    model = tmp_path/'model.pt'
    agent.save(model, {'meaning': 'round trip'})
    loaded, metadata = Agent.load(model)
    assert metadata['meaning'] == 'round trip'
    assert torch.equal(agent.online(torch.ones(2)), loaded.online(torch.ones(2)))


def test_failed_learned_policy_is_not_replaced_by_dijkstra():
    class BadPolicy:
        config = Config()

        def act(self, observation):
            return 3  # South points into the bottom perimeter at the spawn.

    result = rollout(Maze(), BadPolicy(), 1, 35, max_steps=5)
    assert not result['success']
    assert result['wall_attempts'] == 5
    assert result['path'] == [1]*6


def test_heldout_initial_pairs_are_reproducible_and_not_training_resets():
    train, heldout = split_pairs([4, 11, 35], 7)
    assert len(train) == 84 and len(heldout) == 21
    assert not set(train) & set(heldout)
    assert (train, heldout) == split_pairs([4, 11, 35], 7)


def test_densified_path_keeps_bends_clear_and_rejects_wall_connector():
    maze = Maze()
    points = route_points(maze, maze.dijkstra(1, 35))
    assert points[0] == maze.xy(1) and points[-1] == maze.xy(35)
    assert all(np.linalg.norm(np.subtract(a, b)) <= .025001
               for a, b in zip(points, points[1:]))
    assert min(maze.clearance(*p) for p in points) >= .0949
    with pytest.raises(ValueError, match='clearance'):
        route_points(maze, [1, 7], actual_start=(-1.5, -1.25))


def test_pose_must_match_a_discrete_cell_before_route_execution():
    maze = Maze()
    assert maze.cell_at(.77, -1.23) == 4
    assert maze.cell_at(-.775, 1.02) == 31  # A live pose between cell centers.
    with pytest.raises(ValueError, match='too far'):
        maze.cell_at(0., 0.)


def test_active_map_guard_accepts_reference_and_rejects_changed_wall():
    from nav_msgs.msg import MapMetaData
    maze = Maze()
    raw = (maze.world.parents[2]/'maze_navigation/maps/maze_map.pgm').read_bytes()
    pixels = np.frombuffer(raw.split(b'\n', 3)[3], dtype=np.uint8).reshape(128, 128)
    cells = np.flipud((pixels < 90).astype(np.int8)*100)
    info = MapMetaData(width=128, height=128, resolution=.025)
    info.origin.position.x = info.origin.position.y = -1.6
    info.origin.orientation.w = 1.
    assert maze.matches_map(info, cells.reshape(-1))
    cells = cells.copy()
    cells[0, 0] = 100-cells[0, 0]
    assert not maze.matches_map(info, cells.reshape(-1))


def test_mid_edge_start_does_not_reverse_to_an_already_passed_cell_center():
    points = route_points(Maze(), [1, 7], actual_start=(-.775, -1.1))
    assert points[0] == (-.775, -1.1)
    assert all(b[1] >= a[1] for a, b in zip(points, points[1:]))
    assert points[-1] == (-.75, -.75)


def test_waypoint_tolerance_preserves_and_restores_original_controller_value():
    from types import MethodType, SimpleNamespace
    from maze_dqn.follow import Execution
    applied = []

    def set_value(request):
        applied.append(request.parameters[0].value.double_value)
        return SimpleNamespace(results=[SimpleNamespace(successful=True)])

    client = SimpleNamespace(
        original_xy_tolerance=None, wait=lambda future: future,
        get_parameters_client=SimpleNamespace(call_async=lambda request:
            SimpleNamespace(values=[SimpleNamespace(double_value=.06)])),
        set_parameters_client=SimpleNamespace(call_async=set_value))
    client.waypoint_tolerance = MethodType(Execution.waypoint_tolerance, client)
    client.waypoint_tolerance(.08)
    Execution.restore_tolerance(client)
    assert applied == [.08, .06]
    assert client.original_xy_tolerance == .06


def test_unexpected_controller_tolerance_is_not_overwritten():
    from types import SimpleNamespace
    from maze_dqn.follow import Execution
    client = SimpleNamespace(
        original_xy_tolerance=None, wait=lambda future: future,
        get_parameters_client=SimpleNamespace(call_async=lambda request:
            SimpleNamespace(values=[SimpleNamespace(double_value=.25)])))
    with pytest.raises(RuntimeError, match='Unexpected project goal tolerance'):
        Execution.waypoint_tolerance(client, .08)


def test_failed_final_rotation_cannot_be_reported_as_route_success():
    import math
    from types import SimpleNamespace
    from maze_dqn.follow import Execution
    node = SimpleNamespace(maze=Maze(), truth={'x': .25, 'y': -1.25, 'yaw': 0.},
                           stop_requested=False, original_xy_tolerance=.06,
                           waypoint_tolerance=lambda value: None, restore_tolerance=lambda: None,
                           path_message=lambda points, yaw: points)
    node.pose = lambda: node.truth.copy()
    attempted = []

    def execute(path, timeout, target, yaw, intermediate=False, spin_yaw=None):
        attempted.append(spin_yaw is not None)
        node.truth.update(x=target[0], y=target[1], yaw=yaw)
        return {'passed': spin_yaw is None, 'minimum_clearance': .095,
                'moving_commands': 10, 'trajectory': []}

    node.execute = execute
    result = Execution.execute_route(node, [3, 4], 30, math.pi/2)
    assert attempted == [False, True]
    assert result['waypoints_completed'] == 1
    assert not result['passed']  # Translation passed, but arrival orientation did not.
