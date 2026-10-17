"""Cell-level MDP derived from the unchanged corrected Gazebo maze geometry."""
import hashlib
import heapq
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ACTIONS = ((1, 0), (0, 1), (-1, 0), (0, -1))  # east, north, west, south
ACTION_NAMES = ('east', 'north', 'west', 'south')
DEFAULT_GOALS = (4, 11, 35)


def default_world():
    source = Path(__file__).resolve().parents[2]/'maze_world/worlds/maze_fixed.sdf'
    if source.exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('maze_world'))/'worlds/maze_fixed.sdf'


class Maze:
    size = 6
    cells = 36
    spacing = 0.5
    radius = 0.13
    observation_size = 78

    def __init__(self, world=None):
        self.world = Path(world) if world else default_world()
        raw = self.world.read_bytes()
        self.world_hash = hashlib.sha256(raw).hexdigest()
        self.walls = []
        for link in ET.fromstring(raw).findall(".//model[@name='maze_walls']/link"):
            pose = list(map(float, link.findtext('pose').split()))
            size = list(map(float, link.findtext('collision/geometry/box/size').split()))
            if pose[3:] != [0., 0., 0.]:
                raise ValueError('Maze cells require axis-aligned SDF walls')
            self.walls.append((pose[0], pose[1], size[0]/2, size[1]/2))
        if len(self.walls) != 20:
            raise ValueError('Expected the corrected maze with 20 wall boxes')
        self.edges = np.full((self.cells, 4), -1, dtype=np.int64)
        for cell in range(self.cells):
            if self.clearance(*self.xy(cell)) <= 0:
                raise ValueError(f'Cell {cell} does not fit the Burger envelope')
            col, row = cell % self.size, cell // self.size
            for action, (dx, dy) in enumerate(ACTIONS):
                if 0 <= col+dx < self.size and 0 <= row+dy < self.size:
                    other = (row+dy)*self.size+col+dx
                    a, b = self.xy(cell), self.xy(other)
                    if all(self.clearance(a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
                           > 0 for t in np.linspace(0, 1, 51)):
                        self.edges[cell, action] = other
        for cell in range(self.cells):
            for action, other in enumerate(self.edges[cell]):
                if other >= 0 and self.edges[other, (action+2) % 4] != cell:
                    raise ValueError('Inconsistent maze adjacency')
        if len(self.distances(0)) != self.cells:
            raise ValueError('The reference maze cell graph is disconnected')
        self.signature = hashlib.sha256(json.dumps({
            'world': self.world_hash, 'radius': self.radius,
            'edges': self.edges.tolist(), 'observation_version': 1}, sort_keys=True).encode()
        ).hexdigest()

    def xy(self, cell):
        if not 0 <= cell < self.cells:
            raise ValueError(f'Invalid cell {cell}; use 0..35')
        return (-1.25+self.spacing*(cell % self.size),
                -1.25+self.spacing*(cell // self.size))

    def cell_at(self, x, y, tolerance=0.30):
        cell = min(range(self.cells), key=lambda c: math.dist(self.xy(c), (x, y)))
        if math.dist(self.xy(cell), (x, y)) > tolerance:
            raise ValueError('Robot is too far from a cell center for discrete route execution')
        return cell

    def matches_map(self, info, data):
        if (info.width, info.height) != (128, 128) or abs(info.resolution-.025) > 1e-6:
            return False
        if abs(info.origin.position.x+1.6) > 1e-6 or abs(info.origin.position.y+1.6) > 1e-6:
            return False
        q = info.origin.orientation
        if abs(q.x)+abs(q.y)+abs(q.z) > 1e-6 or abs(abs(q.w)-1.) > 1e-6:
            return False
        actual = np.asarray(data).reshape(128, 128)
        if (actual < 0).any():
            return False
        coords = -1.6+(np.arange(128)+.5)*.025
        xx, yy = np.meshgrid(coords, coords)
        expected = np.zeros((128, 128), dtype=bool)
        for x, y, hx, hy in self.walls:
            expected |= (np.abs(xx-x) <= hx+1e-9) & (np.abs(yy-y) <= hy+1e-9)
        return bool(np.array_equal(actual >= 65, expected))

    def clearance(self, x, y):
        return min(math.hypot(max(abs(x-a)-hx, 0), max(abs(y-b)-hy, 0))-self.radius
                   for a, b, hx, hy in self.walls)

    def observe(self, cell, goal):
        state = np.zeros(self.observation_size, dtype=np.float32)
        state[cell], state[self.cells+goal] = 1., 1.
        state[72:76] = self.edges[cell] < 0
        x, y = self.xy(cell)
        gx, gy = self.xy(goal)
        state[76:] = ((gx-x)/2.5, (gy-y)/2.5)
        return state

    def transition(self, cell, goal, action, gamma=0.98):
        if cell == goal:
            raise ValueError('Reset after goal termination')
        if action not in range(4):
            raise ValueError('Action must be 0..3')
        successor = int(self.edges[cell, action])
        collision = successor < 0
        successor = cell if collision else successor
        terminated = successor == goal
        # Potential shaping uses Manhattan distance only; no Dijkstra labels/rewards.
        potential = lambda c: -(abs(c % 6-goal % 6)+abs(c // 6-goal // 6))/10.
        shaped = gamma*(0. if terminated else potential(successor))-potential(cell)
        reward = (10. if terminated else -1. if collision else -0.05)+shaped
        return successor, reward, terminated, collision

    def distances(self, start):
        distances, frontier = {start: 0}, [(0, start)]
        while frontier:
            cost, cell = heapq.heappop(frontier)
            if cost != distances[cell]:
                continue
            for other in self.edges[cell]:
                other = int(other)
                if other >= 0 and (other not in distances or cost+1 < distances[other]):
                    distances[other] = cost+1
                    heapq.heappush(frontier, (cost+1, other))
        return distances

    def dijkstra(self, start, goal):
        costs, previous, frontier = {start: 0}, {}, [(0, start)]
        while frontier:
            cost, cell = heapq.heappop(frontier)
            if cell == goal:
                path = [goal]
                while path[-1] != start:
                    path.append(previous[path[-1]])
                return list(reversed(path))
            if cost != costs[cell]:
                continue
            for other in self.edges[cell]:
                other = int(other)
                if other >= 0 and (other not in costs or cost+1 < costs[other]):
                    costs[other], previous[other] = cost+1, cell
                    heapq.heappush(frontier, (cost+1, other))
        raise ValueError('No path')


def rollout(maze, agent, start, goal, max_steps=72):
    """Greedy learned policy only: no action masking, search, or fallback."""
    cell, path, actions, collisions, total = start, [start], [], 0, 0.
    for _ in range(max_steps):
        if cell == goal:
            break
        action = agent.act(maze.observe(cell, goal))
        cell, reward, _, collision = maze.transition(cell, goal, action, agent.config.gamma)
        path.append(cell)
        actions.append(action)
        collisions += int(collision)
        total += reward
    return {'start': start, 'goal': goal, 'success': cell == goal, 'path': path,
            'actions': actions, 'steps': len(actions), 'wall_attempts': collisions,
            'return': total}
