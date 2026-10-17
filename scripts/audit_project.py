#!/usr/bin/python3
"""Read-only structural and corrected-world/reference-map alignment audit."""
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    nav = ROOT / 'src/maze_navigation'
    world = ROOT / 'src/maze_world'
    for path in list(nav.rglob('*.py')) + list((world/'launch').glob('*.py')):
        ast.parse(path.read_text(), filename=str(path))
    for path in [nav/'package.xml', world/'package.xml']:
        ET.parse(path)
    meta = yaml.safe_load((nav/'maps/maze_map.yaml').read_text())
    image = (nav/'maps'/meta['image']).read_bytes()
    magic, dimensions, maximum, pixels = image.split(b'\n', 3)
    width, height = map(int, dimensions.split())
    assert magic == b'P5' and maximum == b'255' and len(pixels) == width*height
    expected = (128, 128, .025, [-1.6, -1.6, 0.])
    assert (width, height, meta['resolution'], meta['origin']) == expected
    tree = ET.parse(world/'worlds/maze_fixed.sdf')
    walls = []
    for link in tree.findall(".//model[@name='maze_walls']/link"):
        pose = list(map(float, link.findtext('pose').split()))
        size = list(map(float, link.findtext('collision/geometry/box/size').split()))
        assert pose[3:] == [0., 0., 0.], 'Audit expects axis-aligned walls'
        walls.append((pose[0], pose[1], size[0]/2, size[1]/2))
    mismatch, occupied = 0, 0
    for row in range(height):
        y = meta['origin'][1] + (height-row-.5)*meta['resolution']
        for col in range(width):
            x = meta['origin'][0] + (col+.5)*meta['resolution']
            predicted = any(abs(x-wx) <= hx+1e-9 and abs(y-wy) <= hy+1e-9
                            for wx, wy, hx, hy in walls)
            actual = pixels[row*width+col] < 90
            mismatch += predicted != actual
            occupied += actual
    assert mismatch == 0, f'{mismatch} pixels differ from corrected SDF geometry'
    params = yaml.safe_load((nav/'config/nav2_params.yaml').read_text())
    for name, value in params.items():
        if 'ros__parameters' in value:
            assert value['ros__parameters']['use_sim_time'], name
    for name in ['controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor']:
        assert params[name]['ros__parameters']['enable_stamped_cmd_vel'], name
    assert params['amcl']['ros__parameters']['set_initial_pose']
    assert not params['planner_server']['ros__parameters']['GridBased']['use_astar']
    world_hash = hashlib.sha256((world/'worlds/maze_fixed.sdf').read_bytes()).hexdigest()
    report = {'passed': True, 'wall_links': len(walls), 'map_size': [width, height],
              'occupied_pixels': occupied, 'mismatched_pixels': mismatch,
              'world_sha256': world_hash,
              'map_sha256': hashlib.sha256(image).hexdigest()}
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
