import os
import statistics

from assessment.fuel_layout import generate_world
import pytest

WORLD = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'assessment.world')


def layout(seed=1, **kw):
    args = {'reactor_fill_percentage': 80, 'replacement_percentage': 20, 'initial_mean_level': 50}
    args.update(kw)
    return generate_world(WORLD, seed, **args)


def rod_poses(tree):
    world = tree.getroot().find('world')
    return {m.get('name'): tuple(float(v) for v in m.find('pose').text.split()[:3])
            for m in world.findall('model')
            if m.get('name').startswith(('fuel_rod', 'control_rod'))}


def test_same_seed_same_layout():
    (t1, m1), (t2, m2) = layout(seed=7), layout(seed=7)
    assert m1 == m2
    assert rod_poses(t1) == rod_poses(t2)
    assert layout(seed=8)[1]['fuel_rods'] != m1['fuel_rods']


def test_counts_and_placement():
    tree, manifest = layout()
    poses = rod_poses(tree)
    fuel = {n: p for n, p in poses.items() if n.startswith('fuel_rod')}
    in_reactor = [n for n, (x, y, z) in fuel.items() if x < 0]
    in_storage = [n for n, (x, y, z) in fuel.items() if x > 0]
    assert len(in_reactor) == round(93 * 0.8)
    assert len(in_storage) == round(117 * 0.2)
    assert set(fuel) == set(manifest['fuel_rods'])
    assert len(manifest['control_rods']) == 24
    assert manifest['reactor_fuel_cell_count'] == 93
    # one rod per cell
    assert len({(x, y) for x, y, z in poses.values()}) == len(poses)
    # replacements are full
    assert all(manifest['fuel_rods'][n] == 100.0 for n in in_storage)


@pytest.mark.parametrize('distribution', ['uniform', 'normal'])
def test_levels(distribution):
    _, manifest = layout(reactor_fill_percentage=100, replacement_percentage=0,
                         initial_mean_level=60, initial_level_distribution=distribution,
                         initial_level_spread=10)
    levels = list(manifest['fuel_rods'].values())
    assert all(0 <= v <= 100 for v in levels)
    assert abs(statistics.mean(levels) - 60) < 4
    if distribution == 'uniform':
        assert min(levels) >= 50 and max(levels) <= 70


def test_cell_index():
    cells = layout()[1]['cells']
    assert len(cells) == 93 + 24 + 117
    assert cells['A0'] == [-8.75, 2.0, 'reactor', False]    # north-west corner of the reactor
    assert cells['M8'] == [-2.75, -2.0, 'reactor', False]   # south-east corner of the reactor
    assert cells['N0'] == [2.75, 2.0, 'storage', False]     # storage pool starts at N
    assert cells['Z8'] == [8.75, -2.0, 'storage', False]
    assert cells['B1'] == [-8.25, 1.5, 'reactor', True]     # first control cell
    assert sum(c[3] for c in cells.values()) == 24


def test_rejects_bad_parameters():
    with pytest.raises(ValueError):
        layout(reactor_fill_percentage=120)
    with pytest.raises(ValueError):
        layout(initial_level_distribution='poisson')


def test_manifest_is_served_once():
    from assessment.fuel_layout import read_manifest, serve_manifest
    _, manifest = layout(seed=3)
    path = serve_manifest(manifest)
    assert read_manifest(path) == manifest
    assert not os.path.exists(path)
    with pytest.raises(RuntimeError, match='another process has read it'):
        read_manifest(path)
