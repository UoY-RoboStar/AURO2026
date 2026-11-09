"""
Fuel rod layout for the assessment world.

The world file ships with every reactor fuel cell holding a fuel rod. At launch, generate_world()
replaces those rods with a random layout: which reactor fuel cells hold a rod, which storage cells
hold replacement rods, and the power level of each rod. The control rods stay where they are.

The power levels never reach Gazebo. They go, with the cell geometry the reactor monitor needs,
into a manifest that only the reactor_monitor node receives: serve_manifest() hands it over once,
through a named pipe, so it is never stored in a file or a parameter that other nodes could read.
"""

import atexit
import copy
import json
import os
import random
import re
import shutil
import tempfile
import threading
import xml.etree.ElementTree as ET

# Height (z) of a rod's model origin when it stands in a cell: the world places rods at -3.845 and
# they settle 5 mm onto the cell floor.
ROD_SEATED_Z = -3.85
ROD_HALF_HEIGHT = 0.95  # rod collision box is 1.9 m tall, centred on the model origin
CELL_TOP_Z = -2.895     # cell rims: cells are 2 m tall, standing on the pool floor at -4.895

DISTRIBUTIONS = ('uniform', 'normal')


def _cell_centres(world, model_name):
    """(x, y) of every cell in a cell-grid model, from its '<kind>_RR_CC_visual' visuals."""
    model = world.find(f"./model[@name='{model_name}']")
    centres = []
    for visual in model.iter('visual'):
        if re.fullmatch(r'\w+_\d\d_\d\d_visual', visual.get('name', '')):
            x, y = (float(v) for v in visual.find('pose').text.split()[:2])
            centres.append((x, y))
    return centres


def cell_index(world):
    """
    Return {index: (x, y, area, control_cell)} for every cell of the reactor and the storage pool.

    Both pools form one grid: columns are lettered west to east (A = westmost; the reactor is A-M
    and the storage pool N-Z), rows are numbered north to south (0 = northmost). `world` is the
    <world> element of the assessment world.
    """
    cells = [(x, y, 'reactor', False) for x, y in _cell_centres(world, 'fuel_cells')]
    cells += [(x, y, 'reactor', True) for x, y in _cell_centres(world, 'control_cells')]
    cells += [(x, y, 'storage', False) for x, y in _cell_centres(world, 'storage_cells')]
    columns = sorted({x for x, _, _, _ in cells})
    rows = sorted({y for _, y, _, _ in cells}, reverse=True)
    if len(columns) > 26:
        raise ValueError(f'{len(columns)} cell columns, but only 26 letters')
    return {f'{chr(ord("A") + columns.index(x))}{rows.index(y)}': (x, y, area, control)
            for x, y, area, control in cells}


def _draw_level(rng, mean, distribution, spread):
    if distribution == 'uniform':
        level = rng.uniform(mean - spread, mean + spread)
    else:
        level = rng.gauss(mean, spread)
    return round(min(100.0, max(0.0, level)), 1)


def generate_world(world_file, seed, reactor_fill_percentage, replacement_percentage,
                   initial_mean_level, initial_level_distribution='uniform',
                   initial_level_spread=25.0, replacement_level=100.0):
    """
    Return (ElementTree of the world with the generated fuel rods, manifest dict).

    reactor_fill_percentage   share of the reactor's fuel cells that hold a rod
    replacement_percentage    share of the storage cells that hold a replacement rod
    initial_mean_level        mean power level of the rods in the reactor; individual levels are
                              drawn from initial_level_distribution ('uniform': mean +/- spread,
                              'normal': standard deviation spread) and clipped to 0-100
    replacement_level         power level of every replacement rod
    """
    for name, value in (('reactor_fill_percentage', reactor_fill_percentage),
                        ('replacement_percentage', replacement_percentage),
                        ('initial_mean_level', initial_mean_level),
                        ('replacement_level', replacement_level)):
        if not 0.0 <= value <= 100.0:
            raise ValueError(f'{name} must be between 0 and 100, got {value}')
    if initial_level_distribution not in DISTRIBUTIONS:
        raise ValueError(f'initial_level_distribution must be one of {DISTRIBUTIONS}, '
                         f'got {initial_level_distribution!r}')
    if initial_level_spread < 0.0:
        raise ValueError(f'initial_level_spread must be >= 0, got {initial_level_spread}')

    tree = ET.parse(world_file)
    world = tree.getroot().find('world')
    fuel_cells = _cell_centres(world, 'fuel_cells')
    control_cells = _cell_centres(world, 'control_cells')
    storage_cells = _cell_centres(world, 'storage_cells')

    # Use the world's own fuel rod as the template, and drop all of the shipped fuel rods.
    shipped = [m for m in world.findall('model') if m.get('name', '').startswith('fuel_rod_')]
    template = copy.deepcopy(shipped[0])
    insert_at = list(world).index(shipped[0])
    for model in shipped:
        world.remove(model)

    rng = random.Random(seed)
    reactor = rng.sample(fuel_cells, round(len(fuel_cells) * reactor_fill_percentage / 100))
    storage = rng.sample(storage_cells, round(len(storage_cells) * replacement_percentage / 100))
    rods = ([(cell, _draw_level(rng, initial_mean_level, initial_level_distribution,
                                initial_level_spread)) for cell in reactor]
            + [(cell, round(replacement_level, 1)) for cell in storage])
    # Neutral names, assigned in random order, so a name reveals neither the level nor the cell.
    rng.shuffle(rods)

    levels = {}
    template_z = template.find('pose').text.split()[2]
    for i, ((x, y), level) in enumerate(rods):
        name = f'fuel_rod_{i:03d}'
        model = copy.deepcopy(template)
        model.set('name', name)
        model.find('pose').text = f'{x:g} {y:g} {template_z} 0 0 0'
        world.insert(insert_at + i, model)
        levels[name] = level

    manifest = {
        'seed': seed,
        'fuel_rods': levels,
        'control_rods': sorted(m.get('name') for m in world.findall('model')
                               if m.get('name', '').startswith('control_rod_')),
        'reactor_cells': [list(c) for c in fuel_cells + control_cells],
        'reactor_fuel_cell_count': len(fuel_cells),
        'cells': {k: list(v) for k, v in cell_index(world).items()},
        'cell_top_z': CELL_TOP_Z,
        'storage_cells': [list(c) for c in storage_cells],
        'rod_seated_z': ROD_SEATED_Z,
        'rod_half_height': ROD_HALF_HEIGHT,
    }
    return tree, manifest


def serve_manifest(manifest):
    """
    Make a manifest readable once, by read_manifest(), and return the path to read it from.

    The path is a named pipe in a new private directory: its contents never touch the disk. A
    background thread waits for the first process to open it, removes the path at once, so no
    other process can open it, and writes the manifest to that one reader. If another process got
    there first, read_manifest() fails, so the theft does not go unnoticed. Whatever is left is
    removed when this process exits.
    """
    directory = tempfile.mkdtemp(prefix='assessment_')
    atexit.register(shutil.rmtree, directory, ignore_errors=True)
    path = os.path.join(directory, 'manifest')
    os.mkfifo(path, 0o600)
    data = json.dumps(manifest).encode()

    def serve():
        try:
            with open(path, 'wb') as pipe:  # blocks until a reader opens the other end
                os.unlink(path)
                pipe.write(data)
        except OSError:
            pass  # the reader went away, or the directory went at exit

    threading.Thread(target=serve, name='manifest', daemon=True).start()
    return path


def read_manifest(path):
    """Return the manifest served at path by serve_manifest(); it can be read only once."""
    try:
        with open(path, 'rb') as pipe:
            data = pipe.read()
    except FileNotFoundError:
        raise RuntimeError(f'the fuel manifest at {path} is gone: another process has read it')
    try:
        return json.loads(data)
    except ValueError:
        raise RuntimeError(f'the fuel manifest at {path} is incomplete: another process has '
                           'read part of it')
