#!/usr/bin/env python3
"""
Render a top-down map of the reactor and storage pool with every cell's index written on it.

    assessment/scripts/cell_map.py [-o assessment/docs/cell_map.png]

Runs a headless Gazebo server on a copy of the assessment world without fuel rods and without the
crane (its rails would hide row 4), with one top-down camera above the centre of each pool. One
camera above the middle would not do: seen from there, the divider between the pools hides the
cell columns next to it. The cameras' intrinsics are known, so each cell centre (from
fuel_layout.cell_index, the mapping the cell_pose service uses) is projected into the image
exactly. The two views are placed side by side. Needs the workspace built and sourced.
"""

import argparse
import math
import os
import signal
import subprocess
import tempfile
import threading
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from assessment.fuel_layout import cell_index, CELL_TOP_Z, generate_world
from gz.msgs.image_pb2 import Image
from gz.transport import Node
from PIL import Image as PILImage, ImageDraw, ImageFont

WIDTH, HEIGHT = 1200, 820           # each of the two views
HFOV = math.radians(60)
VIEW_WIDTH = 10.25                  # metres covered across a view at the cell rims
CAMERAS = {'reactor': -5.75, 'storage': 5.75}   # x of each view's centre (pool centres)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def camera_height():
    """Camera z so that a view is VIEW_WIDTH wide at the cell rims."""
    return CELL_TOP_Z + (VIEW_WIDTH / 2) / math.tan(HFOV / 2)


def project(area, x, y, z=CELL_TOP_Z):
    """Pixel in the combined image of a world point seen by the camera above `area`'s pool."""
    f = (WIDTH / 2) / math.tan(HFOV / 2)
    depth = camera_height() - z
    offset = 0 if area == 'reactor' else WIDTH
    return offset + WIDTH / 2 + f * (x - CAMERAS[area]) / depth, HEIGHT / 2 - f * y / depth


def map_world(path):
    share = get_package_share_directory('assessment')
    tree, _ = generate_world(os.path.join(share, 'worlds', 'assessment.world'), seed=0,
                             reactor_fill_percentage=0, replacement_percentage=0,
                             initial_mean_level=50)
    world = tree.getroot().find('world')
    world.remove(world.find("./model[@name='crane']"))
    for area, x in CAMERAS.items():
        # Pitch down, then yaw by 90 deg so that image up is +y (north), image right +x (east)
        world.append(ET.fromstring(f"""
          <model name="cell_map_{area}"><static>true</static>
            <pose>{x} 0 {camera_height():.4f} 0 1.5707963 1.5707963</pose>
            <link name="link"><sensor name="camera" type="camera">
              <always_on>true</always_on><update_rate>2</update_rate>
              <topic>/cell_map/{area}</topic>
              <camera><horizontal_fov>{HFOV}</horizontal_fov>
                <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format>
                </image><clip><near>0.1</near><far>100</far></clip></camera>
            </sensor></link></model>"""))
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return os.path.dirname(share)


def grab_frames(world_file, resource_path):
    """Return {area: PIL image} from the two map cameras."""
    frames, lock, done = {}, threading.Lock(), threading.Event()

    def subscriber(area):
        def on_image(msg):
            with lock:
                frames[area] = PILImage.frombytes('RGB', (msg.width, msg.height), msg.data)
                if len(frames) == len(CAMERAS):
                    done.set()
        return on_image

    env = dict(os.environ)
    env['GZ_SIM_RESOURCE_PATH'] = f'{resource_path}:{env.get("GZ_SIM_RESOURCE_PATH", "")}'
    server = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', '-v1', world_file],
                              env=env, start_new_session=True,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    node = Node()
    try:
        for area in CAMERAS:
            node.subscribe(Image, f'/cell_map/{area}', subscriber(area))
        if not done.wait(120):
            raise RuntimeError('no images from the map cameras within 120 s')
        done.clear()
        with lock:
            frames.clear()
        done.wait(10)  # take later frames, once everything has settled
        with lock:
            return dict(frames)
    finally:
        for area in CAMERAS:
            node.unsubscribe(f'/cell_map/{area}')
        os.killpg(server.pid, signal.SIGTERM)
        server.wait(timeout=30)


def compose(frames, cells):
    img = PILImage.new('RGB', (2 * WIDTH, HEIGHT))
    img.paste(frames['reactor'], (0, 0))
    img.paste(frames['storage'], (WIDTH, 0))
    draw = ImageDraw.Draw(img)
    draw.rectangle((WIDTH - 3, 0, WIDTH + 2, HEIGHT), fill='black')
    small, large = ImageFont.truetype(FONT, 17), ImageFont.truetype(FONT, 30)

    def text(xy, s, font, fill='black', stroke='white'):
        draw.text(xy, s, font=font, fill=fill, anchor='mm', stroke_width=3, stroke_fill=stroke)

    for index, (x, y, area, _) in cells.items():
        text(project(area, x, y), index, small)
    columns = sorted({(i[0], c[0], c[2]) for i, c in cells.items()})
    rows = sorted({(int(i[1:]), c[1]) for i, c in cells.items()})
    top = max(y for _, y in rows) + 0.55
    for letter, x, area in columns:
        text(project(area, x, top), letter, large)
    for area, side in (('reactor', -9.3), ('storage', 9.3)):
        for number, y in rows:
            text(project(area, side, y), str(number), large)
    for area, title in (('reactor', 'REACTOR (A-M)'), ('storage', 'STORAGE POOL (N-Z)')):
        text(project(area, CAMERAS[area], top + 0.65), title, large, fill='white', stroke='black')
    text(project('reactor', -9.3, top + 0.65), 'N ^', large, fill='white', stroke='black')
    return img


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('-o', '--output', default='cell_map.png')
    args = parser.parse_args()
    share = get_package_share_directory('assessment')
    cells = cell_index(ET.parse(os.path.join(share, 'worlds', 'assessment.world'))
                       .getroot().find('world'))
    with tempfile.TemporaryDirectory() as tmp:
        world_file = os.path.join(tmp, 'cell_map.world')
        frames = grab_frames(world_file, map_world(world_file))
    compose(frames, cells).save(args.output)
    print(f'wrote {args.output} ({len(cells)} cells)')


if __name__ == '__main__':
    main()
