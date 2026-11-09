"""ArUco tags for the arena's walls, made at launch."""

import math

import cv2
import numpy as np

CELLS = 6  # of a 4x4 marker, with its black border
CELL_PIXELS = 80
MARKER_SIZE = 0.3  # m
TAG_HEIGHT = 0.25  # m: half way up the walls
WALL_FACE = 2.34  # m from the centre, just off the inside face of the walls in week_6.world
ALONG_WALL = (-1.25, 1.25)  # m, the places for tags on each wall


def tags():
    """Yield each tag's id, position (x, y) and normal (x, y), going around the arena."""
    tag_id = 0
    for normal_x, normal_y in ((-1, 0), (0, -1), (1, 0), (0, 1)):
        for along in ALONG_WALL:
            yield (tag_id,
                   (-normal_x * WALL_FACE - normal_y * along,
                    -normal_y * WALL_FACE + normal_x * along),
                   (normal_x, normal_y))
            tag_id += 1


def tag_image(tag_id):
    """Return the tag with its white quiet zone of one cell, without which it cannot be found."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, CELLS * CELL_PIXELS)
    return np.pad(marker, CELL_PIXELS, constant_values=255)


def visual(tag_id, position, normal, texture):
    """Return a tag's SDF, on a plane without collision: the lidar must not see it."""
    panel = MARKER_SIZE * (CELLS + 2) / CELLS
    # Upright: the plane's x along the wall, y up, and z (its visible side) along the normal
    yaw = math.atan2(normal[0], -normal[1])
    return f"""
      <visual name="tag_{tag_id}">
        <pose>{position[0]} {position[1]} {TAG_HEIGHT} {math.pi / 2:.6f} 0 {yaw:.6f}</pose>
        <geometry><plane><normal>0 0 1</normal><size>{panel} {panel}</size></plane></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr><metal>
            <albedo_map>file://{texture}</albedo_map>
            <metalness>0</metalness>
            <roughness>1</roughness>
          </metal></pbr>
        </material>
      </visual>"""


def write_tags(new_file):
    """Write the tags' model and textures, to paths from new_file(suffix): return the model's."""
    visuals = ''
    for tag_id, position, normal in tags():
        texture = new_file('.png')
        cv2.imwrite(texture, tag_image(tag_id))
        visuals += visual(tag_id, position, normal, texture)
    model = new_file('.sdf')
    with open(model, 'w') as f:
        f.write(f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="aruco_tags">
    <static>true</static>
    <link name="link">{visuals}
    </link>
  </model>
</sdf>
""")
    return model
