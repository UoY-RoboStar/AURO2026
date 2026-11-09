import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest
from week_6.aruco_tags import tag_image, tags, write_tags
from week_6.visual_sensor import find_barrels

WALL_INSIDE = 2.35


def blank():
    return np.full((480, 640, 3), 200, np.uint8)


def test_tags_are_unique_and_on_the_walls_facing_in():
    placed = list(tags())
    assert len({tag_id for tag_id, _, _ in placed}) == len(placed) == 8
    for _, (x, y), (normal_x, normal_y) in placed:
        assert max(abs(x), abs(y)) == pytest.approx(WALL_INSIDE, abs=0.02)
        assert x * normal_x + y * normal_y < 0


@pytest.mark.parametrize('tag_id', [0, 7])
def test_tag_images_can_be_detected(tag_id):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    _, ids, _ = cv2.aruco.detectMarkers(tag_image(tag_id), dictionary)
    assert ids.flatten().tolist() == [tag_id]


def test_tag_model_has_no_collision(tmp_path):
    paths = iter(tmp_path / f'{i}' for i in range(100))
    model = ET.parse(write_tags(lambda suffix: str(next(paths)) + suffix))
    assert len(model.findall('.//visual')) == 8
    assert not model.findall('.//collision')


@pytest.mark.parametrize('bgr, colour', [((0, 0, 255), 'red'), ((255, 0, 0), 'blue')])
def test_finds_a_barrel(bgr, colour):
    image = blank()
    cv2.rectangle(image, (100, 200), (139, 279), bgr, cv2.FILLED)
    assert find_barrels(image) == [(colour, pytest.approx(119.5), pytest.approx(239.5), 40, 80)]


def test_ignores_yellow_walls_specks_and_slivers():
    image = blank()
    image[:100] = (0, 255, 255)
    cv2.rectangle(image, (100, 200), (105, 205), (0, 0, 255), cv2.FILLED)
    cv2.rectangle(image, (200, 200), (399, 209), (0, 0, 255), cv2.FILLED)
    assert find_barrels(image) == []


def test_red_either_side_of_the_hue_wrap_is_one_barrel():
    image = blank()
    cv2.rectangle(image, (100, 200), (119, 279), (0, 0, 255), cv2.FILLED)
    cv2.rectangle(image, (120, 200), (139, 279), (30, 0, 255), cv2.FILLED)
    assert len(find_barrels(image)) == 1
