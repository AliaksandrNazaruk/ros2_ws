import math

import pytest

from aehub_symovo_map_mirror.symovo_map_geometry import SymovoMapGeometry


def test_height_width_meters():
    g = SymovoMapGeometry(width_px=100, height_px=200, resolution=0.05, offset_x=1.0, offset_y=2.0)
    assert pytest.approx(g.width_m) == 5.0
    assert pytest.approx(g.height_m) == 10.0


def test_yaml_origin_modes():
    g = SymovoMapGeometry(width_px=10, height_px=10, resolution=1.0, offset_x=3.0, offset_y=-4.0)
    assert g.yaml_origin(origin_mode="origin_offsets") == (3.0, -4.0, 0.0)
    assert g.yaml_origin(origin_mode="origin_neg_offsets") == (-3.0, 4.0, 0.0)


def test_pose_transform_subtract_offsets():
    g = SymovoMapGeometry(width_px=10, height_px=10, resolution=1.0, offset_x=3.0, offset_y=4.0)
    x, y, th = g.transform_pose(x=10.0, y=20.0, theta=0.5, mode="pose_subtract_offsets")
    assert x == 7.0
    assert y == 16.0
    assert th == 0.5


def test_pose_transform_flip_y_after_subtract():
    # height_m = 10
    g = SymovoMapGeometry(width_px=10, height_px=10, resolution=1.0, offset_x=0.0, offset_y=0.0)
    x, y, th = g.transform_pose(x=2.0, y=3.0, theta=-1.0, mode="pose_subtract_offsets_flip_y")
    assert x == 2.0
    assert y == 7.0
    assert th == -1.0


def test_pose_identity():
    g = SymovoMapGeometry(width_px=1, height_px=1, resolution=1.0, offset_x=99.0, offset_y=99.0)
    x, y, th = g.transform_pose(x=1.0, y=2.0, theta=math.pi, mode="pose_identity")
    assert x == 1.0
    assert y == 2.0
    assert th == math.pi

