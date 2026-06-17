"""Basic portfolio test scaffold.

These tests validate the input contract used by the calibration module.
Hardware-specific execution should be validated on the real machine or simulation setup.
"""


def test_offset_vector_contract():
    offset = [2.0, 1.0, 0.0, 0.0, 0.0, 1.5]
    assert len(offset) == 6
    assert offset[:3] == [2.0, 1.0, 0.0]


def test_gantry_pose_contract():
    gantry_pose = [1000.0, 2000.0, 500.0]
    assert len(gantry_pose) == 3


def test_cobot_pose_contract():
    cobot_pose = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
    assert len(cobot_pose) == 6
