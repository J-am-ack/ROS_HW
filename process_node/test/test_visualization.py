"""Checks that the three video views describe the command and odometry."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from process_node.config import NavigationConfig
from process_node.perception import PoseState
from process_node.visualization import (DecisionVisualizer, RiskRecorder,
                                        check_path_risk, predict_path)


def test_prediction_direction_stop_and_map_alignment():
    cfg = NavigationConfig()
    pose = PoseState(0.0, 0.0, 0.0, True)
    view = DecisionVisualizer(cv2, '/missing/map.png')
    assert view.map_point(0.0, 0.0) == (80, 90)
    assert view.map_point(40.0, -35.0) == (540, 482)
    assert all(point == (0.0, 0.0)
               for point in predict_path(pose, 0, 0, cfg))
    straight = predict_path(pose, 0, -100, cfg)
    right = predict_path(pose, 360, -100, cfg)
    assert straight[-1][0] > 2.0
    assert abs(straight[-1][1]) < 1e-8
    assert right[-1][1] < 0.0
    view.step_simulation(pose, 0, -100, 0.0, cfg)
    view.step_simulation(pose, 360, -100, 0.2, cfg)
    assert view.simulated_pose.x > 0.0
    assert view.simulated_pose.y < 0.0
    previous = view.simulated_pose.x
    view.step_simulation(pose, 0, 0, 0.4, cfg)
    assert view.simulated_pose.x == previous


def test_camera_text_wheel_and_preview_use_separate_panels():
    cfg = NavigationConfig()
    pose = PoseState(0.0, 0.0, math.pi / 2, True)
    view = DecisionVisualizer(cv2, '/missing/map.png')
    context = SimpleNamespace(completed_right_turns=3, offset_stage=1,
                              stop_reason='front 0.20 m')
    frame = np.zeros((480, 640, 3), np.uint8)
    camera = view.camera_view(frame, 120, -60, 'CORRIDOR',
                              'FOLLOW', context)
    assert camera.shape == (480, 940, 3)
    assert np.count_nonzero(camera[20:210, 640:]) > 0
    assert np.count_nonzero(camera[245:450, 640:]) > 0
    prediction = predict_path(pose, 0, -100, cfg)
    preview = view.preview_view(pose, None, prediction, 0, -100, 'FOLLOW')
    course = view.map_view(pose, prediction, 'FOLLOW')
    assert preview.shape == (560, 620, 3)
    assert course.shape == (590, 660, 3)


def test_path_risk_near_wall_and_event_recording(tmp_path):
    cfg = NavigationConfig()
    pose = PoseState(0.0, 0.0, 0.0, True)
    prediction = predict_path(pose, 0, -100, cfg)

    def scan_at(x, y):
        ranges = [math.inf] * 181
        angle = math.atan2(y, x)
        index = round(math.degrees(angle)) + 90
        ranges[index] = math.hypot(x, y)
        return SimpleNamespace(ranges=ranges, angle_min=-math.pi / 2,
                               angle_increment=math.pi / 180,
                               range_min=0.05, range_max=10.0)

    hit = check_path_risk(pose, scan_at(1.0, 0.1), prediction, -100, cfg)
    assert hit.level == 'COLLISION_RISK'
    assert hit.clearance < 0.32
    near = check_path_risk(pose, scan_at(1.0, 0.45), prediction, -100, cfg)
    assert near.level == 'NEAR'
    clear = check_path_risk(pose, scan_at(1.0, 1.0), prediction, -100, cfg)
    assert clear.level == 'CLEAR'
    assert check_path_risk(pose, scan_at(1.0, 0.1),
                           predict_path(pose, 0, 0, cfg), 0,
                           cfg).level == 'STOPPED'

    recorder = RiskRecorder(str(tmp_path / 'risks.csv'))
    near.kind = 'WALL_OR_CORNER'
    hit.kind = 'OBSTACLE'
    assert recorder.update(near, 1.0, 'RIGHT_CORNER', 'TURN_RIGHT')
    assert not recorder.update(near, 1.1, 'RIGHT_CORNER', 'TURN_RIGHT')
    assert recorder.update(hit, 1.2, 'OBSTACLE', 'FOLLOW')
    assert not recorder.update(hit, 1.3, 'OBSTACLE', 'FOLLOW')
    assert not recorder.update(clear, 1.4, 'CORRIDOR', 'FOLLOW')
    assert recorder.update(hit, 1.5, 'OBSTACLE', 'FOLLOW')
    assert recorder.count == 3
    assert len((tmp_path / 'risks.csv').read_text().splitlines()) == 4
