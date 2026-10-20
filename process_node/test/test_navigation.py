"""Synthetic-scan tests for the ROS-independent navigation layers."""

import math
from types import SimpleNamespace

from process_node.behaviors import BehaviorManager
from process_node.config import NavigationConfig
from process_node.controller import SteeringController
from process_node.perception import (PoseState, ScanPerception,
                                     angle_to_index, index_to_angle,
                                     normalize_angle)
from process_node.scene_detector import SceneDetector, SceneType
from process_node.state_machine import NavigationFSM, State


def scan_for(sectors=None, default=1.2):
    """Simulate a 270 degree scan with one reading per degree."""
    values = [default] * 271
    for start, stop, distance in sectors or []:
        for angle in range(start, stop + 1):
            values[angle + 135] = distance
    return SimpleNamespace(ranges=values, angle_min=math.radians(-135),
                           angle_increment=math.radians(1),
                           range_min=0.05, range_max=10.0)


def make_stack():
    cfg = NavigationConfig()
    return cfg, ScanPerception(cfg), SceneDetector(cfg), NavigationFSM(cfg)


def test_angles_and_invalid_scan():
    cfg, perception, _, _ = make_stack()
    scan = scan_for()
    assert angle_to_index(0, scan.angle_min, scan.angle_increment) == 135
    assert abs(index_to_angle(135, scan.angle_min,
                              scan.angle_increment)) < 1e-10
    assert abs(normalize_angle(-math.pi - 0.1) - (math.pi - 0.1)) < 1e-10
    scan.ranges[135] = math.nan
    scan.ranges[136] = math.inf
    assert perception.update(scan).scan_valid
    scan.ranges = [math.nan] * len(scan.ranges)
    assert not perception.update(scan).scan_valid


def test_scene_detection():
    _, perception, detector, _ = make_stack()
    assert detector.detect(perception.update(scan_for())) == SceneType.CORRIDOR
    left = scan_for([(-12, 12, 0.75), (15, 115, 2.5)])
    assert detector.detect(perception.update(left)) == SceneType.LEFT_CORNER
    right = scan_for([(-12, 12, 0.75), (-115, -15, 2.5)])
    assert detector.detect(perception.update(right)) == SceneType.RIGHT_CORNER
    obstacle = scan_for([(-8, 8, 0.75), (-40, -12, 1.7),
                         (12, 40, 1.7)])
    assert detector.detect(perception.update(obstacle)) == SceneType.OBSTACLE


def test_turn_confirm_and_recovery():
    cfg, perception, detector, fsm = make_stack()
    pose = PoseState(valid=True)
    normal = perception.update(scan_for(default=2.0))
    corner = perception.update(scan_for([(-12, 12, 0.75),
                                         (15, 115, 2.5)]))
    for tick in range(cfg.init_valid_frames):
        fsm.update(detector.detect(normal), normal, pose, tick * 0.1)
    assert fsm.context.state == State.FOLLOW
    for tick in range(cfg.corner_confirm_frames):
        fsm.update(SceneType.LEFT_CORNER, corner, pose, 1.0 + tick * 0.1)
    assert fsm.context.state == State.APPROACH_LEFT_CORNER
    fsm.update(SceneType.LEFT_CORNER, corner, pose, 1.6)
    assert fsm.context.state == State.TURN_LEFT
    pose.yaw = math.radians(65)
    for tick in range(cfg.recovery_confirm_frames):
        fsm.update(SceneType.CORRIDOR, normal, pose, 2.2 + tick * 0.1)
    assert fsm.context.state == State.RECOVER_CENTER


def test_avoid_and_emergency_controller():
    cfg, perception, detector, fsm = make_stack()
    normal = perception.update(scan_for(default=2.0))
    obstacle = perception.update(scan_for([(-8, 8, 0.75),
                                           (-40, -12, 1.7),
                                           (12, 40, 1.7)]))
    pose = PoseState()
    for tick in range(cfg.init_valid_frames):
        fsm.update(SceneType.CORRIDOR, normal, pose, tick * 0.1)
    for tick in range(cfg.obstacle_confirm_frames):
        fsm.update(SceneType.OBSTACLE, obstacle, pose, 1.0 + tick * 0.1)
    assert fsm.context.state in (State.AVOID_LEFT, State.AVOID_RIGHT)
    controller = SteeringController(cfg)
    target = BehaviorManager(cfg).run(fsm.context.state, obstacle,
                                      pose, fsm.context)
    first = controller.update(target, fsm.context.state, obstacle, 1.5)
    assert abs(first.steer) <= cfg.max_steer_step
    assert abs(first.speed) <= cfg.speed_avoid
    near = perception.update(scan_for([(-12, 12, 0.25)]))
    fsm.update(detector.detect(near), near, pose, 1.6)
    assert fsm.context.state == State.EMERGENCY_STOP
    stopped = controller.update(target, fsm.context.state, near, 1.6)
    assert stopped.steer == stopped.speed == 0
