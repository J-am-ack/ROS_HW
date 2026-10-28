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
    obstacle_near_opening = scan_for([(-8, 8, 0.75), (55, 115, 2.5)])
    assert detector.detect(perception.update(obstacle_near_opening)) == (
        SceneType.OBSTACLE)


def test_turn_confirm_and_recovery():
    cfg, perception, detector, fsm = make_stack()
    cfg.route_right_turns = 0
    cfg.allow_left_turns = True
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
    pose = PoseState(valid=True)
    for tick in range(cfg.init_valid_frames):
        fsm.update(SceneType.CORRIDOR, normal, pose, tick * 0.1)
    # Obstacle avoidance is armed only after right turn 3 and the route offset.
    fsm.context.completed_right_turns = 2
    fsm.context.offset_stage = 2
    for tick in range(cfg.obstacle_confirm_frames):
        fsm.update(SceneType.OBSTACLE, obstacle, pose, 0.5 + tick * 0.1)
    assert fsm.context.state == State.FOLLOW
    fsm.context.completed_right_turns = cfg.route_right_turns
    fsm.context.offset_stage = 2
    for tick in range(cfg.obstacle_confirm_frames):
        fsm.update(SceneType.OBSTACLE, obstacle, pose, 1.0 + tick * 0.1)
    assert fsm.context.state in (State.AVOID_LEFT, State.AVOID_RIGHT)
    initial_side = fsm.context.state
    fsm.context.avoid_phase = fsm.context.avoid_phase.PASS
    assert BehaviorManager(cfg).run(initial_side, obstacle, pose,
                                    fsm.context).steer_override == 0
    fsm.context.avoid_phase = fsm.context.avoid_phase.TRANSITION
    returning = BehaviorManager(cfg).run(initial_side, obstacle, pose,
                                         fsm.context).steer_override
    assert returning == (-cfg.avoid_steer if initial_side == State.AVOID_LEFT
                         else cfg.avoid_steer)
    fsm.context.avoid_phase = fsm.context.avoid_phase.DEVIATE
    controller = SteeringController(cfg)
    target = BehaviorManager(cfg).run(fsm.context.state, obstacle,
                                      pose, fsm.context)
    first = controller.update(target, fsm.context.state, obstacle, 1.5)
    assert abs(first.steer) <= cfg.max_steer_step
    assert abs(first.speed) <= cfg.speed_avoid
    near = perception.update(scan_for([(-12, 12, 0.25)]))
    fsm.update(detector.detect(near), near, pose, 1.6)
    assert fsm.context.state != State.EMERGENCY_STOP
    fsm.update(detector.detect(near), near, pose, 1.7)
    assert fsm.context.state == State.EMERGENCY_STOP
    stopped = controller.update(target, fsm.context.state, near, 1.6)
    assert stopped.steer == stopped.speed == 0
    for tick in range(cfg.emergency_release_frames):
        fsm.update(SceneType.CORRIDOR, normal, pose, 2.0 + tick * 0.1)
    assert fsm.context.state == State.RECOVER_CENTER


def test_route_right_approach_survives_one_noisy_frame():
    cfg, perception, detector, fsm = make_stack()
    cfg.route_first_turn_min_distance = 0.0
    pose = PoseState(valid=True)
    normal = perception.update(scan_for(default=2.0))
    right = perception.update(scan_for([(-12, 12, 1.8),
                                        (-115, -15, 2.5)]))
    close = perception.update(scan_for([(-12, 12, 1.2),
                                        (-115, -15, 2.5)]))
    for tick in range(cfg.init_valid_frames):
        fsm.update(detector.detect(normal), normal, pose, tick * 0.1)
    for tick in range(cfg.corner_confirm_frames):
        fsm.update(detector.detect(right), right, pose, 1.0 + tick * 0.1)
    assert fsm.context.state == State.APPROACH_RIGHT_CORNER
    fsm.update(SceneType.CORRIDOR, normal, pose, 1.3)
    assert fsm.context.state == State.APPROACH_RIGHT_CORNER
    fsm.update(detector.detect(close), close, pose, 1.6)
    assert fsm.context.state == State.TURN_RIGHT


def test_two_right_turns_then_offset_then_third_right():
    cfg, perception, detector, fsm = make_stack()
    cfg.route_first_turn_min_distance = 0.0
    cfg.route_next_turn_min_distance = 0.0
    cfg.route_final_turn_min_distance = 0.0
    cfg.route_offset_min_distance = 0.0
    pose = PoseState(valid=True)
    normal = perception.update(scan_for(default=2.0))
    right = perception.update(scan_for([(-12, 12, 1.2),
                                        (-115, -15, 2.5)]))
    now = 0.0
    for _ in range(cfg.init_valid_frames):
        fsm.update(SceneType.CORRIDOR, normal, pose, now)
        now += 0.1
    assert fsm.context.state == State.FOLLOW

    expected = [(State.TURN_RIGHT, 'MAJOR_RIGHT', 1, 0),
                (State.TURN_RIGHT, 'MAJOR_RIGHT', 2, 0),
                (State.TURN_LEFT, 'OFFSET_LEFT', 2, 1),
                (State.TURN_RIGHT, 'OFFSET_RIGHT', 2, 2),
                (State.TURN_RIGHT, 'MAJOR_RIGHT', 3, 2)]
    for turn, role, right_count, offset_stage in expected:
        trigger = right if role == 'MAJOR_RIGHT' else normal
        scene = detector.detect(trigger)
        for _ in range(cfg.corner_confirm_frames):
            fsm.update(scene, trigger, pose, now)
            now += 0.1
        assert fsm.context.state in (State.APPROACH_RIGHT_CORNER,
                                     State.APPROACH_LEFT_CORNER)
        now += cfg.approach_min_time + 0.1
        fsm.update(scene, trigger, pose, now)
        assert fsm.context.state == turn
        assert fsm.context.active_turn_role == role
        now += cfg.min_turn_time + 0.1
        angle = (cfg.offset_turn_min_angle_deg if role.startswith('OFFSET')
                 else cfg.turn_min_angle_deg) + 10.0
        pose.yaw += math.radians(angle if turn == State.TURN_LEFT else -angle)
        for _ in range(cfg.recovery_confirm_frames):
            fsm.update(SceneType.CORRIDOR, normal, pose, now)
            now += 0.1
        assert fsm.context.state == State.RECOVER_CENTER
        assert fsm.context.completed_right_turns == right_count
        assert fsm.context.offset_stage == offset_stage
        for _ in range(cfg.recovery_confirm_frames):
            fsm.update(SceneType.CORRIDOR, normal, pose, now)
            now += 0.1
        assert fsm.context.state == State.FOLLOW


def test_offset_precedes_final_right_and_both_wait_for_distance():
    cfg, perception, _, fsm = make_stack()
    normal = perception.update(scan_for(default=2.0))
    pose = PoseState(x=20.0, valid=True)
    ctx = fsm.context
    ctx.state = State.FOLLOW
    ctx.completed_right_turns = 2
    ctx.route_anchor_valid = True
    # A candidate right corner before the offset must not become turn 3.
    right = perception.update(scan_for([(-12, 12, 1.2),
                                        (-115, -15, 2.5)]))
    for tick in range(cfg.corner_confirm_frames + 1):
        fsm.update(SceneType.RIGHT_CORNER, right, pose, tick * 0.1)
    assert ctx.state == State.FOLLOW
    pose.x = cfg.route_offset_min_distance - 0.5
    for tick in range(cfg.corner_confirm_frames):
        fsm.update(SceneType.CORRIDOR, normal, pose, 1.0 + tick * 0.1)
    assert ctx.state == State.FOLLOW

    pose.x += 1.0
    for tick in range(cfg.corner_confirm_frames):
        fsm.update(SceneType.CORRIDOR, normal, pose, 2.0 + tick * 0.1)
    assert ctx.state == State.APPROACH_LEFT_CORNER

    ctx.state = State.FOLLOW
    ctx.offset_stage = 2
    ctx.route_anchor_x = pose.x
    ctx.corner_left_count = 0
    pose.x += cfg.route_final_turn_min_distance - 0.5
    for tick in range(cfg.corner_confirm_frames + 1):
        fsm.update(SceneType.CORRIDOR, normal, pose, 3.0 + tick * 0.1)
    assert ctx.state == State.FOLLOW
    pose.x += 1.0
    for tick in range(cfg.corner_confirm_frames):
        fsm.update(SceneType.CORRIDOR, normal, pose, 4.0 + tick * 0.1)
    assert ctx.state == State.APPROACH_RIGHT_CORNER


def test_last_right_turn_after_s_bend_finishes_route():
    cfg, perception, detector, fsm = make_stack()
    cfg.route_finish_right_min_distance = 2.0
    normal = perception.update(scan_for(default=2.0))
    right = perception.update(scan_for([(-12, 12, 1.2),
                                        (-115, -15, 2.5)]))
    pose = PoseState(x=0.0, y=0.0, yaw=0.0, valid=True)
    ctx = fsm.context
    ctx.state = State.FOLLOW
    ctx.completed_right_turns = cfg.route_right_turns
    ctx.offset_stage = 2
    ctx.route_anchor_valid = True
    pose.x = cfg.route_finish_right_min_distance - 0.5
    for tick in range(cfg.corner_confirm_frames + 1):
        fsm.update(SceneType.CORRIDOR, normal, pose, tick * 0.1)
    assert ctx.state == State.FOLLOW
    pose.x += 1.0
    for tick in range(cfg.corner_confirm_frames):
        fsm.update(SceneType.RIGHT_CORNER, right, pose, 1.0 + tick * 0.1)
    assert ctx.state == State.APPROACH_RIGHT_CORNER
    now = 1.0 + cfg.corner_confirm_frames * 0.1 + cfg.approach_min_time + 0.1
    fsm.update(SceneType.CORRIDOR, normal, pose, now)
    assert ctx.state == State.TURN_RIGHT
    assert ctx.active_turn_role == 'FINAL_RIGHT'

    pose.yaw = math.radians(-cfg.turn_min_angle_deg - 5)
    for tick in range(cfg.recovery_confirm_frames):
        fsm.update(SceneType.CORRIDOR, normal, pose,
                   now + cfg.min_turn_time + 0.1 + tick * 0.1)
    assert ctx.state == State.RECOVER_CENTER
    for tick in range(cfg.recovery_confirm_frames):
        fsm.update(SceneType.CORRIDOR, normal, pose,
                   now + cfg.min_turn_time + 0.6 + tick * 0.1)
    assert ctx.state == State.FINISH
    assert ctx.final_right_completed


def test_arbitrary_obstacles_use_alternating_s_curve():
    cfg, perception, detector, fsm = make_stack()
    normal = perception.update(scan_for(default=2.0))
    obstacle = perception.update(scan_for([(-8, 8, 0.75),
                                           (-40, -12, 1.7),
                                           (12, 40, 1.7)]))
    pose = PoseState(valid=True)
    ctx = fsm.context
    ctx.state = State.FOLLOW
    ctx.completed_right_turns = cfg.route_right_turns
    ctx.offset_stage = 2
    now = 0.0

    expected_sides = (State.AVOID_LEFT, State.AVOID_RIGHT,
                      State.AVOID_LEFT, State.AVOID_RIGHT)
    assert BehaviorManager(cfg).run(State.FOLLOW, normal, pose,
                                    ctx).preferred_speed == cfg.speed_s_curve
    for index, expected_side in enumerate(expected_sides):
        for _ in range(cfg.obstacle_confirm_frames):
            fsm.update(SceneType.OBSTACLE, obstacle, pose, now)
            now += 0.1
        assert ctx.state == expected_side
        assert ctx.avoid_count == index + 1
        assert ctx.completed_avoids == index

        # Deviation ends, then clear scans confirm the cone is behind us.
        fsm.update(SceneType.CORRIDOR, normal, pose,
                   now + cfg.avoid_deviate_time + 0.1)
        now += cfg.avoid_deviate_time + 0.2
        deadline = now + cfg.avoid_pass_time + 1.0
        while ctx.avoid_phase.name != 'TRANSITION' and now < deadline:
            fsm.update(SceneType.CORRIDOR, normal, pose, now)
            now += 0.1
        assert ctx.avoid_phase.name == 'TRANSITION'
        assert ctx.completed_avoids == index + 1
        transition_target = BehaviorManager(cfg).run(
            ctx.state, normal, pose, ctx)
        previous_direction = (1 if expected_side == State.AVOID_LEFT else -1)
        assert transition_target.steer_override == (
            -previous_direction * cfg.avoid_steer)
        if index < len(expected_sides) - 1:
            # A brief clear gap must keep the diagonal command active.
            fsm.update(SceneType.CORRIDOR, normal, pose, now + 0.8)
            assert ctx.state == expected_side
            assert ctx.avoid_phase.name == 'TRANSITION'
            for _ in range(cfg.obstacle_confirm_frames):
                fsm.update(SceneType.OBSTACLE, obstacle, pose, now)
                now += 0.1
            assert ctx.state == expected_sides[index + 1]
        else:
            fsm.update(SceneType.CORRIDOR, normal, pose,
                       now + cfg.avoid_transition_time + 0.1)
            assert ctx.state == State.FOLLOW

    assert BehaviorManager(cfg).run(State.FOLLOW, normal, pose,
                                    ctx).preferred_speed == cfg.speed_s_curve


def test_final_right_uses_corner_and_distance_without_obstacle_count():
    cfg, perception, detector, fsm = make_stack()
    cfg.route_finish_right_min_distance = 0.0
    normal = perception.update(scan_for(default=2.0))
    right = perception.update(scan_for([(-12, 12, 1.2),
                                        (-115, -15, 2.5)]))
    pose = PoseState(valid=True)
    ctx = fsm.context
    ctx.state = State.FOLLOW
    ctx.completed_right_turns = cfg.route_right_turns
    ctx.offset_stage = 2
    ctx.route_anchor_valid = False

    for completed in (0, 1, 4):
        ctx.completed_avoids = completed
        ctx.state = State.FOLLOW
        ctx.corner_right_count = 0
        for tick in range(cfg.corner_confirm_frames + 1):
            fsm.update(SceneType.CORRIDOR, normal, pose,
                       completed * 2.0 + tick * 0.1)
        assert ctx.state == State.FOLLOW
        for tick in range(cfg.corner_confirm_frames):
            fsm.update(SceneType.RIGHT_CORNER, right, pose,
                       completed * 2.0 + 0.6 + tick * 0.1)
        assert ctx.state == State.APPROACH_RIGHT_CORNER


def test_final_right_yields_to_an_obstacle_in_approach():
    cfg, perception, detector, fsm = make_stack()
    right = perception.update(scan_for([(-12, 12, 1.2),
                                        (-115, -15, 2.5)]))
    obstacle = perception.update(scan_for([(-8, 8, 0.75),
                                           (-40, -12, 1.7),
                                           (12, 40, 1.7)]))
    pose = PoseState(valid=True)
    ctx = fsm.context
    ctx.state = State.FOLLOW
    ctx.completed_right_turns = cfg.route_right_turns
    ctx.offset_stage = 2
    ctx.route_anchor_valid = False
    for tick in range(cfg.corner_confirm_frames):
        fsm.update(detector.detect(right), right, pose, tick * 0.1)
    assert ctx.state == State.APPROACH_RIGHT_CORNER
    fsm.update(detector.detect(obstacle), obstacle, pose, 1.0)
    assert ctx.state == State.FOLLOW
    for tick in range(cfg.obstacle_confirm_frames):
        fsm.update(detector.detect(obstacle), obstacle, pose, 1.1 + tick * 0.1)
    assert ctx.state in (State.AVOID_LEFT, State.AVOID_RIGHT)
    for tick in range(cfg.corner_confirm_frames + 1):
        fsm.update(detector.detect(right), right, pose, 1.5 + tick * 0.1)
    assert ctx.state in (State.AVOID_LEFT, State.AVOID_RIGHT)
