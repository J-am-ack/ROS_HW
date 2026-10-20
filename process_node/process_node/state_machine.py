"""Navigation state and hysteresis, with no ROS dependency."""

import math
from dataclasses import dataclass
from enum import Enum, auto

from .perception import normalize_angle
from .scene_detector import SceneType


class State(Enum):
    INIT = auto()
    FOLLOW = auto()
    APPROACH_LEFT_CORNER = auto()
    APPROACH_RIGHT_CORNER = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    AVOID_LEFT = auto()
    AVOID_RIGHT = auto()
    RECOVER_CENTER = auto()
    FINISH = auto()
    EMERGENCY_STOP = auto()


class AvoidPhase(Enum):
    DEVIATE = auto()
    PASS = auto()
    RETURN = auto()


@dataclass
class FSMContext:
    state: State = State.INIT
    state_enter_time: float = 0.0
    turn_start_yaw: float = 0.0
    turn_has_odom: bool = False
    avoid_phase: AvoidPhase = AvoidPhase.DEVIATE
    phase_enter_time: float = 0.0
    valid_count: int = 0
    invalid_count: int = 0
    corner_left_count: int = 0
    corner_right_count: int = 0
    obstacle_count: int = 0
    clear_count: int = 0
    recover_count: int = 0
    unknown_since: float = -1.0
    previous_scene: SceneType = SceneType.UNKNOWN


class NavigationFSM:
    def __init__(self, config):
        self.cfg = config
        self.context = FSMContext()

    def _enter(self, state, now, pose):
        ctx = self.context
        previous = ctx.state
        ctx.state = state
        ctx.state_enter_time = now
        ctx.recover_count = 0
        ctx.clear_count = 0
        if state in (State.TURN_LEFT, State.TURN_RIGHT):
            ctx.turn_has_odom = pose.valid
            ctx.turn_start_yaw = pose.yaw
        if state in (State.AVOID_LEFT, State.AVOID_RIGHT):
            ctx.avoid_phase = AvoidPhase.DEVIATE
            ctx.phase_enter_time = now
        if state == State.FOLLOW:
            ctx.corner_left_count = 0
            ctx.corner_right_count = 0
            ctx.obstacle_count = 0
        return previous != state

    def update(self, scene, perception, pose, now):
        cfg, ctx = self.cfg, self.context
        p = perception
        ctx.previous_scene = scene
        ctx.valid_count = ctx.valid_count + 1 if p.scan_valid else 0
        ctx.invalid_count = ctx.invalid_count + 1 if not p.scan_valid else 0
        if not p.scan_valid:
            if ctx.invalid_count >= cfg.invalid_stop_frames:
                self._enter(State.EMERGENCY_STOP, now, pose)
            return ctx.state

        # Emergency checks are evaluated even during locked turn/avoid states.
        if p.front_distance < cfg.emergency_distance or p.dead_end_detected:
            self._enter(State.EMERGENCY_STOP, now, pose)
            return ctx.state
        if scene == SceneType.UNKNOWN:
            if ctx.unknown_since < 0:
                ctx.unknown_since = now
            if now - ctx.unknown_since >= cfg.unknown_stop_seconds:
                self._enter(State.EMERGENCY_STOP, now, pose)
                return ctx.state
        else:
            ctx.unknown_since = -1.0

        if ctx.state == State.FINISH:
            return ctx.state
        if ctx.state == State.EMERGENCY_STOP:
            # A latched emergency is deliberately reset only on a fresh
            # run; transient clearance cannot restart the vehicle.
            return ctx.state
        if ctx.state == State.INIT:
            if ctx.valid_count >= cfg.init_valid_frames:
                self._enter(State.FOLLOW, now, pose)
            return ctx.state

        if ctx.state in (State.TURN_LEFT, State.TURN_RIGHT):
            elapsed = now - ctx.state_enter_time
            if elapsed >= cfg.max_turn_time:
                self._enter(State.EMERGENCY_STOP, now, pose)
                return ctx.state
            direction = 1 if ctx.state == State.TURN_LEFT else -1
            yaw_done = (ctx.turn_has_odom and pose.valid and
                        direction * normalize_angle(pose.yaw - ctx.turn_start_yaw)
                        >= math.radians(cfg.turn_min_angle_deg))
            open_new_path = (p.front_distance > cfg.front_safe_distance
                             and p.corridor_detected)
            if elapsed >= cfg.min_turn_time and open_new_path and (
                    yaw_done or not ctx.turn_has_odom):
                ctx.recover_count += 1
                if ctx.recover_count >= cfg.recovery_confirm_frames:
                    self._enter(State.RECOVER_CENTER, now, pose)
            else:
                ctx.recover_count = 0
            return ctx.state

        if ctx.state in (State.AVOID_LEFT, State.AVOID_RIGHT):
            elapsed = now - ctx.state_enter_time
            if elapsed >= cfg.max_avoid_time:
                self._enter(State.EMERGENCY_STOP, now, pose)
                return ctx.state
            phase_elapsed = now - ctx.phase_enter_time
            if ctx.avoid_phase == AvoidPhase.DEVIATE:
                if phase_elapsed >= cfg.avoid_deviate_time:
                    ctx.avoid_phase = AvoidPhase.PASS
                    ctx.phase_enter_time = now
            elif ctx.avoid_phase == AvoidPhase.PASS:
                clear = (not p.obstacle_front and
                         p.front_distance > cfg.obstacle_exit_distance)
                ctx.clear_count = ctx.clear_count + 1 if clear else 0
                if (phase_elapsed >= cfg.avoid_pass_time and
                        ctx.clear_count >= cfg.obstacle_clear_frames):
                    ctx.avoid_phase = AvoidPhase.RETURN
                    ctx.phase_enter_time = now
            elif phase_elapsed >= cfg.avoid_return_time:
                self._enter(State.RECOVER_CENTER, now, pose)
            return ctx.state

        if ctx.state == State.RECOVER_CENTER:
            recovered = (p.corridor_detected and
                         p.front_distance > cfg.front_safe_distance and
                         abs(p.center_error) < cfg.center_recover_threshold)
            ctx.recover_count = ctx.recover_count + 1 if recovered else 0
            if ctx.recover_count >= cfg.recovery_confirm_frames:
                self._enter(State.FOLLOW, now, pose)
            return ctx.state

        if ctx.state in (State.APPROACH_LEFT_CORNER,
                         State.APPROACH_RIGHT_CORNER):
            left = ctx.state == State.APPROACH_LEFT_CORNER
            still_corner = (p.left_corner_detected if left
                            else p.right_corner_detected)
            if not still_corner:
                self._enter(State.RECOVER_CENTER, now, pose)
            elif (now - ctx.state_enter_time >= cfg.approach_min_time and
                  p.front_distance < cfg.turn_trigger_distance):
                self._enter(State.TURN_LEFT if left else State.TURN_RIGHT,
                            now, pose)
            return ctx.state

        # FOLLOW: only new decisions are debounced; ongoing actions above
        # keep priority until their explicit exit conditions are met.
        if scene == SceneType.UNKNOWN:
            self._enter(State.RECOVER_CENTER, now, pose)
            return ctx.state
        ctx.obstacle_count = (ctx.obstacle_count + 1 if p.obstacle_front
                              else 0)
        ctx.corner_left_count = (ctx.corner_left_count + 1
                                 if p.left_corner_detected else 0)
        ctx.corner_right_count = (ctx.corner_right_count + 1
                                  if p.right_corner_detected else 0)
        if ctx.obstacle_count >= cfg.obstacle_confirm_frames:
            go_left = p.front_left_distance >= p.front_right_distance
            self._enter(State.AVOID_LEFT if go_left else State.AVOID_RIGHT,
                        now, pose)
        elif ctx.corner_left_count >= cfg.corner_confirm_frames:
            self._enter(State.APPROACH_LEFT_CORNER, now, pose)
        elif ctx.corner_right_count >= cfg.corner_confirm_frames:
            self._enter(State.APPROACH_RIGHT_CORNER, now, pose)
        return ctx.state
