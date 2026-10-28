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
    TRANSITION = auto()


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
    completed_right_turns: int = 0
    offset_stage: int = 0  # 0: before left, 1: before right, 2: complete
    final_right_completed: bool = False
    active_turn_role: str = ''
    avoid_count: int = 0
    completed_avoids: int = 0
    pending_avoid_completion: bool = False
    obstacle_ready: bool = True
    approach_lost_count: int = 0
    emergency_count: int = 0
    stop_reason: str = ''
    interrupted_state: State = State.INIT
    emergency_release_count: int = 0
    route_anchor_x: float = 0.0
    route_anchor_y: float = 0.0
    route_anchor_valid: bool = False


class NavigationFSM:
    def __init__(self, config):
        self.cfg = config
        self.context = FSMContext()

    def _finish_turn(self, pose):
        ctx = self.context
        if ctx.active_turn_role == 'MAJOR_RIGHT':
            ctx.completed_right_turns = min(
                self.cfg.route_right_turns, ctx.completed_right_turns + 1)
        elif ctx.active_turn_role == 'OFFSET_LEFT':
            ctx.offset_stage = 1
        elif ctx.active_turn_role == 'OFFSET_RIGHT':
            ctx.offset_stage = 2
        elif ctx.active_turn_role == 'FINAL_RIGHT':
            ctx.final_right_completed = True
        if pose.valid:
            ctx.route_anchor_x = pose.x
            ctx.route_anchor_y = pose.y
            ctx.route_anchor_valid = True
        ctx.active_turn_role = ''

    def _enter(self, state, now, pose, reason=''):
        ctx = self.context
        previous = ctx.state
        if state == State.EMERGENCY_STOP and previous != State.EMERGENCY_STOP:
            ctx.interrupted_state = previous
            ctx.stop_reason = reason
            ctx.emergency_release_count = 0
        if state == State.RECOVER_CENTER and previous in (
                State.TURN_LEFT, State.TURN_RIGHT):
            self._finish_turn(pose)
        if state == State.RECOVER_CENTER:
            ctx.pending_avoid_completion = (
                previous in (State.AVOID_LEFT, State.AVOID_RIGHT) and
                ctx.avoid_phase == AvoidPhase.TRANSITION)
        ctx.state = state
        ctx.state_enter_time = now
        ctx.recover_count = 0
        ctx.clear_count = 0
        if state in (State.TURN_LEFT, State.TURN_RIGHT):
            ctx.turn_has_odom = pose.valid
            ctx.turn_start_yaw = pose.yaw
            if (self.cfg.route_offset_enabled and
                    ctx.completed_right_turns == self.cfg.route_right_turns - 1 and
                    state == State.TURN_LEFT and ctx.offset_stage == 0):
                ctx.active_turn_role = 'OFFSET_LEFT'
            elif (self.cfg.route_offset_enabled and
                  ctx.completed_right_turns == self.cfg.route_right_turns - 1 and
                  state == State.TURN_RIGHT and ctx.offset_stage == 1):
                ctx.active_turn_role = 'OFFSET_RIGHT'
            elif state == State.TURN_RIGHT:
                if (ctx.completed_right_turns >= self.cfg.route_right_turns and
                        ctx.offset_stage == 2 and
                        not ctx.final_right_completed):
                    ctx.active_turn_role = 'FINAL_RIGHT'
                else:
                    ctx.active_turn_role = 'MAJOR_RIGHT'
            else:
                ctx.active_turn_role = 'FREE_LEFT'
        if state in (State.AVOID_LEFT, State.AVOID_RIGHT):
            ctx.avoid_phase = AvoidPhase.DEVIATE
            ctx.phase_enter_time = now
            ctx.avoid_count += 1
            ctx.obstacle_ready = False
            ctx.obstacle_count = 0
        if state in (State.APPROACH_LEFT_CORNER,
                     State.APPROACH_RIGHT_CORNER):
            ctx.approach_lost_count = 0
        if state == State.FOLLOW:
            if previous == State.INIT and pose.valid:
                ctx.route_anchor_x = pose.x
                ctx.route_anchor_y = pose.y
                ctx.route_anchor_valid = True
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
                self._enter(State.EMERGENCY_STOP, now, pose,
                            'invalid scan')
            return ctx.state

        # Emergency checks are evaluated even during locked turn/avoid states.
        # A dead-end classification is not by itself an emergency: at a
        # corner the narrow front sector can look closed for several frames.
        # Only the explicit near-collision distance forces an immediate stop.
        ctx.emergency_count = (ctx.emergency_count + 1
                               if p.front_distance < cfg.emergency_distance
                               else 0)
        if ctx.emergency_count >= cfg.emergency_confirm_frames:
            self._enter(State.EMERGENCY_STOP, now, pose,
                        'front %.2f m' % p.front_distance)
            return ctx.state
        if scene == SceneType.UNKNOWN:
            if ctx.unknown_since < 0:
                ctx.unknown_since = now
            if now - ctx.unknown_since >= cfg.unknown_stop_seconds:
                self._enter(State.EMERGENCY_STOP, now, pose,
                            'unknown scene timeout')
                return ctx.state
        else:
            ctx.unknown_since = -1.0

        if ctx.state == State.FINISH:
            return ctx.state
        if ctx.state == State.EMERGENCY_STOP:
            safe = (p.front_distance > cfg.front_safe_distance and
                    scene != SceneType.UNKNOWN)
            ctx.emergency_release_count = (
                ctx.emergency_release_count + 1 if safe else 0)
            if ctx.emergency_release_count >= cfg.emergency_release_frames:
                # A recorded turn may continue in the bag while this node is
                # stopped. Count it only if odometry confirms a real yaw.
                if (ctx.interrupted_state in (State.TURN_LEFT,
                                              State.TURN_RIGHT) and
                        ctx.turn_has_odom and pose.valid):
                    direction = (1 if ctx.interrupted_state == State.TURN_LEFT
                                 else -1)
                    angle = (cfg.offset_turn_min_angle_deg
                             if ctx.active_turn_role.startswith('OFFSET')
                             else cfg.turn_min_angle_deg)
                    if (direction * normalize_angle(
                            pose.yaw - ctx.turn_start_yaw) >=
                            math.radians(angle)):
                        self._finish_turn(pose)
                self._enter(State.RECOVER_CENTER, now, pose)
            return ctx.state
        if ctx.state == State.INIT:
            if ctx.valid_count >= cfg.init_valid_frames:
                self._enter(State.FOLLOW, now, pose)
            return ctx.state

        if ctx.state in (State.TURN_LEFT, State.TURN_RIGHT):
            elapsed = now - ctx.state_enter_time
            if elapsed >= cfg.max_turn_time:
                self._enter(State.EMERGENCY_STOP, now, pose,
                            'turn timeout')
                return ctx.state
            direction = 1 if ctx.state == State.TURN_LEFT else -1
            required_angle = (cfg.offset_turn_min_angle_deg
                              if ctx.active_turn_role.startswith('OFFSET')
                              else cfg.turn_min_angle_deg)
            yaw_done = (ctx.turn_has_odom and pose.valid and
                        direction * normalize_angle(pose.yaw - ctx.turn_start_yaw)
                        >= math.radians(required_angle))
            if ctx.active_turn_role.startswith('OFFSET'):
                open_new_path = p.front_distance > cfg.emergency_distance
            else:
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
                self._enter(State.EMERGENCY_STOP, now, pose,
                            'avoid timeout')
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
                    # The first cone is passed. Start moving diagonally toward
                    # the opposite side immediately; do not wait to recenter.
                    ctx.completed_avoids += 1
                    ctx.obstacle_ready = True
                    ctx.avoid_phase = AvoidPhase.TRANSITION
                    ctx.phase_enter_time = now
            elif ctx.avoid_phase == AvoidPhase.TRANSITION:
                ctx.obstacle_count = (ctx.obstacle_count + 1
                                      if p.obstacle_front else 0)
                if ctx.obstacle_count >= cfg.obstacle_confirm_frames:
                    # Enter the next pass directly from the diagonal motion.
                    # Side alternation follows the number already passed.
                    go_left = ((ctx.completed_avoids % 2 == 0) ==
                               cfg.s_curve_first_left)
                    self._enter(State.AVOID_LEFT if go_left
                                else State.AVOID_RIGHT, now, pose)
                elif phase_elapsed >= cfg.avoid_transition_time:
                    # If the next cone is farther away, resume lane following
                    # without forcing a center-recovery maneuver.
                    self._enter(State.FOLLOW, now, pose)
            return ctx.state

        if ctx.state == State.RECOVER_CENTER:
            recovered = (p.corridor_detected and
                         p.front_distance > cfg.front_safe_distance and
                         abs(p.center_error) < cfg.center_recover_threshold)
            ctx.recover_count = ctx.recover_count + 1 if recovered else 0
            if ctx.recover_count >= cfg.recovery_confirm_frames:
                if ctx.pending_avoid_completion:
                    ctx.pending_avoid_completion = False
                if ctx.interrupted_state in (State.AVOID_LEFT,
                                             State.AVOID_RIGHT):
                    # A stopped attempt does not count as passed.
                    ctx.obstacle_ready = True
                if ctx.final_right_completed:
                    self._enter(State.FINISH, now, pose)
                else:
                    self._enter(State.FOLLOW, now, pose)
            return ctx.state

        if ctx.state in (State.APPROACH_LEFT_CORNER,
                         State.APPROACH_RIGHT_CORNER):
            left = ctx.state == State.APPROACH_LEFT_CORNER
            offset_approach = (
                cfg.route_offset_enabled and
                ctx.completed_right_turns == cfg.route_right_turns - 1 and
                ((left and ctx.offset_stage == 0) or
                 (not left and ctx.offset_stage == 1)))
            route_third_right = (
                not left and ctx.completed_right_turns == 2 and
                (not cfg.route_offset_enabled or ctx.offset_stage == 2))
            route_finish_right = (
                not left and
                ctx.completed_right_turns >= cfg.route_right_turns and
                (not cfg.route_offset_enabled or ctx.offset_stage == 2) and
                not ctx.final_right_completed)
            if route_finish_right and p.obstacle_front:
                self._enter(State.FOLLOW, now, pose)
                return ctx.state
            opening = p.left_opening if left else p.right_opening
            still_corner = opening > cfg.opening_distance
            ctx.approach_lost_count = (0 if still_corner else
                                       ctx.approach_lost_count + 1)
            elapsed = now - ctx.state_enter_time
            if (elapsed >= cfg.approach_min_time and
                    (offset_approach or route_third_right or
                     route_finish_right or
                     p.front_distance < cfg.turn_trigger_distance) and
                    (offset_approach or route_third_right or
                     route_finish_right or still_corner or
                     ctx.approach_lost_count < cfg.approach_lost_frames)):
                self._enter(State.TURN_LEFT if left else State.TURN_RIGHT,
                            now, pose)
            elif (not (offset_approach or route_third_right or
                       route_finish_right) and
                  (elapsed >= cfg.approach_max_time or
                  (elapsed >= cfg.approach_min_time and
                   ctx.approach_lost_count >= cfg.approach_lost_frames))):
                self._enter(State.RECOVER_CENTER, now, pose)
            return ctx.state

        # FOLLOW: only new decisions are debounced; ongoing actions above
        # keep priority until their explicit exit conditions are met.
        if scene == SceneType.UNKNOWN:
            self._enter(State.RECOVER_CENTER, now, pose)
            return ctx.state
        obstacle_mode = (cfg.s_curve_after_route and
                         ctx.completed_right_turns >= cfg.route_right_turns and
                         (not cfg.route_offset_enabled or ctx.offset_stage == 2))
        ctx.obstacle_count = (ctx.obstacle_count + 1
                              if obstacle_mode and ctx.obstacle_ready and
                              p.obstacle_front else 0)
        distance = (math.hypot(pose.x - ctx.route_anchor_x,
                               pose.y - ctx.route_anchor_y)
                    if ctx.route_anchor_valid and pose.valid else math.nan)
        # With odometry, use route distance. Without it, accept only an
        # observed corner; never treat an unknown distance as infinity.
        def at_corner(threshold, detected):
            return (distance >= threshold if math.isfinite(distance)
                    else detected)

        if ctx.completed_right_turns == 0:
            desired = ('RIGHT' if at_corner(
                cfg.route_first_turn_min_distance,
                p.right_corner_detected) else '')
        elif ctx.completed_right_turns == 1:
            desired = ('RIGHT' if at_corner(
                cfg.route_next_turn_min_distance,
                p.right_corner_detected) else '')
        elif ctx.completed_right_turns == 2:
            if cfg.route_offset_enabled and ctx.offset_stage == 0:
                desired = ('LEFT' if at_corner(
                    cfg.route_offset_min_distance,
                    p.left_corner_detected) else '')
            elif cfg.route_offset_enabled and ctx.offset_stage == 1:
                desired = 'RIGHT'
            else:
                desired = ('RIGHT' if at_corner(
                    cfg.route_final_turn_min_distance,
                    p.right_corner_detected) else '')
        elif ctx.completed_right_turns < cfg.route_right_turns:
            desired = 'RIGHT'
        elif (ctx.completed_right_turns >= cfg.route_right_turns and
              (not cfg.route_offset_enabled or ctx.offset_stage == 2) and
              not ctx.final_right_completed):
            desired = ('RIGHT' if p.right_corner_detected and at_corner(
                cfg.route_finish_right_min_distance,
                p.right_corner_detected) else '')
        else:
            desired = ''
        offset_right = (cfg.route_offset_enabled and
                        ctx.completed_right_turns == cfg.route_right_turns - 1 and
                        ctx.offset_stage == 1)
        offset_left = (cfg.route_offset_enabled and
                       ctx.completed_right_turns == cfg.route_right_turns - 1 and
                       ctx.offset_stage == 0)
        final_right = (ctx.completed_right_turns == 2 and
                       (not cfg.route_offset_enabled or ctx.offset_stage == 2) and
                       distance >= cfg.route_final_turn_min_distance)
        finish_right = (ctx.completed_right_turns >= cfg.route_right_turns and
                        (not cfg.route_offset_enabled or
                         ctx.offset_stage == 2) and
                        not ctx.final_right_completed and
                        p.right_corner_detected and
                        distance >= cfg.route_finish_right_min_distance)
        right_candidate = (desired == 'RIGHT' and not p.obstacle_front and
            (offset_right or
            final_right or
            finish_right or
            p.right_corner_detected or (
                p.front_distance < cfg.corner_trigger_distance and
                p.right_opening > cfg.opening_distance)))
        left_candidate = (desired == 'LEFT' and not p.obstacle_front and
            (offset_left or
            p.left_corner_detected or (
                p.front_distance < cfg.corner_trigger_distance and
                p.left_opening > cfg.opening_distance)))
        ctx.corner_left_count = (ctx.corner_left_count + 1
                                 if (left_candidate or
                                     cfg.allow_left_turns and
                                     p.left_corner_detected and
                                     not desired) else 0)
        ctx.corner_right_count = (ctx.corner_right_count + 1
                                  if right_candidate else 0)
        if ctx.obstacle_count >= cfg.obstacle_confirm_frames:
            # Alternate the side after each passed obstacle, for any number
            # of obstacles encountered on the last straight.
            go_left = ((ctx.completed_avoids % 2 == 0) ==
                       cfg.s_curve_first_left)
            chosen_clearance = (p.front_left_distance if go_left
                                else p.front_right_distance)
            other_clearance = (p.front_right_distance if go_left
                               else p.front_left_distance)
            if chosen_clearance < cfg.passage_clearance:
                if other_clearance >= cfg.passage_clearance:
                    go_left = not go_left
                else:
                    self._enter(State.EMERGENCY_STOP, now, pose,
                                'no obstacle passage')
                    return ctx.state
            self._enter(State.AVOID_LEFT if go_left else State.AVOID_RIGHT,
                        now, pose)
        elif ctx.corner_right_count >= cfg.corner_confirm_frames:
            self._enter(State.APPROACH_RIGHT_CORNER, now, pose)
        elif ctx.corner_left_count >= cfg.corner_confirm_frames:
            self._enter(State.APPROACH_LEFT_CORNER, now, pose)
        return ctx.state
