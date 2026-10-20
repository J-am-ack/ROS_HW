"""Translate navigation states into controller targets."""

import math
from dataclasses import dataclass

from .state_machine import AvoidPhase, State


@dataclass
class BehaviorTarget:
    error: float = 0.0
    preferred_speed: float = 0.0
    steer_override: float = None


class BehaviorManager:
    def __init__(self, config):
        self.cfg = config

    def run(self, state, perception, pose, context):
        cfg = self.cfg
        p = perception
        if not p.scan_valid or state in (State.INIT, State.FINISH,
                                        State.EMERGENCY_STOP):
            return BehaviorTarget(steer_override=0.0)
        center = p.center_error if p.corridor_detected else p.free_angle
        if state == State.FOLLOW:
            return BehaviorTarget(center, cfg.speed_follow)
        if state == State.RECOVER_CENTER:
            return BehaviorTarget(center, cfg.speed_recovery)
        if state in (State.APPROACH_LEFT_CORNER,
                     State.APPROACH_RIGHT_CORNER):
            direction = 1 if state == State.APPROACH_LEFT_CORNER else -1
            return BehaviorTarget(center, cfg.speed_approach,
                                  direction * cfg.turn_steer * 0.30)
        if state in (State.TURN_LEFT, State.TURN_RIGHT):
            direction = 1 if state == State.TURN_LEFT else -1
            return BehaviorTarget(0.0, cfg.speed_turn,
                                  direction * cfg.turn_steer)
        if state in (State.AVOID_LEFT, State.AVOID_RIGHT):
            direction = 1 if state == State.AVOID_LEFT else -1
            if context.avoid_phase == AvoidPhase.DEVIATE:
                steer = direction * cfg.avoid_steer
            elif context.avoid_phase == AvoidPhase.PASS:
                steer = direction * cfg.avoid_steer * 0.20
            else:
                steer = -direction * cfg.avoid_steer * 0.70
            return BehaviorTarget(0.0, cfg.speed_avoid, steer)
        return BehaviorTarget(steer_override=0.0)
