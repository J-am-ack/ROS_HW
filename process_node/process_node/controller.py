"""Filtered PD steering, rate limiting, and state dependent speed."""

import math
from dataclasses import dataclass

from .state_machine import State


@dataclass
class ControlCommand:
    steer: int = 0
    speed: int = 0


class SteeringController:
    def __init__(self, config):
        self.cfg = config
        self.previous_error = 0.0
        self.previous_derivative = 0.0
        self.previous_steer = 0.0
        self.previous_time = None
        self.previous_state = None

    def update(self, target, state, perception, now):
        cfg = self.cfg
        if (not perception.scan_valid or state in
                (State.INIT, State.FINISH, State.EMERGENCY_STOP) or
                perception.front_distance < cfg.emergency_distance):
            self.previous_steer = 0.0
            self.previous_derivative = 0.0
            self.previous_time = now
            self.previous_state = state
            return ControlCommand()
        if state != self.previous_state:
            self.previous_error = target.error
            self.previous_derivative = 0.0
        dt = 0.0 if self.previous_time is None else now - self.previous_time
        dt = max(0.0, min(dt, 0.5))
        if target.steer_override is None:
            raw_d = ((target.error - self.previous_error) / dt
                     if dt >= 0.01 else 0.0)
            derivative = (cfg.derivative_alpha * self.previous_derivative
                          + (1 - cfg.derivative_alpha) * raw_d)
            raw = cfg.kp * target.error + cfg.kd * derivative
            self.previous_derivative = derivative
        else:
            raw = target.steer_override
            self.previous_derivative = 0.0
        raw = max(-cfg.steer_max, min(cfg.steer_max, raw))
        step = max(-cfg.max_steer_step,
                   min(cfg.max_steer_step, raw - self.previous_steer))
        logical_steer = self.previous_steer + step
        if state == State.RECOVER_CENTER:
            logical_steer = max(-cfg.recovery_steer_max,
                                min(cfg.recovery_steer_max, logical_steer))
        scale = 1.0 - cfg.steer_speed_reduction * (
            abs(logical_steer) / cfg.steer_max)
        speed = int(round(cfg.speed_forward_sign * target.preferred_speed
                          * scale))
        self.previous_error = target.error
        self.previous_steer = logical_steer
        self.previous_time = now
        self.previous_state = state
        return ControlCommand(int(round(cfg.steer_sign * logical_steer)), speed)
