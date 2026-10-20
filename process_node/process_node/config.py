"""Tunable parameters for local navigation."""

from dataclasses import dataclass


@dataclass
class NavigationConfig:
    front_angle_deg: float = 0.0
    steer_sign: int = 1
    speed_forward_sign: int = -1
    sector_front: tuple = (-12.0, 12.0)
    sector_front_left: tuple = (12.0, 40.0)
    sector_left: tuple = (55.0, 85.0)
    sector_far_left: tuple = (85.0, 115.0)
    sector_front_right: tuple = (-40.0, -12.0)
    sector_right: tuple = (-85.0, -55.0)
    sector_far_right: tuple = (-115.0, -85.0)
    minimum_sector_samples: int = 3
    minimum_scan_fraction: float = 0.35
    distance_percentile: float = 30.0
    emergency_distance: float = 0.35
    front_safe_distance: float = 1.30
    corner_trigger_distance: float = 1.10
    turn_trigger_distance: float = 0.85
    obstacle_trigger_distance: float = 1.20
    obstacle_exit_distance: float = 1.45
    opening_distance: float = 1.65
    opposite_opening_distance: float = 1.35
    passage_clearance: float = 0.90
    wall_max_distance: float = 2.40
    wall_min_distance: float = 0.25
    center_recover_threshold: float = 0.20
    obstacle_local_ratio_max: float = 0.65
    init_valid_frames: int = 3
    corner_confirm_frames: int = 3
    obstacle_confirm_frames: int = 3
    recovery_confirm_frames: int = 4
    obstacle_clear_frames: int = 3
    invalid_stop_frames: int = 3
    unknown_stop_seconds: float = 3.0
    approach_min_time: float = 0.35
    min_turn_time: float = 0.45
    max_turn_time: float = 4.0
    turn_min_angle_deg: float = 55.0
    min_avoid_time: float = 0.50
    max_avoid_time: float = 6.0
    avoid_deviate_time: float = 0.55
    avoid_pass_time: float = 0.65
    avoid_return_time: float = 0.55
    kp: float = 175.0
    kd: float = 18.0
    derivative_alpha: float = 0.75
    steer_max: int = 600
    max_steer_step: int = 90
    turn_steer: int = 360
    avoid_steer: int = 260
    recovery_steer_max: int = 300
    speed_follow: int = 100
    speed_approach: int = 65
    speed_turn: int = 48
    speed_avoid: int = 55
    speed_recovery: int = 40
    steer_speed_reduction: float = 0.35
    log_period_seconds: float = 1.0
