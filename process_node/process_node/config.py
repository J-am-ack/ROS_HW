"""Tunable parameters for local navigation."""

from dataclasses import dataclass


@dataclass
class NavigationConfig:
    # Course intent from the supplied map.
    route_right_turns: int = 3
    allow_left_turns: bool = False
    route_first_turn_min_distance: float = 36.5
    route_next_turn_min_distance: float = 29.0
    route_offset_enabled: bool = True
    # The third straight starts after right turn 2. Its left/right offset is
    # around 22-26 m from that corner; right turn 3 follows about 9 m later.
    route_offset_min_distance: float = 21.5
    route_final_turn_min_distance: float = 9.0
    # Distance from the third main turn/last S-bend anchor to the final
    # right turn at the start/finish area.
    route_finish_right_min_distance: float = 30.0
    offset_turn_min_angle_deg: float = 30.0
    # Enable obstacle S-bend only after the last main right turn.
    s_curve_after_route: bool = True
    s_curve_first_left: bool = True
    front_angle_deg: float = 0.0
    # The old Nav2 adapter used a negative gain from ROS positive yaw (left)
    # to the vehicle's steer command.
    steer_sign: int = -1
    speed_forward_sign: int = -1
    sector_front: tuple = (-20.0, 20.0)
    sector_front_left: tuple = (20.0, 50.0)
    sector_left: tuple = (55.0, 85.0)
    sector_far_left: tuple = (85.0, 115.0)
    sector_front_right: tuple = (-50.0, -20.0)
    sector_right: tuple = (-85.0, -55.0)
    sector_far_right: tuple = (-115.0, -85.0)
    minimum_sector_samples: int = 3
    # The recorded lidar has a narrow valid region around 0 degrees; requiring
    # 35% valid points would incorrectly turn a usable front sector into NaN.
    minimum_scan_fraction: float = 0.05
    distance_percentile: float = 30.0
    emergency_distance: float = 0.35
    front_safe_distance: float = 1.30
    # A real corner can expose the side opening before the front wall is very
    # close.  The old 1.10 m value made the vehicle pass the first bend.
    corner_trigger_distance: float = 2.40
    turn_trigger_distance: float = 2.00
    obstacle_trigger_distance: float = 1.20
    obstacle_exit_distance: float = 1.45
    opening_distance: float = 1.65
    opposite_opening_distance: float = 1.35
    passage_clearance: float = 0.90
    wall_max_distance: float = 2.40
    wall_min_distance: float = 0.25
    center_recover_threshold: float = 0.20
    obstacle_local_ratio_max: float = 0.65
    # A narrow central obstacle can resemble a corner when one side opens.
    # Only let a strongly localized blockage override that corner label.
    obstacle_corner_override_ratio: float = 0.30
    init_valid_frames: int = 3
    corner_confirm_frames: int = 3
    obstacle_confirm_frames: int = 3
    recovery_confirm_frames: int = 4
    obstacle_clear_frames: int = 3
    invalid_stop_frames: int = 15
    emergency_confirm_frames: int = 2
    emergency_release_frames: int = 5
    unknown_stop_seconds: float = 3.0
    approach_min_time: float = 0.35
    approach_max_time: float = 2.0
    approach_lost_frames: int = 5
    min_turn_time: float = 0.45
    max_turn_time: float = 15.0
    turn_min_angle_deg: float = 55.0
    min_avoid_time: float = 0.50
    max_avoid_time: float = 6.0
    avoid_deviate_time: float = 0.55
    avoid_pass_time: float = 0.65
    # Continue the diagonal search long enough to see the next closely spaced
    # cone on the new approach line; do not recenter after a short gap.
    avoid_transition_time: float = 2.00
    kp: float = 175.0
    kd: float = 18.0
    derivative_alpha: float = 0.75
    steer_max: int = 600
    max_steer_step: int = 90
    turn_steer: int = 360
    avoid_steer: int = 260
    recovery_steer_max: int = 300
    # Keep the normal cruise speed conservative for the recorded corridor.
    speed_follow: int = 60
    speed_s_curve: int = 55
    speed_approach: int = 65
    speed_turn: int = 48
    speed_avoid: int = 55
    speed_recovery: int = 40
    steer_speed_reduction: float = 0.35
    log_period_seconds: float = 1.0
