"""Robust, angle based LaserScan features; independent of ROS imports."""

import math
from dataclasses import dataclass


@dataclass
class PoseState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    valid: bool = False


@dataclass
class PerceptionState:
    far_left_distance: float = math.nan
    left_distance: float = math.nan
    front_left_distance: float = math.nan
    front_distance: float = math.nan
    front_right_distance: float = math.nan
    right_distance: float = math.nan
    far_right_distance: float = math.nan
    left_opening: float = 0.0
    front_opening: float = 0.0
    right_opening: float = 0.0
    corridor_detected: bool = False
    center_error: float = 0.0
    heading_error: float = 0.0
    obstacle_front: bool = False
    obstacle_left: bool = False
    obstacle_right: bool = False
    left_corner_detected: bool = False
    right_corner_detected: bool = False
    dead_end_detected: bool = False
    scan_valid: bool = False
    confidence: float = 0.0
    free_angle: float = 0.0
    front_blocked_ratio: float = 0.0


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def index_to_angle(index, angle_min, angle_increment):
    return angle_min + index * angle_increment


def angle_to_index(angle, angle_min, angle_increment):
    if not math.isfinite(angle_increment) or angle_increment == 0:
        raise ValueError('Invalid scan angle increment')
    return int(round((angle - angle_min) / angle_increment))


def percentile(values, percent):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = (len(ordered) - 1) * percent / 100.0
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


class ScanPerception:
    def __init__(self, config):
        self.cfg = config

    def update(self, scan, pose=None):
        cfg = self.cfg
        result = PerceptionState()
        ranges = list(scan.ranges)
        increment = float(scan.angle_increment)
        if not ranges or not math.isfinite(increment) or increment == 0:
            return result
        minimum = float(scan.range_min)
        maximum = float(scan.range_max)
        if not (math.isfinite(minimum) and math.isfinite(maximum)
                and 0 <= minimum < maximum):
            return result
        clean = [float(value) if math.isfinite(value)
                 and minimum <= value <= maximum else math.nan
                 for value in ranges]
        angles = [math.degrees(normalize_angle(
            index_to_angle(i, scan.angle_min, increment)
            - math.radians(cfg.front_angle_deg)))
            for i in range(len(clean))]

        def sector(bounds):
            indices = [i for i, angle in enumerate(angles)
                       if bounds[0] <= angle < bounds[1]]
            values = [clean[i] for i in indices if math.isfinite(clean[i])]
            covered = len(indices) >= cfg.minimum_sector_samples
            valid = covered and len(values) >= cfg.minimum_sector_samples
            valid &= len(values) >= cfg.minimum_scan_fraction * len(indices)
            return (percentile(values, cfg.distance_percentile) if valid
                    else math.nan, values, covered)

        names = ('far_left', 'left', 'front_left', 'front',
                 'front_right', 'right', 'far_right')
        data = {name: sector(getattr(cfg, 'sector_' + name)) for name in names}
        for name in names:
            setattr(result, name + '_distance', data[name][0])
        required = ('left', 'front_left', 'front', 'front_right', 'right')
        result.scan_valid = all(math.isfinite(data[name][0]) for name in required)
        result.confidence = sum(math.isfinite(data[name][0]) for name in names) / len(names)
        if not result.scan_valid:
            return result

        left, front, right = (result.left_distance,
                              result.front_distance, result.right_distance)
        fl, fr = result.front_left_distance, result.front_right_distance
        result.left_opening = max(left, fl)
        result.right_opening = max(right, fr)
        result.front_opening = front
        left_wall = cfg.wall_min_distance <= left <= cfg.wall_max_distance
        right_wall = cfg.wall_min_distance <= right <= cfg.wall_max_distance
        result.corridor_detected = left_wall and right_wall
        if result.corridor_detected:
            result.center_error = left - right
        result.obstacle_left = fl < cfg.obstacle_trigger_distance
        result.obstacle_right = fr < cfg.obstacle_trigger_distance

        frontal = [clean[i] for i, angle in enumerate(angles)
                   if -40.0 <= angle <= 40.0 and math.isfinite(clean[i])]
        result.front_blocked_ratio = (sum(v < cfg.obstacle_trigger_distance
                                         for v in frontal) / len(frontal)
                                      if frontal else 1.0)
        result.left_corner_detected = (
            front < cfg.corner_trigger_distance
            and result.left_opening > cfg.opening_distance
            and result.right_opening < cfg.opposite_opening_distance)
        result.right_corner_detected = (
            front < cfg.corner_trigger_distance
            and result.right_opening > cfg.opening_distance
            and result.left_opening < cfg.opposite_opening_distance)
        result.dead_end_detected = (
            front < cfg.corner_trigger_distance
            and result.left_opening < cfg.passage_clearance
            and result.right_opening < cfg.passage_clearance)
        result.obstacle_front = (
            result.corridor_detected
            and front < cfg.obstacle_trigger_distance
            and result.front_blocked_ratio < cfg.obstacle_local_ratio_max
            and max(fl, fr) > cfg.passage_clearance
            and not (result.left_corner_detected
                     or result.right_corner_detected))

        # Maximise the low percentile of a rolling angular window, with a
        # mild preference for driving straight in open areas.
        candidates = []
        for center in range(-60, 61, 5):
            values = [clean[i] for i, angle in enumerate(angles)
                      if abs(angle - center) <= 8.0 and math.isfinite(clean[i])]
            if len(values) >= cfg.minimum_sector_samples:
                clearance = percentile(values, cfg.distance_percentile)
                if clearance > cfg.passage_clearance:
                    candidates.append((clearance - abs(center) * 0.008, center))
        if candidates:
            result.free_angle = math.radians(max(candidates)[1])
        return result
