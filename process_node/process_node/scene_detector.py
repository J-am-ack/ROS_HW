"""Classify a preprocessed scan into one local scene."""

from enum import Enum, auto


class SceneType(Enum):
    CORRIDOR = auto()
    LEFT_CORNER = auto()
    RIGHT_CORNER = auto()
    OBSTACLE = auto()
    OPEN_SPACE = auto()
    DEAD_END = auto()
    UNKNOWN = auto()


class SceneDetector:
    def __init__(self, config):
        self.cfg = config

    def detect(self, perception):
        p = perception
        if not p.scan_valid:
            return SceneType.UNKNOWN
        if p.dead_end_detected:
            return SceneType.DEAD_END
        if p.left_corner_detected:
            return SceneType.LEFT_CORNER
        if p.right_corner_detected:
            return SceneType.RIGHT_CORNER
        if p.obstacle_front:
            return SceneType.OBSTACLE
        if p.corridor_detected:
            return SceneType.CORRIDOR
        if p.front_distance > self.cfg.front_safe_distance:
            return SceneType.OPEN_SPACE
        return SceneType.UNKNOWN
