"""OpenCV views for recorded video, command preview, and odometry on a map."""

import math
import os
import csv
from dataclasses import dataclass

import numpy as np

from .perception import PoseState


@dataclass
class PathRisk:
    level: str = 'CLEAR'
    clearance: float = math.inf
    forward: float = math.inf
    point: tuple = None
    kind: str = ''


def check_path_risk(pose, scan, prediction, speed, config,
                    collision_radius=0.32, warning_radius=0.52):
    """Compare the predicted vehicle centerline with valid lidar returns.

    Returns represent nearby surfaces, not proof of physical contact. Only
    returns in front of the vehicle and path points reached within the preview
    are considered. A stopped command has no future collision prediction.
    """
    if (not pose.valid or scan is None or len(prediction) < 2 or
            speed * config.speed_forward_sign <= 0):
        return PathRisk(level='STOPPED' if speed == 0 else 'UNAVAILABLE')
    path = np.asarray([
        ((x - pose.x) * math.cos(pose.yaw) +
         (y - pose.y) * math.sin(pose.y),
         -(x - pose.x) * math.sin(pose.yaw) +
         (y - pose.y) * math.cos(pose.yaw))
        for x, y in prediction[1:]], dtype=float)
    if not np.isfinite(path).all():
        return PathRisk(level='UNAVAILABLE')
    ranges = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    valid = (np.isfinite(ranges) & (ranges >= scan.range_min) &
             (ranges <= scan.range_max) &
             (np.abs(angles) <= math.radians(105)))
    ranges, angles = ranges[valid], angles[valid]
    if not len(ranges):
        return PathRisk(level='UNAVAILABLE')
    forward = ranges * np.cos(angles)
    left = ranges * np.sin(angles)
    valid = (forward > 0.12) & (forward < max(1.0, path[:, 0].max() + 0.7))
    obstacles = np.column_stack((forward[valid], left[valid]))
    if not len(obstacles):
        return PathRisk()
    # Distance from each return to every sampled centerline point. The
    # 0.1-second spacing in predict_path is finer than the vehicle radius.
    distances = np.linalg.norm(obstacles[:, None, :] - path[None, :, :],
                               axis=2)
    obstacle_index, _ = np.unravel_index(np.argmin(distances),
                                         distances.shape)
    clearance = float(distances.min())
    point = tuple(float(v) for v in obstacles[obstacle_index])
    level = ('COLLISION_RISK' if clearance <= collision_radius else
             'NEAR' if clearance <= warning_radius else 'CLEAR')
    return PathRisk(level, clearance, point[0], point)


class RiskRecorder:
    """Write one CSV row on each new risk episode, not on every scan frame."""

    def __init__(self, path):
        self.path = path
        self.last_level = 'CLEAR'
        self.count = 0

    def update(self, risk, stamp, scene, state):
        active = risk.level in ('NEAR', 'COLLISION_RISK')
        previous_active = self.last_level in ('NEAR', 'COLLISION_RISK')
        new_event = active and (not previous_active or
                                risk.level == 'COLLISION_RISK' and
                                self.last_level == 'NEAR')
        self.last_level = risk.level
        if not new_event:
            return False
        self.count += 1
        if self.path:
            directory = os.path.dirname(os.path.abspath(self.path))
            os.makedirs(directory, exist_ok=True)
            exists = os.path.isfile(self.path) and os.path.getsize(self.path) > 0
            with open(self.path, 'a', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                if not exists:
                    writer.writerow(('bag_time', 'level', 'kind', 'clearance_m',
                                     'forward_m', 'scene', 'state'))
                writer.writerow((f'{stamp:.3f}', risk.level, risk.kind,
                                 f'{risk.clearance:.3f}',
                                 f'{risk.forward:.3f}', scene, state))
        return True


def predict_path(pose, steer, speed, config, horizon=4.0, dt=0.1,
                 nominal_speed=0.55, wheelbase=0.38):
    """A constant-command bicycle-model preview in odometry coordinates."""
    if not pose.valid or config.steer_max <= 0 or config.speed_follow <= 0:
        return []
    forward = speed * config.speed_forward_sign
    velocity = max(0.0, forward / config.speed_follow) * nominal_speed
    angle = -steer / config.steer_max * math.radians(30.0)
    angle = max(-math.radians(30.0), min(math.radians(30.0), angle))
    x, y, yaw = pose.x, pose.y, pose.yaw
    result = [(x, y)]
    for _ in range(int(horizon / dt)):
        x += velocity * math.cos(yaw) * dt
        y += velocity * math.sin(yaw) * dt
        yaw += velocity * math.tan(angle) / wheelbase * dt
        result.append((x, y))
    return result


class DecisionVisualizer:
    def __init__(self, cv2, map_path, map_origin_px=(80.0, 90.0),
                 map_pixels_per_meter=(11.5, 11.2)):
        self.cv2 = cv2
        self.map_path = map_path
        self.map_origin_px = map_origin_px
        self.map_pixels_per_meter = map_pixels_per_meter
        self.map_image = cv2.imread(map_path) if os.path.isfile(map_path) else None
        self.actual_path = []
        self.simulated_path = []
        self.simulated_pose = PoseState()
        self.simulated_time = None

    def add_pose(self, pose):
        if not pose.valid:
            return
        point = (pose.x, pose.y)
        if not self.actual_path or math.dist(point, self.actual_path[-1]) >= 0.10:
            self.actual_path.append(point)

    def step_simulation(self, pose, steer, speed, stamp, config,
                        nominal_speed=0.55, wheelbase=0.38):
        """Integrate emitted commands without snapping back to recorded odom."""
        if not pose.valid:
            return
        if self.simulated_time is None or stamp < self.simulated_time:
            self.simulated_pose = PoseState(pose.x, pose.y, pose.yaw, True)
            self.simulated_path = [(pose.x, pose.y)]
            if self.simulated_time is not None:
                self.actual_path = [(pose.x, pose.y)]
            self.simulated_time = stamp
            return
        dt = stamp - self.simulated_time
        self.simulated_time = stamp
        if dt <= 0 or dt > 1.0:
            return
        current = self.simulated_pose
        velocity = max(0.0, speed * config.speed_forward_sign /
                       config.speed_follow) * nominal_speed
        angle = max(-math.radians(30.0), min(
            math.radians(30.0),
            -steer / config.steer_max * math.radians(30.0)))
        yaw_change = velocity * math.tan(angle) / wheelbase * dt
        mid_yaw = current.yaw + yaw_change / 2.0
        x = current.x + velocity * math.cos(mid_yaw) * dt
        y = current.y + velocity * math.sin(mid_yaw) * dt
        self.simulated_pose = PoseState(x, y, current.yaw + yaw_change, True)
        if math.dist((x, y), self.simulated_path[-1]) >= 0.10:
            self.simulated_path.append((x, y))

    def map_point(self, x, y):
        ox, oy = self.map_origin_px
        sx, sy = self.map_pixels_per_meter
        return int(round(ox + x * sx)), int(round(oy - y * sy))

    @staticmethod
    def _text(cv2, image, label, x, y, color=(230, 230, 230), scale=0.55):
        cv2.putText(image, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, 1, cv2.LINE_AA)

    def camera_view(self, frame, steer, speed, scene, state, context):
        cv2 = self.cv2
        height, width = frame.shape[:2]
        canvas = np.full((max(height, 480), width + 300, 3), 30, np.uint8)
        canvas[:height, :width] = frame
        cx, cy, radius = width + 150, 116, 73
        cv2.circle(canvas, (cx, cy), radius, (0, 210, 0), 2, cv2.LINE_AA)
        cv2.circle(canvas, (cx, cy), 6, (200, 200, 200), -1, cv2.LINE_AA)
        angle = steer * 90.0 / 600.0
        end = (int(cx + radius * math.cos(math.radians(90 - angle))),
               int(cy - radius * math.sin(math.radians(90 - angle))))
        cv2.line(canvas, (cx, cy), end, (255, 150, 30), 3, cv2.LINE_AA)
        x = width + 20
        self._text(cv2, canvas, 'Steering wheel', x + 55, 222, scale=0.6)
        cv2.line(canvas, (x, 239), (width + 280, 239), (90, 90, 90), 1)
        labels = [f'Scene: {scene}', f'State: {state}',
                  f'Steer: {steer}', f'Speed: {speed}',
                  f'Right turns: {context.completed_right_turns}/3',
                  f'Offset: {context.offset_stage}/2']
        for index, label in enumerate(labels):
            self._text(cv2, canvas, label, x, 267 + index * 29,
                       (60, 180, 255) if state == 'EMERGENCY_STOP'
                       and index == 1 else (230, 230, 230))
        if state == 'EMERGENCY_STOP':
            reason = context.stop_reason[:32]
            self._text(cv2, canvas, f'Reason: {reason}', x, 455,
                       (50, 80, 255), 0.45)
        return canvas

    def _local_point(self, pose, x, y, center=(310, 470), scale=75.0):
        dx, dy = x - pose.x, y - pose.y
        forward = dx * math.cos(pose.yaw) + dy * math.sin(pose.yaw)
        left = -dx * math.sin(pose.yaw) + dy * math.cos(pose.yaw)
        return int(center[0] - left * scale), int(center[1] - forward * scale)

    def preview_view(self, pose, scan, prediction, steer, speed, state,
                     risk=None, event_count=0):
        cv2 = self.cv2
        canvas = np.full((560, 620, 3), 26, np.uint8)
        self._text(cv2, canvas, 'Decision preview (constant command)', 18, 29,
                   (255, 255, 255), 0.7)
        self._text(cv2, canvas, 'Forward', 275, 65, (150, 150, 150))
        for distance in (1, 2, 3, 4):
            cy = 470 - distance * 75
            cv2.line(canvas, (25, cy), (595, cy), (55, 55, 55), 1)
            self._text(cv2, canvas, f'{distance}m', 28, cy - 5,
                       (130, 130, 130), 0.4)
        cv2.line(canvas, (310, 85), (310, 500), (65, 65, 65), 1)
        if scan is not None:
            step = max(1, len(scan.ranges) // 240)
            for i in range(0, len(scan.ranges), step):
                r = float(scan.ranges[i])
                if not math.isfinite(r) or r < scan.range_min or r > 5.0:
                    continue
                angle = scan.angle_min + i * scan.angle_increment
                x = int(310 - r * math.sin(angle) * 75)
                y = int(470 - r * math.cos(angle) * 75)
                if 0 <= x < 620 and 70 <= y < 510:
                    cv2.circle(canvas, (x, y), 2, (30, 160, 255), -1)
        if pose.valid and len(prediction) > 1:
            pts = [self._local_point(pose, x, y) for x, y in prediction]
            cv2.polylines(canvas, [np.array(pts, dtype=np.int32)], False,
                          (255, 80, 210), 3, cv2.LINE_AA)
        if risk is not None and risk.point is not None and risk.level in (
                'NEAR', 'COLLISION_RISK'):
            forward, left = risk.point
            location = (int(310 - left * 75), int(470 - forward * 75))
            color = ((0, 60, 255) if risk.level == 'COLLISION_RISK'
                     else (0, 210, 255))
            cv2.circle(canvas, location, 11, color, 2, cv2.LINE_AA)
        cv2.fillPoly(canvas, [np.array([(310, 453), (297, 485), (323, 485)],
                                       dtype=np.int32)], (60, 220, 70))
        if risk is not None:
            color = ((0, 60, 255) if risk.level == 'COLLISION_RISK' else
                     (0, 210, 255) if risk.level == 'NEAR' else
                     (90, 220, 90))
            detail = (f'{risk.clearance:.2f}m  {risk.kind}'
                      if math.isfinite(risk.clearance) else '')
            self._text(cv2, canvas,
                       f'Path risk: {risk.level} {detail} | events: {event_count}',
                       20, 508, color, 0.55)
        self._text(cv2, canvas, f'{state}  steer={steer}  speed={speed}',
                   20, 530, (255, 255, 255))
        self._text(cv2, canvas, 'Magenta: 4s estimate  Orange: lidar',
                   20, 550, (180, 180, 180), 0.46)
        return canvas

    def map_view(self, pose, prediction, state):
        cv2 = self.cv2
        if self.map_image is None:
            canvas = np.full((590, 660, 3), 45, np.uint8)
            self._text(cv2, canvas, 'Map image unavailable', 20, 35,
                       (60, 160, 255), 0.8)
        else:
            h, w = self.map_image.shape[:2]
            canvas = np.full((h + 95, w, 3), 32, np.uint8)
            canvas[:h, :w] = self.map_image
        if len(self.actual_path) > 1:
            pts = np.array([self.map_point(x, y) for x, y in self.actual_path],
                           dtype=np.int32)
            cv2.polylines(canvas, [pts], False, (70, 230, 90),
                          2, cv2.LINE_AA)
        if len(self.simulated_path) > 1:
            pts = np.array([self.map_point(x, y)
                            for x, y in self.simulated_path], dtype=np.int32)
            cv2.polylines(canvas, [pts], False, (255, 180, 50),
                          2, cv2.LINE_AA)
        if len(prediction) > 1:
            pts = np.array([self.map_point(x, y) for x, y in prediction],
                           dtype=np.int32)
            cv2.polylines(canvas, [pts], False, (255, 80, 210),
                          3, cv2.LINE_AA)
        if pose.valid:
            center = self.map_point(pose.x, pose.y)
            cv2.circle(canvas, center, 6, (255, 255, 255), -1, cv2.LINE_AA)
            tip = (int(center[0] + 16 * math.cos(pose.yaw)),
                   int(center[1] - 16 * math.sin(pose.yaw)))
            cv2.arrowedLine(canvas, center, tip, (30, 30, 240),
                            2, cv2.LINE_AA, tipLength=0.4)
        if self.simulated_pose.valid:
            cv2.circle(canvas, self.map_point(
                self.simulated_pose.x, self.simulated_pose.y),
                6, (255, 180, 50), -1, cv2.LINE_AA)
        h = canvas.shape[0]
        self._text(cv2, canvas, 'Green: recorded odom path', 15, h - 83,
                   (70, 230, 90))
        self._text(cv2, canvas, 'Blue: integrated command estimate', 15, h - 61,
                   (255, 180, 50))
        self._text(cv2, canvas, 'Magenta: 4s command estimate', 15, h - 39,
                   (255, 80, 210))
        self._text(cv2, canvas, f'State: {state} | white/red: current pose',
                   15, h - 15, (230, 230, 230))
        return canvas
