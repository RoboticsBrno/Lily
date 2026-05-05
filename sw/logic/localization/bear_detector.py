from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from math import cos, inf, pi, sin, sqrt
from typing import Optional

from geometry.shapes import Point, ShapeGroup, Vector, sum_vec
from geometry.util import dist2, lerp_pt
from geometry.transforms import Pose

import numpy as np

from localization.types import LidarMeasurementsRel


@dataclass
class DBSCANConfig:
    eps: float = 0.05
    min_samples: int = 3


class DBSCAN:
    def __init__(self, config: DBSCANConfig) -> None:
        self.eps2 = config.eps * config.eps
        self.min_samples = config.min_samples

    def fit(self, points: list[Point]) -> dict[int, list[Point]]:
        xs = np.array([point.x for point in points], dtype="f")
        ys = np.array([point.y for point in points], dtype="f")

        labels: list[Optional[int]] = [None] * len(points)
        cluster_id = 0

        for i in range(len(points)):
            if labels[i] is not None:
                continue

            neighbors = self._region_query(xs, ys, i)

            if len(neighbors) < self.min_samples:
                labels[i] = -1  # Noise
                continue

            cluster_id += 1
            labels[i] = cluster_id

            work = deque(neighbors)
            while work:
                n = work.popleft()

                if labels[n] == -1:
                    labels[n] = cluster_id
                    continue
                if labels[n] is not None:
                    continue

                labels[n] = cluster_id
                neighbors = self._region_query(xs, ys, n)
                if len(neighbors) >= self.min_samples:
                    work.extend(neighbors)

        clusters: dict[int, list[Point]] = {}
        for label, point in zip(labels, points):
            if label is None or label == -1:
                continue
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(point)

        return clusters

    def _region_query(self, xs, ys, index: int) -> list[int]:
        dxs = xs - xs[index]
        dys = ys - ys[index]
        dist2s = dxs * dxs + dys * dys
        return np.where(dist2s <= self.eps2)[0].tolist()


@dataclass
class BearDetectionConfig:
    min_distance: float = 0.1
    max_distance: float = 2
    feature_detection_points: int = 7
    feature_threshold: float = 0.015
    line_covariance_ratio_threshold: float = 0.01
    min_samples: int = feature_detection_points // 2 + 1
    dbscan_config: DBSCANConfig = field(default_factory=DBSCANConfig)
    match_threshold: float = 0.3


@dataclass
class BearDetection:
    position: Point
    confidence: float


class BearDetector:
    def __init__(self, world: ShapeGroup, config: BearDetectionConfig) -> None:
        self.world = world
        self.config = config
        self._scan_buffer: deque[tuple[Point, float, float]] = deque(maxlen=1000)
        self._candidate_points: deque[list[Point]] = deque([[]] * 1)
        self._waiting: deque[tuple[Point, float, float]] \
            = deque(([(Point(0, 0), 0.0, 1.0)] * (config.feature_detection_points)))
        self._last_angle: float = 0.0
        self._feat_vec = np.array([0.0, 0.0], dtype="f")
        self._candidate_detections: list[BearDetection] = []
        self._last_detection: Optional[BearDetection] = None

    def _end_revolution(self) -> None:
        # end of revolution
        points = self._candidate_points[-1]
        clusters = self._scan(points)
        matched, new = self._match_detections([point for point, _ in clusters])

        purge: set[int] = set()
        for idx, match in enumerate(matched):
            if match is not None:
                self._candidate_detections[idx].position = lerp_pt(self._candidate_detections[idx].position, match, 0.5)
                self._candidate_detections[idx].confidence = min(5.0, self._candidate_detections[idx].confidence + 1.0)
            else:
                self._candidate_detections[idx].confidence = max(0.0, self._candidate_detections[idx].confidence - 1.0)
                if self._candidate_detections[idx].confidence <= 0.0:
                    purge.add(idx)
        for point in new:
            self._candidate_detections.append(BearDetection(position=point, confidence=1.0))

        self._candidate_detections = [d for idx, d in enumerate(self._candidate_detections) if idx not in purge]
        if self._candidate_detections:
            best = max(self._candidate_detections, key=lambda d: d.confidence)
            if best.confidence >= 3.0:
                self._last_detection = best
        else:
            self._last_detection = None

        self._candidate_points.popleft()
        self._candidate_points.append([])

    def update(self, robot_pose: Pose, delta_x: float, delta_y: float,
               delta_theta: float, measurements: LidarMeasurementsRel) -> list[tuple[Point, Vector]]:
        output = []

        world_angles = (measurements.angles + robot_pose.yaw) % (2 * pi)
        sin_ = sin(robot_pose.yaw)
        cos_ = cos(robot_pose.yaw)
        world_xs = measurements.dxs * cos_ - measurements.dys * sin_ + robot_pose.x
        world_ys = measurements.dxs * sin_ + measurements.dys * cos_ + robot_pose.y

        for i in range(len(measurements.dxs)):
            dist = measurements.distances[i]
            x = world_xs[i]
            y = world_ys[i]
            angle = world_angles[i]
            if angle > 1.5 * pi and self._last_angle < 0.5 * pi:
                self._end_revolution()

            self._last_angle = angle
            if not self._is_distance_in_range(dist):
                continue

            world_point = Point(x, y)

            self._waiting.append((world_point, angle, dist))

            mid = self.config.feature_detection_points // 2
            self._feat_vec = (
                self._feat_vec
                + self._waiting[mid][0].to_array() * self.config.feature_detection_points
                - self._waiting[mid + 1][0].to_array() * self.config.feature_detection_points
                - self._waiting[0][0].to_array()
                + self._waiting[-1][0].to_array()
            )
            self._waiting.popleft()

            mid_pt, mid_angle, mid_distance = self._waiting[mid]
            feat_vec = self._feat_vec * (1 / (self.config.feature_detection_points * (mid_distance + 0.2)))
            feat_val = float(np.linalg.norm(feat_vec))
            if feat_val > self.config.feature_threshold:
                self._candidate_points[-1].append(mid_pt)
                self._process_block()
            else:
                self._scan_buffer.append((mid_pt, mid_angle, feat_val))
            output.append((mid_pt, Vector(feat_vec[0], feat_vec[1])))

        return output

    def _is_distance_in_range(self, distance: float) -> bool:
        return self.config.min_distance <= distance < self.config.max_distance

    def _process_block(self) -> None:
        if len(self._scan_buffer) < self.config.feature_detection_points:
            self._candidate_points[-1].extend(point for point, _, _ in self._scan_buffer)
            self._scan_buffer.clear()
            return

        block_points = [point for point, _, _ in self._scan_buffer]
        cov = self._calc_cov(block_points)
        if cov > self.config.line_covariance_ratio_threshold:
            self._candidate_points[-1].extend(block_points)

        self._scan_buffer.clear()

    def _calc_cov(self, points: list[Point]) -> float:
        if len(points) < 2:
            return inf

        count = float(len(points))
        mean_x = sum(point.x for point in points) / count
        mean_y = sum(point.y for point in points) / count

        cov_xx = sum((point.x - mean_x) * (point.x - mean_x) for point in points) / count
        cov_xy = sum((point.x - mean_x) * (point.y - mean_y) for point in points) / count
        cov_yy = sum((point.y - mean_y) * (point.y - mean_y) for point in points) / count

        trace = cov_xx + cov_yy
        discriminant = max(0.0, (cov_xx - cov_yy) * (cov_xx - cov_yy) + 4.0 * cov_xy * cov_xy)
        root = sqrt(discriminant)
        major = (trace + root) / 2.0
        minor = (trace - root) / 2.0

        if major <= 1e-12:
            return inf

        return minor / major

    def _scan(self, points: list[Point]) -> list[tuple[Point, int]]:
        scanner = DBSCAN(self.config.dbscan_config)
        clusters = scanner.fit(points)

        candidates: list[tuple[Point, int]] = []
        for cluster in clusters.values():
            if len(cluster) < self.config.min_samples:
                continue
            if self._calc_cov(cluster) <= self.config.line_covariance_ratio_threshold:
                continue

            vec = sum_vec(
                Vector(0, 0),
                *[point.to_vector() for point in cluster]
            ).scaled(1 / len(cluster))
            candidates.append((Point(vec.x, vec.y), len(cluster)))

        return candidates

    def _match_detections(self, points: list[Point]) -> tuple[list[Optional[Point]], list[Point]]:
        matched: list[Optional[Point]] = [None] * len(self._candidate_detections)
        new: list[Point] = []

        for point in points:
            closest = None
            closest_dist2 = inf
            for idx, detection in enumerate(self._candidate_detections):
                d2 = dist2(detection.position, point)
                if d2 < closest_dist2:
                    closest_dist2 = d2
                    closest = idx

            if closest is not None and closest_dist2 < self.config.match_threshold * self.config.match_threshold:
                matched[closest] = point
            else:
                new.append(point)

        return matched, new

    def get_estimate(self) -> Optional[BearDetection]:
        return self._last_detection
