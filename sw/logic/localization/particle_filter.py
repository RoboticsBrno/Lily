"""Particle filter implementation for robot localization."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, exp, pi
import random
from typing import Optional

from comm.messages import Measurements
from geometry import (
    ShapeGroup,
)
from geometry.transforms import Pose, rotation, translation
from geometry.util import lerp

import numpy as np

from map.raster import RasterMap


@dataclass
class ParticleFilterConfig:
    num_particles: int
    wheel_base: float
    ticks_per_meter: float
    position_noise: float
    heading_noise: float
    estimate_smoothing_alpha: float
    lidar_likelihood_map: RasterMap

    def __post_init__(self) -> None:
        if not 0.0 <= self.estimate_smoothing_alpha <= 1.0:
            raise ValueError("estimate_smoothing_alpha must be in range [0.0, 1.0]")


@dataclass
class Encoders:
    left: float
    right: float


@dataclass
class Particle:
    pose: Pose
    weight: float = 1.0


class ParticleFilterLocalizer:
    def __init__(self, world: ShapeGroup, config: ParticleFilterConfig, initial_pose: Pose) -> None:
        self.world = world
        self.config = config
        self.last_encoders: Optional[Encoders] = None
        self._smoothed_pose = initial_pose

        n = config.num_particles
        self.xs = np.full(n, initial_pose.x, dtype="f")
        self.ys = np.full(n, initial_pose.y, dtype="f")
        self.thetas = np.full(n, initial_pose.yaw, dtype="f")
        self.weights = np.full(n, 1.0 / n, dtype="f")

        self._normalize_weights()

    def update(self, measurements: Measurements) -> None:
        encoders = Encoders(
            left=measurements.encoders.left_ticks / self.config.ticks_per_meter,
            right=measurements.encoders.right_ticks / self.config.ticks_per_meter,
        )

        if self.last_encoders is not None:
            self._apply_motion_model(encoders)

        self._apply_sensor_model([(beam.angle, beam.distance) for beam in measurements.lidar])
        self._normalize_weights()
        self._resample_particles()

        self.last_encoders = encoders

    def estimate_pose(self) -> Pose:
        x = np.sum(self.xs * self.weights)
        y = np.sum(self.ys * self.weights)
        sin_sum = np.sum(np.sin(self.thetas) * self.weights)
        cos_sum = np.sum(np.cos(self.thetas) * self.weights)
        theta = atan2(sin_sum, cos_sum)
        raw_pose = Pose(x, y, theta)

        if self.config.estimate_smoothing_alpha >= 1.0:
            self._smoothed_pose = raw_pose
            return raw_pose

        self._smoothed_pose = lerp(self._smoothed_pose, raw_pose, self.config.estimate_smoothing_alpha)
        return self._smoothed_pose

    def get_estimate(self) -> Pose:
        return self._smoothed_pose

    def _apply_motion_model(self, enc: Encoders) -> None:
        if self.last_encoders is None:
            return

        delta_left = enc.left - self.last_encoders.left
        delta_right = enc.right - self.last_encoders.right

        delta_theta = (delta_right - delta_left) / self.config.wheel_base
        delta_x = (delta_left + delta_right) / 2
        delta_y = 0

        movement_transform = translation(delta_x, delta_y).compose(rotation(delta_theta))

        n = len(self.xs)
        position_noise_x = np.random.normal(0.0, self.config.position_noise, size=n)
        position_noise_y = np.random.normal(0.0, self.config.position_noise, size=n)
        heading_noise = np.random.normal(0.0, self.config.heading_noise, size=n)

        cos_t = np.cos(self.thetas)
        sin_t = np.sin(self.thetas)
        self.xs += cos_t * delta_x - sin_t * delta_y + position_noise_x
        self.ys += sin_t * delta_x + cos_t * delta_y + position_noise_y
        self.thetas = (self.thetas + delta_theta + heading_noise + pi) % (2 * pi) - pi

        self._smoothed_pose = self._smoothed_pose.to_transform().compose(movement_transform).to_pose()

    def _apply_sensor_model(self, lidar_measurements: list[tuple[float, float]]) -> None:
        lidar_measurements = list(filter(lambda m: m[1] < 8.0, lidar_measurements))
        for _ in range(len(lidar_measurements) // 50):
            choices = np.random.choice(len(lidar_measurements), size=len(self.xs))
            angles = np.array([lidar_measurements[i][0] for i in choices], dtype="f")
            distances = np.array([lidar_measurements[i][1] for i in choices], dtype="f")

            lidar_xs = self.xs + np.cos(self.thetas + angles) * distances
            lidar_ys = self.ys + np.sin(self.thetas + angles) * distances
            likelihoods = self.config.lidar_likelihood_map.get_many(lidar_xs, lidar_ys)
            self.weights *= likelihoods

    def _gaussian(self, mu: float, sigma: float, x: float) -> float:
        if sigma <= 0:
            raise ValueError("Sigma must be positive")
        return (1.0 / (sigma * (2.0 * pi) ** 0.5)) * exp(-0.5 * ((x - mu) / sigma) ** 2) + 0.2

    def _normalize_weights(self) -> None:
        total_weight: float = np.sum(self.weights)
        if total_weight == 0.0:
            self.weights.fill(1.0 / len(self.weights))
            return

        self.weights /= total_weight

    def _resample_particles(self) -> None:
        tiny_parts = np.sum(self.weights < (0.25 / len(self.weights)))
        if tiny_parts < len(self.weights) * 0.5:
            # skip resampling
            return

        indices = list(range(len(self.weights)))
        random.shuffle(indices)

        threshold = 0.5 / len(self.weights)
        new_xs = np.empty_like(self.xs)
        new_ys = np.empty_like(self.ys)
        new_thetas = np.empty_like(self.thetas)
        cumm_w = 0.0
        idx = 0
        for i in range(len(self.weights)):
            src = indices[i]
            cumm_w += self.weights[src]
            while cumm_w > threshold:
                new_xs[idx] = self.xs[src]
                new_ys[idx] = self.ys[src]
                new_thetas[idx] = self.thetas[src]
                idx += 1
                cumm_w -= 1 / len(self.weights)

        self.xs = new_xs
        self.ys = new_ys
        self.thetas = new_thetas
        self.weights.fill(1.0 / len(self.weights))
