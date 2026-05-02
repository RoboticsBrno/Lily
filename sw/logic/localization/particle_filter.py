"""Particle filter implementation for robot localization."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, exp, pi, sin
from random import gauss
import random
from typing import Optional

from comm.messages import Measurements
from geometry import (
    ShapeGroup,
    wrap_angle,
)
from geometry.shapes import Point
from geometry.transforms import Pose, rotation, translation
from geometry.util import find_nearest, lerp


@dataclass
class ParticleFilterConfig:
    num_particles: int = 100
    wheel_base: float = 0.2
    ticks_per_meter: float = 1000.0
    position_noise: float = 0.01
    heading_noise: float = 0.01
    lidar_likelihood_stddev: float = 0.1
    estimate_smoothing_alpha: float = 1.0

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
        self.particles: list[Particle] = []
        self.last_encoders: Optional[Encoders] = None
        self._smoothed_pose: Optional[Pose] = None

        self.particles = []
        for _ in range(self.config.num_particles):
            x = initial_pose.x + gauss(0.0, config.position_noise * 10)
            y = initial_pose.y + gauss(0.0, config.position_noise * 10)
            theta = wrap_angle(initial_pose.yaw + gauss(0.0, config.heading_noise) * 10)
            self.particles.append(Particle(Pose(x, y, theta)))

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
        x = sum(p.pose.x * p.weight for p in self.particles)
        y = sum(p.pose.y * p.weight for p in self.particles)
        sin_sum = sum(sin(p.pose.yaw) * p.weight for p in self.particles)
        cos_sum = sum(cos(p.pose.yaw) * p.weight for p in self.particles)
        theta = atan2(sin_sum, cos_sum)
        raw_pose = Pose(x, y, theta)

        if self._smoothed_pose is None or self.config.estimate_smoothing_alpha >= 1.0:
            self._smoothed_pose = raw_pose
            return raw_pose

        self._smoothed_pose = lerp(self._smoothed_pose, raw_pose, self.config.estimate_smoothing_alpha)
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

        for particle in self.particles:
            noise_transform = translation(
                random.gauss(0, self.config.position_noise),
                random.gauss(0, self.config.position_noise),
            ).compose(rotation(random.gauss(0, self.config.heading_noise)))
            particle.pose = particle.pose.to_transform().compose(movement_transform).compose(noise_transform).to_pose()

        if self._smoothed_pose:
            self._smoothed_pose = self._smoothed_pose.to_transform().compose(movement_transform).to_pose()

    def _apply_sensor_model(self, lidar_measurements: list[tuple[float, float]]) -> None:
        lidar_measurements = list(filter(lambda m: m[1] < 8.0, lidar_measurements))
        for particle in self.particles:
            for angle, distance in random.sample(lidar_measurements, len(lidar_measurements) // 50):
                lidar_point = Point(
                    particle.pose.x + cos(particle.pose.yaw + angle) * distance,
                    particle.pose.y + sin(particle.pose.yaw + angle) * distance,
                )
                _, nearest_distance = find_nearest(self.world, lidar_point)
                likelihood = self._gaussian(0.0, self.config.lidar_likelihood_stddev, nearest_distance)
                particle.weight *= likelihood

    def _gaussian(self, mu: float, sigma: float, x: float) -> float:
        if sigma <= 0:
            raise ValueError("Sigma must be positive")
        return (1.0 / (sigma * (2.0 * pi) ** 0.5)) * exp(-0.5 * ((x - mu) / sigma) ** 2) + 0.2

    def _normalize_weights(self) -> None:
        total_weight = sum(p.weight for p in self.particles)
        if total_weight == 0.0:
            for particle in self.particles:
                particle.weight = 1.0 / len(self.particles)
            return

        for particle in self.particles:
            particle.weight /= total_weight

    def _resample_particles(self) -> None:
        tiny_parts = sum([1 for p in self.particles if p.weight < (0.25 / len(self.particles))])
        if tiny_parts < len(self.particles) * 0.5:
            # skip resampling
            return

        random.shuffle(self.particles)
        cumm_w = 0.0
        threshold = 0.5 / len(self.particles)
        new_parts: list[Particle] = []
        print("Resampling", tiny_parts, "tiny particles")
        for p in self.particles:
            cumm_w += p.weight
            while cumm_w > threshold:
                new_parts.append(Particle(pose=p.pose, weight=1.0 / len(self.particles)))
                cumm_w -= 1 / len(self.particles)

        self.particles = new_parts
