from __future__ import annotations

from dataclasses import dataclass
from math import pi
import random

from comm.messages import EncodersMeasurement, LidarMeasurement
from geometry.raycast import RaycastGroup, RaycastGroupItem, raycast_from_angle
from geometry.shapes import Circle, Point, ShapeGroup
from geometry.transforms import Pose


@dataclass(frozen=True)
class LidarSensorConfig:
    rotation_speed_hz: float = 10.0
    measurement_frequency_hz: float = 400.0
    angle_min: float = -pi
    angle_max: float = pi
    max_distance: float = 8.0
    world_max_incidence_angle: float = pi / 2.0
    bear_max_incidence_angle: float = pi / 2.0
    distance_noise_stddev: float = 0.0
    angle_noise_stddev: float = 0.0
    random_distance_probability: float = 0.0

    def __post_init__(self) -> None:
        if self.rotation_speed_hz <= 0.0:
            raise ValueError("rotation_speed_hz must be positive")
        if self.measurement_frequency_hz <= 0.0:
            raise ValueError("measurement_frequency_hz must be positive")
        if self.angle_max <= self.angle_min:
            raise ValueError("angle_max must be greater than angle_min")
        if self.max_distance <= 0.0:
            raise ValueError("max_distance must be positive")
        if self.world_max_incidence_angle < 0.0 or self.world_max_incidence_angle > (pi / 2.0):
            raise ValueError("world_max_incidence_angle must be in [0, pi/2]")
        if self.bear_max_incidence_angle < 0.0 or self.bear_max_incidence_angle > (pi / 2.0):
            raise ValueError("bear_max_incidence_angle must be in [0, pi/2]")
        if self.distance_noise_stddev < 0.0:
            raise ValueError("distance_noise_stddev must be non-negative")
        if self.angle_noise_stddev < 0.0:
            raise ValueError("angle_noise_stddev must be non-negative")
        if self.random_distance_probability < 0.0 or self.random_distance_probability > 1.0:
            raise ValueError("random_distance_probability must be in [0, 1]")


@dataclass(frozen=True)
class MotorConfig:
    ticks_per_meter: float = 1000.0
    max_speed: float = 0.6

    def __post_init__(self) -> None:
        if self.ticks_per_meter <= 0.0:
            raise ValueError("ticks_per_meter must be positive")
        if self.max_speed <= 0.0:
            raise ValueError("max_speed must be positive")


class MotorSimulator:
    def __init__(self, config: MotorConfig):
        self.config = config

        self._left_tick_position = 0.0
        self._right_tick_position = 0.0
        self._left_power = 0.0
        self._right_power = 0.0

    def set_motor_power(self, left_power: float, right_power: float) -> None:
        self._left_power = _clamp(left_power, -1.0, 1.0)
        self._right_power = _clamp(right_power, -1.0, 1.0)

    def step(self, dt: float, timestamp_ms: int) -> tuple[float, float, EncodersMeasurement]:
        if dt < 0.0:
            raise ValueError("dt must be non-negative")

        left_speed = self._left_power * self.config.max_speed
        right_speed = self._right_power * self.config.max_speed

        self._left_tick_position += left_speed * dt * self.config.ticks_per_meter
        self._right_tick_position += right_speed * dt * self.config.ticks_per_meter

        encoder = EncodersMeasurement(
            left_ticks=int(round(self._left_tick_position)),
            right_ticks=int(round(self._right_tick_position)),
        )
        return left_speed, right_speed, encoder


class LidarSensorSimulator:
    def __init__(self, world: ShapeGroup, config: LidarSensorConfig, bear: Circle):
        self.world = world
        self.bear = bear
        self.config = config

        self._raycast_target = RaycastGroup(
            items=[
                RaycastGroupItem(
                    shape=self.world,
                    max_angle_of_incidence=self.config.world_max_incidence_angle,
                ),
                RaycastGroupItem(
                    shape=self.bear,
                    max_angle_of_incidence=self.config.bear_max_incidence_angle,
                ),
            ]
        )

        self._angle_span = self.config.angle_max - self.config.angle_min
        self._angle_increment = -(
            self._angle_span
            * self.config.rotation_speed_hz
            / self.config.measurement_frequency_hz
        )
        self._current_angle = self.config.angle_min
        self._sample_residual = 0.0

    def step(
        self,
        dt: float,
        pose: Pose,
        timestamp_ms: int,
    ) -> list[LidarMeasurement]:
        if dt < 0.0:
            raise ValueError("dt must be non-negative")

        sample_count_float = dt * self.config.measurement_frequency_hz + self._sample_residual
        sample_count = int(sample_count_float)
        self._sample_residual = sample_count_float - sample_count

        if sample_count <= 0:
            return []

        origin = Point(pose.x, pose.y)

        measurements: list[LidarMeasurement] = []
        for _ in range(sample_count):
            relative_angle = self._current_angle
            world_angle = pose.yaw + relative_angle
            hit = raycast_from_angle(
                shape=self._raycast_target,
                origin=origin,
                angle_radians=world_angle,
                max_distance=self.config.max_distance,
            )
            distance = self.config.max_distance
            if hit is not None:
                distance = hit.distance

            measured_angle = relative_angle
            if self.config.angle_noise_stddev > 0.0:
                angle_noise = random.gauss(0.0, self.config.angle_noise_stddev)
                max_noise = 0.4 * abs(self._angle_increment)
                measured_angle += _clamp(angle_noise, -max_noise, max_noise)

            if self.config.random_distance_probability > 0.0:
                if random.random() < self.config.random_distance_probability:
                    distance = random.uniform(0.0, self.config.max_distance)

            if hit is not None and self.config.distance_noise_stddev > 0.0:
                distance += random.gauss(0.0, self.config.distance_noise_stddev)
                distance = _clamp(distance, 0.0, self.config.max_distance)

            measurements.append(
                LidarMeasurement(
                    angle=measured_angle,
                    distance=distance,
                )
            )

            self._current_angle = self._wrap_angle(
                self._current_angle + self._angle_increment
            )

        return measurements

    def _wrap_angle(self, angle: float) -> float:
        wrapped = (angle - self.config.angle_min) % self._angle_span
        return self.config.angle_min + wrapped


def _clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))
