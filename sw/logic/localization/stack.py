from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np

from comm.messages import Measurements
from geometry.shapes import Point, ShapeGroup, Vector
from localization.bear_detector import BearDetector
from localization.particle_filter import ParticleFilterLocalizer
from localization.types import Encoders, LidarMeasurementsRel


@dataclass
class RobotParams:
    ticks_per_meter: float
    wheel_base: float


class LocalizationStack:
    world: ShapeGroup
    localizer: ParticleFilterLocalizer
    bear_detector: BearDetector
    lidar_history: deque[tuple[Point, Vector]]
    last_encoders: Optional[Encoders] = None
    params: RobotParams

    def __init__(self, world: ShapeGroup, localizer: ParticleFilterLocalizer, bear_detector: BearDetector, robot_params: RobotParams, history_len: int) -> None:
        self.world = world
        self.localizer = localizer
        self.bear_detector = bear_detector
        self.lidar_history = deque(maxlen=history_len)
        self.last_encoders: Optional[Encoders] = None
        self.params = robot_params

    def on_measurements(self, measurements: Measurements) -> None:
        enc = Encoders(
            left=measurements.encoders.left_ticks / self.params.ticks_per_meter,
            right=measurements.encoders.right_ticks / self.params.ticks_per_meter,
        )

        if self.last_encoders is not None:
            delta_left = enc.left - self.last_encoders.left
            delta_right = enc.right - self.last_encoders.right

            delta_theta = (delta_right - delta_left) / self.params.wheel_base
            delta_x = (delta_left + delta_right) / 2
            delta_y = 0
        else:
            delta_theta = 0
            delta_x = 0
            delta_y = 0

        self.last_encoders = enc

        lidar_angles = np.array([beam.angle for beam in measurements.lidar], dtype="f") + np.linspace(-delta_theta, 0, len(measurements.lidar), dtype="f")
        lidar_distances = np.array([beam.distance for beam in measurements.lidar], dtype="f")
        lidar_dxs = np.cos(lidar_angles) * lidar_distances + np.linspace(-delta_x, 0, len(lidar_angles), dtype="f")
        lidar_dys = np.sin(lidar_angles) * lidar_distances + np.linspace(-delta_y, 0, len(lidar_angles), dtype="f")

        measurements_rel = LidarMeasurementsRel(lidar_dxs, lidar_dys, lidar_angles, lidar_distances)

        self.localizer.update(delta_x, delta_y, delta_theta, measurements_rel)
        estimated_pose = self.localizer.estimate_pose()

        feature_points = self.bear_detector.update(estimated_pose, delta_x, delta_y, delta_theta, measurements_rel)
        for point, feature in feature_points:
            self.lidar_history.append((point, feature))
