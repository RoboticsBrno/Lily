from collections import deque

from comm.messages import Measurements
from geometry.shapes import Point, ShapeGroup, Vector
from localization.bear_detector import BearDetector
from localization.particle_filter import ParticleFilterLocalizer


class LocalizationStack:
    world: ShapeGroup
    localizer: ParticleFilterLocalizer
    bear_detector: BearDetector
    lidar_history: deque[tuple[Point, Vector]]

    def __init__(self, world: ShapeGroup, localizer: ParticleFilterLocalizer, bear_detector: BearDetector, history_len: int) -> None:
        self.world = world
        self.localizer = localizer
        self.bear_detector = bear_detector
        self.lidar_history = deque(maxlen=history_len)

    def on_measurements(self, measurement: Measurements) -> None:
        self.localizer.update(measurement)
        estimated_pose = self.localizer.estimate_pose()

        feature_points = self.bear_detector.update(estimated_pose, measurement.lidar)
        for point, feature in feature_points:
            self.lidar_history.append((point, feature))
