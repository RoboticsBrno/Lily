from dataclasses import dataclass

import numpy as np


@dataclass
class Encoders:
    left: float
    right: float


@dataclass
class LidarMeasurementsRel:
    dxs: np.ndarray
    dys: np.ndarray
    angles: np.ndarray
    distances: np.ndarray
