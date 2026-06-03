from dataclasses import dataclass
from typing import List, Union


# Commands


@dataclass
class MoveCommand:
    left_speed: float
    right_speed: float


@dataclass
class ClawCommand:
    pwm: int


@dataclass
class ArmCommand:
    pass

# Sensor measurements


@dataclass
class LidarMeasurement:
    angle: float
    distance: float


@dataclass
class EncodersMeasurement:
    left_ticks: int
    right_ticks: int


@dataclass
class Measurements:
    timestamp: int
    lidar: List[LidarMeasurement]
    encoders: EncodersMeasurement


Command = Union[MoveCommand, ClawCommand, ArmCommand]
