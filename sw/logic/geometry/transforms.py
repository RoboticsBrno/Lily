from __future__ import annotations
from math import atan2, cos, sin
import numpy as np


class Pose:
    def __init__(self, x: float, y: float, yaw: float) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw

    def to_transform(self) -> Transformation:
        return translation(self.x, self.y).compose(rotation(self.yaw))

    def to_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


class Transformation:
    def __init__(self, matrix: list[list[float]] | np.ndarray) -> None:
        self.matrix = np.array(matrix, dtype="f")

    def compose(self, other: "Transformation") -> "Transformation":
        return Transformation(self.matrix @ other.matrix)

    def apply_to_point(self, x: float, y: float) -> tuple[float, float]:
        point = self.matrix @ np.array([x, y, 1.0], dtype="f")
        return (float(point[0]), float(point[1]))

    def apply_to_vector(self, x: float, y: float) -> tuple[float, float]:
        vector = self.matrix @ np.array([x, y, 0.0], dtype="f")
        return (float(vector[0]), float(vector[1]))

    def inverse(self) -> "Transformation":
        try:
            return Transformation(np.linalg.inv(self.matrix))
        except np.linalg.LinAlgError as exc:
            raise ValueError("Transformation is not invertible") from exc

    def to_pose(self) -> Pose:
        x = float(self.matrix[0, 2])
        y = float(self.matrix[1, 2])
        yaw = atan2(float(self.matrix[1, 0]), float(self.matrix[0, 0]))
        return Pose(x, y, yaw)


def translation(tx: float, ty: float) -> Transformation:
    return Transformation(
        np.array([
            [1.0, 0.0, tx],
            [0.0, 1.0, ty],
            [0.0, 0.0, 1.0],
        ], dtype="f")
    )


def rotation(angle: float) -> Transformation:
    c = cos(angle)
    s = sin(angle)
    return Transformation(
        np.array([
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype="f")
    )


def scaling(sx: float, sy: float) -> Transformation:
    return Transformation(
        np.array([
            [sx, 0.0, 0.0],
            [0.0, sy, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype="f")
    )
