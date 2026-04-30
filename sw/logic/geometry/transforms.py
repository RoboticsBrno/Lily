from __future__ import annotations
from math import atan2, cos, sin


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
    def __init__(self, matrix: list[list[float]]) -> None:
        self.matrix = matrix

    def compose(self, other: "Transformation") -> "Transformation":
        return Transformation(_matmul(self.matrix, other.matrix))

    def apply_to_point(self, x: float, y: float) -> tuple[float, float]:
        px = self.matrix[0][0] * x + self.matrix[0][1] * y + self.matrix[0][2]
        py = self.matrix[1][0] * x + self.matrix[1][1] * y + self.matrix[1][2]
        return (px, py)

    def inverse(self) -> "Transformation":
        a = self.matrix[0][0]
        b = self.matrix[0][1]
        tx = self.matrix[0][2]
        c = self.matrix[1][0]
        d = self.matrix[1][1]
        ty = self.matrix[1][2]

        det = a * d - b * c
        if abs(det) < 1e-12:
            raise ValueError("Transformation is not invertible")

        inv_det = 1.0 / det
        ia = d * inv_det
        ib = -b * inv_det
        ic = -c * inv_det
        idd = a * inv_det
        itx = -(ia * tx + ib * ty)
        ity = -(ic * tx + idd * ty)

        return Transformation(
            [
                [ia, ib, itx],
                [ic, idd, ity],
                [0.0, 0.0, 1.0],
            ]
        )

    def to_pose(self) -> Pose:
        x = self.matrix[0][2]
        y = self.matrix[1][2]
        yaw = atan2(self.matrix[1][0], self.matrix[0][0])
        return Pose(x, y, yaw)


def _matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def translation(tx: float, ty: float) -> Transformation:
    return Transformation(
        [
            [1.0, 0.0, tx],
            [0.0, 1.0, ty],
            [0.0, 0.0, 1.0],
        ]
    )


def rotation(angle: float) -> Transformation:
    c = cos(angle)
    s = sin(angle)
    return Transformation(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )


def scaling(sx: float, sy: float) -> Transformation:
    return Transformation(
        [
            [sx, 0.0, 0.0],
            [0.0, sy, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
