from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Union

import numpy as np

from .transforms import Transformation


ShapeItem = Union[
    "Point",
    "Circle",
    "Line",
    "ShapeGroup",
]


@dataclass(frozen=True)
class Vector:
    x: float
    y: float

    def to_array(self) -> np.ndarray:
        return np.asarray([self.x, self.y], dtype="f")

    @classmethod
    def from_array(cls, array: np.ndarray) -> "Vector":
        x, y = np.asarray(array, dtype="f")
        return cls(float(x), float(y))

    @classmethod
    def from_points(cls, start: "Point", end: "Point") -> "Vector":
        return cls(x=end.x - start.x, y=end.y - start.y)

    def magnitude(self) -> float:
        return hypot(self.x, self.y)

    def normalized(self) -> "Vector":
        mag = self.magnitude()
        if mag == 0.0:
            raise ValueError("Cannot normalize zero-length vector")
        return Vector(self.x / mag, self.y / mag)

    def dot(self, other: "Vector") -> float:
        return self.x * other.x + self.y * other.y

    def cross(self, other: "Vector") -> float:
        return self.x * other.y - self.y * other.x

    def scaled(self, factor: float) -> "Vector":
        return Vector(self.x * factor, self.y * factor)

    def apply_transform(self, transform: Transformation) -> "Vector":
        x, y = transform.apply_to_vector(self.x, self.y)
        return Vector(x=x, y=y)

    def to_tuple(self) -> tuple[float, float]:
        return (self.x, self.y)

    def invert(self) -> "Vector":
        return Vector(-self.x, -self.y)


@dataclass(frozen=True)
class Point:
    x: float
    y: float

    def to_array(self) -> np.ndarray:
        return np.asarray([self.x, self.y], dtype="f")

    @classmethod
    def from_array(cls, array: np.ndarray) -> "Point":
        x, y = np.asarray(array, dtype="f")
        return cls(float(x), float(y))

    def apply_transform(self, transform: Transformation) -> Point:
        x, y = transform.apply_to_point(self.x, self.y)
        return Point(x=x, y=y)

    def translated(self, vector: Vector) -> Point:
        return Point(x=self.x + vector.x, y=self.y + vector.y)

    def to_vector(self) -> Vector:
        return Vector(x=self.x, y=self.y)


@dataclass(frozen=True)
class Circle:
    center: Point
    radius: float

    def apply_transform(self, transform: Transformation) -> Circle:
        """
        For non-uniform transforms, the true result is an ellipse. This method
        keeps the Circle type by using the average transformed radius measured
        on the local x and y axes.
        """
        transformed_center = self.center.apply_transform(transform)
        edge_x = Point(self.center.x + self.radius, self.center.y).apply_transform(
            transform
        )
        edge_y = Point(self.center.x, self.center.y + self.radius).apply_transform(
            transform
        )
        rx = hypot(edge_x.x - transformed_center.x, edge_x.y - transformed_center.y)
        ry = hypot(edge_y.x - transformed_center.x, edge_y.y - transformed_center.y)
        return Circle(center=transformed_center, radius=(rx + ry) / 2.0)


@dataclass(frozen=True)
class Line:
    a: Point
    b: Point

    def apply_transform(self, transform: Transformation) -> Line:
        return Line(
            a=self.a.apply_transform(transform),
            b=self.b.apply_transform(transform),
        )


@dataclass(frozen=True)
class ShapeGroup:
    shapes: list[ShapeItem]

    def apply_transform(self, transform: Transformation) -> ShapeGroup:
        return ShapeGroup(
            shapes=[shape.apply_transform(transform) for shape in self.shapes]
        )


def diff(a: Point, b: Point) -> Vector:
    return Vector(x=b.x - a.x, y=b.y - a.y)


def sum_vec(*vectors: Vector) -> Vector:
    x = sum(v.x for v in vectors)
    y = sum(v.y for v in vectors)
    return Vector(x=x, y=y)
