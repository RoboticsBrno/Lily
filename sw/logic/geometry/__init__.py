"""Generic geometry models."""

from .raycast import (
    RaycastGroup,
    RaycastGroupItem,
    RaycastHit,
    raycast,
    raycast_from_angle,
)
from .shapes import (
    Circle,
    Line,
    Point,
    ShapeGroup,
    Vector,
)
from .util import (
    angular_distance,
    dist2,
    find_nearest,
    shape_bounds,
    wrap_angle,
)

LineSegment = Line

__all__ = [
    "LineSegment",
    "Line",
    "Circle",
    "Point",
    "Vector",
    "ShapeGroup",
    "RaycastGroup",
    "RaycastGroupItem",
    "RaycastHit",
    "raycast",
    "raycast_from_angle",
    "wrap_angle",
    "dist2",
    "angular_distance",
    "find_nearest",
    "shape_bounds",
]
