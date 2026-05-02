from __future__ import annotations

from dataclasses import dataclass
from math import acos, cos, inf, isfinite, pi, sin, sqrt
from typing import Optional, Union

from .shapes import Circle, Line, Point, ShapeGroup, ShapeItem, Vector


EPSILON = 1e-9


@dataclass(frozen=True)
class RaycastHit:
    point: Point
    distance: float
    shape: ShapeItem
    angle_of_incidence: Optional[float]


@dataclass(frozen=True)
class RaycastGroupItem:

    shape: ShapeItem
    max_angle_of_incidence: float = pi / 2.0

    def __post_init__(self) -> None:
        if self.max_angle_of_incidence < 0.0 or self.max_angle_of_incidence > (pi / 2.0):
            raise ValueError("max_angle_of_incidence must be in [0, pi/2]")


@dataclass(frozen=True)
class RaycastGroup:
    items: list[RaycastGroupItem]


RaycastTarget = Union[ShapeItem, RaycastGroup]


def raycast(
    shape: RaycastTarget,
    origin: Point,
    direction: Vector,
    max_distance: float = inf,
) -> Optional[RaycastHit]:
    if direction.magnitude() < EPSILON:
        raise ValueError("Ray direction must be non-zero")
    if max_distance < 0.0:
        raise ValueError("max_distance must be non-negative")

    if isinstance(shape, Point):
        return _raycast_point(shape, origin, direction, max_distance)
    if isinstance(shape, Line):
        return _raycast_line(shape, origin, direction, max_distance)
    if isinstance(shape, Circle):
        return _raycast_circle(shape, origin, direction, max_distance)
    if isinstance(shape, RaycastGroup):
        return _raycast_group_with_incidence_limits(shape, origin, direction, max_distance)
    if isinstance(shape, ShapeGroup):
        return _raycast_group(shape, origin, direction, max_distance)
    return None


def raycast_from_angle(
    shape: RaycastTarget,
    origin: Point,
    angle_radians: float,
    max_distance: float = inf,
) -> Optional[RaycastHit]:
    return raycast(
        shape=shape,
        origin=origin,
        direction=Vector(cos(angle_radians), sin(angle_radians)),
        max_distance=max_distance,
    )


def _raycast_group(
    group: ShapeGroup,
    origin: Point,
    direction: Vector,
    max_distance: float,
) -> Optional[RaycastHit]:
    best_hit: Optional[RaycastHit] = None
    current_limit = max_distance
    for item in group.shapes:
        hit = raycast(item, origin, direction, max_distance=current_limit)
        if hit is None:
            continue
        if best_hit is None or hit.distance < best_hit.distance:
            best_hit = hit
            current_limit = hit.distance
    return best_hit


def _raycast_group_with_incidence_limits(
    group: RaycastGroup,
    origin: Point,
    direction: Vector,
    max_distance: float,
) -> Optional[RaycastHit]:
    nearest_hit: Optional[RaycastHit] = None
    nearest_max_incidence: Optional[float] = None
    current_limit = max_distance
    for item in group.items:
        hit = raycast(item.shape, origin, direction, max_distance=current_limit)
        if hit is None:
            continue
        if nearest_hit is None or hit.distance < nearest_hit.distance:
            nearest_hit = hit
            nearest_max_incidence = item.max_angle_of_incidence
            current_limit = hit.distance

    if nearest_hit is None:
        return None

    incidence = nearest_hit.angle_of_incidence
    if incidence is None:
        return None
    if nearest_max_incidence is not None and incidence > nearest_max_incidence:
        return None
    return nearest_hit


def _raycast_point(
    target: Point,
    origin: Point,
    direction: Vector,
    max_distance: float,
) -> Optional[RaycastHit]:
    to_target = Vector.from_points(origin, target)
    cross = to_target.cross(direction)
    if abs(cross) > EPSILON:
        return None

    denom = direction.dot(direction)
    t = to_target.dot(direction) / denom
    if t < 0.0:
        return None

    return _hit_from_t(target, t, direction, max_distance)


def _raycast_line(
    segment: Line,
    origin: Point,
    direction: Vector,
    max_distance: float,
) -> Optional[RaycastHit]:
    segment_vector = Vector.from_points(segment.a, segment.b)
    q_minus_p = Vector.from_points(origin, segment.a)

    r_cross_s = direction.cross(segment_vector)
    q_minus_p_cross_r = q_minus_p.cross(direction)

    if abs(r_cross_s) < EPSILON:
        # if collinear, choose the nearest endpoint in front.
        if abs(q_minus_p_cross_r) >= EPSILON:
            return None
        best_hit: Optional[RaycastHit] = None
        for endpoint in (segment.a, segment.b):
            candidate = _raycast_point(endpoint, origin, direction, max_distance)
            if candidate is not None and (best_hit is None or candidate.distance < best_hit.distance):
                best_hit = candidate
        return best_hit

    t = q_minus_p.cross(segment_vector) / r_cross_s
    u = q_minus_p_cross_r / r_cross_s
    if t < 0.0 or u < -EPSILON or u > 1.0 + EPSILON:
        return None

    hit_point = origin.translated(direction.scaled(t))
    sx, sy = segment_vector.to_tuple()
    incidence = _incidence_angle_from_normal(direction, Vector(sy, -sx))
    return _hit_from_t(
        hit_point,
        t,
        direction,
        max_distance,
        segment,
        angle_of_incidence=incidence,
    )


def _raycast_circle(
    circle: Circle,
    origin: Point,
    direction: Vector,
    max_distance: float,
) -> Optional[RaycastHit]:
    center_to_origin = Vector.from_points(circle.center, origin)

    a = direction.dot(direction)
    b = 2.0 * center_to_origin.dot(direction)
    c = center_to_origin.dot(center_to_origin) - circle.radius * circle.radius

    discriminant = b * b - 4.0 * a * c
    if discriminant < -EPSILON:
        return None
    discriminant = max(discriminant, 0.0)
    root = sqrt(discriminant)

    t1 = (-b - root) / (2.0 * a)
    t2 = (-b + root) / (2.0 * a)
    candidates = [t for t in (t1, t2) if t >= 0.0]
    if not candidates:
        return None

    t = min(candidates)
    hit_point = origin.translated(direction.scaled(t))
    normal = Vector.from_points(circle.center, hit_point)
    incidence = _incidence_angle_from_normal(direction, normal)
    return _hit_from_t(
        hit_point,
        t,
        direction,
        max_distance,
        circle,
        angle_of_incidence=incidence,
    )


def _incidence_angle_from_normal(
    direction: Vector,
    normal: Vector,
) -> Optional[float]:
    ray_norm = direction.magnitude()
    normal_norm = normal.magnitude()
    if ray_norm < EPSILON or normal_norm < EPSILON:
        return None

    dot = direction.dot(normal) / (ray_norm * normal_norm)
    # Use absolute value so incidence is independent of chosen normal orientation.
    clamped = min(1.0, max(0.0, abs(dot)))
    return acos(clamped)


def _hit_from_t(
    point: Point,
    t: float,
    direction: Vector,
    max_distance: float,
    shape: Optional[ShapeItem] = None,
    angle_of_incidence: Optional[float] = None,
) -> Optional[RaycastHit]:
    distance = t * direction.magnitude()
    if isfinite(max_distance) and distance > max_distance + EPSILON:
        return None
    if distance < -EPSILON:
        return None
    return RaycastHit(
        point=point,
        distance=max(distance, 0.0),
        shape=shape if shape is not None else point,
        angle_of_incidence=angle_of_incidence,
    )
