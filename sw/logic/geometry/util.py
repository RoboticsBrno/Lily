from math import pi, sqrt

from geometry.shapes import Circle, Line, Point, ShapeGroup, ShapeItem
from geometry.transforms import Pose


def wrap_angle(angle: float) -> float:
    """Wrap an angle in radians to the interval [-pi, pi)."""
    return (angle + pi) % (2.0 * pi) - pi


def wrap_angle_pos(angle: float) -> float:
    """Wrap an angle in radians to the interval [0, 2*pi)."""
    return angle % (2.0 * pi)


def angular_distance(from_angle: float, to_angle: float) -> float:
    """Return the shortest signed rotation from from_angle to to_angle."""
    return wrap_angle(to_angle - from_angle)


def angular_distance_pos(from_angle: float, to_angle: float) -> float:
    """Return the shortest positive rotation from from_angle to to_angle."""
    return wrap_angle_pos(to_angle - from_angle)


def dist2(p1: Point, p2: Point) -> float:
    """Return the squared distance between two points."""
    return (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2


def lerp(a: Pose, b: Pose, t: float) -> Pose:
    """Linearly interpolate between two poses."""
    return Pose(
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.yaw + t * wrap_angle(b.yaw - a.yaw),
    )


def lerp_pt(a: Point, b: Point, t: float) -> Point:
    """Linearly interpolate between two points."""
    return Point(
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
    )


def shape_bounds(shape: ShapeItem) -> tuple[float, float, float, float]:
    if isinstance(shape, Point):
        return shape.x, shape.y, shape.x, shape.y
    if isinstance(shape, Line):
        return (
            min(shape.a.x, shape.b.x),
            min(shape.a.y, shape.b.y),
            max(shape.a.x, shape.b.x),
            max(shape.a.y, shape.b.y),
        )
    if isinstance(shape, Circle):
        return (
            shape.center.x - shape.radius,
            shape.center.y - shape.radius,
            shape.center.x + shape.radius,
            shape.center.y + shape.radius,
        )
    if isinstance(shape, ShapeGroup):
        min_x = float("inf")
        min_y = float("inf")
        max_x = float("-inf")
        max_y = float("-inf")
        has_shape = False

        for s in shape.shapes:
            has_shape = True
            shape_min_x, shape_min_y, shape_max_x, shape_max_y = shape_bounds(s)
            min_x = min(min_x, shape_min_x)
            min_y = min(min_y, shape_min_y)
            max_x = max(max_x, shape_max_x)
            max_y = max(max_y, shape_max_y)

        if not has_shape:
            raise ValueError("shapes must contain at least one shape")

        return min_x, min_y, max_x, max_y
    raise TypeError(f"Unsupported shape type: {type(shape)!r}")


def find_nearest(
    input: ShapeItem,
    point: Point,
) -> tuple[ShapeItem, float]:
    nearest_shape: ShapeItem | None = None
    nearest_dist2 = float("inf")
    pending: list[ShapeItem] = [input]

    while pending:
        shape = pending.pop()
        if isinstance(shape, ShapeGroup):
            pending.extend(shape.shapes)
            continue

        if isinstance(shape, Point):
            dx = point.x - shape.x
            dy = point.y - shape.y
            candidate_dist2 = dx * dx + dy * dy
        elif isinstance(shape, Circle):
            dx = point.x - shape.center.x
            dy = point.y - shape.center.y
            center_distance = sqrt(dx * dx + dy * dy)
            distance = max(0.0, center_distance - shape.radius)
            candidate_dist2 = distance * distance
        elif isinstance(shape, Line):
            ax, ay = shape.a.x, shape.a.y
            bx, by = shape.b.x, shape.b.y
            px, py = point.x, point.y

            abx = bx - ax
            aby = by - ay
            apx = px - ax
            apy = py - ay
            ab_len2 = abx * abx + aby * aby

            if ab_len2 == 0.0:
                dx = apx
                dy = apy
                candidate_dist2 = dx * dx + dy * dy
            else:
                t = (apx * abx + apy * aby) / ab_len2
                t = max(0.0, min(1.0, t))
                closest_x = ax + t * abx
                closest_y = ay + t * aby
                dx = px - closest_x
                dy = py - closest_y
                candidate_dist2 = dx * dx + dy * dy
        else:
            raise TypeError(f"Unsupported shape type: {type(shape)!r}")

        if candidate_dist2 < nearest_dist2:
            nearest_shape = shape
            nearest_dist2 = candidate_dist2

    if nearest_shape is None:
        raise ValueError("group must contain at least one shape")

    return nearest_shape, sqrt(nearest_dist2)
