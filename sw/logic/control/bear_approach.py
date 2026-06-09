from __future__ import annotations
import math

from geometry.shapes import Point
from geometry.util import in_square
from map.types import VectorMap


WALL_THRESHOLD = 0.25
WALL_APPROACH_DISTANCE = 0.40
CORNER_OFFSET = 0.15  # side offset for corner goal
CORNER_APPROACH_PT_A = (CORNER_OFFSET, 0.00)  # side, fwd
CORNER_APPROACH_PT_B = (CORNER_OFFSET, 0.30)  # side, fwd
CORNER_APPROACH_PT_C = (0.40, 0.80)  # side, fwd
WALL_OFFSET = 0.10


def n_approach(start: Point, detected_bear: Point) -> list[Point]:
    return [
        Point(detected_bear.x, 2.80 - WALL_APPROACH_DISTANCE),
        Point(detected_bear.x, 2.80 - WALL_OFFSET)
    ]


def s_approach(start: Point, detected_bear: Point) -> list[Point]:
    return [
        Point(detected_bear.x, 1.40 + WALL_APPROACH_DISTANCE),
        Point(detected_bear.x, 1.40 + WALL_OFFSET)
    ]


def w_approach(start: Point, detected_bear: Point) -> list[Point]:
    return [
        Point(0.00 + WALL_APPROACH_DISTANCE, detected_bear.y),
        Point(0.00 + WALL_OFFSET, detected_bear.y)
    ]


def e_approach(start: Point, detected_bear: Point) -> list[Point]:
    return [
        Point(1.40 - WALL_APPROACH_DISTANCE, detected_bear.y),
        Point(1.40 - WALL_OFFSET, detected_bear.y)
    ]


def nw_approach(start: Point, detected_bear: Point) -> list[Point]:
    off_x = start.x
    off_y = 2.80 - start.y
    a, b, c = (
        (CORNER_APPROACH_PT_A, CORNER_APPROACH_PT_B, CORNER_APPROACH_PT_C)
        if off_x < off_y
        else (tuple(reversed(CORNER_APPROACH_PT_A)), tuple(reversed(CORNER_APPROACH_PT_B)), tuple(reversed(CORNER_APPROACH_PT_C)))
    )

    res = [
        Point(0.00 + c[0], 2.80 - c[1]),
        Point(0.00 + b[0], 2.80 - b[1]),
        Point(0.00 + a[0], 2.80 - a[1]),
    ]
    if off_x - c[0] < off_y - c[1]:
        res.pop(0)

    return res


def sw_approach(start: Point, detected_bear: Point) -> list[Point]:
    off_x = start.x
    off_y = start.y - 1.40
    a, b, c = (
        (CORNER_APPROACH_PT_A, CORNER_APPROACH_PT_B, CORNER_APPROACH_PT_C)
        if off_x < off_y
        else (tuple(reversed(CORNER_APPROACH_PT_A)), tuple(reversed(CORNER_APPROACH_PT_B)), tuple(reversed(CORNER_APPROACH_PT_C)))
    )

    res = [
        Point(0.00 + c[0], 1.40 + c[1]),
        Point(0.00 + b[0], 1.40 + b[1]),
        Point(0.00 + a[0], 1.40 + a[1]),
    ]
    if off_x - c[0] < off_y - c[1]:
        res.pop(0)

    return res


def ne_approach(start: Point, detected_bear: Point) -> list[Point]:
    off_x = 1.40 - start.x
    off_y = 2.80 - start.y
    a, b, c = (
        (CORNER_APPROACH_PT_A, CORNER_APPROACH_PT_B, CORNER_APPROACH_PT_C)
        if off_x < off_y
        else (tuple(reversed(CORNER_APPROACH_PT_A)), tuple(reversed(CORNER_APPROACH_PT_B)), tuple(reversed(CORNER_APPROACH_PT_C)))
    )

    res = [
        Point(1.40 - c[0], 2.80 - c[1]),
        Point(1.40 - b[0], 2.80 - b[1]),
        Point(1.40 - a[0], 2.80 - a[1]),
    ]
    if off_x - c[0] < off_y - c[1]:
        res.pop(0)

    return res


WALLS_CORNERS = {  # N, S, W, E
    (True, False, False, False): n_approach,
    (False, True, False, False): s_approach,
    (False, False, True, False): w_approach,
    (False, False, False, True): e_approach,
    (True, False, True, False): nw_approach,
    (True, False, False, True): ne_approach,
    (False, True, True, False): sw_approach,
}


def plan_bear_approach_path(start: Point, detected_bear: Point, world: VectorMap) -> list[Point]:
    if in_square(Point(1.25, 1.20), Point(1.40, 2.6), detected_bear):  # entrance wall
        return [start, Point(1.25, detected_bear.y - 0.30), Point(1.25, detected_bear.y)]

    if in_square(Point(1.00, 1.40), Point(1.4, 2.6), detected_bear):  # entrance
        return [start, detected_bear]

    at_wall_n = detected_bear.y > (2.80 - WALL_THRESHOLD)
    at_wall_s = detected_bear.y < (1.40 + WALL_THRESHOLD)
    at_wall_w = detected_bear.x < (0.00 + WALL_THRESHOLD)
    at_wall_e = detected_bear.x > (1.40 - WALL_THRESHOLD)

    if any((at_wall_n, at_wall_s, at_wall_w, at_wall_e)):  # corner/edge
        approach_fn = WALLS_CORNERS[(at_wall_n, at_wall_s, at_wall_w, at_wall_e)]
        return [start] + approach_fn(start, detected_bear)

    return [start, detected_bear]  # center
