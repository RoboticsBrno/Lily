from __future__ import annotations
import math

from geometry.shapes import Point
from geometry.util import in_square
from params import ROBOT_BODY_RADIUS
from map.types import VectorMap


WALL_THRESHOLD = 0.25
APPROACH_DISTANCE = 0.40
APPROACH_ANGLE_CORNER = math.pi / 6
APPROACH_DISTANCE_CORNER_SMALL = APPROACH_DISTANCE * math.cos(APPROACH_ANGLE_CORNER)
APPROACH_DISTANCE_CORNER_LARGE = APPROACH_DISTANCE * math.sin(APPROACH_ANGLE_CORNER)
CORNER_OFFSET = 0.14
WALL_OFFSET = 0.15


def n_approach(detected_bear: Point) -> list[Point]:
    return [
        Point(detected_bear.x, 2.80 - APPROACH_DISTANCE),
        Point(detected_bear.x, 2.80 - WALL_OFFSET)
    ]


def s_approach(detected_bear: Point) -> list[Point]:
    return [
        Point(detected_bear.x, 1.40 + APPROACH_DISTANCE),
        Point(detected_bear.x, 1.40 + WALL_OFFSET)
    ]


def w_approach(detected_bear: Point) -> list[Point]:
    return [
        Point(0.00 + APPROACH_DISTANCE, detected_bear.y),
        Point(0.00 + WALL_OFFSET, detected_bear.y)
    ]


def e_approach(detected_bear: Point) -> list[Point]:
    return [
        Point(1.40 - APPROACH_DISTANCE, detected_bear.y),
        Point(1.40 - WALL_OFFSET, detected_bear.y)
    ]


def nw_approach(detected_bear: Point) -> list[Point]:
    off_x = detected_bear.x
    off_y = detected_bear.y - 2.80
    a, b = (
        (APPROACH_DISTANCE_CORNER_LARGE, APPROACH_DISTANCE_CORNER_SMALL)
        if off_y > off_x
        else (APPROACH_DISTANCE_CORNER_SMALL, APPROACH_DISTANCE_CORNER_LARGE)
    )

    return [
        Point(0.00 + a, 2.80 - b),
        Point(0.00 + CORNER_OFFSET, 2.80 - CORNER_OFFSET),
    ]


def sw_approach(detected_bear: Point) -> list[Point]:
    off_x = detected_bear.x
    off_y = detected_bear.y - 1.40
    a, b = (
        (APPROACH_DISTANCE_CORNER_LARGE, APPROACH_DISTANCE_CORNER_SMALL)
        if off_y < off_x
        else (APPROACH_DISTANCE_CORNER_SMALL, APPROACH_DISTANCE_CORNER_LARGE)
    )
    return [
        Point(0.00 + a, 1.40 + b),
        Point(0.00 + CORNER_OFFSET, 1.40 + CORNER_OFFSET),
    ]


def ne_approach(detected_bear: Point) -> list[Point]:
    off_x = detected_bear.x - 1.40
    off_y = detected_bear.y - 2.80
    a, b = (
        (APPROACH_DISTANCE_CORNER_LARGE, APPROACH_DISTANCE_CORNER_SMALL)
        if off_y > -off_x
        else (APPROACH_DISTANCE_CORNER_SMALL, APPROACH_DISTANCE_CORNER_LARGE)
    )
    return [
        Point(1.40 - a, 2.80 - b),
        Point(1.40 - CORNER_OFFSET, 2.80 - CORNER_OFFSET),
    ]


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
        return [start] + approach_fn(detected_bear)

    return [start, detected_bear]  # center
