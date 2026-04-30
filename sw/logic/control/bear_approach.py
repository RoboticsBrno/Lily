from __future__ import annotations

from geometry.shapes import Point
from map.types import VectorMap


def plan_bear_approach_path(start: Point, detected_bear: Point, world: VectorMap) -> list[Point]:
    return [start, detected_bear]
