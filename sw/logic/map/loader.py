import json
from pathlib import Path
from typing import Any, Union

from geometry.shapes import Line, Point, ShapeGroup, ShapeItem


def load_world_from_json(path: Union[str, Path]) -> ShapeGroup:
    file_path = Path(path)
    with file_path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)

    walls = payload["walls"]

    segments: list[ShapeItem] = [
        _segment_from_wall(wall) for wall in walls
    ]
    return ShapeGroup(shapes=segments)


def _segment_from_wall(wall: Any) -> Line:
    return Line(
        a=Point(
            float(wall["x1_m"]),
            float(wall["y1_m"]),
        ),
        b=Point(
            float(wall["x2_m"]),
            float(wall["y2_m"]),
        ),
    )
