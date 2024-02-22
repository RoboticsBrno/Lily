from geometry import Point, Vector
from typing import Optional, Protocol


class World(Protocol):
    def get_pos(self) -> Point: ...
    def get_heading(self) -> Vector: ...
    def raycast(self, origin: Point, direction: Vector) -> Optional[tuple[Point, float, float]]: ...


class Link(Protocol):
    def read_all(self) -> bytes: ...
    def write(self, data: bytes) -> int: ...
