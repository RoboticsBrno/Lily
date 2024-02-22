from abc import ABC, abstractmethod
import math
from typing import Optional, Union, overload
from vizctl import VizCtl


class Drawable(ABC):
    @abstractmethod
    def draw(self, viz: VizCtl, layer: int, color: str = 'black') -> None:
        pass


COLORS = {
    'black': (0, 0, 0),
    'white': (7, 7, 3),
    'red': (7, 0, 0),
    'green': (0, 7, 0),
    'blue': (0, 0, 3),
    'yellow': (7, 7, 0),
    'cyan': (0, 7, 3),
    'magenta': (7, 0, 3)
}


Num = int | float


class Point:
    def __init__(self, x: Num, y: Num):
        self.x = x
        self.y = y

    def __repr__(self) -> str:
        return f'Point({self.x}, {self.y})'

    def __add__(self, other: 'Vector') -> 'Point':
        return Point(self.x + other.x, self.y + other.y)

    @overload
    def __sub__(self, other: 'Point') -> 'Vector': ...
    @overload
    def __sub__(self, other: 'Vector') -> 'Point': ...

    def __sub__(self, other: Union['Point', 'Vector']) -> Union['Point', 'Vector']:
        if isinstance(other, Point):
            return Vector(self.x - other.x, self.y - other.y)

        return Point(self.x - other.x, self.y - other.y)

    def tup(self) -> tuple[Num, Num]:
        return (self.x, self.y)

    def draw(self, viz: VizCtl, layer: int, color: str = 'black', radius: int = 1) -> None:
        viz.draw_point(round(self.x), round(self.y), layer, COLORS[color], radius)


class Vector:
    def __init__(self, x: Num, y: Num):
        self.x = x
        self.y = y

    def __repr__(self) -> str:
        return f'Point({self.x}, {self.y})'

    def __add__(self, other: 'Vector') -> 'Vector':
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Vector') -> 'Vector':
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: Num) -> 'Vector':
        return Vector(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar: Num) -> 'Vector':
        return Vector(self.x / scalar, self.y / scalar)

    def __floordiv__(self, scalar: Num) -> 'Vector':
        return Vector(self.x // scalar, self.y // scalar)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Vector):
            raise NotImplementedError

        return self.x == other.x and self.y == other.y

    def tup(self) -> tuple[Num, Num]:
        return (self.x, self.y)

    def angle(self) -> float:
        return math.atan2(self.y, self.x)

    def length(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def length2(self) -> float:
        return self.x ** 2 + self.y ** 2

    def dot(self, other: 'Vector') -> float:
        return self.x * other.x + self.y * other.y

    def cross(self, other: 'Vector') -> float:
        return self.x * other.y - self.y * other.x

    def normalize(self) -> 'Vector':
        return self / self.length()

    def normal(self) -> 'Vector':
        return Vector(-self.y, self.x)

    def rotate(self, angle: float) -> 'Vector':
        cos = math.cos(angle)
        sin = math.sin(angle)
        return Vector(
            self.x * cos - self.y * sin,
            self.x * sin + self.y * cos
        )


class Obstacle(Drawable):
    @abstractmethod
    def raycast(self, start: Point, direction: Vector) -> Optional[tuple[Point, float]]:
        '''
        Returns the intersection point and the angle of the ray
        '''
        pass

    @abstractmethod
    def translate(self, vec: Vector) -> 'Obstacle':
        pass


class Line(Obstacle):
    def __init__(self, start: Point, end: Point):
        self.start = start
        self.end = end

    def __repr__(self) -> str:
        return f'Line({self.start}, {self.end})'

    def draw(self, viz: VizCtl, layer: int, color: str = 'black', thickness: int = 1) -> None:
        viz.draw_line(round(self.start.x), round(self.start.y), round(self.end.x), round(self.end.y),
                      layer, COLORS[color], thickness)

    def raycast(self, start: Point, direction: Vector) -> Optional[tuple[Point, float]]:
        '''
        Returns the intersection point and the angle of the ray
        '''
        # https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
        x1, y1 = self.start.tup()
        x2, y2 = self.end.tup()
        x3, y3 = start.tup()
        x4, y4 = (start + direction).tup()
        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        # If the lines are parallel
        if denominator == 0:
            return None

        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator
        u = ((x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)) / denominator

        # If the intersection is not on the line segment
        if not (0 <= t <= 1) or not (0 <= u):
            return None

        line_direction = self.end - self.start

        return start + direction * u, math.acos(line_direction.normalize().dot(direction.normalize()))

    def translate(self, vec: Vector) -> 'Line':
        return Line(self.start + vec, self.end + vec)


class Square(Obstacle):
    def __init__(self, a: Point, b: Point):
        x1, y1 = a.tup()
        x2, y2 = b.tup()
        self.a = Point(min(x1, x2), min(y1, y2))
        self.b = Point(max(x1, x2), max(y1, y2))

    def __repr__(self) -> str:
        return f'Square({self.a}, {self.b})'

    def draw(self, viz: VizCtl, layer: int, color: str = 'black', thickness: int = 1) -> None:
        viz.draw_rectangle(round(self.a.x), round(self.a.y), round(self.b.x), round(self.b.y),
                           layer, COLORS[color], thickness)

    def raycast(self, start: Point, direction: Vector) -> Optional[tuple[Point, float]]:
        points = [
            self.a,
            Point(self.a.x, self.b.y),
            self.b,
            Point(self.b.x, self.a.y)
        ]

        min_distance = float('inf')
        min_intersection = None

        for i in range(4):
            line = Line(points[i], points[(i + 1) % 4])
            res = line.raycast(start, direction)
            if res is not None and res[1] < min_distance:
                point, dst = res
                if self.a.x <= point.x <= self.b.x and self.a.y <= point.y <= self.b.y:
                    min_distance = dst
                    min_intersection = point

        if min_intersection is None:
            return None

        return min_intersection, min_distance


class Circle(Obstacle):
    def __init__(self, center: Point, radius: int):
        self.center = center
        self.radius = radius

    def __repr__(self) -> str:
        return f'Circle({self.center}, {self.radius})'

    def draw(self, viz: VizCtl, layer: int, color: str = 'black', thickness: int = 1) -> None:
        viz.draw_circle(round(self.center.x), round(self.center.y), self.radius, layer, COLORS[color], thickness)

    def raycast(self, start: Point, direction: Vector) -> Optional[tuple[Point, float]]:
        # https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
        a = direction.length2()
        b = 2 * direction.dot(start - self.center)
        c = (start - self.center).length2() - self.radius ** 2
        discriminant = b ** 2 - 4 * a * c
        if discriminant < 0:
            return None
        t = (-b - math.sqrt(discriminant)) / (2 * a)

        if t < 0:
            return None

        angle = math.acos(direction.normalize().dot((start + direction * t - self.center).normalize().normal()))

        return start + direction * t, angle

    def translate(self, vec: Vector) -> 'Circle':
        return Circle(self.center + vec, self.radius)
