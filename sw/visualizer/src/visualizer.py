from abc import ABC, abstractmethod
import pygame
from typing import Iterator


class Color(ABC):
    @abstractmethod
    def rgb(self) -> tuple[int, int, int]:
        raise NotImplementedError()

    def __iter__(self) -> Iterator[int]:
        return iter(self.rgb())


class Rgb(Color):
    def __init__(self, color: tuple[int, int, int]):
        self.r, self.g, self.b = color

    def rgb(self) -> tuple[int, int, int]:
        return self.r, self.g, self.b

    def __repr__(self) -> str:
        return f"Rgb({self.r}, {self.g}, {self.b})"


class Hsl(Color):
    def __init__(self, h: int, s: int, l_: int):
        self.h = h
        self.s = s
        self.l_ = l_

    def rgb(self) -> tuple[int, int, int]:
        h = self.h / 360
        s = self.s / 100
        l_ = self.l_ / 100
        if s == 0:
            return int(l_ * 255), int(l_ * 255), int(l_ * 255)
        if l_ < 0.5:
            q = l_ * (1 + s)
        else:
            q = l_ + s - l_ * s
        p = 2 * l_ - q
        r = self._hue_to_rgb(p, q, h + 1 / 3)
        g = self._hue_to_rgb(p, q, h)
        b = self._hue_to_rgb(p, q, h - 1 / 3)
        return int(r * 255), int(g * 255), int(b * 255)

    def _hue_to_rgb(self, p: float, q: float, t: float) -> float:
        if t < 0:
            t += 1
        if t > 1:
            t -= 1
        if t < 1 / 6:
            return p + (q - p) * 6 * t
        if t < 1 / 2:
            return q
        if t < 2 / 3:
            return p + (q - p) * (2 / 3 - t) * 6
        return p

    def __repr__(self) -> str:
        return f"Hsl({self.h}, {self.s}, {self.l_})"


BLACK = Rgb((0, 0, 0))


####################


class Vector:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __repr__(self) -> str:
        return f"Vector({self.x}, {self.y})"

    def __add__(self, other: "Vector") -> "Vector":
        return Vector(self.x + other.x, self.y + other.y)

    def __mul__(self, scalar: float) -> "Vector":
        return Vector(self.x * scalar, self.y * scalar)

    def __iter__(self) -> Iterator[float]:
        yield self.x
        yield self.y


class Graphics(ABC):
    @abstractmethod
    def draw(self, surface: pygame.Surface) -> None:
        pass

    @abstractmethod
    def translate(self, vec: Vector) -> "Graphics":
        pass

    @abstractmethod
    def scale(self, factor: float) -> "Graphics":
        pass


class Point(Graphics):
    def __init__(self, x: float, y: float, color: Color = BLACK, thickness: int = 1):
        self.x = x
        self.y = y
        self.color = color
        self.thickness = thickness

    def __repr__(self) -> str:
        return f"Point({self.x}, {self.y}, {self.color}, {self.thickness})"

    def translate(self, vec: Vector) -> "Point":
        return Point(self.x + vec.x, self.y + vec.y, self.color, self.thickness)

    def scale(self, factor: float) -> "Point":
        return Point(self.x * factor, self.y * factor, self.color, self.thickness)

    def __add__(self, vec: Vector) -> "Point":
        return Point(self.x + vec.x, self.y + vec.y, self.color, self.thickness)

    def __iter__(self) -> Iterator[float]:
        yield self.x
        yield self.y

    def draw(self, surface: pygame.Surface) -> None:
        surface.set_at((int(self.x), int(self.y)), tuple(self.color))


class Line(Graphics):
    def __init__(self, start: Point, end: Point, color: Color = BLACK, thickness: int = 1):
        self.start = start
        self.end = end
        self.color = color
        self.thickness = thickness

    def __repr__(self) -> str:
        return f"Line({self.start}, {self.end}, {self.color}, {self.thickness})"

    def translate(self, vec: Vector) -> "Line":
        return Line(self.start + vec, self.end + vec, self.color, self.thickness)

    def scale(self, factor: float) -> "Line":
        return Line(self.start.scale(factor), self.end.scale(factor), self.color, self.thickness)

    def draw(self, surface: pygame.Surface) -> None:
        pygame.draw.line(surface, tuple(self.color), tuple(self.start), tuple(self.end), self.thickness)


class Circle(Graphics):
    def __init__(self, center: Point, radius: float, color: Color = BLACK, thickness: int = 1):
        self.center = center
        self.radius = radius
        self.color = color
        self.thickness = thickness

    def __repr__(self) -> str:
        return f"Circle({self.center}, {self.radius}, {self.color}, {self.thickness})"

    def translate(self, vec: Vector) -> "Circle":
        return Circle(self.center + vec, self.radius, self.color, self.thickness)

    def scale(self, factor: float) -> "Circle":
        return Circle(self.center.scale(factor), self.radius * factor, self.color, self.thickness)

    def draw(self, surface: pygame.Surface) -> None:
        pygame.draw.circle(surface, tuple(self.color), tuple(self.center), self.radius, self.thickness)


class Rectangle(Graphics):
    def __init__(self, p1: Point, p2: Point, color: Color = BLACK, thickness: int = 1):
        self.p1 = p1
        self.p2 = p2
        self.color = color
        self.thickness = thickness

    def __repr__(self) -> str:
        return f"Rectangle({self.p1}, {self.p2}, {self.color}, {self.thickness})"

    def translate(self, vec: Vector) -> "Rectangle":
        return Rectangle(self.p1 + vec, self.p2 + vec, self.color, self.thickness)

    def scale(self, factor: float) -> "Rectangle":
        return Rectangle(self.p1.scale(factor), self.p2.scale(factor), self.color, self.thickness)

    def draw(self, surface: pygame.Surface) -> None:
        width = self.p2.x - self.p1.x
        height = self.p2.y - self.p1.y

        pygame.draw.rect(surface, tuple(self.color), (self.p1.x, self.p1.y, width, height), self.thickness)


####################


class Value:
    pass


####################


class Viz:
    def __init__(self, width: int, height: int):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        self.running = True

        self._layers: dict[int, pygame.Surface] = {}
        self._kvs: dict[str, Graphics | Value] = {}

    def clear_screen(self) -> None:
        self._layers.clear()

    def clear_layer(self, layer: int) -> None:
        if layer in self._layers:
            del self._layers[layer]

    def draw(self, graphics: Graphics, layer: int) -> None:
        if layer not in self._layers:
            self._layers[layer] = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)

        graphics.draw(self._layers[layer])

    def kv(self, id: str, value: Value | None) -> None:
        if value is None:
            del self._kvs[id]
        else:
            self._kvs[id] = value

    def run(self) -> None:
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                    continue

            self.screen.fill((255, 255, 255))

            for layer in sorted(self._layers.keys()):
                self.screen.blit(self._layers[layer], (0, 0))

            pygame.display.flip()

        pygame.quit()
