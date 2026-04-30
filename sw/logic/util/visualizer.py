from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional, Tuple, Union

from geometry.shapes import Circle, Line, Point, ShapeGroup
from geometry.transforms import Transformation, scaling, translation

import pygame


Color = Tuple[int, int, int]
OnTickCallback = Callable[[float], None]
OnEventCallback = Callable[[Any], None]
Shape = Union[Circle, Point, Line, ShapeGroup]
Drawable = Shape


@dataclass
class CameraState:
    zoom_px_per_meter: float
    pan_x_px: float
    pan_y_px: float


@dataclass
class Visualizer:
    def __init__(
        self,
        on_tick: Optional[OnTickCallback] = None,
        on_event: Optional[OnEventCallback] = None,
        window_size: Tuple[int, int] = (1024, 768),
        title: str = "World Visualizer",
    ) -> None:
        self.on_tick = on_tick
        self.on_event = on_event
        self.window_size = window_size
        self.title = title

        self.background_color: Color = (16, 18, 22)
        self.wall_color: Color = (225, 228, 236)
        self.grid_color: Color = (40, 44, 52)

        self.min_zoom_px_per_meter = 25.0
        self.max_zoom_px_per_meter = 2500.0
        self.zoom_factor_per_step = 1.1

        self._dragging = False
        self._camera = CameraState(zoom_px_per_meter=100.0, pan_x_px=0.0, pan_y_px=0.0)
        self._screen: Any = None
        self._font: Any = None

    def run(self, target_fps: int = 60) -> None:
        pygame.init()
        screen = pygame.display.set_mode(self.window_size, pygame.RESIZABLE)
        pygame.display.set_caption(self.title)
        clock = pygame.time.Clock()
        self._screen = screen
        self._font = pygame.font.Font(None, 24)

        self._center_camera()

        running = True
        while running:
            dt_seconds = clock.tick(target_fps) / 1000.0

            self._clear_frame(screen)

            for event in pygame.event.get():
                if self.on_event is not None:
                    self.on_event(event)

                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.VIDEORESIZE:
                    self.window_size = (max(1, event.w), max(1, event.h))
                    screen = pygame.display.set_mode(self.window_size, pygame.RESIZABLE)
                    self._screen = screen
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:
                        self._dragging = True
                    elif event.button == 4:  # Wheel up (pygame < 2 fallback)
                        self._zoom_at_point(1.0, pygame.mouse.get_pos())
                    elif event.button == 5:  # Wheel down (pygame < 2 fallback)
                        self._zoom_at_point(-1.0, pygame.mouse.get_pos())
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        self._dragging = False
                elif event.type == pygame.MOUSEMOTION and self._dragging:
                    rel_x, rel_y = event.rel
                    self._camera.pan_x_px += rel_x
                    self._camera.pan_y_px += rel_y
                elif event.type == pygame.MOUSEWHEEL:
                    self._zoom_at_point(float(event.y), pygame.mouse.get_pos())

            self._draw_grid(screen)

            if self.on_tick is not None:
                self.on_tick(dt_seconds)
            pygame.display.flip()

        self._screen = None
        pygame.quit()

    def world_to_screen(self, x: float, y: float) -> Tuple[float, float]:
        return self._world_to_screen_transform().apply_to_point(x, y)

    def screen_to_world(self, x_px: float, y_px: float) -> Tuple[float, float]:
        return self._world_to_screen_transform().inverse().apply_to_point(x_px, y_px)

    def draw(
        self,
        shape: Drawable,
        color: Color = (255, 255, 255),
        width_px: int = 0,
        point_radius_px: int = 3,
    ) -> None:
        if self._screen is None:
            raise RuntimeError("draw() can only be used while the visualizer is running")

        if not isinstance(shape, (Circle, Point, Line, ShapeGroup)):
            raise TypeError("Unsupported shape type.")

        self._draw_shape(
            self._screen,
            shape,
            color=color,
            width_px=width_px,
            point_radius_px=max(1, point_radius_px),
        )

    def draw_text(
        self,
        text: str,
        x: float,
        y: float,
        color: Color = (255, 255, 255),
        font_size: int = 24,
        world_coordinates: bool = False,
    ) -> None:
        if self._screen is None:
            raise RuntimeError("draw_text() can only be used while the visualizer is running")

        if world_coordinates:
            x, y = self.world_to_screen(x, y)

        font = pygame.font.Font(None, font_size)
        text_surface = font.render(text, True, color)
        text_rect = text_surface.get_rect(topleft=(int(round(x)), int(round(y))))
        self._screen.blit(text_surface, text_rect)

    def _clear_frame(self, screen: Any) -> None:
        screen.fill(self.background_color)

    def _draw_shape(
        self,
        screen: Any,
        shape: Shape,
        color: Color,
        width_px: int,
        point_radius_px: int,
    ) -> None:
        if isinstance(shape, ShapeGroup):
            for child_shape in shape.shapes:
                self._draw_shape(
                    screen,
                    child_shape,
                    color=color,
                    width_px=width_px,
                    point_radius_px=point_radius_px,
                )
            return

        screen_shape = shape.apply_transform(self._world_to_screen_transform())

        if isinstance(screen_shape, Circle):
            center = (int(round(screen_shape.center.x)), int(round(screen_shape.center.y)))
            radius_px = max(1, int(round(screen_shape.radius)))
            normalized_width_px = max(1, width_px)
            pygame.draw.circle(
                screen,
                color,
                center,
                radius_px,
                width=normalized_width_px,
            )
        elif isinstance(screen_shape, Point):
            center = (int(round(screen_shape.x)), int(round(screen_shape.y)))
            pygame.draw.circle(
                screen,
                color,
                center,
                point_radius_px,
                width=0,
            )
        elif isinstance(screen_shape, Line):
            start = (int(round(screen_shape.a.x)), int(round(screen_shape.a.y)))
            end = (int(round(screen_shape.b.x)), int(round(screen_shape.b.y)))
            normalized_width_px = max(1, width_px if width_px > 0 else 2)
            pygame.draw.line(
                screen,
                color,
                start,
                end,
                width=normalized_width_px,
            )
        else:
            left = int(round(min(screen_shape.a.x, screen_shape.b.x)))
            top = int(round(min(screen_shape.a.y, screen_shape.b.y)))
            rect_width_px = max(1, int(round(abs(screen_shape.b.x - screen_shape.a.x))))
            rect_height_px = max(1, int(round(abs(screen_shape.b.y - screen_shape.a.y))))
            normalized_width_px = max(0, width_px)
            pygame.draw.rect(
                screen,
                color,
                pygame.Rect(left, top, rect_width_px, rect_height_px),
                width=normalized_width_px,
            )

    def _draw_grid(self, screen: Any) -> None:
        width, height = self.window_size
        meters_per_grid = 0.25
        step_px = self._camera.zoom_px_per_meter * meters_per_grid
        if step_px < 16:
            return

        start_x = self._camera.pan_x_px % step_px
        x = start_x
        while x <= width:
            pygame.draw.line(screen, self.grid_color, (x, 0), (x, height), width=1)
            x += step_px

        start_y = self._camera.pan_y_px % step_px
        y = start_y
        while y <= height:
            pygame.draw.line(screen, self.grid_color, (0, y), (width, y), width=1)
            y += step_px

    def _world_to_screen_transform(self) -> Transformation:
        return translation(self._camera.pan_x_px, self._camera.pan_y_px).compose(
            scaling(self._camera.zoom_px_per_meter, -self._camera.zoom_px_per_meter)
        )

    def _center_camera(self) -> None:
        width, height = self.window_size
        self._camera.pan_x_px = width / 2.0
        self._camera.pan_y_px = height / 2.0
        self._camera.zoom_px_per_meter = 100.0

    def _zoom_at_point(self, wheel_delta: float, mouse_pos: Tuple[int, int]) -> None:
        if wheel_delta == 0:
            return

        mouse_x, mouse_y = mouse_pos
        world_before = self.screen_to_world(float(mouse_x), float(mouse_y))

        if wheel_delta > 0:
            zoom_multiplier = self.zoom_factor_per_step ** wheel_delta
        else:
            zoom_multiplier = (1.0 / self.zoom_factor_per_step) ** (-wheel_delta)

        new_zoom = self._camera.zoom_px_per_meter * zoom_multiplier
        self._camera.zoom_px_per_meter = max(
            self.min_zoom_px_per_meter,
            min(self.max_zoom_px_per_meter, new_zoom),
        )

        # Keep the world point under the cursor fixed while zooming.
        self._camera.pan_x_px = mouse_x - world_before[0] * self._camera.zoom_px_per_meter
        self._camera.pan_y_px = mouse_y + world_before[1] * self._camera.zoom_px_per_meter
