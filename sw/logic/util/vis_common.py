from collections import deque
from math import cos, hypot, sin
from typing import Any

from geometry.shapes import Circle, Line, Point, Vector
from geometry.transforms import Pose
from localization.particle_filter import ParticleFilterLocalizer
from localization.bear_detector import BearDetector
from util.keyboard_controller import KeyboardRobotController
from util.visualizer import Visualizer

import pygame


MIN_BEAR_DIAMETER_M = 0.10
KEY_MAP = {
    pygame.K_w: "w",
    pygame.K_s: "s",
    pygame.K_a: "a",
    pygame.K_d: "d",
    pygame.K_UP: "up",
    pygame.K_DOWN: "down",
    pygame.K_LEFT: "left",
    pygame.K_RIGHT: "right",
    pygame.K_SPACE: "space",
    pygame.K_x: "x",
    pygame.K_o: "o",
    pygame.K_c: "c",
}


def handle_ui_control_event(
    event,
    visualizer: Visualizer,
    bear: Circle,
    resizing_bear_with_right_drag: bool,
) -> bool:
    if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
        world_x, world_y = visualizer.screen_to_world(float(event.pos[0]), float(event.pos[1]))
        object.__setattr__(bear, "center", Point(world_x, world_y))
        return True

    if event.type == pygame.MOUSEBUTTONUP and event.button == 3:
        print(
            f"Set bear position to ({bear.center.x:.2f}, {bear.center.y:.2f}) and radius to {bear.radius:.2f})"
        )
        return False

    if event.type == pygame.MOUSEMOTION and resizing_bear_with_right_drag:
        world_x, world_y = visualizer.screen_to_world(float(event.pos[0]), float(event.pos[1]))
        dx = world_x - bear.center.x
        dy = world_y - bear.center.y
        object.__setattr__(bear, "radius", max(MIN_BEAR_DIAMETER_M / 2.0, hypot(dx, dy)))
        return resizing_bear_with_right_drag

    return resizing_bear_with_right_drag


def handle_robot_control_event(
    event,
    controller: KeyboardRobotController,
) -> None:
    key_name = KEY_MAP.get(getattr(event, "key", -1))
    if key_name is not None:
        if event.type == pygame.KEYDOWN:
            controller.key_down(key_name)
        elif event.type == pygame.KEYUP:
            controller.key_up(key_name)


def draw_particles(visualizer: Visualizer, localizer: ParticleFilterLocalizer) -> None:
    for particle in localizer.particles:
        visualizer.draw(
            Circle(Point(particle.pose.x, particle.pose.y), radius=(particle.weight * len(localizer.particles)) / 300),
            color=(0, 100, 0),
        )
        visualizer.draw(
            Line(
                a=Point(particle.pose.x, particle.pose.y),
                b=Point(
                    particle.pose.x + 0.01 * cos(particle.pose.yaw),
                    particle.pose.y + 0.01 * sin(particle.pose.yaw),
                ),
            ),
            color=(0, 100, 0),
            width_px=1,
        )


def draw_lidar_history(
    visualizer: Visualizer,
    lidar_history: deque[tuple[Point, Vector]],
    show_magnitude: bool = False,
) -> None:
    max_feature = 0.05
    for lidar_point, feature in list(lidar_history):
        magnitude = feature.magnitude()
        normalized = 0.0 if max_feature <= 0.0 else max(0.0, min(1.0, magnitude / max_feature))
        red = int(255 * normalized)
        blue = int(255 * (1.0 - normalized))
        if show_magnitude:
            vector_tip = lidar_point.translated(feature.scaled(3))
            visualizer.draw(
                Line(a=lidar_point, b=vector_tip),
                color=(red, 0, blue),
                width_px=2,
            )
            visualizer.draw_text(
                f"{magnitude:.2f}",
                vector_tip.x + 0.02,
                vector_tip.y + 0.02,
                color=(red, 0, blue),
                font_size=18,
                world_coordinates=True,
            )
        else:
            visualizer.draw(
                lidar_point,
                color=(red, 0, blue),
                point_radius_px=3,
            )


def draw_estimated_pose(
    visualizer: Visualizer,
    estimated: Pose,
    body_radius: float,
) -> None:
    visualizer.draw(
        Circle(center=Point(estimated.x, estimated.y), radius=body_radius),
        color=(90, 255, 90),
        width_px=2,
    )
    visualizer.draw(
        Line(
            a=Point(estimated.x, estimated.y),
            b=Point(
                estimated.x + body_radius * cos(estimated.yaw),
                estimated.y + body_radius * sin(estimated.yaw),
            ),
        ),
        color=(90, 255, 200),
        width_px=2,
    )


def draw_bear_detection(visualizer: Visualizer, bear_detection: Any, estimated: Pose) -> None:
    if bear_detection is None:
        return

    detected_position = bear_detection.position
    visualizer.draw(
        bear_detection.position,
        color=(255, 70, 70),
        width_px=3,
    )
    visualizer.draw(
        detected_position,
        color=(255, 255, 120),
        point_radius_px=4,
    )
    visualizer.draw(
        Line(
            a=Point(estimated.x, estimated.y),
            b=detected_position,
        ),
        color=(255, 140, 140),
        width_px=1,
    )


def draw_bear(visualizer: Visualizer, bear: Circle) -> None:
    visualizer.draw(
        Circle(center=bear.center, radius=bear.radius),
        color=(190, 140, 70),
        width_px=2,
    )
    visualizer.draw(
        bear.center,
        color=(255, 220, 160),
        point_radius_px=3,
    )


def draw_candidate_points(visualizer: Visualizer, bear_detector: BearDetector) -> None:
    for reverse_candidates in list(bear_detector._candidate_points):
        for point in reverse_candidates:
            visualizer.draw(
                point,
                color=(255, 110, 0),
                point_radius_px=2,
            )


def draw_path(visualizer: Visualizer, path: list[Point]) -> None:
    if len(path) < 2:
        return

    for i in range(len(path) - 1):
        visualizer.draw(Line(a=path[i], b=path[i + 1]), color=(120, 170, 255), width_px=2)
    for p in path:
        visualizer.draw(p, color=(190, 220, 255), point_radius_px=3)
