from __future__ import annotations

from collections import deque
from math import ceil, cos, hypot, sin
from pathlib import Path
from typing import Any

from comm.binary_serializer import BinarySerializer
from comm.controller import Controller
from comm.recording_transport import RecordingTransport
from comm.replay_transport import ReplayTransport
from comm.types import Transport
from geometry.shapes import Circle, Line, Point, Vector
from geometry.transforms import Pose
from localization import BearDetectionConfig, BearDetector
from localization.particle_filter import ParticleFilterConfig, ParticleFilterLocalizer
from map.loader import load_world_from_json
from util.keyboard_controller import KeyboardRobotController
from util.visualizer import Visualizer

import pygame


TARGET_FPS = 60
SIM_PORT = 5005
CONTROLLER_RECEIVE_PORT = 5006
LIDAR_MAX_DISTANCE = 8.0
LIDAR_HZ = 10
LIDAR_SAMPLE = 3900
LIDAR_HISTORY = ceil(LIDAR_SAMPLE / LIDAR_HZ / 2)
WHEEL_BASE = 0.25
PARTICLE_COUNT = 400
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


def resolve_map_path(repo_root: Path, map_arg: str) -> Path:
    map_path = Path(map_arg)
    if not map_path.is_absolute():
        map_path = repo_root / map_path
    return map_path


def create_default_bear() -> Circle:
    return Circle(
        center=Point(0.05, 1.45),
        radius=0.05,
    )


def build_keyboard_controller(
    transport: Transport,
    recording_path: str = "recording.csv",
    max_measurements: int = 4000,
) -> KeyboardRobotController:
    return KeyboardRobotController(
        controller=Controller(
            transport=RecordingTransport(transport, recording_path),
            serializer=BinarySerializer(),
        ),
        max_measurements=max_measurements,
    )


def build_replay_player(
    recording_path: str = "recording.csv",
    speed: float = 1.0,
    max_measurements: int = 4000,
) -> KeyboardRobotController:
    return KeyboardRobotController(
        controller=Controller(
            transport=ReplayTransport(recording_path, speed=speed),
            serializer=BinarySerializer(),
        ),
        max_measurements=max_measurements,
    )


def build_localization_stack(
    map_path: Path,
    initial_pose: Pose,
) -> tuple[Any, ParticleFilterLocalizer, BearDetector]:
    world = load_world_from_json(map_path)
    localizer = ParticleFilterLocalizer(
        world=world,
        config=ParticleFilterConfig(
            num_particles=PARTICLE_COUNT,
            wheel_base=WHEEL_BASE,
            ticks_per_meter=1000.0,
            position_noise=0.007,
            heading_noise=0.01,
            lidar_likelihood_stddev=0.08,
            estimate_smoothing_alpha=0.1,
        ),
        initial_pose=initial_pose,
    )
    bear_detector = BearDetector(
        world=world,
        config=BearDetectionConfig(),
    )
    return world, localizer, bear_detector


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


def process_measurements(
    controller: KeyboardRobotController,
    localizer: ParticleFilterLocalizer,
    bear_detector: BearDetector,
    lidar_history: deque[tuple[Point, Vector]],
) -> tuple[Pose, Any]:
    for measurement in controller.get_measurements():
        localizer.update(measurement)
        estimated_pose = localizer.estimate_pose()

        feature_points = bear_detector.update(estimated_pose, measurement.lidar)
        for point, feature in feature_points:
            lidar_history.append((point, feature))

    return localizer.estimate_pose(), bear_detector.get_estimate()


def draw_particles(visualizer: Visualizer, localizer: ParticleFilterLocalizer) -> None:
    for particle in localizer.particles:
        visualizer.draw(
            Circle(Point(particle.pose.x, particle.pose.y), radius=(particle.weight * PARTICLE_COUNT) / 300),
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
    for lidar_point, feature in lidar_history:
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


def draw_candidate_points(visualizer: Visualizer, bear_detector: BearDetector) -> None:
    for reverse_candidates in bear_detector._candidate_points:
        for point in reverse_candidates:
            visualizer.draw(
                point,
                color=(255, 110, 0),
                point_radius_px=2,
            )
