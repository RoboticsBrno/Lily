from __future__ import annotations

from math import ceil
from pathlib import Path

from comm.binary_serializer import BinarySerializer
from comm.controller import Controller
from comm.recording_transport import RecordingTransport
from comm.replay_transport import ReplayTransport
from comm.types import Transport
from geometry.shapes import Circle, Point
from geometry.transforms import Pose
from localization import BearDetectionConfig, BearDetector
from localization.particle_filter import ParticleFilterConfig, ParticleFilterLocalizer
from localization.stack import LocalizationStack
from map.loader import load_world_from_json
from map.raster import load_raster_map
from util.keyboard_controller import KeyboardRobotController


TARGET_FPS = 60
SIM_PORT = 5005
CONTROLLER_RECEIVE_PORT = 5006
LIDAR_MAX_DISTANCE = 8.0
LIDAR_HZ = 10
LIDAR_SAMPLE = 3900
LIDAR_HISTORY = ceil(LIDAR_SAMPLE / LIDAR_HZ / 2)
WHEEL_BASE = 0.25
PARTICLE_COUNT = 200


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


def build_controller(
    transport: Transport,
    recording_path: str = "recording.csv",
) -> Controller:
    return Controller(
        transport=RecordingTransport(transport, recording_path),
        serializer=BinarySerializer(),
    )


def build_replay_controller(
    recording_path: str = "recording.csv",
    speed: float = 1.0
) -> Controller:
    return Controller(
        transport=ReplayTransport(recording_path, speed=speed),
        serializer=BinarySerializer(),
    )


def connect_keyboard_ctrl(
    controller: Controller
) -> KeyboardRobotController:
    return KeyboardRobotController(controller)


def build_localization_stack(
    map_path: Path,
    initial_pose: Pose,
) -> LocalizationStack:
    world = load_world_from_json(map_path)
    localizer = ParticleFilterLocalizer(
        world=world,
        config=ParticleFilterConfig(
            num_particles=PARTICLE_COUNT,
            wheel_base=WHEEL_BASE,
            ticks_per_meter=1000.0,
            position_noise=0.007,
            heading_noise=0.01,
            estimate_smoothing_alpha=0.1,
            lidar_likelihood_map=load_raster_map("data/map_lidar_likelihood.npz"),
        ),
        initial_pose=initial_pose,
    )
    bear_detector = BearDetector(
        world=world,
        config=BearDetectionConfig(),
    )

    return LocalizationStack(world, localizer, bear_detector, LIDAR_HISTORY)
