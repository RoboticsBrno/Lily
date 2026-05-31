from __future__ import annotations

from math import ceil
from pathlib import Path

from comm.binary_serializer import BinarySerializer
from comm.controller import Controller
from comm.recording_transport import RecordingTransport
from comm.replay_transport import ReplayTransport
from comm.types import Transport
from comm.udp_transport import UdpTransport
from sim.robot import RobotConfig
from sim.sensors import LidarSensorConfig, MotorConfig
from sim.server import RobotSimulatorServer, create_server_from_map
from geometry.shapes import Circle, Point, Vector
from geometry.transforms import Pose
from localization import BearDetectionConfig, BearDetector, DBSCANConfig
from localization.particle_filter import ParticleFilterConfig, ParticleFilterLocalizer
from localization.stack import LocalizationStack, RobotParams
from map.loader import load_world_from_json
from map.raster import load_raster_map
from util.keyboard_controller import KeyboardRobotController
from params import (
    BEAR_CONFIDENCE_THRESHOLD,
    BEAR_DBSCAN_EPS,
    BEAR_DBSCAN_MIN_SAMPLES,
    BEAR_FEATURE_DETECTION_POINTS,
    BEAR_FEATURE_THRESHOLD,
    BEAR_LINE_COVARIANCE_RATIO,
    BEAR_MATCH_THRESHOLD,
    BEAR_MAX_DISTANCE,
    BEAR_MIN_DISTANCE,
    DEFAULT_BEAR_RADIUS,
    DEFAULT_BEAR_X,
    DEFAULT_BEAR_Y,
    KB_MAX_SPEED,
    KB_MOVE_POWER,
    KB_TURN_POWER,
    ROBOT_TICKS_PER_METER,
    ROBOT_WHEEL_BASE,
    LIDAR_OFFSET,
    SIM_CLAW_CLOSED_ANGLE,
    SIM_CLAW_LENGTH,
    SIM_CLAW_OFFSET,
    SIM_CLAW_OPEN_ANGLE,
    SIM_LIDAR_ANGLE_MAX,
    SIM_LIDAR_ANGLE_MIN,
    SIM_LIDAR_ANGLE_NOISE,
    SIM_LIDAR_BEAR_MAX_INCIDENCE,
    SIM_LIDAR_DIST_NOISE,
    SIM_LIDAR_MAX_RANGE,
    SIM_LIDAR_RANDOM_DIST_PROB,
    SIM_LIDAR_ROTATION_HZ,
    SIM_LIDAR_SAMPLE_RATE,
    SIM_LIDAR_WORLD_MAX_INCIDENCE,
    SIM_MOTOR_MAX_SPEED,
    SIM_PUBLISH_HZ,
    ROBOT_BODY_RADIUS,
    PF_HEADING_NOISE,
    PF_BLOCKED_HEADING_NOISE,
    PF_NUM_PARTICLES,
    PF_POSITION_NOISE,
    PF_SMOOTHING_ALPHA,
)


TARGET_FPS = 60
SIM_PORT = 5005
CONTROLLER_RECEIVE_PORT = 5006
LIDAR_HISTORY = ceil(SIM_LIDAR_SAMPLE_RATE / SIM_LIDAR_ROTATION_HZ / 2)


def resolve_map_path(repo_root: Path, map_arg: str) -> Path:
    map_path = Path(map_arg)
    if not map_path.is_absolute():
        map_path = repo_root / map_path
    return map_path


def create_default_bear() -> Circle:
    return Circle(
        center=Point(DEFAULT_BEAR_X, DEFAULT_BEAR_Y),
        radius=DEFAULT_BEAR_RADIUS,
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
    return KeyboardRobotController(
        controller,
        move_power=KB_MOVE_POWER,
        turn_power=KB_TURN_POWER,
        max_speed=KB_MAX_SPEED,
    )


def build_localization_stack(
    map_path: Path,
    initial_pose: Pose,
) -> LocalizationStack:
    world = load_world_from_json(map_path)
    localizer = ParticleFilterLocalizer(
        world=world,
        config=ParticleFilterConfig(
            num_particles=PF_NUM_PARTICLES,
            position_noise=PF_POSITION_NOISE,
            heading_noise=PF_HEADING_NOISE,
            blocked_heading_noise=PF_BLOCKED_HEADING_NOISE,
            estimate_smoothing_alpha=PF_SMOOTHING_ALPHA,
            lidar_likelihood_map=load_raster_map("data/map_lidar_likelihood.npz"),
            motion_map=load_raster_map("data/map_motion_model.npz"),
        ),
        initial_pose=initial_pose,
    )
    bear_detector = BearDetector(
        world=world,
        config=BearDetectionConfig(
            min_distance=BEAR_MIN_DISTANCE,
            max_distance=BEAR_MAX_DISTANCE,
            feature_detection_points=BEAR_FEATURE_DETECTION_POINTS,
            feature_threshold=BEAR_FEATURE_THRESHOLD,
            line_covariance_ratio_threshold=BEAR_LINE_COVARIANCE_RATIO,
            dbscan_config=DBSCANConfig(
                eps=BEAR_DBSCAN_EPS,
                min_samples=BEAR_DBSCAN_MIN_SAMPLES,
            ),
            match_threshold=BEAR_MATCH_THRESHOLD,
            confidence_threshold=BEAR_CONFIDENCE_THRESHOLD,
        ),
    )

    return LocalizationStack(
        world,
        LIDAR_OFFSET,
        localizer,
        bear_detector,
        RobotParams(
            wheel_base=ROBOT_WHEEL_BASE,
            ticks_per_meter=ROBOT_TICKS_PER_METER,
        ),
        LIDAR_HISTORY
    )


def make_default_sim(
    map_path: Path,
    publish_hz: float = SIM_PUBLISH_HZ,
    max_speed: float = SIM_MOTOR_MAX_SPEED,
    random_distance_probability: float = SIM_LIDAR_RANDOM_DIST_PROB,
    transport: Transport | None = None,
) -> tuple[RobotSimulatorServer, Circle]:
    bear = create_default_bear()

    if transport is None:
        transport = UdpTransport(
            host="127.0.0.1",
            port=CONTROLLER_RECEIVE_PORT,
            receive_port=SIM_PORT,
        )

    server = create_server_from_map(
        map_path=map_path,
        robot_config=RobotConfig(
            wheel_base=ROBOT_WHEEL_BASE,
            radius=ROBOT_BODY_RADIUS,
            claw_offset=SIM_CLAW_OFFSET,
            claw_length=SIM_CLAW_LENGTH,
            claw_open_angle=SIM_CLAW_OPEN_ANGLE,
            claw_closed_angle=SIM_CLAW_CLOSED_ANGLE,
        ),
        lidar_config=LidarSensorConfig(
            rotation_speed_hz=SIM_LIDAR_ROTATION_HZ,
            measurement_frequency_hz=SIM_LIDAR_SAMPLE_RATE,
            angle_min=SIM_LIDAR_ANGLE_MIN,
            angle_max=SIM_LIDAR_ANGLE_MAX,
            max_distance=SIM_LIDAR_MAX_RANGE,
            lidar_offset=LIDAR_OFFSET,
            world_max_incidence_angle=SIM_LIDAR_WORLD_MAX_INCIDENCE,
            bear_max_incidence_angle=SIM_LIDAR_BEAR_MAX_INCIDENCE,
            distance_noise_stddev=SIM_LIDAR_DIST_NOISE,
            angle_noise_stddev=SIM_LIDAR_ANGLE_NOISE,
            random_distance_probability=random_distance_probability,
        ),
        motor_config=MotorConfig(
            ticks_per_meter=ROBOT_TICKS_PER_METER,
            max_speed=max_speed,
        ),
        bear=bear,
        transport=transport,
        publish_hz=publish_hz,
    )
    return server, bear
