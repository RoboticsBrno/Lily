from __future__ import annotations

from collections import deque
from enum import Enum, auto
from math import ceil, cos, hypot, pi, sin
from pathlib import Path
import time

from comm.controller import Controller
from comm.binary_serializer import BinarySerializer
from comm.messages import ClawAction, ClawCommand, Measurements, MoveCommand, SubscribeCommand
from comm.recording_transport import RecordingTransport
from comm.udp_transport import UdpTransport
from control.pure_pursuit import PurePursuitConfig, PurePursuitController
from control.bear_approach import plan_bear_approach_path
from geometry.shapes import Line, Point, Vector
from geometry.util import angular_distance, dist2
from localization.particle_filter import ParticleFilterLocalizer
from localization.bear_detector import BearDetector
from sim.robot import RobotConfig
from sim.sensors import LidarSensorConfig, MotorConfig
from sim.server import create_server_from_map
from util.remote_control_common import (
    build_localization_stack,
    create_default_bear,
    draw_bear,
    draw_bear_detection,
    draw_candidate_points,
    draw_estimated_pose,
    draw_lidar_history,
    draw_particles,
    handle_ui_control_event,
    resolve_map_path,
)
from util.visualizer import Visualizer

TARGET_FPS = 60
SIM_PORT = 5005
CONTROLLER_RECEIVE_PORT = 5006
LIDAR_MAX_DISTANCE = 8.0
LIDAR_HZ = 10
LIDAR_SAMPLE = 3900
LIDAR_HISTORY = ceil(LIDAR_SAMPLE / LIDAR_HZ)
WHEEL_BASE = 0.25
PARTICLE_COUNT = 200
PURSUIT_LOOKAHEAD = 0.18
COMMAND_SCALE = 0.5
GOAL_TOLERANCE_M = 0.08
MIN_BEAR_DIAMETER_M = 0.10
MAX_SPEED = 1

RETRIEVE_BLINDSPOT_RANGE = 0.5 * pi
RETRIEVE_BLINDSPOT_OFFSET = 0


class PursuitState(Enum):
    INIT = auto()
    START = auto()
    SEEKING_STARTUP_PATH = auto()
    SEEKING_DETECTED_BEAR = auto()
    CAPTURE_BEAR = auto()
    SEEKING_RETURN_PATH = auto()
    FINISHED = auto()


def _build_startup_s_path() -> list[Point]:
    return [
        Point(0.20, 0.10),
        Point(0.20, 0.75),
        Point(0.35, 0.90),
        Point(0.45, 0.90),
        Point(0.85, 0.50),
        Point(0.95, 0.50),
        Point(1.15, 0.70),
        Point(1.15, 1.50),
        Point(1.00, 1.80),
        Point(0.70, 2.10),
        # Point(0.70, 2.10),
        # Point(1.0, 2.1),
        # Point(1.2, 2.4),
    ]


class PursuitStateMachine:
    def __init__(
        self,
        pursuit: PurePursuitController,
        controller: Controller,
        localizer: ParticleFilterLocalizer,
        bear_detector: BearDetector,
        lidar_history: deque[tuple[Point, Vector]],
    ) -> None:
        self.planned_path: list[Point] = []
        self.pursuit = pursuit
        self.controller = controller
        self.localizer = localizer
        self.bear_detector = bear_detector
        self.lidar_history = lidar_history
        self.state = PursuitState.INIT
        self.delay_end = 0.0

    def _on_last_segment(self) -> bool:
        return len(self.planned_path) >= 2 and self.pursuit.current_segment >= len(self.planned_path) - 3

    def _should_replan_to_bear(self, estimated, bear_detection) -> bool:
        return (
            self._on_last_segment()
            and bear_detection is not None
            and bear_detection.position.y > 1.4
            and dist2(estimated, bear_detection.position) > (0.5 * 0.5)
        )

    def _replan_to_bear(self, estimated, bear_detection) -> None:
        assert bear_detection is not None
        start = Point(estimated.x, estimated.y)
        self.planned_path = plan_bear_approach_path(
            start=start,
            detected_bear=bear_detection.position,
            world=self.bear_detector.world,
        )
        self.pursuit.update_plan(self.planned_path, False)

    def _at_goal(self, estimated) -> bool:
        if not self.planned_path:
            return True
        goal = self.planned_path[-1]
        return hypot(goal.x - estimated.x, goal.y - estimated.y) <= GOAL_TOLERANCE_M

    def _stop_command(self) -> MoveCommand:
        return MoveCommand(left_speed=0, right_speed=0)

    def _follow_path(self, estimated) -> MoveCommand:
        control = self.pursuit.compute(estimated)
        return MoveCommand(
            left_speed=_clamp_scale(control.left_power, -1, 1, MAX_SPEED),
            right_speed=_clamp_scale(control.right_power, -1, 1, MAX_SPEED),
        )

    def _loop(self, estimated, bear_detection) -> None:
        if self.state == PursuitState.FINISHED:
            self.controller.send_command(self._stop_command())
            return

        if self.state == PursuitState.INIT:
            self.delay_end = time.time() + 2.0
            self.state = PursuitState.START
            self.controller.send_command(ClawCommand(ClawAction.CLOSE))
            return

        if self.state == PursuitState.START:
            if time.time() > self.delay_end:
                self.planned_path = _build_startup_s_path()
                self.pursuit.update_plan(self.planned_path, True)
                self.state = PursuitState.SEEKING_STARTUP_PATH
                self.controller.send_command(self._follow_path(estimated))
            return

        if self.state == PursuitState.SEEKING_STARTUP_PATH:
            if self._should_replan_to_bear(estimated, bear_detection):
                self._replan_to_bear(estimated, bear_detection)
                self.state = PursuitState.SEEKING_DETECTED_BEAR
                self.controller.send_command(ClawCommand(ClawAction.OPEN))

            self.controller.send_command(self._follow_path(estimated))
            return

        if self.state == PursuitState.SEEKING_DETECTED_BEAR:
            if self._at_goal(estimated):
                self.state = PursuitState.CAPTURE_BEAR
                self.delay_end = time.time() + 2.0
                self.controller.send_command(self._stop_command())
                self.controller.send_command(ClawCommand(ClawAction.CLOSE))
                return

            self.controller.send_command(self._follow_path(estimated))
            return

        if self.state == PursuitState.CAPTURE_BEAR:
            if time.time() >= self.delay_end:
                self.state = PursuitState.SEEKING_RETURN_PATH
                self.planned_path = list(reversed(_build_startup_s_path()))
                self.pursuit.update_plan(self.planned_path, True)
                self.controller.send_command(self._follow_path(estimated))
            return

        if self.state == PursuitState.SEEKING_RETURN_PATH:
            if self._at_goal(estimated):
                self.state = PursuitState.FINISHED
                self.controller.send_command(self._stop_command())
                return

            self.controller.send_command(self._follow_path(estimated))
            return

    def process_tick(self, measurements: list[Measurements]):
        for m in measurements:
            if self.state in [PursuitState.CAPTURE_BEAR, PursuitState.SEEKING_RETURN_PATH]:
                m.lidar = list(filter(lambda lm: abs(angular_distance(lm.angle, RETRIEVE_BLINDSPOT_OFFSET)) > RETRIEVE_BLINDSPOT_RANGE / 2.0, m.lidar))
            self.localizer.update(m)
            estimated_pose = self.localizer.estimate_pose()
            feature_points = self.bear_detector.update(estimated_pose, m.lidar)
            for point, feature in feature_points:
                self.lidar_history.append((point, feature))

        estimated = self.localizer.estimate_pose()
        bear_detection = self.bear_detector.get_estimate()
        self._loop(estimated, bear_detection)
        return estimated, bear_detection


def _draw_path(visualizer: Visualizer, path: list[Point]) -> None:
    if len(path) < 2:
        return

    for i in range(len(path) - 1):
        visualizer.draw(Line(a=path[i], b=path[i + 1]), color=(120, 170, 255), width_px=2)
    for p in path:
        visualizer.draw(p, color=(190, 220, 255), point_radius_px=3)


def _clamp_scale(value: float, low: float = -1.0, high: float = 1.0, scale: float = 1.0) -> float:
    return max(low, min(high, value)) * scale


def main() -> None:
    repo_root = Path(__file__).resolve().parent
    map_path = resolve_map_path(repo_root, "data/map_bear_rescue.json")

    bear = create_default_bear()

    server = create_server_from_map(
        map_path=map_path,
        robot_config=RobotConfig(wheel_base=WHEEL_BASE),
        lidar_config=LidarSensorConfig(
            rotation_speed_hz=LIDAR_HZ,
            measurement_frequency_hz=LIDAR_SAMPLE,
            max_distance=LIDAR_MAX_DISTANCE,
            world_max_incidence_angle=pi / 10,
            bear_max_incidence_angle=pi / 2.0,
            distance_noise_stddev=0.003,
            angle_noise_stddev=0.001,
        ),
        motor_config=MotorConfig(
            ticks_per_meter=1000.0,
            max_speed=MAX_SPEED,
        ),
        bear=bear,
        transport=UdpTransport(
            host="127.0.0.1",
            port=CONTROLLER_RECEIVE_PORT,
            receive_port=SIM_PORT,
        ),
        publish_hz=30.0,
    )

    controller = Controller(
        transport=RecordingTransport(
            UdpTransport(
                host="127.0.0.1",
                port=SIM_PORT,
                receive_port=CONTROLLER_RECEIVE_PORT,
            ),
            "recording.csv",
        ),
        serializer=BinarySerializer(),
    )

    visualizer = Visualizer(
        title="Simulator + Pure Pursuit Demo",
    )

    world, localizer, bear_detector = build_localization_stack(map_path, server.get_true_pose())

    measurement_queue: deque[Measurements] = deque(maxlen=4000)

    def _enqueue_measurements(measurements: Measurements) -> None:
        measurement_queue.append(measurements)

    controller.set_measurement_callback(_enqueue_measurements)

    lidar_history: deque[tuple[Point, Vector]] = deque(maxlen=LIDAR_HISTORY)
    pursuit_state_machine = PursuitStateMachine(
        pursuit=PurePursuitController(PurePursuitConfig(lookahead_distance=PURSUIT_LOOKAHEAD)),
        controller=controller,
        localizer=localizer,
        bear_detector=bear_detector,
        lidar_history=lidar_history,
    )
    resizing_bear_with_right_drag = False

    def on_event(event) -> None:
        nonlocal resizing_bear_with_right_drag
        resizing_bear_with_right_drag = handle_ui_control_event(
            event,
            visualizer,
            bear,
            resizing_bear_with_right_drag,
        )

    def on_tick(dt_seconds: float) -> None:
        _ = dt_seconds

        truth = server.get_true_pose()
        heading_length = server.robot.config.diameter * 0.5
        measurements: list[Measurements] = list(measurement_queue)
        measurement_queue.clear()
        estimated, bear_detection = pursuit_state_machine.process_tick(measurements)

        visualizer.draw(world, color=(224, 228, 236), width_px=3)
        draw_bear(visualizer, bear)
        _draw_path(visualizer, pursuit_state_machine.planned_path)
        robot_center = Point(truth.x, truth.y)
        heading_tip = Point(
            truth.x + heading_length * cos(truth.yaw),
            truth.y + heading_length * sin(truth.yaw),
        )

        visualizer.draw(server.robot.get_body_circle(), color=(255, 120, 90), width_px=2)
        visualizer.draw(Line(a=robot_center, b=heading_tip), color=(255, 200, 90), width_px=2)
        left_claw, right_claw = server.robot.get_claw_segments()
        visualizer.draw(left_claw, color=(255, 150, 120), width_px=2)
        visualizer.draw(right_claw, color=(255, 150, 120), width_px=2)
        visualizer.draw(robot_center, color=(255, 255, 255), point_radius_px=2)

        draw_estimated_pose(visualizer, estimated, heading_length)
        draw_bear_detection(visualizer, bear_detection, estimated)
        draw_particles(visualizer, localizer)
        # draw_lidar_history(visualizer, lidar_history, show_magnitude=False)
        draw_candidate_points(visualizer, bear_detector)

    visualizer.on_tick = on_tick
    visualizer.on_event = on_event

    server.start()
    controller.start()
    controller.send_command(SubscribeCommand())
    try:
        visualizer.run(target_fps=max(1, TARGET_FPS))
    finally:
        controller.send_command(MoveCommand(left_speed=0, right_speed=0))
        controller.stop()
        server.stop()


if __name__ == "__main__":
    main()
