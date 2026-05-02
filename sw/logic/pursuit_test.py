from __future__ import annotations

from collections import deque
from enum import Enum, auto
from math import ceil, cos, hypot, pi, sin
from pathlib import Path
import time
from typing import Optional

from comm.controller import Controller
from comm.messages import ClawAction, ClawCommand, Measurements, MoveCommand, SubscribeCommand
from comm.udp_transport import UdpTransport
from control.pure_pursuit import PurePursuitConfig, PurePursuitController
from control.bear_approach import plan_bear_approach_path
from geometry.shapes import Line, Point, Vector
from geometry.util import dist2
from localization.bear_detector import BearDetection
from geometry.transforms import Pose
from sim.robot import RobotConfig
from sim.sensors import LidarSensorConfig, MotorConfig
from sim.server import create_server_from_map
from logic.util.init_common import (
    CONTROLLER_RECEIVE_PORT,
    LIDAR_HZ,
    LIDAR_MAX_DISTANCE,
    LIDAR_SAMPLE,
    SIM_PORT,
    TARGET_FPS,
    WHEEL_BASE,
    LocalizationStack,
    build_controller,
    build_localization_stack,
    create_default_bear,
    resolve_map_path,
)
from util.vis_common import (
    draw_bear,
    draw_bear_detection,
    draw_candidate_points,
    draw_estimated_pose,
    # draw_lidar_history,
    draw_particles,
    draw_path,
    handle_ui_control_event
)
from util.visualizer import Visualizer


PURSUIT_LOOKAHEAD = 0.18
COMMAND_SCALE = 0.5
GOAL_TOLERANCE_M = 0.08
MAX_SPEED = 1

RETRIEVE_BLINDSPOT_RANGE = 0.5 * pi
RETRIEVE_BLINDSPOT_OFFSET = 0
ROBOT_BODY_RADIUS = 0.125


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
        localization: LocalizationStack
    ) -> None:
        self.planned_path: list[Point] = []
        self.pursuit = pursuit
        self.controller = controller
        self.localization = localization
        self.state = PursuitState.INIT
        self.delay_end = 0.0

    def _on_last_segment(self) -> bool:
        return len(self.planned_path) >= 2 and self.pursuit.current_segment >= len(self.planned_path) - 3

    def _should_replan_to_bear(self, estimated: Pose, bear_detection: Optional[BearDetection]) -> bool:
        return (
            self._on_last_segment()
            and bear_detection is not None
            and bear_detection.position.y > 1.4
            and dist2(Point(estimated.x, estimated.y), bear_detection.position) > (0.5 * 0.5)
        )

    def _replan_to_bear(self, estimated, bear_detection) -> None:
        assert bear_detection is not None
        start = Point(estimated.x, estimated.y)
        self.planned_path = plan_bear_approach_path(
            start=start,
            detected_bear=bear_detection.position,
            world=self.localization.world,
        )
        self.pursuit.update_plan(self.planned_path, False)

    def _at_goal(self, estimated: Pose) -> bool:
        if not self.planned_path:
            return True
        goal = self.planned_path[-1]
        return hypot(goal.x - estimated.x, goal.y - estimated.y) <= GOAL_TOLERANCE_M

    def _stop_command(self) -> MoveCommand:
        return MoveCommand(left_speed=0, right_speed=0)

    def _follow_path(self, estimated: Pose) -> MoveCommand:
        control = self.pursuit.compute(estimated)
        return MoveCommand(
            left_speed=_clamp_scale(control.left_power, -1, 1, MAX_SPEED),
            right_speed=_clamp_scale(control.right_power, -1, 1, MAX_SPEED),
        )

    def loop(self) -> None:
        estimated = self.localization.localizer.get_estimate()
        bear_detection = self.localization.bear_detector.get_estimate()
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

    visualizer = Visualizer(title="Simulator + Pure Pursuit Demo")

    localization = build_localization_stack(map_path, server.get_true_pose())
    controller = build_controller(
        UdpTransport(
            host="127.0.0.1",
            port=SIM_PORT,
            receive_port=CONTROLLER_RECEIVE_PORT,
        )
    )

    def on_measurements(measurements: Measurements):
        print("on measurements")
        localization.on_measurements(measurements)
        pursuit_state_machine.loop()

    controller.set_measurement_callback(on_measurements)

    pursuit_state_machine = PursuitStateMachine(
        pursuit=PurePursuitController(PurePursuitConfig(lookahead_distance=PURSUIT_LOOKAHEAD)),
        controller=controller,
        localization=localization
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

    def on_ui_tick(dt_seconds: float) -> None:
        _ = dt_seconds
        controller.send_command(SubscribeCommand())

        truth = server.get_true_pose()
        estimated = localization.localizer.get_estimate()
        bear_detection = localization.bear_detector.get_estimate()
        print(f"State: {pursuit_state_machine.state}, Estimated: {estimated.x:.3f}, {estimated.y:.3f}, {estimated.yaw:.2f}")

        visualizer.draw(localization.world, color=(224, 228, 236))
        draw_bear(visualizer, bear)
        draw_path(visualizer, pursuit_state_machine.planned_path)
        heading_length = ROBOT_BODY_RADIUS
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
        draw_particles(visualizer, localization.localizer)
        # draw_lidar_history(visualizer, lidar_history, show_magnitude=False)
        draw_candidate_points(visualizer, localization.bear_detector)

    visualizer.on_tick = on_ui_tick
    visualizer.on_event = on_event

    visualizer.init()
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
