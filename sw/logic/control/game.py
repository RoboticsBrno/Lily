

from enum import Enum, auto
import time
from typing import Optional

from comm.controller import Controller
from comm.messages import ClawCommand, MoveCommand
from control.bear_approach import plan_bear_approach_path
from control.pure_pursuit import PurePursuitController, ReverseMode
from geometry.shapes import Point
from geometry.transforms import Pose
from geometry.util import dist, dist2
from localization.bear_detector import BearDetection
from localization.stack import LocalizationStack

from params import (
    CLAW_PWM_FREE,
    GAME_30CM_TIMEOUT,
    GAME_CLAW_CLOSE_DELAY,
    GAME_GOAL_TOLERANCE,
    GAME_MAX_SPEED,
    GAME_PUSH_TIME,
    GAME_RETURN_RAMP_UP_TIME,
    GAME_STARTUP_DELAY,
    CLAW_PWM_CLOSE,
    CLAW_PWM_OPEN,
)


def _build_startup_s_path() -> list[Point]:
    return [
        Point(0.20, 0.10),
        Point(0.20, 0.75),
        Point(0.35, 0.90),
        Point(0.45, 0.90),
        Point(0.85, 0.50),
        Point(0.95, 0.50),
        Point(1.15, 0.70),
        Point(1.15, 1.60),
    ]


class PursuitState(Enum):
    INIT = auto()
    START = auto()
    SEEKING_STARTUP_PATH = auto()
    SEEKING_DETECTED_BEAR = auto()
    BEAR_PUSH = auto()
    CAPTURE_BEAR = auto()
    RETURN_REVERSE = auto()
    SEEKING_RETURN_PATH = auto()
    FINISHED = auto()


class GameStateMachine:
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
        self._start_requested = False
        self._last_loop_time = 0.0

    def request_start(self) -> None:
        self._start_requested = True

    def _on_last_segment(self) -> bool:
        return len(self.planned_path) >= 2 and self.pursuit.current_segment >= len(self.planned_path) - 3

    def _should_replan_to_bear(self, estimated: Pose, bear_detection: Optional[BearDetection]) -> bool:
        if bear_detection is None or bear_detection.position.y < 1.40:
            return False

        if estimated.x > 0.9 and 0.5 < estimated.y < 1.5:  # early detection
            return (
                (bear_detection.position.x > 1.1 or bear_detection.position.y > 1.7)
                and dist2(Point(estimated.x, estimated.y), bear_detection.position) > (0.5 * 0.5)
            )
        else:  # in area
            return (
                estimated.y > 1.5
                and dist2(Point(estimated.x, estimated.y), bear_detection.position) > (0.6 * 0.6)
            )

    def _replan_to_bear(self, estimated, bear_detection) -> None:
        assert bear_detection is not None
        start = Point(estimated.x, estimated.y)
        self.planned_path = plan_bear_approach_path(
            start=start,
            detected_bear=bear_detection.position,
            world=self.localization.world,
        )
        self.pursuit.update_plan(self.planned_path, ReverseMode.Disallow)

    def _stop_command(self) -> MoveCommand:
        return MoveCommand(left_speed=0, right_speed=0)

    def _follow_path(self, estimated: Pose, speed: float, dt: float) -> MoveCommand:
        control = self.pursuit.compute(estimated, dt)
        return MoveCommand(
            left_speed=_clamp_scale(control.left_power, -1, 1, speed),
            right_speed=_clamp_scale(control.right_power, -1, 1, speed),
        )

    def loop(self) -> None:
        now = time.time()
        dt = now - self._last_loop_time if self._last_loop_time > 0 else 0.0
        self._last_loop_time = now

        estimated = self.localization.localizer.get_estimate()
        bear_detection = self.localization.bear_detector.get_estimate()
        if self.state == PursuitState.FINISHED:
            self.controller.send_command(self._stop_command())
            self.controller.send_command(ClawCommand(pwm=CLAW_PWM_FREE))
            return

        if self.state == PursuitState.INIT:
            self.controller.send_command(self._stop_command())
            if self._start_requested:
                self.delay_end = time.time() + GAME_STARTUP_DELAY
                self.state = PursuitState.START
                self.controller.send_command(ClawCommand(pwm=CLAW_PWM_CLOSE))
                self._start_requested = False
            return

        if self.state == PursuitState.START:
            if time.time() > self.delay_end:
                self.planned_path = _build_startup_s_path() + [
                    Point(1.00, 2.10),
                    Point(0.50, 2.30),
                    Point(0.40, 1.80),
                    Point(1.1, 2.4),
                ]
                self.pursuit.update_plan(self.planned_path, ReverseMode.Disallow)
                self.state = PursuitState.SEEKING_STARTUP_PATH
                self.controller.send_command(self._follow_path(estimated, GAME_MAX_SPEED, dt))
            return

        if self.state == PursuitState.SEEKING_STARTUP_PATH:
            if self._should_replan_to_bear(estimated, bear_detection):
                self._replan_to_bear(estimated, bear_detection)
                self.state = PursuitState.SEEKING_DETECTED_BEAR
                self.controller.send_command(ClawCommand(pwm=CLAW_PWM_OPEN))
                self.delay_end = 0.0

            self.controller.send_command(self._follow_path(estimated, GAME_MAX_SPEED, dt))
            return

        if self.state == PursuitState.SEEKING_DETECTED_BEAR:
            if self.delay_end == 0.0 and dist2(Point(estimated.x, estimated.y), self.planned_path[-1]) < (0.3 * 0.3):
                self.controller.send_command(ClawCommand(pwm=CLAW_PWM_FREE))
                self.delay_end = time.time() + GAME_30CM_TIMEOUT
            if self.pursuit.at_goal(estimated, GAME_GOAL_TOLERANCE) or (self.delay_end != 0.0 and time.time() >= self.delay_end):
                self.state = PursuitState.BEAR_PUSH
                self.delay_end = time.time() + GAME_PUSH_TIME
                return

            speed = GAME_MAX_SPEED * min(1, dist(Point(estimated.x, estimated.y), self.planned_path[-1]) / 0.8)
            self.controller.send_command(self._follow_path(estimated, speed, dt))
            return

        if self.state == PursuitState.BEAR_PUSH:
            if time.time() >= self.delay_end:
                self.state = PursuitState.CAPTURE_BEAR
                self.delay_end = time.time() + GAME_CLAW_CLOSE_DELAY
                self.controller.send_command(self._stop_command())
                self.controller.send_command(ClawCommand(pwm=CLAW_PWM_CLOSE))

        if self.state == PursuitState.CAPTURE_BEAR:
            if time.time() >= self.delay_end:
                self.state = PursuitState.SEEKING_RETURN_PATH
                self.delay_end = time.time() + GAME_RETURN_RAMP_UP_TIME
                self.planned_path = list(reversed(_build_startup_s_path()))
                self.pursuit.update_plan(self.planned_path, ReverseMode.Force)
                self.controller.send_command(self._follow_path(estimated, 0, dt))
            return

        if self.state == PursuitState.SEEKING_RETURN_PATH:
            speed = GAME_MAX_SPEED * min(
                1,
                1 - (self.delay_end - time.time()) / GAME_RETURN_RAMP_UP_TIME,
                dist(Point(estimated.x, estimated.y), self.planned_path[-1]) / 0.3
            )
            if self.pursuit.at_goal(estimated, GAME_GOAL_TOLERANCE):
                self.state = PursuitState.FINISHED
                self.controller.send_command(self._stop_command())
                return

            self.controller.send_command(self._follow_path(estimated, speed, dt))
            return


def _clamp_scale(value: float, low: float = -1.0, high: float = 1.0, scale: float = 1.0) -> float:
    return max(low, min(high, value)) * scale
