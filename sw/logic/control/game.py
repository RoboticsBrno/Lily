

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
        Point(0.2, 0.1),
        Point(0.1987, 0.1507),
        Point(0.1954, 0.2023),
        Point(0.1905, 0.2545),
        Point(0.1848, 0.307),
        Point(0.179, 0.3597),
        Point(0.1735, 0.4121),
        Point(0.1692, 0.4641),
        Point(0.1665, 0.5154),
        Point(0.1661, 0.5657),
        Point(0.1688, 0.6148),
        Point(0.175, 0.6624),
        Point(0.1855, 0.7083),
        Point(0.2009, 0.7521),
        Point(0.2253, 0.7951),
        Point(0.2593, 0.8358),
        Point(0.2995, 0.8708),
        Point(0.3424, 0.8966),
        Point(0.3907, 0.9086),
        Point(0.4424, 0.9028),
        Point(0.4843, 0.8822),
        Point(0.5226, 0.8534),
        Point(0.5582, 0.8183),
        Point(0.5919, 0.7784),
        Point(0.6243, 0.7356),
        Point(0.6562, 0.6914),
        Point(0.6882, 0.6475),
        Point(0.7211, 0.6056),
        Point(0.7554, 0.5674),
        Point(0.792, 0.5345),
        Point(0.8315, 0.5087),
        Point(0.8768, 0.4929),
        Point(0.9297, 0.494),
        Point(0.9744, 0.5116),
        Point(1.0172, 0.5388),
        Point(1.058, 0.5729),
        Point(1.0948, 0.612),
        Point(1.1257, 0.654),
        Point(1.1488, 0.6971),
        Point(1.1649, 0.7406),
        Point(1.1773, 0.7856),
        Point(1.1864, 0.8321),
        Point(1.1925, 0.8797),
        Point(1.196, 0.9285),
        Point(1.1971, 0.9782),
        Point(1.1963, 1.0287),
        Point(1.1938, 1.08),
        Point(1.1899, 1.1317),
        Point(1.1851, 1.1839),
        Point(1.1796, 1.2364),
        Point(1.1737, 1.289),
        Point(1.1678, 1.3417),
        Point(1.1623, 1.3941),
        Point(1.1574, 1.4464),
        Point(1.1535, 1.4982),
        Point(1.1509, 1.5494),
        Point(1.15, 1.6),
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
                self.delay_end = time.time() + GAME_30CM_TIMEOUT
            if self.pursuit.at_goal(estimated, GAME_GOAL_TOLERANCE) or (self.delay_end != 0.0 and time.time() >= self.delay_end):
                self.state = PursuitState.BEAR_PUSH
                self.delay_end = time.time() + GAME_PUSH_TIME
                return

            speed = GAME_MAX_SPEED * min(1, max(0.2, dist(Point(estimated.x, estimated.y), self.planned_path[-1]) / 0.6))
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
                dist(Point(estimated.x, estimated.y), self.planned_path[-1]) / 0.2
            )
            if self.pursuit.at_goal(estimated, GAME_GOAL_TOLERANCE):
                self.state = PursuitState.FINISHED
                self.controller.send_command(self._stop_command())
                return

            self.controller.send_command(self._follow_path(estimated, speed, dt))
            return


def _clamp_scale(value: float, low: float = -1.0, high: float = 1.0, scale: float = 1.0) -> float:
    return max(low, min(high, value)) * scale
