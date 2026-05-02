

from enum import Enum, auto
from math import hypot
import time
from typing import Optional

from comm.controller import Controller
from comm.messages import ClawAction, ClawCommand, MoveCommand
from control.bear_approach import plan_bear_approach_path
from control.pure_pursuit import PurePursuitController
from geometry.shapes import Point
from geometry.transforms import Pose
from geometry.util import dist2
from localization.bear_detector import BearDetection
from localization.stack import LocalizationStack


GOAL_TOLERANCE_M = 0.08
MAX_SPEED = 1


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


class PursuitState(Enum):
    INIT = auto()
    START = auto()
    SEEKING_STARTUP_PATH = auto()
    SEEKING_DETECTED_BEAR = auto()
    CAPTURE_BEAR = auto()
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
