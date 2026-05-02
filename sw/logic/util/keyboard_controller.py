from __future__ import annotations

from typing import Optional

from comm import (
    ClawAction,
    ClawCommand,
    Controller,
    MoveCommand,
)


class KeyboardRobotController:
    def __init__(
        self,
        controller: Controller,
        move_power: float = 1.0,
        turn_power: float = 0.75,
        max_speed: float = 1.2
    ) -> None:
        if move_power < 0.0:
            raise ValueError("move_power must be non-negative")
        if turn_power < 0.0:
            raise ValueError("turn_power must be non-negative")

        self.move_power = move_power
        self.turn_power = turn_power
        self._max_speed = max_speed

        self._controller = controller

        self._running = False
        self._last_move: tuple[Optional[float], Optional[float]] = (None, None)

        self._forward = False
        self._backward = False
        self._left = False
        self._right = False

    def key_down(self, key: str) -> None:
        normalized = key.lower()

        if normalized in ("w", "up"):
            self._forward = True
            self._send_move_from_state()
        elif normalized in ("s", "down"):
            self._backward = True
            self._send_move_from_state()
        elif normalized in ("a", "left"):
            self._left = True
            self._send_move_from_state()
        elif normalized in ("d", "right"):
            self._right = True
            self._send_move_from_state()
        elif normalized in ("space", "x"):
            self.send_stop()
        elif normalized == "o":
            self._controller.send_command(ClawCommand(action=ClawAction.OPEN))
        elif normalized == "c":
            self._controller.send_command(ClawCommand(action=ClawAction.CLOSE))

    def key_up(self, key: str) -> None:
        normalized = key.lower()

        if normalized in ("w", "up"):
            self._forward = False
            self._send_move_from_state()
        elif normalized in ("s", "down"):
            self._backward = False
            self._send_move_from_state()
        elif normalized in ("a", "left"):
            self._left = False
            self._send_move_from_state()
        elif normalized in ("d", "right"):
            self._right = False
            self._send_move_from_state()

    def send_stop(self) -> None:
        self._forward = False
        self._backward = False
        self._left = False
        self._right = False
        self._send_move(0, 0)

    def _send_move_from_state(self) -> None:
        linear = float(self._forward) - float(self._backward)
        turn = float(self._left) - float(self._right)

        left_power = linear * self.move_power - turn * self.turn_power
        right_power = linear * self.move_power + turn * self.turn_power

        self._send_move(self._clamp_scale(left_power), self._clamp_scale(right_power))

    def _send_move(self, left_speed: int, right_speed: int) -> None:
        if (left_speed, right_speed) == self._last_move:
            return

        self._controller.send_command(
            MoveCommand(left_speed=left_speed, right_speed=right_speed)
        )
        self._last_move = (left_speed, right_speed)

    def _clamp_scale(self, value: float, low: float = -1.0, high: float = 1.0) -> int:
        return round(max(low, min(high, value)) * self._max_speed)
